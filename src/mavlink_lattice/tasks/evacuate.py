"""NGC2 Evacuate: a pickup and a dropoff position for a vehicle the operator drives.

The bridge never drives the vehicle for this task. It uploads the next position to PX4 as a one-waypoint mission and
says so in a STATUSTEXT. The operator loads that mission into the ground station, plans the route to it and starts it.
The pickup goes first, so a staged mission started as-is stops there. The task moves on from a position once the
vehicle has stopped there, and is done at the last one.

Staging waits in two cases, and tells the operator why:
- The vehicle is in Mission mode, armed or not. PX4 starts a mission uploaded then, or on its next arming.
- After the first position, the vehicle's mission is one the operator planned and hasn't finished. A route through the
  pickup to the dropoff is one.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time

from pymavlink.dialects.v20 import common as mavlink

from .. import geo
from ..config import Config
from ..lattice.entity import Route
from ..vehicle import CommandError, Mode
from .base import Executor, TaskContext, TaskRejected

log = logging.getLogger(__name__)

_LAT_LON = (("lat", "lon"), ("latitude", "longitude"), ("latitudeDegrees", "longitudeDegrees"))
_WRAPPERS = ("lla", "point", "position", "llaPoint", "location")
# Stopped within the arrival radius for this long means the vehicle is at the position, not driving past it.
STOPPED_MPS = 0.5
DWELL_S = 3.0
STAGE_RETRY_S = 5.0
# QGC lists NOTICE quietly. The operator has to act on these.
_SHOW = mavlink.MAV_SEVERITY_WARNING
# Missions an upload may replace: nothing left to drive.
_REPLACEABLE = {mavlink.MISSION_STATE_NO_MISSION, mavlink.MISSION_STATE_NOT_STARTED, mavlink.MISSION_STATE_COMPLETE}


class _HeldBack(Exception):
    pass


def _camel(name: str) -> str:
    head, *rest = name.split("_")
    return head + "".join(w[:1].upper() + w[1:] for w in rest)


def at_path(spec: dict, path: str):
    """The value at a dotted proto field path, written in snake_case or in proto JSON's camelCase. None if it's absent."""
    node = spec
    for key in path.split("."):
        if not isinstance(node, dict):
            return None
        node = node[key] if key in node else node.get(_camel(key))
    return node


def find_lla(node, depth: int = 3) -> tuple[float, float] | None:
    if not isinstance(node, dict):
        return None
    for lat_key, lon_key in _LAT_LON:
        lat, lon = node.get(lat_key), node.get(lon_key)
        if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
            return (float(lat), float(lon)) if -90 <= lat <= 90 and -180 <= lon <= 180 else None
    if depth:
        for key in _WRAPPERS:
            if key in node:
                return find_lla(node[key], depth - 1)
    return None


class Evacuate(Executor):
    drives = False

    @classmethod
    def advertised_type(cls, cfg: Config) -> str:
        return cfg.tasks.evacuate.type_url

    def __init__(self, ctx: TaskContext, spec: dict):
        super().__init__(ctx, spec)
        cfg = ctx.cfg.tasks.evacuate
        self.radius_m = cfg.arrival_radius_m
        self.legs: list[tuple[str, tuple[float, float]]] = []
        for name, path in (("pickup", cfg.pickup), ("dropoff", cfg.dropoff)):
            node = at_path(spec, path)
            if node is None:
                continue
            lla = find_lla(node)
            if lla is None:
                log.warning("Evacuate: no usable lat/lon at %r; the task was %s", path, json.dumps(spec, default=str))
                raise TaskRejected(f"{name} at {path} has no valid lat/lon")
            self.legs.append((name, lla))
        if not self.legs:
            log.warning("Evacuate: nothing at %r or %r; the task was %s", cfg.pickup, cfg.dropoff, json.dumps(spec, default=str))
            raise TaskRejected(f"Evacuate has no position at {cfg.pickup} or {cfg.dropoff}")
        self.leg = 0
        self.staged = -1  # index of the leg uploaded to the vehicle
        self._staged_count: int | None = None  # mission items in that upload
        self._retry_at = 0.0
        self._told: str | None = None
        self._stopped_since: float | None = None
        self._edited = False
        self.route = self._route()

    def continue_from(self, old: Executor) -> None:
        self._edited = True
        if not isinstance(old, Evacuate):
            return
        reached = old.legs[: old.leg]
        if len(reached) < len(self.legs) and self.legs[: len(reached)] == reached:
            self.leg = len(reached)
            # The operator may have planned a route to the staged position already. Re-uploading would replace it.
            if old.staged == old.leg and self.legs[self.leg] == old.legs[old.leg]:
                self.staged = self.leg
            self._staged_count = old._staged_count
            self.route = self._route()

    def _route(self) -> Route:
        name, (lat, lon) = self.legs[self.leg]
        return Route(name, lat, lon)

    async def start(self) -> None:
        v = self.ctx.vehicle
        v.notify("Lattice Evacuate task updated" if self._edited else "Lattice Evacuate task", _SHOW)
        for name, (lat, lon) in self.legs[self.leg :]:
            v.notify(f"{name.capitalize()} {lat:.6f}, {lon:.6f}", _SHOW)
        await self._stage()

    def _hold_back(self, text: str) -> None:
        if self._told != text:
            self._told = text
            self.ctx.vehicle.notify(text, _SHOW)

    def _replaceable(self) -> bool:
        s = self.ctx.vehicle.state
        # Older PX4 versions report no mission state. Then only a mission the size of ours counts as ours.
        return s.mission_state in _REPLACEABLE or s.mission_total == self._staged_count

    async def _stage(self) -> None:
        if self.staged == self.leg or time.monotonic() < self._retry_at:
            return
        v, (name, target) = self.ctx.vehicle, self.legs[self.leg]
        if not v.link_ok():
            return
        try:
            # The mode is only as fresh as the last heartbeat, which comes at 1 Hz. Decide on a new one.
            await v.wait_message(lambda m: m.get_type() == "HEARTBEAT", 2.0)
        except asyncio.TimeoutError:
            return
        if v.state.mode == Mode.MISSION:
            self._hold_back(f"Lattice: leave Mission to load the {name}")
            return
        if self.leg > 0 and not self._replaceable():
            self._hold_back(f"Lattice: clear the mission to load the {name}")
            return

        def still_out_of_mission() -> None:
            if v.state.mode == Mode.MISSION:
                raise _HeldBack

        try:
            last = await v.upload_route([target], still_out_of_mission)
        except _HeldBack:
            return
        except CommandError as exc:
            log.warning("Evacuate: uploading the %s failed (will retry): %s", name, exc)
            self._retry_at = time.monotonic() + STAGE_RETRY_S
            return
        self.staged, self._staged_count, self._told = self.leg, last + 1, None
        log.info("Evacuate: %s uploaded as the vehicle's mission", name)
        v.notify(f"Lattice: {name} uploaded as mission", _SHOW)

    async def step(self) -> bool:
        v = self.ctx.vehicle
        if not v.link_ok():
            self._stopped_since = None
            return False
        await self._stage()
        s, (name, target) = v.state, self.legs[self.leg]
        near = s.has_position and geo.distance_m(s.lat, s.lon, *target) <= self.radius_m
        if not near or (s.armed and s.ground_speed_mps > STOPPED_MPS):
            self._stopped_since = None
            return False
        now = time.monotonic()
        if self._stopped_since is None:
            self._stopped_since = now
        if now - self._stopped_since < DWELL_S:
            return False
        log.info("Evacuate: at the %s", name)
        v.notify(f"Lattice: at {name}", _SHOW)
        self.leg += 1
        self._stopped_since = None
        if self.leg == len(self.legs):
            return True
        self.route = self._route()
        return False

    def ended(self, status: str, message: str | None) -> None:
        text = "Lattice Evacuate done" if status == "STATUS_DONE_OK" else f"Lattice Evacuate ended: {message or 'closed in Lattice'}"
        self.ctx.vehicle.notify(text, _SHOW)
