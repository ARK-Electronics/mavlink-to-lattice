"""anduril.tasks.v2.Marshal: drive to an objective (a point, or an entity that may move) and stop there."""

from __future__ import annotations

import asyncio
import time

import httpx

from .. import geo
from ..lattice.client import ApiError
from ..lattice.entity import Route
from ..vehicle import Mode
from .base import TYPE_PREFIX, Executor, TaskContext, TaskFailed, TaskRejected, lla_of

# Re-target an entity objective when it has moved this far; smaller moves aren't worth a new command.
RETARGET_M = 5.0
ENTITY_POLL_S = 3.0
# Entity reads are short and not retried: a slow Lattice mustn't hold up the task's safety checks.
_ENTITY_REQUEST = {"timeout_in_seconds": 5, "max_retries": 0}


class Marshal(Executor):
    type_url = TYPE_PREFIX + "anduril.tasks.v2.Marshal"
    expected_mode = Mode.HOLD  # PX4 drives to a reposition target in Hold and stops there

    def __init__(self, ctx: TaskContext, spec: dict):
        super().__init__(ctx, spec)
        objective = spec.get("objective") or {}
        self.entity_id: str | None = objective.get("entityId")
        self.target: tuple[float, float] | None = None
        self.name = "objective"
        if self.entity_id:
            self.name = self.entity_id
        elif "point" in objective:
            self.target = lla_of(objective["point"], "objective point")
            self.name = objective["point"].get("referenceName") or "objective point"
        else:
            raise TaskRejected("Marshal needs an objective point or entity")
        self._poller: asyncio.Task | None = None
        self._poll_now = asyncio.Event()
        self._latest: tuple[float, float] | None = None
        self._latest_at = 0.0
        self._failure: TaskFailed | None = None
        self._arrived_at: float | None = None

    async def _resolve_entity(self) -> tuple[float, float]:
        try:
            entity = await self.ctx.api.client.entities.get_entity(self.entity_id, request_options=_ENTITY_REQUEST)
        except ApiError as exc:
            if exc.status_code == 404:
                raise TaskFailed(f"objective entity {self.entity_id} not found") from None
            raise
        pos = entity.location.position if entity.location else None
        if not entity.is_live or not pos or pos.latitude_degrees is None:
            raise TaskFailed(f"objective entity {self.entity_id} is gone or has no position")
        if entity.aliases and entity.aliases.name:
            self.name = entity.aliases.name
        return pos.latitude_degrees, pos.longitude_degrees

    async def _poll(self) -> None:
        while True:
            waiter = asyncio.ensure_future(self._poll_now.wait())
            try:
                await asyncio.wait({waiter}, timeout=ENTITY_POLL_S)  # not wait_for: see Vehicle.wait_message
            finally:
                waiter.cancel()
            self._poll_now.clear()
            try:
                self._latest = await self._resolve_entity()
                self._latest_at = time.monotonic()
            except TaskFailed as exc:
                self._failure = exc
                return
            except (ApiError, httpx.HTTPError):
                pass  # Lattice unreachable: keep driving to the last known position

    async def _go(self, target: tuple[float, float]) -> None:
        self.target = target
        self._arrived_at = None
        self.route = Route(self.name, *target)
        await self.ctx.vehicle.reposition(*target)

    async def start(self) -> None:
        if self.entity_id:
            try:
                target = await self._resolve_entity()
            except (ApiError, httpx.HTTPError) as exc:
                raise TaskFailed(f"couldn't read objective entity {self.entity_id}: {exc}") from None
            self._latest_at = time.monotonic()
            await self._go(target)
            self._poller = asyncio.create_task(self._poll())
        else:
            await self._go(self.target)

    async def step(self) -> bool:
        if self._failure:
            raise self._failure
        if self._latest and geo.distance_m(*self._latest, *self.target) > RETARGET_M:
            await self._go(self._latest)
        s = self.ctx.vehicle.state
        if geo.distance_m(s.lat, s.lon, *self.target) > self.ctx.cfg.vehicle.arrival_radius_m:
            self._arrived_at = None
            return False
        if not self.entity_id:
            return True
        # Don't report arrival at where the entity used to be: wait for a position read after arriving.
        if self._arrived_at is None:
            self._arrived_at = time.monotonic()
            self._poll_now.set()
        return self._latest_at > self._arrived_at

    async def close(self) -> None:
        if self._poller:
            self._poller.cancel()
            await asyncio.gather(self._poller, return_exceptions=True)
