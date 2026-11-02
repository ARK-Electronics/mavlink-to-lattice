"""Vehicle connection: telemetry, commands with acknowledgement, and mission upload."""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import Callable

from pymavlink.dialects.v20 import common as mavlink

from ..config import VehicleConfig
from .link import MavlinkLink
from .state import Mode, VehicleState

log = logging.getLogger(__name__)

_STREAMS_HZ = {
    mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,
    mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION: 5,
    mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 1,
    mavlink.MAVLINK_MSG_ID_SYS_STATUS: 1,
    mavlink.MAVLINK_MSG_ID_BATTERY_STATUS: 1,
    mavlink.MAVLINK_MSG_ID_HOME_POSITION: 0.5,
    mavlink.MAVLINK_MSG_ID_MISSION_CURRENT: 1,
}

class CommandError(Exception):
    def __init__(self, what: str, result: int | None = None):
        name = mavlink.enums["MAV_RESULT"][result].name if result in mavlink.enums["MAV_RESULT"] else "no response"
        super().__init__(f"{what}: {name}")
        self.result = result


class Vehicle:
    def __init__(self, cfg: VehicleConfig, link: MavlinkLink | None = None):
        self.cfg = cfg
        self.link = link or MavlinkLink(cfg.mavlink, cfg.source_system or cfg.system_id or 1, cfg.source_component)
        self.link.add_handler(self._on_message)
        self.state = VehicleState(system_id=cfg.system_id)
        self.target_component = mavlink.MAV_COMP_ID_AUTOPILOT1
        self._waiters: list[tuple[Callable[[object], bool], asyncio.Future]] = []
        self._tasks: list[asyncio.Task] = []
        self._was_connected = False
        self._lock = asyncio.Lock()  # one command or mission transaction at a time

    async def start(self) -> None:
        await self.link.open()
        self._tasks = [asyncio.create_task(self._heartbeat_loop(), name="mavlink-heartbeat")]

    async def close(self) -> None:
        for t in self._tasks:
            t.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        await self.link.close()

    def link_ok(self) -> bool:
        return self.state.link_ok(self.cfg.link_timeout_s)

    async def wait_connected(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while not self.link_ok():
            if time.monotonic() > deadline:
                raise TimeoutError("no heartbeat from the autopilot")
            await asyncio.sleep(0.1)

    # --- receive ---

    def _is_target(self, msg) -> bool:
        # Only the autopilot component: a camera, gimbal or Remote ID module on the same system ID sends its own
        # heartbeats, which would otherwise read as disarmed, refresh the link, or change the vehicle type.
        src, comp = msg.get_srcSystem(), msg.get_srcComponent()
        if comp != self.target_component:
            return False
        if self.state.system_id:
            return src == self.state.system_id
        if msg.get_type() == "HEARTBEAT" and msg.autopilot == mavlink.MAV_AUTOPILOT_PX4:
            self.state.system_id = src
            if not self.cfg.source_system:
                self.link.mav.srcSystem = src
            log.info("following autopilot system %d", src)
            return True
        return False

    def _on_message(self, msg) -> None:
        if not self._is_target(msg):
            return
        self.state.update(msg)
        if msg.get_type() == "HEARTBEAT" and not self._was_connected:
            self._was_connected = True
            log.info("vehicle link up")
            self._tasks.append(asyncio.create_task(self._request_streams()))
        for pred, fut in list(self._waiters):
            if not fut.done() and pred(msg):
                fut.set_result(msg)

    async def wait_message(self, pred: Callable[[object], bool], timeout: float):
        fut = asyncio.get_running_loop().create_future()
        entry = (pred, fut)
        self._waiters.append(entry)
        try:
            # asyncio.wait, not wait_for: before Python 3.12, wait_for can swallow a cancel that arrives in the
            # same loop iteration as the message, and a cancelled task would then carry on.
            await asyncio.wait({fut}, timeout=timeout)
        finally:
            self._waiters.remove(entry)
        if not fut.done():
            fut.cancel()
            raise asyncio.TimeoutError
        return fut.result()

    # --- send ---

    async def _heartbeat_loop(self) -> None:
        mav_type = mavlink.MAV_TYPE_GCS if self.cfg.heartbeat_type == "gcs" else mavlink.MAV_TYPE_ONBOARD_CONTROLLER
        while True:
            self.link.send(self.link.mav.heartbeat_encode(mav_type, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavlink.MAV_STATE_ACTIVE))
            if self._was_connected and not self.link_ok():
                self._was_connected = False
                log.warning("vehicle link lost")
            await asyncio.sleep(1.0)

    def notify(self, text: str, severity: int = mavlink.MAV_SEVERITY_NOTICE) -> None:
        """Show a message in the ground station. It gets there only if mavlink-router or PX4's forwarding passes it
        to the GCS link, since PX4 itself only writes it to its log. Cut to MAVLink's 50 characters."""
        self.link.send(self.link.mav.statustext_encode(severity, text[:50].encode()))

    async def request_message(self, msg_id: int) -> None:
        await self.command(mavlink.MAV_CMD_REQUEST_MESSAGE, msg_id, timeout=0.5, retries=2)

    async def _request_streams(self) -> None:
        # AUTOPILOT_VERSION first: its hardware UID names the entity. The rate requests only raise rates of
        # streams PX4 already sends, so a refused one doesn't matter.
        requests = [(mavlink.MAV_CMD_REQUEST_MESSAGE, msg_id, 0) for msg_id in (mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, mavlink.MAVLINK_MSG_ID_HOME_POSITION)]
        requests += [(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz) for msg_id, hz in _STREAMS_HZ.items()]
        for cmd, p1, p2 in requests:
            try:
                await self.command(cmd, p1, p2, timeout=0.5, retries=2)
            except CommandError as exc:
                log.debug("stream request not accepted: %s", exc)

    def _ack_for(self, command: int):
        return lambda m: m.get_type() == "COMMAND_ACK" and m.command == command

    async def command(self, command: int, *params: float, timeout: float = 1.0, retries: int = 3) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        async with self._lock:
            for attempt in range(retries):
                msg = self.link.mav.command_long_encode(self.state.system_id, self.target_component, command, attempt, *p)
                if await self._send_and_await_ack(msg, command, timeout):
                    return
        raise CommandError(mavlink.enums["MAV_CMD"][command].name)

    async def command_int(self, command: int, frame: int, params: tuple[float, float, float, float], x: int, y: int, z: float, timeout: float = 1.0, retries: int = 3) -> None:
        async with self._lock:
            for _ in range(retries):
                msg = self.link.mav.command_int_encode(self.state.system_id, self.target_component, frame, command, 0, 0, *params, x, y, z)
                if await self._send_and_await_ack(msg, command, timeout):
                    return
        raise CommandError(mavlink.enums["MAV_CMD"][command].name)

    async def _send_and_await_ack(self, msg, command: int, timeout: float) -> bool:
        """True when accepted; raises on a definite refusal; False when no answer came (retry)."""
        waiter = asyncio.ensure_future(self.wait_message(self._ack_for(command), timeout))
        await asyncio.sleep(0)  # register the waiter before the reply can arrive
        self.link.send(msg)
        try:
            ack = await waiter
            while ack.result == mavlink.MAV_RESULT_IN_PROGRESS:
                ack = await self.wait_message(self._ack_for(command), 10.0)
        except asyncio.TimeoutError:
            return False
        if ack.result == mavlink.MAV_RESULT_ACCEPTED:
            return True
        raise CommandError(mavlink.enums["MAV_CMD"][command].name, ack.result)

    # --- actions ---

    async def set_mode(self, mode: Mode) -> None:
        # PX4 refuses Mission mode for a moment after an upload, while it checks the new mission.
        deadline = time.monotonic() + 3.0
        while True:
            try:
                await self.command(mavlink.MAV_CMD_DO_SET_MODE, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode.main, mode.sub)
                return
            except CommandError as exc:
                if exc.result != mavlink.MAV_RESULT_TEMPORARILY_REJECTED or time.monotonic() > deadline:
                    raise
            await asyncio.sleep(0.3)

    async def hold(self) -> None:
        await self.set_mode(Mode.HOLD)

    async def stop(self) -> None:
        """Stop where the vehicle is now and hold there.

        Switching to Hold isn't enough: after a reposition PX4 is already in Hold, and re-entering it keeps
        driving to the reposition target. A reposition with lat/lon/alt all NaN is PX4's pause, which holds
        at the current position. NaN only fits COMMAND_LONG's float params, not COMMAND_INT's x/y.
        """
        if not self.state.armed:
            return  # not moving, and a mode change would only get in the way of whoever has it
        nan = math.nan
        # Twice: acks only carry the command ID, so a late ack of an earlier reposition can answer the first.
        for _ in range(2):
            await self.command(mavlink.MAV_CMD_DO_REPOSITION, -1.0, mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0.0, nan, nan, nan, nan)

    async def arm(self) -> None:
        await self.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        # The ack comes before the next heartbeat reports the armed state that tasks are checked against.
        try:
            await self.wait_message(lambda m: m.get_type() == "HEARTBEAT" and m.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 3.0)
        except asyncio.TimeoutError:
            raise CommandError("arming: no armed heartbeat") from None

    async def return_to_launch(self) -> None:
        await self.command(mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    async def reposition(self, lat: float, lon: float) -> None:
        """Drive straight to a point and stop there (PX4 switches to Hold with the point as its target)."""
        speed = self.cfg.speed_mps if self.cfg.speed_mps > 0 else -1.0
        alt = self.state.alt_msl_m if self.state.alt_msl_m is not None else math.nan
        await self.command_int(
            mavlink.MAV_CMD_DO_REPOSITION,
            mavlink.MAV_FRAME_GLOBAL,
            (speed, mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0.0, math.nan),
            int(round(lat * 1e7)),
            int(round(lon * 1e7)),
            alt,
        )

    async def upload_route(self, points: list[tuple[float, float]], before_send: Callable[[], None] | None = None) -> int:
        """Upload waypoints as a mission. Returns the sequence number of the last waypoint."""
        items = []
        if self.cfg.speed_mps > 0:
            items.append((mavlink.MAV_CMD_DO_CHANGE_SPEED, mavlink.MAV_FRAME_MISSION, (1, self.cfg.speed_mps, -1, 0), 0, 0))
        for lat, lon in points:
            items.append((mavlink.MAV_CMD_NAV_WAYPOINT, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, (0, 0, 0, math.nan), int(round(lat * 1e7)), int(round(lon * 1e7))))
        await self.upload_mission(items, before_send)
        return len(items) - 1

    async def upload_mission(self, items: list[tuple], before_send: Callable[[], None] | None = None) -> None:
        """`before_send` is called before each MISSION_COUNT, once the upload holds the link. Raising from it
        abandons the upload before PX4 has taken any of it."""
        sysid, comp = self.state.system_id, self.target_component

        def encode(seq: int):
            cmd, frame, (p1, p2, p3, p4), x, y = items[seq]
            return self.link.mav.mission_item_int_encode(sysid, comp, seq, frame, cmd, 0, 1, p1, p2, p3, p4, x, y, 0.0, mavlink.MAV_MISSION_TYPE_MISSION)

        def is_reply(m) -> bool:
            if m.get_type() == "MISSION_ACK":
                return m.mission_type == mavlink.MAV_MISSION_TYPE_MISSION
            return m.get_type() in ("MISSION_REQUEST_INT", "MISSION_REQUEST") and m.mission_type == mavlink.MAV_MISSION_TYPE_MISSION

        async with self._lock:
            last_sent = self.link.mav.mission_count_encode(sysid, comp, len(items), mavlink.MAV_MISSION_TYPE_MISSION)
            retries = 0
            while True:
                if before_send and last_sent.get_type() == "MISSION_COUNT":
                    before_send()
                waiter = asyncio.ensure_future(self.wait_message(is_reply, 1.5))
                await asyncio.sleep(0)
                self.link.send(last_sent)
                try:
                    reply = await waiter
                except asyncio.TimeoutError:
                    retries += 1
                    if retries > 5:
                        raise CommandError("mission upload") from None
                    continue
                retries = 0
                if reply.get_type() == "MISSION_ACK":
                    if reply.type == mavlink.MAV_MISSION_ACCEPTED:
                        self.state.mission_reached = -1
                        return
                    raise CommandError(f"mission upload rejected: {mavlink.enums['MAV_MISSION_RESULT'][reply.type].name}")
                if not 0 <= reply.seq < len(items):
                    raise CommandError(f"mission upload: vehicle asked for item {reply.seq} of {len(items)}")
                last_sent = encode(reply.seq)

    async def start_mission(self) -> None:
        await self.set_mode(Mode.MISSION)
