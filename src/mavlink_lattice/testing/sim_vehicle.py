"""A minimal simulated PX4 rover over MAVLink, for fast tests of the whole bridge without a PX4 build.

It models only what the bridge relies on: heartbeat with PX4 custom modes, arming, DO_SET_MODE,
DO_REPOSITION (drive straight to the point in Hold), RTL, mission upload and MISSION mode with
MISSION_ITEM_REACHED, and the telemetry the entity is built from. Faults can be injected: a mode change
from elsewhere, disarm, telemetry silence, refused commands. Real PX4 behaviour is covered by the SITL tests.
"""

from __future__ import annotations

import asyncio
import math

from pymavlink.dialects.v20 import common as mavlink

from .. import geo
from ..vehicle.state import Mode

M = mavlink


class _Proto(asyncio.DatagramProtocol):
    def __init__(self, sim: "SimRover"):
        self.sim = sim

    def datagram_received(self, data, addr):
        for msg in self.sim.mav.parse_buffer(data) or []:
            self.sim._handle(msg)


class SimRover:
    def __init__(self, bridge_port: int, *, system_id: int = 1, lat: float = 47.397742, lon: float = 8.545594, speed_mps: float = 5.0, uid: int = 0x00AB12CD34EF5678):
        self.bridge_addr = ("127.0.0.1", bridge_port)
        self.mav = M.MAVLink(None, srcSystem=system_id, srcComponent=M.MAV_COMP_ID_AUTOPILOT1)
        self.lat, self.lon = lat, lon
        self.home = (lat, lon)
        self.alt_msl = 488.0
        self.speed = speed_mps
        self.uid = uid
        self.armed = False
        self.mode = Mode.HOLD
        self.target: tuple[float, float] | None = None
        self.mission: list = []
        self.mission_seq = 0
        self._upload: list | None = None
        self.silent = False
        self.refuse: dict[int, int] = {}  # command -> MAV_RESULT to answer with
        self.ignore: dict[int, int] = {}  # command -> number of times to stay silent
        self.commands: list[int] = []
        self.arm_delay_s = 0.0
        # Another component on the autopilot's system ID (a camera), sending its own disarmed heartbeat.
        self.camera = False
        self.vel = (0.0, 0.0)
        self._transport = None
        self._task: asyncio.Task | None = None

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(lambda: _Proto(self), local_addr=("127.0.0.1", 0), remote_addr=self.bridge_addr)
        self._task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
        if self._transport:
            self._transport.close()

    # --- fault injection ---

    def take_over(self, mode: Mode) -> None:
        """A mode switch from RC or another GCS."""
        self.mode = mode
        if mode != Mode.HOLD:
            self.target = None

    def disarm(self) -> None:
        self.armed = False

    # --- simulation ---

    def _send(self, msg) -> None:
        if not self.silent and self._transport:
            self._transport.sendto(msg.pack(self.mav))

    def _custom_mode(self) -> int:
        return (self.mode.main << 16) | (self.mode.sub << 24)

    def _goal(self) -> tuple[float, float] | None:
        if not self.armed:
            return None
        if self.mode == Mode.HOLD:
            return self.target
        if self.mode == Mode.MISSION and self.mission_seq < len(self.mission):
            item = self.mission[self.mission_seq]
            return (item.x / 1e7, item.y / 1e7)
        if self.mode == Mode.RTL:
            return self.home
        return None

    def _advance_mission(self) -> None:
        while self.mission_seq < len(self.mission) and self.mission[self.mission_seq].command != M.MAV_CMD_NAV_WAYPOINT:
            self.mission_seq += 1

    def _step(self, dt: float) -> None:
        if self.mode == Mode.MISSION:
            self._advance_mission()  # past the last item PX4 stays in Mission, stopped
        goal = self._goal()
        if goal is None:
            self.vel = (0.0, 0.0)
            return
        dist = geo.distance_m(self.lat, self.lon, *goal)
        step = self.speed * dt
        if dist <= max(step, 0.3):
            self.lat, self.lon = goal
            self.vel = (0.0, 0.0)
            if self.mode == Mode.MISSION:
                self._send(self.mav.mission_item_reached_encode(self.mission_seq))
                self.mission_seq += 1
            elif self.mode == Mode.RTL:
                self.armed = False
            return
        north = geo.distance_m(self.lat, self.lon, goal[0], self.lon) * (1 if goal[0] > self.lat else -1)
        east = geo.distance_m(self.lat, self.lon, self.lat, goal[1]) * (1 if goal[1] > self.lon else -1)
        n, e = north / dist * step, east / dist * step
        self.lat, self.lon = geo.offset(self.lat, self.lon, n, e)
        self.vel = (n / dt, e / dt)

    async def _run(self) -> None:
        dt, tick = 0.05, 0
        while True:
            self._step(dt)
            if tick % 20 == 0:
                base = M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (M.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
                self._send(self.mav.heartbeat_encode(M.MAV_TYPE_GROUND_ROVER, M.MAV_AUTOPILOT_PX4, base, self._custom_mode(), M.MAV_STATE_ACTIVE if self.armed else M.MAV_STATE_STANDBY))
                if self.camera:
                    cam = M.MAVLink(None, srcSystem=self.mav.srcSystem, srcComponent=M.MAV_COMP_ID_CAMERA)
                    self._transport.sendto(cam.heartbeat_encode(M.MAV_TYPE_CAMERA, M.MAV_AUTOPILOT_INVALID, 0, 0, M.MAV_STATE_ACTIVE).pack(cam))
                self._send(self.mav.gps_raw_int_encode(0, 3, int(self.lat * 1e7), int(self.lon * 1e7), int(self.alt_msl * 1e3), 80, 120, 0, 0, 12, alt_ellipsoid=int((self.alt_msl + 47.0) * 1e3)))
                self._send(self.mav.home_position_encode(int(self.home[0] * 1e7), int(self.home[1] * 1e7), int(self.alt_msl * 1e3), 0, 0, 0, [1, 0, 0, 0], 0, 0, 0))
                self._send(self.mav.battery_status_encode(0, 0, 0, 0, [16800] + [65535] * 9, 1200, -1, -1, 76))
                prearm = M.MAV_SYS_STATUS_PREARM_CHECK
                self._send(self.mav.sys_status_encode(0, 0, prearm, 500, 16800, 1200, 76, 0, 0, 0, 0, 0, 0))
            if tick % 4 == 0:
                vn, ve = self.vel
                heading = math.atan2(ve, vn) if (vn or ve) else 0.0
                self._send(self.mav.global_position_int_encode(0, int(self.lat * 1e7), int(self.lon * 1e7), int(self.alt_msl * 1e3), 0, int(vn * 100), int(ve * 100), 0, int(math.degrees(heading) % 360 * 100)))
                self._send(self.mav.attitude_quaternion_encode(0, math.cos(heading / 2), 0, 0, math.sin(heading / 2), 0, 0, 0))
            tick += 1
            await asyncio.sleep(dt)

    def _finish_arm(self, armed: bool) -> None:
        self.armed = armed
        self._ack(M.MAV_CMD_COMPONENT_ARM_DISARM, M.MAV_RESULT_ACCEPTED)

    def _ack(self, command: int, result: int) -> None:
        self._send(self.mav.command_ack_encode(command, result))

    def _handle(self, msg) -> None:
        t = msg.get_type()
        if t in ("COMMAND_LONG", "COMMAND_INT"):
            self._command(msg)
        elif t == "MISSION_COUNT":
            self._upload = [None] * msg.count
            if msg.count == 0:
                self.mission = []
                self._send(self.mav.mission_ack_encode(msg.get_srcSystem(), msg.get_srcComponent(), M.MAV_MISSION_ACCEPTED, 0))
            else:
                self._send(self.mav.mission_request_int_encode(msg.get_srcSystem(), msg.get_srcComponent(), 0, 0))
        elif t == "MISSION_ITEM_INT" and self._upload is not None and msg.seq < len(self._upload):
            self._upload[msg.seq] = msg
            if msg.seq + 1 < len(self._upload):
                self._send(self.mav.mission_request_int_encode(msg.get_srcSystem(), msg.get_srcComponent(), msg.seq + 1, 0))
            else:
                self.mission, self._upload, self.mission_seq = self._upload, None, 0
                self._send(self.mav.mission_ack_encode(msg.get_srcSystem(), msg.get_srcComponent(), M.MAV_MISSION_ACCEPTED, 0))

    def _command(self, msg) -> None:
        cmd = msg.command
        self.commands.append(cmd)
        if self.ignore.get(cmd):
            self.ignore[cmd] -= 1
            return
        if cmd in self.refuse:
            self._ack(cmd, self.refuse[cmd])
            return
        if cmd == M.MAV_CMD_COMPONENT_ARM_DISARM:
            if self.arm_delay_s:
                asyncio.get_running_loop().call_later(self.arm_delay_s, self._finish_arm, msg.param1 == 1)
                return
            self.armed = msg.param1 == 1
        elif cmd == M.MAV_CMD_DO_SET_MODE:
            mode = Mode.from_custom_mode((int(msg.param2) << 16) | (int(msg.param3) << 24))
            if mode == Mode.MISSION and not self.mission:
                self._ack(cmd, M.MAV_RESULT_DENIED)
                return
            if mode == Mode.HOLD and self.mode != Mode.HOLD:
                self.target = None  # entering Hold holds here; re-entering it keeps the reposition target, as PX4 does
            self.mode = mode
        elif cmd == M.MAV_CMD_DO_REPOSITION:
            if not self.armed:
                self._ack(cmd, M.MAV_RESULT_DENIED)
                return
            lat, lon = (msg.x / 1e7, msg.y / 1e7) if msg.get_type() == "COMMAND_INT" else (msg.param5, msg.param6)
            # All NaN is PX4's pause: hold at the current position.
            self.mode, self.target = Mode.HOLD, None if math.isnan(lat) or math.isnan(lon) else (lat, lon)
        elif cmd == M.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self.mode = Mode.RTL
        elif cmd == M.MAV_CMD_REQUEST_MESSAGE and int(msg.param1) == M.MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            self._ack(cmd, M.MAV_RESULT_ACCEPTED)
            self._send(self.mav.autopilot_version_encode(0, 0, 0, 0, 0, [0] * 8, [0] * 8, [0] * 8, 0, 0, self.uid))
            return
        elif cmd not in (M.MAV_CMD_SET_MESSAGE_INTERVAL, M.MAV_CMD_REQUEST_MESSAGE):
            self._ack(cmd, M.MAV_RESULT_UNSUPPORTED)
            return
        self._ack(cmd, M.MAV_RESULT_ACCEPTED)
