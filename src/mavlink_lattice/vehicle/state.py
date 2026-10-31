"""Vehicle state assembled from MAVLink telemetry. PX4 only: modes use PX4's custom-mode encoding."""

from __future__ import annotations

import enum
import math
import time
from dataclasses import dataclass, field

from pymavlink.dialects.v20 import common as mavlink


class Mode(enum.Enum):
    """PX4 flight modes as (main mode, auto sub mode)."""

    MANUAL = (1, 0)
    ALTCTL = (2, 0)
    POSCTL = (3, 0)
    HOLD = (4, 3)  # AUTO.LOITER, which rovers call Hold
    MISSION = (4, 4)
    RTL = (4, 5)
    LAND = (4, 6)
    ACRO = (5, 0)
    OFFBOARD = (6, 0)
    STABILIZED = (7, 0)
    UNKNOWN = (0, 0)

    @property
    def main(self) -> int:
        return self.value[0]

    @property
    def sub(self) -> int:
        return self.value[1]

    @classmethod
    def from_custom_mode(cls, custom_mode: int) -> "Mode":
        main, sub = (custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF
        if main != 4:
            sub = 0
        for m in cls:
            if m.value == (main, sub):
                return m
        return cls.UNKNOWN


GROUND = {mavlink.MAV_TYPE_GROUND_ROVER}
SURFACE = {mavlink.MAV_TYPE_SURFACE_BOAT}


@dataclass
class Battery:
    percent: float | None = None
    voltage_v: float | None = None
    current_a: float | None = None
    time_remaining_s: float | None = None


@dataclass
class VehicleState:
    system_id: int = 0
    mav_type: int = mavlink.MAV_TYPE_GENERIC
    autopilot: int = mavlink.MAV_AUTOPILOT_INVALID
    last_heartbeat: float = 0.0  # monotonic
    armed: bool = False
    mode: Mode = Mode.UNKNOWN
    system_status: int = mavlink.MAV_STATE_UNINIT

    lat: float | None = None
    lon: float | None = None
    alt_msl_m: float | None = None
    # GPS_RAW_INT reports both MSL and ellipsoid altitude; their difference is the local geoid separation.
    geoid_sep_m: float | None = None
    vel_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    attitude: tuple[float, float, float, float] | None = None  # (w, x, y, z), FRD to NED
    gps_fix: int = 0

    home: tuple[float, float, float] | None = None  # lat, lon, alt MSL
    battery: Battery = field(default_factory=Battery)
    sensors_unhealthy: int = 0  # MAV_SYS_STATUS_SENSOR bits enabled but unhealthy
    prearm_ok: bool | None = None  # None until the autopilot reports its pre-arm check state

    mission_current: int = -1
    mission_reached: int = -1
    uid: int = 0  # hardware UID from AUTOPILOT_VERSION, 0 if not reported

    def link_ok(self, timeout_s: float, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return self.last_heartbeat > 0 and now - self.last_heartbeat < timeout_s

    @property
    def has_position(self) -> bool:
        return self.lat is not None and self.lon is not None

    @property
    def alt_hae_m(self) -> float | None:
        if self.alt_msl_m is None or self.geoid_sep_m is None:
            return None
        return self.alt_msl_m + self.geoid_sep_m

    @property
    def ground_speed_mps(self) -> float:
        return math.hypot(self.vel_ned[0], self.vel_ned[1])

    @property
    def is_ground(self) -> bool:
        return self.mav_type in GROUND

    def update(self, msg, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        t = msg.get_type()
        if t == "HEARTBEAT":
            self.last_heartbeat = now
            self.mav_type = msg.type
            self.autopilot = msg.autopilot
            self.armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.system_status = msg.system_status
            if msg.base_mode & mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                self.mode = Mode.from_custom_mode(msg.custom_mode)
        elif t == "GLOBAL_POSITION_INT":
            if msg.lat == 0 and msg.lon == 0:
                return
            self.lat, self.lon = msg.lat / 1e7, msg.lon / 1e7
            self.alt_msl_m = msg.alt / 1e3
            self.vel_ned = (msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0)
        elif t == "GPS_RAW_INT":
            self.gps_fix = msg.fix_type
            alt_ellipsoid = getattr(msg, "alt_ellipsoid", 0)
            if msg.fix_type >= 3 and alt_ellipsoid:
                self.geoid_sep_m = (alt_ellipsoid - msg.alt) / 1e3
        elif t == "ATTITUDE_QUATERNION":
            self.attitude = (msg.q1, msg.q2, msg.q3, msg.q4)
        elif t == "HOME_POSITION":
            self.home = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1e3)
        elif t == "SYS_STATUS":
            enabled = msg.onboard_control_sensors_enabled & msg.onboard_control_sensors_present
            prearm = mavlink.MAV_SYS_STATUS_PREARM_CHECK
            self.sensors_unhealthy = enabled & ~msg.onboard_control_sensors_health & ~prearm
            # PX4 sets only the health bit (can arm in the current mode), never present/enabled.
            if self.autopilot == mavlink.MAV_AUTOPILOT_PX4 or msg.onboard_control_sensors_present & prearm:
                self.prearm_ok = bool(msg.onboard_control_sensors_health & prearm)
        elif t == "BATTERY_STATUS":
            if msg.id != 0:
                return
            b = self.battery
            b.percent = None if msg.battery_remaining < 0 else float(msg.battery_remaining)
            cells = [v for v in msg.voltages if v != 0xFFFF]
            b.voltage_v = sum(cells) / 1e3 if cells else None
            b.current_a = None if msg.current_battery < 0 else msg.current_battery / 100.0
            b.time_remaining_s = float(msg.time_remaining) if getattr(msg, "time_remaining", 0) > 0 else None
        elif t == "MISSION_CURRENT":
            self.mission_current = msg.seq
        elif t == "MISSION_ITEM_REACHED":
            self.mission_reached = msg.seq
        elif t == "AUTOPILOT_VERSION":
            self.uid = msg.uid
