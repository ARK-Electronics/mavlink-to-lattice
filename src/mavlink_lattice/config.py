"""TOML configuration. Secrets can come from the environment instead of the file."""

from __future__ import annotations

import dataclasses
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

if sys.version_info >= (3, 11):
    import tomllib
else:  # JetPack 6 ships Python 3.10
    import tomli as tomllib


@dataclass
class VehicleConfig:
    # pymavlink-style address: "udpin:0.0.0.0:14540" listens and replies to whoever sent last;
    # "udpout:host:port" sends to a fixed endpoint.
    mavlink: str = "udpin:127.0.0.1:14540"
    # Autopilot to follow; 0 takes the first PX4 autopilot heard.
    system_id: int = 0
    # Our own MAVLink identity. System ID 0 takes the autopilot's, as a component on the vehicle should. PX4 logs
    # STATUSTEXT only from its own system ID. "onboard_controller" keeps PX4's GCS-loss failsafe tied to the real GCS.
    # "gcs" makes PX4 treat this bridge as a GCS link.
    source_system: int = 0
    source_component: int = 191
    heartbeat_type: str = "onboard_controller"
    link_timeout_s: float = 3.0
    # Arm a disarmed vehicle when a task arrives instead of rejecting the task.
    auto_arm: bool = False
    arrival_radius_m: float = 2.0
    # Ground speed for tasks; 0 leaves the vehicle's configured cruise speed.
    speed_mps: float = 0.0


@dataclass
class EntityConfig:
    # Stable Lattice entity ID. Empty derives one from the autopilot's hardware UID (or its system ID).
    id: str = ""
    name: str = ""
    # Empty derives these from the vehicle's MAV_TYPE.
    platform_type: str = ""
    specific_type: str = ""
    # MIL-STD-2525C symbol ID, e.g. for a friendly unmanned ground vehicle. Empty leaves Lattice's default icon.
    sidc: str = ""
    simulated: bool = False
    publish_period_s: float = 1.0
    # How long Lattice keeps showing the entity after the bridge stops publishing.
    expiry_s: float = 15.0


@dataclass
class LatticeConfig:
    url: str = ""
    # Either a static environment token or OAuth client credentials.
    token: str = ""
    client_id: str = ""
    client_secret: str = ""
    # Only Lattice Sandboxes environments need this second token.
    sandbox_token: str = ""
    heartbeat_interval_s: float = 5.0


TASK_NAMES = ("marshal", "transit", "evacuate")


@dataclass
class EvacuateConfig:
    # The NGC2 Evacuate task's type URL, "type.googleapis.com/<package>.<message>".
    type_url: str = ""
    # Where the pickup and dropoff positions sit in the task, as dotted proto field paths. Each must lead to a message
    # with lat/lon, latitude/longitude or latitude_degrees/longitude_degrees fields. Those fields may also sit below it,
    # inside an lla, point, position, lla_point or location field. A task may carry either position or both.
    pickup: str = "pickup"
    dropoff: str = "dropoff"
    # The vehicle is at a position once it has stopped or disarmed within this distance of it.
    arrival_radius_m: float = 10.0


@dataclass
class TasksConfig:
    # Task types to advertise. Marshal and Transit drive the vehicle from Lattice. Evacuate leaves the driving to the
    # ground station operator.
    enabled: list[str] = field(default_factory=lambda: ["marshal", "transit"])
    evacuate: EvacuateConfig = field(default_factory=EvacuateConfig)


@dataclass
class Config:
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    entity: EntityConfig = field(default_factory=EntityConfig)
    lattice: LatticeConfig = field(default_factory=LatticeConfig)
    tasks: TasksConfig = field(default_factory=TasksConfig)
    # Where the active task ID is kept, so a restarted bridge can close out a task it was running.
    state_dir: str = ""


_ENV = {
    "LATTICE_URL": ("lattice", "url"),
    "LATTICE_TOKEN": ("lattice", "token"),
    "LATTICE_CLIENT_ID": ("lattice", "client_id"),
    "LATTICE_CLIENT_SECRET": ("lattice", "client_secret"),
    "LATTICE_SANDBOX_TOKEN": ("lattice", "sandbox_token"),
}


def _build(cls, data: dict, section: str):
    known = {f.name for f in dataclasses.fields(cls)}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"unknown keys in [{section}]: {', '.join(sorted(unknown))}")
    return cls(**data)


def load(path: str | os.PathLike | None, env: dict[str, str] | None = None) -> Config:
    env = os.environ if env is None else env
    raw = tomllib.loads(Path(path).read_text()) if path else {}
    sections = {"vehicle": VehicleConfig, "entity": EntityConfig, "lattice": LatticeConfig}
    unknown = set(raw) - set(sections) - {"tasks", "state_dir"}
    if unknown:
        raise ValueError(f"unknown config sections: {', '.join(sorted(unknown))}")
    tasks = dict(raw.get("tasks", {}))
    evacuate = _build(EvacuateConfig, tasks.pop("evacuate", {}), "tasks.evacuate")
    cfg = Config(
        **{name: _build(cls, raw.get(name, {}), name) for name, cls in sections.items()},
        tasks=_build(TasksConfig, {**tasks, "evacuate": evacuate}, "tasks"),
        state_dir=raw.get("state_dir", ""),
    )
    for var, (section, key) in _ENV.items():
        if env.get(var):
            setattr(getattr(cfg, section), key, env[var])
    validate(cfg)
    return cfg


def validate(cfg: Config) -> None:
    if not cfg.lattice.url:
        raise ValueError("lattice.url is required")
    if not cfg.lattice.token and not (cfg.lattice.client_id and cfg.lattice.client_secret):
        raise ValueError("set lattice.token, or lattice.client_id and lattice.client_secret")
    if cfg.vehicle.heartbeat_type not in ("onboard_controller", "gcs"):
        raise ValueError("vehicle.heartbeat_type must be 'onboard_controller' or 'gcs'")
    if not cfg.vehicle.mavlink.startswith(("udpin:", "udpout:")):
        raise ValueError("vehicle.mavlink must be udpin:<host>:<port> or udpout:<host>:<port>")
    unknown = set(cfg.tasks.enabled) - set(TASK_NAMES)
    if unknown:
        raise ValueError(f"unknown tasks in tasks.enabled: {', '.join(sorted(unknown))} (known: {', '.join(TASK_NAMES)})")
    if "evacuate" in cfg.tasks.enabled:
        ev = cfg.tasks.evacuate
        if not ev.type_url.startswith("type.googleapis.com/"):
            raise ValueError("tasks.evacuate.type_url must be set to type.googleapis.com/<package>.<message> to enable evacuate")
        if ev.type_url.removeprefix("type.googleapis.com/").startswith("anduril.tasks."):
            raise ValueError("tasks.evacuate.type_url can't be one of Anduril's built-in task types")
        if ev.arrival_radius_m <= 0:
            raise ValueError("tasks.evacuate.arrival_radius_m must be positive")
