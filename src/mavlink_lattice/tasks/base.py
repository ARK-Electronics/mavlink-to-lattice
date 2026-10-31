"""Executor interface. One executor instance runs one Lattice task on the vehicle."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from ..config import Config
from ..lattice.client import LatticeApi
from ..lattice.entity import Route
from ..vehicle import Mode, Vehicle

TYPE_PREFIX = "type.googleapis.com/"


class TaskRejected(Exception):
    """The task can't be accepted (bad specification, or the vehicle can't take it now)."""

    def __init__(self, message: str, code: str = "ERROR_CODE_REJECTED"):
        super().__init__(message)
        self.code = code


class TaskFailed(Exception):
    """An accepted task ended without reaching its goal.

    `stop` is False when someone else now controls the vehicle (an RC or GCS takeover, a disarm) and the
    bridge mustn't command it; otherwise the vehicle is stopped, since it may still be driving to the target.
    """

    def __init__(self, message: str, code: str = "ERROR_CODE_FAILED", stop: bool = True):
        super().__init__(message)
        self.code = code
        self.stop = stop


@dataclass
class TaskContext:
    vehicle: Vehicle
    api: LatticeApi
    cfg: Config


def lla_of(point: dict, what: str) -> tuple[float, float]:
    """(lat, lon) from an anduril.tasks.v2.Point in proto JSON."""
    lla = (point or {}).get("lla") or {}
    lat, lon = lla.get("lat"), lla.get("lon")
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)) or not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise TaskRejected(f"{what} has no valid lat/lon")
    return float(lat), float(lon)


class Executor:
    type_url: ClassVar[str]
    # The vehicle must stay in this mode once it gets there; leaving it (RC or GCS takeover, a failsafe) fails the task.
    expected_mode: ClassVar[Mode]

    def __init__(self, ctx: TaskContext, spec: dict):
        """Parse the task specification (proto JSON, without "@type"); raise TaskRejected if it's unusable."""
        self.ctx = ctx
        self.route: Route | None = None

    async def start(self) -> None:
        """Command the vehicle. Returns once the vehicle accepted; raises TaskFailed or CommandError."""
        raise NotImplementedError

    async def step(self) -> bool:
        """Called every tick while running. True when the task's goal is reached."""
        raise NotImplementedError

    async def finish(self) -> None:
        """Leave the vehicle in a defined state after success."""

    async def close(self) -> None:
        """Release anything the executor started (always called once the task ends)."""
