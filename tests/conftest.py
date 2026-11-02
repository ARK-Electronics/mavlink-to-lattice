from __future__ import annotations

import asyncio
import socket
import time
from collections.abc import Callable

import pytest
from anduril import AsyncLattice, GoogleProtobufAny, Principal, Relations, System

from mavlink_lattice.bridge import Bridge
from mavlink_lattice.config import Config, EntityConfig, LatticeConfig, TasksConfig, VehicleConfig
from mavlink_lattice.testing.fake_lattice import FakeLattice
from mavlink_lattice.testing.sim_vehicle import SimRover

MARSHAL = "type.googleapis.com/anduril.tasks.v2.Marshal"
TRANSIT = "type.googleapis.com/anduril.tasks.v2.Transit"
TOKEN = "test-token"


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


async def wait_until(pred: Callable[[], bool], timeout: float = 10.0, what: str = "condition") -> None:
    deadline = time.monotonic() + timeout
    while not pred():
        if time.monotonic() > deadline:
            raise AssertionError(f"timed out waiting for {what}")
        await asyncio.sleep(0.05)


def point(lat: float, lon: float, name: str = "") -> dict:
    p = {"lla": {"lat": lat, "lon": lon}}
    if name:
        p["referenceName"] = name
    return p


class Harness:
    def __init__(self, fake: FakeLattice, sim: SimRover, cfg: Config):
        self.fake, self.sim, self.cfg = fake, sim, cfg
        self.bridge: Bridge | None = None
        self.run: asyncio.Task | None = None
        self.operator = AsyncLattice(base_url=fake.url, token=TOKEN)

    @property
    def entity_id(self) -> str:
        return self.bridge.entity_id

    async def start_bridge(self) -> None:
        self.bridge = Bridge(self.cfg)
        self.run = asyncio.create_task(self.bridge.run())
        ready = asyncio.create_task(self.bridge.ready.wait())
        await asyncio.wait({ready, self.run}, timeout=10, return_when=asyncio.FIRST_COMPLETED)
        if self.run.done():
            self.run.result()  # raises what stopped the bridge
        if not ready.done():
            ready.cancel()
            raise AssertionError("bridge not ready after 10 s")
        await wait_until(lambda: self.fake.connections(self.entity_id) > 0 and self.entity_id in self.fake.entities, what="bridge connected")
        await wait_until(lambda: self.catalog(), what="task catalog")

    async def stop_bridge(self) -> None:
        if self.run:
            self.run.cancel()
            await asyncio.gather(self.run, return_exceptions=True)
            self.run = None

    def entity(self) -> dict:
        return self.fake.entities[self.entity_id]

    def catalog(self) -> list[str]:
        return [d["taskSpecificationUrl"] for d in (self.entity().get("taskCatalog") or {}).get("taskDefinitions", [])]

    async def create(self, type_url: str, spec: dict, **kwargs) -> str:
        task = await self.operator.tasks.create_task(
            specification=GoogleProtobufAny.model_validate({"@type": type_url, **spec}),
            relations=Relations(assignee=Principal(system=System(entity_id=self.entity_id))),
            author=Principal(system=System(entity_id="operator")),
            **kwargs,
        )
        return task.version.task_id

    def status(self, task_id: str) -> str:
        return self.fake.tasks[task_id].status

    def error(self, task_id: str) -> dict:
        return self.fake.tasks[task_id].task["status"].get("taskError") or {}

    async def wait_status(self, task_id: str, status: str, timeout: float = 15.0) -> None:
        await wait_until(lambda: self.status(task_id) == status, timeout, f"{task_id} {status} (now {self.status(task_id)})")

    async def wait_terminal(self, task_id: str, timeout: float = 30.0) -> str:
        await wait_until(lambda: self.status(task_id) in ("STATUS_DONE_OK", "STATUS_DONE_NOT_OK"), timeout, f"{task_id} to end (now {self.status(task_id)})")
        return self.status(task_id)


@pytest.fixture
async def fake():
    f = FakeLattice(token=TOKEN)
    await f.start()
    yield f
    await f.stop()


@pytest.fixture
def vehicle_cfg() -> VehicleConfig:
    return VehicleConfig(link_timeout_s=1.5, auto_arm=True)


@pytest.fixture
def tasks_cfg() -> TasksConfig:
    return TasksConfig()


@pytest.fixture
async def harness(fake, vehicle_cfg, tasks_cfg, tmp_path):
    port = free_udp_port()
    vehicle_cfg.mavlink = f"udpin:127.0.0.1:{port}"
    cfg = Config(
        vehicle=vehicle_cfg,
        entity=EntityConfig(publish_period_s=0.2, simulated=True),
        lattice=LatticeConfig(url=fake.url, token=TOKEN, heartbeat_interval_s=0.5),
        tasks=tasks_cfg,
        state_dir=str(tmp_path),
    )
    sim = SimRover(port, speed_mps=15.0)
    await sim.start()
    h = Harness(fake, sim, cfg)
    await h.start_bridge()
    yield h
    await h.stop_bridge()
    await sim.stop()
    assert fake.violations == []
