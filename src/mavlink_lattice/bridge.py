"""Wires the vehicle, the Lattice entity publisher, the agent stream and the task manager together."""

from __future__ import annotations

import asyncio
import logging
import time
from datetime import datetime, timezone

import httpx
from pymavlink.dialects.v20 import common as mavlink

from .config import Config
from .lattice import entity as entity_builder
from .lattice.agent import AgentStream
from .lattice.client import ApiError, LatticeApi
from .tasks import TaskContext, TaskManager
from .vehicle import CommandError, Vehicle

log = logging.getLogger(__name__)


class Bridge:
    def __init__(self, cfg: Config, vehicle: Vehicle | None = None):
        self.cfg = cfg
        self.vehicle = vehicle or Vehicle(cfg.vehicle)
        self.entity_id = ""
        self.api: LatticeApi | None = None
        self.manager: TaskManager | None = None
        self.agent: AgentStream | None = None
        self._created = datetime.now(timezone.utc)
        self._last_publish_error = 0.0
        self.ready = asyncio.Event()

    async def _identify(self) -> str:
        if self.cfg.entity.id:
            return self.cfg.entity.id
        deadline = time.monotonic() + 5.0
        while not self.vehicle.state.uid and time.monotonic() < deadline:
            try:
                await self.vehicle.request_message(mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
            except CommandError:
                pass
            await asyncio.sleep(0.5)
        if not self.vehicle.state.uid:
            log.warning("autopilot reported no hardware UID; naming the entity after its system ID")
        return entity_builder.default_entity_id(self.vehicle.state)

    async def run(self) -> None:
        await self.vehicle.start()
        try:
            while True:
                try:
                    await self.vehicle.wait_connected(timeout=10.0)
                    break
                except TimeoutError:
                    log.info("waiting for a PX4 heartbeat on %s", self.cfg.vehicle.mavlink)
            self.entity_id = await self._identify()
            log.info("publishing as Lattice entity %s", self.entity_id)
            self.api = LatticeApi(self.cfg.lattice, self.entity_id)
            ctx = TaskContext(self.vehicle, self.api, self.cfg)
            self.manager = TaskManager(ctx, self.cfg.state_dir)
            await self.manager.recover()
            queue: asyncio.Queue = asyncio.Queue()

            def on_request(event) -> None:
                self.manager.notice(event)
                queue.put_nowait(event)

            self.agent = AgentStream(self.api, on_request, self.manager.on_stream_connected)
            self.ready.set()
            await asyncio.gather(self._publish_loop(), self.agent.run(), self._request_loop(queue), self.manager.background())
        finally:
            if self.manager:
                await self.manager.shutdown()
            await self.vehicle.close()

    async def _request_loop(self, queue: asyncio.Queue) -> None:
        # Serial, so a cancel is never handled before the execute it refers to.
        while True:
            await self.manager.handle(await queue.get())

    def entity(self) -> dict | None:
        s = self.vehicle.state
        if not s.has_position:
            return None
        return entity_builder.build(
            self.cfg.entity,
            self.entity_id,
            s,
            link_ok=self.vehicle.link_ok(),
            now=datetime.now(timezone.utc),
            created=self._created,
            task_types=self.manager.task_types(),
            route=self.manager.route,
        )

    async def _publish_loop(self) -> None:
        while True:
            entity = self.entity()
            if entity:
                try:
                    await self.api.publish_entity(entity)
                except (ApiError, httpx.HTTPError) as exc:
                    if time.monotonic() - self._last_publish_error > 30:
                        self._last_publish_error = time.monotonic()
                        log.warning("publishing the entity failed: %s", getattr(exc, "body", None) or exc)
            await asyncio.sleep(self.cfg.entity.publish_period_s)
