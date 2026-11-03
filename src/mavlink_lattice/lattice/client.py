"""The slice of the Lattice REST API the bridge uses, over Anduril's SDK."""

from __future__ import annotations

from collections.abc import AsyncIterator
from datetime import datetime

from anduril import AsyncLattice, EntityIdsSelector, Principal, System, TaskError, TaskStatus
from anduril.core.api_error import ApiError

from ..config import LatticeConfig

__all__ = ["ApiError", "LatticeApi"]


class LatticeApi:
    def __init__(self, cfg: LatticeConfig, agent_entity_id: str):
        headers = {"anduril-sandbox-authorization": f"Bearer {cfg.sandbox_token}"} if cfg.sandbox_token else None
        auth = {"token": cfg.token} if cfg.token else {"client_id": cfg.client_id, "client_secret": cfg.client_secret}
        # Reconnecting the agent stream is ours to do: the SDK can't tell a stalled stream from a quiet one.
        self.client = AsyncLattice(base_url=cfg.url.rstrip("/"), headers=headers, stream_reconnection_enabled=False, **auth)
        self.agent_entity_id = agent_entity_id
        self.heartbeat_interval_ms = int(cfg.heartbeat_interval_s * 1000)

    @property
    def author(self) -> Principal:
        return Principal(system=System(entity_id=self.agent_entity_id))

    async def publish_entity(self, entity: dict) -> None:
        await self.client.entities.publish_entity(**entity)

    async def get_entity(self, entity_id: str):
        return await self.client.entities.get_entity(entity_id)

    async def get_task(self, task_id: str):
        return await self.client.tasks.get_task(task_id)

    async def update_status(self, task_id: str, status_version: int, status: str, *, error: TaskError | None = None, start_time: datetime | None = None, end_time: datetime | None = None):
        return await self.client.tasks.update_task_status(
            task_id,
            status_version=status_version,
            new_status=TaskStatus(**{k: v for k, v in dict(status=status, task_error=error, start_time=start_time, end_time=end_time).items() if v is not None}),
            author=self.author,
        )

    def stream_as_agent(self) -> AsyncIterator:
        return self.client.tasks.stream_as_agent(
            agent_selector=EntityIdsSelector(entity_ids=[self.agent_entity_id]),
            heartbeat_interval_ms=self.heartbeat_interval_ms,
        )
