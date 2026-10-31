"""Task status updates. Lattice ignores an update unless its status version is higher than the one it holds,
and it bumps the version itself on cancel and complete requests, so the agent tracks the version per task
and re-reads the task when an update doesn't land."""

from __future__ import annotations

import asyncio
import logging
from datetime import datetime, timezone

import httpx
from anduril import TaskError

from .client import ApiError, LatticeApi

log = logging.getLogger(__name__)

TERMINAL = ("STATUS_DONE_OK", "STATUS_DONE_NOT_OK")
# Server-set statuses that mean an operator request is waiting for the agent's answer.
REQUESTS = {"STATUS_CANCEL_REQUESTED": "cancel", "STATUS_COMPLETE_REQUESTED": "complete"}


class TaskStatusReporter:
    def __init__(self, api: LatticeApi, task_id: str, status_version: int, status: str | None = None):
        self.api = api
        self.task_id = task_id
        self.version = status_version
        self.status = status
        # "cancel" or "complete" when a re-read shows an operator request we haven't answered. It may have been
        # sent while the agent stream was down, so the re-read is how the agent learns of it.
        self.requested: str | None = REQUESTS.get(status)

    @property
    def terminal(self) -> bool:
        return self.status in TERMINAL

    async def resync(self) -> None:
        task = await self.api.get_task(self.task_id)
        self.version = task.version.status_version or 0
        self.status = task.status.status if task.status else None
        if self.status in REQUESTS:
            self.requested = REQUESTS[self.status]

    async def resync_quietly(self) -> bool:
        try:
            await self.resync()
            return True
        except (ApiError, httpx.HTTPError) as exc:
            log.warning("task %s: re-reading the task failed: %s", self.task_id, exc)
            return False

    async def set(self, status: str, *, code: str | None = None, message: str | None = None) -> bool:
        """Send a status; True once Lattice holds it.

        Gives up (False) if the task is already terminal on the server, or if an operator request is pending
        and this isn't a terminal answer to it: the caller has to act on `requested` first.
        """
        now = datetime.now(timezone.utc)
        error = TaskError(**{k: v for k, v in dict(code=code, message=message).items() if v}) if code or message else None
        for attempt in range(4):
            if self.terminal or (self.requested and status not in TERMINAL):
                return False
            want = self.version + 1
            try:
                task = await self.api.update_status(
                    self.task_id,
                    want,
                    status,
                    error=error,
                    start_time=now if status == "STATUS_EXECUTING" else None,
                    end_time=now if status in TERMINAL else None,
                )
            except ApiError as exc:
                log.warning("task %s: %s rejected (%s): %s", self.task_id, status, exc.status_code, exc.body)
                await asyncio.sleep(0.5 * 2**attempt)
                if exc.status_code is not None and 400 <= exc.status_code < 500 and exc.status_code != 429:
                    await self.resync_quietly()
                continue
            except httpx.HTTPError as exc:
                log.warning("task %s: %s not sent: %s", self.task_id, status, exc)
                await asyncio.sleep(0.5 * 2**attempt)
                continue
            got = task.version.status_version or 0
            if got >= want and task.status and task.status.status == status:
                self.version = got
                self.status = status
                log.info("task %s: %s%s", self.task_id, status, f" ({message})" if message else "")
                return True
            # Ignored: the server's version moved past ours (a cancel or complete request bumps it).
            await self.resync_quietly()
        log.warning("task %s: couldn't set %s yet", self.task_id, status)
        return False
