"""Runs Lattice tasks on the vehicle: one at a time, a new task supersedes the running one.

Status lifecycle per task: MACHINE_RECEIPT on delivery, then either DONE_NOT_OK/REJECTED or WILCO,
EXECUTING once the vehicle accepted the commands, and DONE_OK or DONE_NOT_OK at the end.

Two invariants, because this drives a real vehicle:
- Every task received stays tracked until Lattice holds a terminal status for it. Unsent terminal statuses
  are retried in the background, and the open set is saved so a restarted bridge closes them out.
- A task that ends stops the vehicle, unless someone else took control (a mode change or a disarm). A stop
  is re-sent while the vehicle keeps moving in an auto mode, since an ack proves the command arrived, not
  that the vehicle stopped.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from collections import OrderedDict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import httpx

from ..lattice.client import ApiError
from ..lattice.status import REQUESTS, TERMINAL, TaskStatusReporter
from ..vehicle import CommandError, Mode
from ..vehicle.state import GROUND, SURFACE
from .base import Executor, TaskContext, TaskFailed, TaskRejected
from .marshal import Marshal
from .transit import Transit

log = logging.getLogger(__name__)

EXECUTORS: dict[str, type[Executor]] = {cls.type_url: cls for cls in (Marshal, Transit)}

TICK_S = 0.2
# Time for PX4 to report the mode a command switched it to.
MODE_GRACE_S = 3.0
UNSENT_RETRY_S = 5.0
# Statuses only this agent sets: a delivered task already in one of them was being run by an earlier bridge.
_AGENT_STARTED = {"STATUS_MACHINE_RECEIPT", "STATUS_ACK", "STATUS_WILCO", "STATUS_EXECUTING", "STATUS_WAITING_FOR_UPDATE"}
_AUTO = {Mode.HOLD, Mode.MISSION}  # the modes our tasks put the vehicle in

_CANCELLED = ("STATUS_DONE_NOT_OK", "ERROR_CODE_CANCELLED", "cancelled by operator")
_COMPLETED = ("STATUS_DONE_OK", None, "completed by operator")


@dataclass
class _Active:
    task_id: str
    definition_version: int
    reporter: TaskStatusReporter
    executor: Executor
    complete_before: datetime | None
    runner: asyncio.Task | None = None
    finishing: bool = False  # the runner has decided the outcome and owns reporting it

    @property
    def requested(self) -> str | None:
        return self.reporter.requested


class TaskManager:
    def __init__(self, ctx: TaskContext, state_dir: str = ""):
        self.ctx = ctx
        self.active: _Active | None = None
        self._open: dict[str, TaskStatusReporter] = {}
        self._unsent: dict[str, tuple[TaskStatusReporter, str, str | None, str | None]] = {}
        self._finished: OrderedDict[str, tuple[str, str | None, str | None]] = OrderedDict()
        self._state_file = Path(state_dir) / "open_tasks.json" if state_dir else None
        self._stop_pending = False  # a stop that didn't get through (link down); retried until it does
        self._stopped_from: Mode | None = None
        self._stop_check_at = 0.0
        self._unsent_at = 0.0
        self._background: set[asyncio.Task] = set()

    def task_types(self) -> list[str]:
        """Task types to advertise now. Only ground and surface vehicles have executors so far."""
        v = self.ctx.vehicle
        if not v.link_ok() or not v.state.has_position or v.state.mav_type not in GROUND | SURFACE:
            return []
        return list(EXECUTORS)

    @property
    def route(self):
        return self.active.executor.route if self.active else None

    # --- requests from the agent stream ---

    def notice(self, event) -> None:
        """Called for each event as it arrives, ahead of the queue: a cancel or complete for the running task
        takes effect at the runner's next tick, not after whatever the request loop is busy with."""
        a = self.active
        if not a or a.finishing:
            return
        if event.cancel_request and event.cancel_request.task_id == a.task_id:
            a.reporter.requested = "cancel"
        elif event.complete_request and event.complete_request.task_id == a.task_id:
            a.reporter.requested = "complete"
        elif event.execute_request and event.execute_request.task and event.execute_request.task.version.task_id == a.task_id:
            status = event.execute_request.task.status.status if event.execute_request.task.status else None
            if status in REQUESTS:
                a.reporter.requested = REQUESTS[status]

    def on_stream_connected(self) -> None:
        """A cancel sent while the stream was down may never arrive; re-read the running task to find it."""
        if self.active:
            t = asyncio.ensure_future(self.active.reporter.resync_quietly())
            self._background.add(t)
            t.add_done_callback(self._background.discard)

    async def handle(self, event) -> None:
        try:
            if event.execute_request and event.execute_request.task:
                await self._execute(event.execute_request.task)
            elif event.cancel_request:
                await self._end_on_request(event.cancel_request.task_id, _CANCELLED)
            elif event.complete_request:
                await self._end_on_request(event.complete_request.task_id, _COMPLETED)
        except (ApiError, httpx.HTTPError) as exc:
            log.error("Lattice request failed while handling a task: %s", exc)

    async def _execute(self, task) -> None:
        task_id = task.version.task_id
        status = task.status.status if task.status else None
        definition = task.version.definition_version or 0
        a = self.active
        if a and a.task_id == task_id:
            if definition > a.definition_version and not a.finishing:
                await self._replace_definition(a, task)
            return  # a redelivery; notice() already picked up any pending request
        if task_id in self._finished:
            await self._report_again(task_id)
            return
        if status in TERMINAL or task_id in self._open:
            return
        reporter = TaskStatusReporter(self.ctx.api, task_id, task.version.status_version or 0, status)
        self._track(reporter)
        if status in _AGENT_STARTED:
            await self._stop_vehicle()
            await self._end(reporter, "STATUS_DONE_NOT_OK", "ERROR_CODE_FAILED", "bridge restarted while running the task")
            return
        if reporter.requested:
            await self._end(reporter, *(_CANCELLED if reporter.requested == "cancel" else _COMPLETED))
            return
        await reporter.set("STATUS_MACHINE_RECEIPT")
        if await self._closed_meanwhile(reporter):
            return
        try:
            executor, complete_before = self._accept(task)
            await self._prepare_vehicle()
        except TaskRejected as exc:
            await self._end(reporter, "STATUS_DONE_NOT_OK", exc.code, str(exc))
            return
        if self.active:
            # The new task's commands replace the old target; its runner stops the vehicle if they fail.
            await self._stop_active("STATUS_DONE_NOT_OK", "ERROR_CODE_CANCELLED", f"superseded by task {task_id}", stop=False)
        await reporter.set("STATUS_WILCO")
        if await self._closed_meanwhile(reporter):
            await self._stop_vehicle()
            return
        active = _Active(task_id, definition, reporter, executor, complete_before)
        active.runner = asyncio.create_task(self._run(active), name=f"task-{task_id}")
        self.active = active
        self._stop_pending, self._stopped_from = False, None  # the new task commands the vehicle now

    async def _closed_meanwhile(self, reporter: TaskStatusReporter) -> bool:
        """The task ended on the server, or an operator asked to end it, before we started running it."""
        if reporter.terminal:
            self._open.pop(reporter.task_id, None)
            self._save()
            return True
        if reporter.requested:
            await self._end(reporter, *(_CANCELLED if reporter.requested == "cancel" else _COMPLETED))
            return True
        return False

    def _accept(self, task) -> tuple[Executor, datetime | None]:
        spec = task.specification
        type_url = spec.type if spec else None
        cls = EXECUTORS.get(type_url)
        if cls is None:
            raise TaskRejected(f"unsupported task type {type_url}")
        constraints = task.execution_constraints
        complete_before = constraints.complete_before if constraints else None
        if complete_before and complete_before <= datetime.now(timezone.utc):
            raise TaskRejected("completeBefore has already passed", "ERROR_CODE_TIMEOUT")
        if constraints and constraints.start_after and constraints.start_after > datetime.now(timezone.utc):
            raise TaskRejected("scheduled tasks (startAfter) aren't supported")
        return cls(self.ctx, dict(spec.model_extra or {})), complete_before

    async def _prepare_vehicle(self) -> None:
        v = self.ctx.vehicle
        if not v.link_ok():
            raise TaskRejected("no link to the vehicle")
        if v.state.mav_type not in GROUND | SURFACE:
            raise TaskRejected("tasks are only supported on ground and surface vehicles so far")
        if not v.state.has_position or v.state.gps_fix < 3:
            raise TaskRejected("vehicle has no global position")
        if not v.state.armed:
            if not self.ctx.cfg.vehicle.auto_arm:
                raise TaskRejected("vehicle is disarmed")
            if v.state.prearm_ok is False:
                raise TaskRejected("vehicle is disarmed and failing its pre-arm checks")
            try:
                await v.arm()
            except CommandError as exc:
                raise TaskRejected(f"arming failed: {exc}") from None

    async def _replace_definition(self, a: _Active, task) -> None:
        """The running task was edited: run the new definition under the same task."""
        a.runner.cancel()
        await asyncio.gather(a.runner, return_exceptions=True)
        await a.executor.close()
        try:
            executor, complete_before = self._accept(task)
        except TaskRejected as exc:
            self.active = None
            await self._stop_vehicle()
            await self._end(a.reporter, "STATUS_DONE_NOT_OK", exc.code, f"edited task rejected: {exc}")
            return
        log.info("task %s: definition %d replaces %d", a.task_id, task.version.definition_version, a.definition_version)
        new = _Active(a.task_id, task.version.definition_version or 0, a.reporter, executor, complete_before)
        new.runner = asyncio.create_task(self._run(new), name=f"task-{a.task_id}")
        self.active = new

    async def _end_on_request(self, task_id: str, outcome: tuple[str, str | None, str]) -> None:
        if self.active and self.active.task_id == task_id:
            return  # notice() flagged it; the runner stops the vehicle and answers
        if task_id in self._finished:
            await self._report_again(task_id)  # it already ended here; repeat how, if Lattice missed it
            return
        reporter = self._open.get(task_id) or TaskStatusReporter(self.ctx.api, task_id, 0)
        if not await reporter.resync_quietly() and task_id not in self._open:
            return
        if task_id in self._open:
            await self._stop_vehicle()  # a task from before a restart, or one we were still starting
        await self._end(reporter, *outcome)

    # --- running ---

    async def _stop_active(self, status: str, code: str | None, message: str, stop: bool) -> None:
        active, self.active = self.active, None
        if active.finishing:
            # The task already reached its own end; let it report that rather than overwrite it.
            await asyncio.gather(active.runner, return_exceptions=True)
            return
        active.runner.cancel()
        await asyncio.gather(active.runner, return_exceptions=True)
        await active.executor.close()
        if stop:
            await self._stop_vehicle()
        await self._end(active.reporter, status, code, message)

    async def _run(self, active: _Active) -> None:
        ex = active.executor
        executing: asyncio.Task | None = None
        try:
            try:
                await ex.start()
                # In the background: a slow Lattice mustn't delay the checks below.
                executing = asyncio.create_task(active.reporter.set("STATUS_EXECUTING"))
                outcome = await self._supervise(active)
            except (TaskFailed, CommandError) as exc:
                outcome = ("STATUS_DONE_NOT_OK", getattr(exc, "code", "ERROR_CODE_FAILED"), str(exc), getattr(exc, "stop", True))
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.exception("task %s: executor error", active.task_id)
                outcome = ("STATUS_DONE_NOT_OK", "ERROR_CODE_FAILED", f"bridge error: {exc}", True)
        except asyncio.CancelledError:
            if executing:
                executing.cancel()
            raise
        active.finishing = True
        status, code, message, stop = outcome
        await ex.close()
        if stop == "finish":
            try:
                await ex.finish()
            except CommandError as exc:
                log.warning("task %s: reached the goal, but %s", active.task_id, exc)
                await self._stop_vehicle()
        elif stop:
            await self._stop_vehicle()
        if executing:
            await asyncio.gather(executing, return_exceptions=True)  # EXECUTING must land before the end
        if status:
            await self._end(active.reporter, status, code, message)
        else:
            self._open.pop(active.task_id, None)
        if self.active is active:
            self.active = None
        self._save()

    async def _supervise(self, active: _Active):
        """Watch the running task. Returns (status, code, message, stop) once it should end."""
        ex, v = active.executor, self.ctx.vehicle
        started, mode_reached = time.monotonic(), False
        while True:
            if active.requested:
                status, code, message = _CANCELLED if active.requested == "cancel" else _COMPLETED
                return status, code, message, True
            if active.reporter.terminal:
                return None, None, None, True  # closed on the server by someone else
            s = v.state
            if not v.link_ok():
                raise TaskFailed("lost the link to the vehicle")
            if not s.armed:
                raise TaskFailed("vehicle disarmed", stop=False)
            if s.mode == ex.expected_mode:
                mode_reached = True
            elif mode_reached:
                raise TaskFailed(f"vehicle left {ex.expected_mode.name} for {s.mode.name}", stop=False)
            elif time.monotonic() - started > MODE_GRACE_S:
                raise TaskFailed(f"vehicle didn't switch to {ex.expected_mode.name} (in {s.mode.name})")
            if active.complete_before and datetime.now(timezone.utc) > active.complete_before:
                raise TaskFailed("completeBefore passed", "ERROR_CODE_TIMEOUT")
            if await ex.step():
                return "STATUS_DONE_OK", None, None, "finish"
            await asyncio.sleep(TICK_S)

    async def _stop_vehicle(self) -> None:
        s = self.ctx.vehicle.state
        self._stopped_from = s.mode if s.armed else None
        self._stop_check_at = time.monotonic() + MODE_GRACE_S
        try:
            await self.ctx.vehicle.stop()
            self._stop_pending = False
        except CommandError as exc:
            self._stop_pending = True
            log.warning("couldn't stop the vehicle (will retry): %s", exc)

    async def _end(self, reporter: TaskStatusReporter, status: str, code: str | None, message: str | None) -> None:
        self._finished[reporter.task_id] = (status, code, message)
        while len(self._finished) > 200:
            self._finished.popitem(last=False)
        if await reporter.set(status, code=code, message=message) or reporter.terminal:
            self._open.pop(reporter.task_id, None)
            self._unsent.pop(reporter.task_id, None)
        else:
            self._unsent[reporter.task_id] = (reporter, status, code, message)
        self._save()

    async def _report_again(self, task_id: str) -> None:
        """A finished task was delivered again; Lattice may have missed our final status."""
        reporter = TaskStatusReporter(self.ctx.api, task_id, 0)
        if await reporter.resync_quietly() and not reporter.terminal:
            await self._end(reporter, *self._finished[task_id])

    # --- background: stop watchdog, unsent statuses ---

    async def background(self) -> None:
        while True:
            await asyncio.sleep(1.0)
            await self._watch_stop()
            if self._unsent and time.monotonic() - self._unsent_at > UNSENT_RETRY_S:
                self._unsent_at = time.monotonic()
                for reporter, status, code, message in list(self._unsent.values()):
                    await self._end(reporter, status, code, message)

    async def _watch_stop(self) -> None:
        v = self.ctx.vehicle
        if self.active or not v.link_ok():
            return
        s = v.state
        # Only while it's in a mode we command: an operator driving it by hand isn't overridden, and a
        # disarmed vehicle has nothing left to stop.
        if not s.armed or s.mode not in _AUTO:
            self._stop_pending = False
            self._stopped_from = None
            return
        # A stop from Mission switches to Hold; still in Mission means the stop didn't take effect.
        unchanged = self._stopped_from == Mode.MISSION and s.mode == Mode.MISSION and time.monotonic() > self._stop_check_at
        if self._stop_pending or unchanged:
            log.warning("vehicle %s; stopping it", "didn't leave Mission" if unchanged else "link is back")
            await self._stop_vehicle()

    # --- restart recovery ---

    def _track(self, reporter: TaskStatusReporter) -> None:
        self._open[reporter.task_id] = reporter
        self._save()

    def _save(self) -> None:
        if not self._state_file:
            return
        try:
            self._state_file.parent.mkdir(parents=True, exist_ok=True)
            self._state_file.write_text(json.dumps({"open": sorted(self._open)}))
        except OSError as exc:
            log.warning("couldn't save open tasks: %s", exc)

    async def recover(self) -> None:
        """Stop the vehicle and fail the tasks a previous run of the bridge left open."""
        if not self._state_file or not self._state_file.exists():
            return
        try:
            task_ids = json.loads(self._state_file.read_text()).get("open", [])
        except (OSError, ValueError):
            task_ids = []
        if not task_ids:
            return
        await self._stop_vehicle()
        for task_id in task_ids:
            reporter = TaskStatusReporter(self.ctx.api, task_id, 0)
            self._open[task_id] = reporter
            await reporter.resync_quietly()
            await self._end(reporter, "STATUS_DONE_NOT_OK", "ERROR_CODE_FAILED", "bridge restarted while running the task")

    async def shutdown(self, timeout_s: float = 10.0) -> None:
        """Stop the vehicle and close every open task, within a bounded time."""
        a, self.active = self.active, None
        if a and a.runner and not a.runner.done():
            if not a.finishing:
                a.runner.cancel()
            await asyncio.wait({a.runner}, timeout=timeout_s / 2)
            await a.executor.close()
        if self._open:
            await self._stop_vehicle()

        async def close_all():
            for reporter in list(self._open.values()):
                await self._end(reporter, "STATUS_DONE_NOT_OK", "ERROR_CODE_FAILED", "bridge stopped")

        done, _ = await asyncio.wait({asyncio.ensure_future(close_all())}, timeout=timeout_s / 2)
        if not done:
            log.warning("bridge stopped with tasks Lattice may still show open: %s", ", ".join(self._open))
