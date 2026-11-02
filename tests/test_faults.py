"""Failure handling: whatever goes wrong, the vehicle stops and every task ends with a terminal status."""

from __future__ import annotations

import asyncio
from datetime import datetime, timedelta, timezone

import pytest
from anduril import ExecutionConstraints
from pymavlink.dialects.v20 import common as mavlink

from mavlink_lattice import geo
from mavlink_lattice.tasks import manager
from mavlink_lattice.vehicle import Mode

from .conftest import MARSHAL, TRANSIT, point, wait_until
from .test_bridge import _publish_target, ahead


def stopped(h) -> bool:
    return h.sim.target is None and h.sim.vel == (0.0, 0.0)


async def drive_far(h) -> str:
    task_id = await h.create(MARSHAL, {"objective": {"point": point(*ahead(h, 2000))}})
    await h.wait_status(task_id, "STATUS_EXECUTING")
    await wait_until(lambda: h.sim.vel != (0.0, 0.0), what="driving")
    return task_id


async def test_failed_task_stops_vehicle(harness):
    await _publish_target(harness, "casualty-2", *ahead(harness, 2000), expiry_s=2.0)
    task_id = await harness.create(MARSHAL, {"objective": {"entityId": "casualty-2"}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert "gone" in harness.error(task_id)["message"]
    await wait_until(lambda: stopped(harness), timeout=3, what="vehicle stopped")


async def test_superseding_task_that_fails_to_start_stops_vehicle(harness):
    first = await drive_far(harness)
    harness.sim.refuse[mavlink.MAV_CMD_DO_SET_MODE] = mavlink.MAV_RESULT_DENIED
    second = await harness.create(TRANSIT, {"plan": {"route": {"path": [{"waypoint": {"llaPoint": point(*ahead(harness, 30))}}]}}})
    assert await harness.wait_terminal(second) == "STATUS_DONE_NOT_OK"
    assert harness.error(first)["message"] == f"superseded by task {second}"
    assert "DENIED" in harness.error(second)["message"]
    await wait_until(lambda: stopped(harness), timeout=3, what="vehicle stopped")


async def test_cancel_through_lattice_errors(harness):
    task_id = await drive_far(harness)
    harness.fake.fail("GET", "/api/v1/tasks/", count=3)
    harness.fake.fail("PUT", "/status", count=2)
    await harness.operator.tasks.cancel_task(task_id)
    await wait_until(lambda: stopped(harness), timeout=3, what="vehicle stopped")
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["code"] == "ERROR_CODE_CANCELLED"


@pytest.mark.parametrize("redeliver", [True, False])
async def test_cancel_sent_while_stream_stalled(harness, redeliver):
    """Lattice thinks it delivered the cancel; the bridge finds it by re-reading the task after reconnecting."""
    harness.fake.redeliver_on_connect = redeliver
    task_id = await drive_far(harness)
    harness.fake.stall_streams()
    await harness.operator.tasks.cancel_task(task_id)
    assert await harness.wait_terminal(task_id, timeout=10) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["code"] == "ERROR_CODE_CANCELLED"
    await wait_until(lambda: stopped(harness), timeout=1, what="vehicle stopped")


async def test_other_component_heartbeats_ignored(harness):
    harness.sim.camera = True
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 40))}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"
    assert harness.bridge.vehicle.state.armed


async def test_terminal_status_retried_after_outage(harness, monkeypatch):
    monkeypatch.setattr(manager, "UNSENT_RETRY_S", 0.5)
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 40))}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    harness.fake.fail("PUT", "/status", seconds=12)
    await wait_until(lambda: geo.distance_m(harness.sim.lat, harness.sim.lon, *harness.sim.target or (0, 0)) < 1 or harness.sim.target is None, timeout=10, what="arrival")
    assert await harness.wait_terminal(task_id, timeout=30) == "STATUS_DONE_OK"


async def test_shutdown_while_starting_a_task(harness):
    harness.sim.arm_delay_s = 2.0
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 40))}})
    await harness.wait_status(task_id, "STATUS_MACHINE_RECEIPT")
    await harness.stop_bridge()
    assert harness.status(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["message"] == "bridge stopped"


async def test_cancel_while_starting_a_task(harness):
    harness.sim.arm_delay_s = 1.5
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 40))}})
    await harness.wait_status(task_id, "STATUS_MACHINE_RECEIPT")
    await harness.operator.tasks.cancel_task(task_id)
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["code"] == "ERROR_CODE_CANCELLED"
    assert "STATUS_EXECUTING" not in harness.fake.statuses(task_id)
    await wait_until(lambda: stopped(harness), timeout=1, what="vehicle stopped")


async def test_edited_task_runs_the_new_definition(harness):
    task_id = await drive_far(harness)
    near = ahead(harness, 0, 30)
    harness.fake.edit(task_id, {"@type": MARSHAL, "objective": {"point": point(*near)}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"
    assert geo.distance_m(harness.sim.lat, harness.sim.lon, *near) < 2.5


async def test_task_from_a_previous_run_is_failed(harness, tmp_path):
    """Without the state file, a redelivered task the bridge was running before a crash is failed, not rerun."""
    task_id = await drive_far(harness)
    manager_ = harness.bridge.manager
    manager_.active, manager_._open = None, {}  # crash: no clean shutdown, no state file
    await harness.stop_bridge()
    (tmp_path / "open_tasks.json").unlink()
    await harness.start_bridge()
    assert await harness.wait_terminal(task_id, timeout=5) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["message"] == "bridge restarted while running the task"
    await wait_until(lambda: stopped(harness), timeout=3, what="vehicle stopped")


async def test_slow_lattice_doesnt_delay_safety_checks(harness):
    """The EXECUTING update hangs for 10 s; the completeBefore stop still happens on time."""
    deadline = datetime.now(timezone.utc) + timedelta(seconds=5)
    harness.fake.fail("PUT", "/status", count=1, delay_s=10.0, body_contains="STATUS_EXECUTING")
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 2000))}}, execution_constraints=ExecutionConstraints(complete_before=deadline))
    await wait_until(lambda: harness.sim.vel != (0.0, 0.0), timeout=5, what="driving")
    await wait_until(lambda: datetime.now(timezone.utc) > deadline, timeout=10, what="deadline")
    await wait_until(lambda: stopped(harness), timeout=1.5, what="stopped at completeBefore")
    assert await harness.wait_terminal(task_id, timeout=30) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["code"] == "ERROR_CODE_TIMEOUT"
    assert harness.fake.statuses(task_id)[-2:] == ["STATUS_EXECUTING", "STATUS_DONE_NOT_OK"]


async def test_operator_mission_after_a_stop_is_left_alone(harness):
    """The stop watchdog re-stops a vehicle that stayed in Mission, not one an operator put back in Mission later."""
    path = [{"waypoint": {"llaPoint": point(*ahead(harness, 2000))}}]
    task_id = await harness.create(TRANSIT, {"plan": {"route": {"path": path}}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    await wait_until(lambda: harness.bridge.vehicle.state.mode == Mode.MISSION, what="bridge sees Mission")
    await harness.operator.tasks.cancel_task(task_id)
    await harness.wait_terminal(task_id)
    await wait_until(lambda: stopped(harness) and harness.sim.mode == Mode.HOLD, timeout=3, what="stopped")
    await asyncio.sleep(1.5)  # the watchdog sees the stop took effect
    harness.sim.take_over(Mode.MISSION)
    await asyncio.sleep(manager.MODE_GRACE_S + 1.5)
    assert harness.sim.mode == Mode.MISSION and harness.sim.vel != (0.0, 0.0)
