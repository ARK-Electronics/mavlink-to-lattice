"""Evacuate: the bridge stages pickup and dropoff for the operator and reports progress, but never drives."""

from __future__ import annotations

import asyncio

import pytest
from pymavlink.dialects.v20 import common as mavlink

from mavlink_lattice import geo
from mavlink_lattice.config import EvacuateConfig, TasksConfig
from mavlink_lattice.tasks import evacuate
from mavlink_lattice.vehicle import Mode

from .conftest import MARSHAL, point, wait_until
from .test_bridge import DONE, ahead

# Stands in for the NGC2 Evacuate type, whose definition isn't public.
EVACUATE = "type.googleapis.com/test.ngc2.v1.Evacuate"
# Commands that would move or arm the vehicle. The bridge must send none of them for an Evacuate task.
DRIVING = {mavlink.MAV_CMD_DO_REPOSITION, mavlink.MAV_CMD_DO_SET_MODE, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH}


@pytest.fixture
def tasks_cfg() -> TasksConfig:
    return TasksConfig(
        enabled=["marshal", "evacuate"],
        evacuate=EvacuateConfig(type_url=EVACUATE, pickup="pickup_location", dropoff="dropoff_location", arrival_radius_m=5.0),
    )


@pytest.fixture(autouse=True)
def short_dwell(monkeypatch):
    monkeypatch.setattr(evacuate, "DWELL_S", 0.5)


def spec(pickup=None, dropoff=None) -> dict:
    s = {"casevacRequestId": "line-9-ref"}
    if pickup:
        s["pickupLocation"] = point(*pickup)
    if dropoff:
        s["dropoffLocation"] = point(*dropoff)
    return s


def staged(h) -> list[tuple[float, float]]:
    return [(i.x / 1e7, i.y / 1e7) for i in h.sim.mission if i.command == mavlink.MAV_CMD_NAV_WAYPOINT]


def at(a: tuple[float, float], b: tuple[float, float]) -> bool:
    return geo.distance_m(*a, *b) < 0.5


def operator_starts_mission(h) -> None:
    h.sim.armed = True
    h.sim.take_over(Mode.MISSION)


def bridge_commands(h) -> set[int]:
    return set(h.sim.commands) & DRIVING


async def test_catalog(harness):
    assert harness.catalog() == [MARSHAL, EVACUATE]


async def test_pickup_then_dropoff(harness):
    pickup, dropoff = ahead(harness, 40), ahead(harness, 40, 40)
    task_id = await harness.create(EVACUATE, spec(pickup, dropoff))
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    await wait_until(lambda: staged(harness) and at(staged(harness)[0], pickup), what="pickup staged")
    assert len(staged(harness)) == 1  # one leg at a time, so the mission as uploaded stops at the pickup
    assert not harness.sim.armed  # the bridge doesn't arm
    await wait_until(lambda: harness.entity().get("routeDetails", {}).get("destinationName") == "pickup", what="route to pickup")
    assert "Lattice: pickup uploaded as mission" in harness.sim.statustexts
    assert any(t.startswith("Pickup ") for t in harness.sim.statustexts) and any(t.startswith("Dropoff ") for t in harness.sim.statustexts)

    operator_starts_mission(harness)
    await wait_until(lambda: "Lattice: at pickup" in harness.sim.statustexts, what="at pickup")
    # Still in Mission mode at the pickup: an upload now would start the dropoff leg, now or on the next arming.
    await wait_until(lambda: "Lattice: leave Mission to load the dropoff" in harness.sim.statustexts, what="told to leave Mission")
    harness.sim.disarm()  # loading, which doesn't fail the task
    await asyncio.sleep(1.5)
    assert at(staged(harness)[0], pickup)
    harness.sim.take_over(Mode.HOLD)
    await wait_until(lambda: at(staged(harness)[0], dropoff), what="dropoff staged")
    assert harness.status(task_id) == "STATUS_EXECUTING"
    await wait_until(lambda: harness.entity()["routeDetails"]["destinationName"] == "dropoff", what="route to dropoff")

    operator_starts_mission(harness)
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"
    assert harness.fake.statuses(task_id) == DONE
    assert at((harness.sim.lat, harness.sim.lon), dropoff)
    await wait_until(lambda: harness.sim.statustexts[-1] == "Lattice Evacuate done", timeout=2, what="operator told")
    assert bridge_commands(harness) == set()


async def test_driving_past_isnt_arriving(harness):
    task_id = await harness.create(EVACUATE, spec(dropoff=ahead(harness, 30)))
    await wait_until(lambda: staged(harness), what="staged")
    # The operator's own route runs through the point and on, without stopping there.
    harness.sim.mission = [_waypoint(harness, p) for p in (ahead(harness, 60), ahead(harness, 60, 30))]
    harness.sim.mission_seq = 0
    operator_starts_mission(harness)
    await wait_until(lambda: harness.sim.mission_seq >= 2, timeout=20, what="route finished")
    assert harness.status(task_id) == "STATUS_EXECUTING"


def _waypoint(h, p):
    return h.sim.mav.mission_item_int_encode(1, 1, 0, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, int(p[0] * 1e7), int(p[1] * 1e7), 0, 0)


@pytest.mark.parametrize("which", ["pickup", "dropoff"])
async def test_one_position(harness, which):
    p = ahead(harness, 20)
    task_id = await harness.create(EVACUATE, spec(**{which: p}))
    await wait_until(lambda: staged(harness) and at(staged(harness)[0], p), what="staged")
    operator_starts_mission(harness)
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"
    assert f"Lattice: at {which}" in harness.sim.statustexts


async def test_operator_drives_by_hand(harness):
    """FPV for the last stretch: the vehicle gets there in Manual, with no mission at all."""
    p = ahead(harness, 20)
    task_id = await harness.create(EVACUATE, spec(pickup=p))
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    harness.sim.armed = True
    harness.sim.take_over(Mode.MANUAL)
    harness.sim.lat, harness.sim.lon = geo.offset(*p, 2.0, 0.0)
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"
    assert harness.sim.mode == Mode.MANUAL


async def test_cancel_leaves_vehicle_to_operator(harness):
    task_id = await harness.create(EVACUATE, spec(pickup=ahead(harness, 2000)))
    await wait_until(lambda: staged(harness), what="staged")
    operator_starts_mission(harness)
    await wait_until(lambda: harness.sim.vel != (0.0, 0.0), what="driving")
    await harness.operator.tasks.cancel_task(task_id)
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["code"] == "ERROR_CODE_CANCELLED"
    assert "Lattice Evacuate ended: cancelled by operator" in harness.sim.statustexts
    assert harness.sim.mode == Mode.MISSION and harness.sim.vel != (0.0, 0.0)
    assert bridge_commands(harness) == set()


async def test_waits_while_a_mission_runs(harness):
    far = ahead(harness, 2000)
    harness.sim.mission = [_waypoint(harness, far)]
    harness.sim.mission_seq = 0
    operator_starts_mission(harness)
    task_id = await harness.create(EVACUATE, spec(pickup=ahead(harness, 20)))
    await wait_until(lambda: "Lattice: leave Mission to load the pickup" in harness.sim.statustexts, what="told to wait")
    assert staged(harness) == [pytest.approx(far, abs=1e-6)]  # the running mission is untouched
    harness.sim.take_over(Mode.HOLD)
    await wait_until(lambda: "Lattice: pickup uploaded as mission" in harness.sim.statustexts, what="staged")
    assert harness.status(task_id) == "STATUS_EXECUTING"


async def test_supersedes_driving_task_and_stops_it(harness):
    first = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 2000))}})
    await harness.wait_status(first, "STATUS_EXECUTING")
    await wait_until(lambda: harness.sim.vel != (0.0, 0.0), what="driving")
    second = await harness.create(EVACUATE, spec(pickup=ahead(harness, 20)))
    assert await harness.wait_terminal(first) == "STATUS_DONE_NOT_OK"
    assert harness.error(first)["message"] == f"superseded by task {second}"
    await wait_until(lambda: harness.sim.target is None and harness.sim.vel == (0.0, 0.0), timeout=3, what="stopped")
    await harness.wait_status(second, "STATUS_EXECUTING")


async def test_edit_keeps_the_operators_route(harness):
    """Lattice adds the dropoff to the running task after the operator planned a route to the pickup."""
    pickup, dropoff = ahead(harness, 30), ahead(harness, 30, 30)
    task_id = await harness.create(EVACUATE, spec(pickup=pickup))
    await wait_until(lambda: staged(harness), what="pickup staged")
    route = [_waypoint(harness, ahead(harness, 15, 5)), _waypoint(harness, pickup)]
    harness.sim.mission, harness.sim.mission_seq = route, 0
    harness.fake.edit(task_id, {"@type": EVACUATE, **spec(pickup, dropoff)})
    await wait_until(lambda: "Lattice Evacuate task updated" in harness.sim.statustexts, what="operator told")
    await asyncio.sleep(1.5)
    assert harness.sim.mission is route
    operator_starts_mission(harness)
    await wait_until(lambda: "Lattice: at pickup" in harness.sim.statustexts, what="at pickup")
    harness.sim.take_over(Mode.HOLD)
    await wait_until(lambda: at(staged(harness)[0], dropoff), what="dropoff staged")
    operator_starts_mission(harness)
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"


async def test_keeps_the_operators_route_through_the_pickup(harness):
    pickup, dropoff = ahead(harness, 30), ahead(harness, 30, 30)
    task_id = await harness.create(EVACUATE, spec(pickup, dropoff))
    await wait_until(lambda: staged(harness), what="pickup staged")
    route = [_waypoint(harness, pickup), _waypoint(harness, dropoff)]
    harness.sim.mission, harness.sim.mission_seq = route, 0
    operator_starts_mission(harness)
    await wait_until(lambda: harness.sim.mission_seq >= 1, what="past the pickup waypoint")
    harness.sim.take_over(Mode.HOLD)  # stops there to load
    await wait_until(lambda: "Lattice: clear the mission to load the dropoff" in harness.sim.statustexts, what="told to clear")
    assert harness.sim.mission is route
    harness.sim.mission = []  # the operator clears it
    await wait_until(lambda: staged(harness) and at(staged(harness)[0], dropoff), what="dropoff staged")
    assert harness.status(task_id) == "STATUS_EXECUTING"


async def test_px4_without_mission_state(harness):
    """Older PX4 doesn't report MISSION_CURRENT's mission state: the bridge replaces only a mission the size of its own."""
    harness.sim.reports_mission_state = False
    pickup, dropoff = ahead(harness, 30), ahead(harness, 30, 30)
    task_id = await harness.create(EVACUATE, spec(pickup, dropoff))
    await wait_until(lambda: staged(harness), what="pickup staged")
    operator_starts_mission(harness)
    await wait_until(lambda: "Lattice: at pickup" in harness.sim.statustexts, what="at pickup")
    harness.sim.take_over(Mode.HOLD)
    await wait_until(lambda: at(staged(harness)[0], dropoff), what="dropoff staged over the bridge's own mission")
    harness.sim.mission = [_waypoint(harness, ahead(harness, 5)), _waypoint(harness, dropoff)]
    await asyncio.sleep(1.5)  # the operator's upload shows in MISSION_CURRENT before the edit arrives
    harness.fake.edit(task_id, {"@type": EVACUATE, **spec(pickup, ahead(harness, 0, 50))})
    await wait_until(lambda: "Lattice: clear the mission to load the dropoff" in harness.sim.statustexts, what="told to clear")
    assert len(harness.sim.mission) == 2


async def test_marshal_cancelled_before_wilco_leaves_operator_alone(harness):
    task_id = await harness.create(EVACUATE, spec(pickup=ahead(harness, 2000)))
    await wait_until(lambda: staged(harness), what="staged")
    operator_starts_mission(harness)
    await wait_until(lambda: harness.sim.vel != (0.0, 0.0) and harness.bridge.vehicle.state.armed, what="driving")
    before = len(harness.sim.commands)
    harness.fake.fail("PUT", "/status", delay_s=1.5, body_contains="STATUS_WILCO", count=1)
    marshal = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 10))}})
    await harness.wait_status(marshal, "STATUS_MACHINE_RECEIPT")
    await harness.operator.tasks.cancel_task(marshal)
    assert await harness.wait_terminal(marshal) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["message"] == f"superseded by task {marshal}"
    assert set(harness.sim.commands[before:]) & DRIVING == set()
    assert harness.sim.mode == Mode.MISSION and harness.sim.vel != (0.0, 0.0)


async def test_bridge_stop_with_a_superseded_task_unreported(harness):
    """A driving task the Evacuate superseded is still open, because its status can't reach Lattice. Shutdown still
    leaves the operator's vehicle alone."""
    first = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 2000))}})
    await harness.wait_status(first, "STATUS_EXECUTING")
    harness.fake.fail("PUT", f"/tasks/{first}/status")
    second = await harness.create(EVACUATE, spec(pickup=ahead(harness, 2000, 100)))
    # The Evacuate starts once the bridge gives up on the old task's status for now. That takes its retries, then the SDK's.
    await harness.wait_status(second, "STATUS_EXECUTING", timeout=60)
    assert first in harness.bridge.manager._open
    await wait_until(lambda: staged(harness), what="staged")
    operator_starts_mission(harness)
    await wait_until(lambda: harness.sim.vel != (0.0, 0.0), what="driving")
    before = len(harness.sim.commands)
    await harness.stop_bridge()
    assert set(harness.sim.commands[before:]) & DRIVING == set()
    assert harness.sim.vel != (0.0, 0.0)
    harness.fake.clear_faults()


async def test_edit_to_evacuate_stops_the_driving_task(harness):
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 2000))}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    await wait_until(lambda: harness.sim.vel != (0.0, 0.0), what="driving")
    harness.fake.edit(task_id, {"@type": EVACUATE, **spec(pickup=ahead(harness, 20))})
    await wait_until(lambda: harness.sim.target is None and harness.sim.vel == (0.0, 0.0), timeout=3, what="stopped")
    await wait_until(lambda: "Lattice Evacuate task updated" in harness.sim.statustexts, what="staged")


async def test_upload_abandoned_by_its_guard(harness):
    before = harness.sim.mission

    class Held(Exception):
        pass

    def guard() -> None:
        raise Held

    with pytest.raises(Held):
        await harness.bridge.vehicle.upload_route([ahead(harness, 10)], guard)
    assert harness.sim.mission is before


async def test_no_position_rejected(harness):
    task_id = await harness.create(EVACUATE, {"casevacRequestId": "x"})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id) == {"code": "ERROR_CODE_REJECTED", "message": "Evacuate has no position at pickup_location or dropoff_location"}


async def test_bridge_stop_leaves_vehicle_driving(harness):
    task_id = await harness.create(EVACUATE, spec(pickup=ahead(harness, 2000)))
    await wait_until(lambda: staged(harness), what="staged")
    operator_starts_mission(harness)
    await wait_until(lambda: harness.sim.vel != (0.0, 0.0), what="driving")
    await harness.stop_bridge()
    assert harness.error(task_id)["message"] == "bridge stopped"
    await wait_until(lambda: harness.sim.statustexts[-1] == "Lattice Evacuate ended: bridge stopped", timeout=2, what="operator told")
    assert harness.sim.vel != (0.0, 0.0) and bridge_commands(harness) == set()


async def test_bridge_crash_leaves_vehicle_driving(harness, tmp_path):
    task_id = await harness.create(EVACUATE, spec(pickup=ahead(harness, 2000)))
    await wait_until(lambda: staged(harness), what="staged")
    operator_starts_mission(harness)
    state = (tmp_path / "open_tasks.json").read_text()
    assert task_id in state
    manager = harness.bridge.manager
    manager.active, manager._open = None, {}  # skip the clean shutdown
    await harness.stop_bridge()
    (tmp_path / "open_tasks.json").write_text(state)
    harness.fake.redeliver_on_connect = False
    await harness.start_bridge()
    await harness.wait_terminal(task_id, timeout=5)
    assert harness.error(task_id)["message"] == "bridge restarted while running the task"
    assert harness.sim.mode == Mode.MISSION and harness.sim.vel != (0.0, 0.0)
    assert bridge_commands(harness) == set()


@pytest.mark.parametrize(
    "node, lla",
    [
        ({"lla": {"lat": 1.5, "lon": 2.5}}, (1.5, 2.5)),
        ({"latitudeDegrees": 1.5, "longitudeDegrees": 2.5}, (1.5, 2.5)),
        ({"point": {"lla": {"lat": 1.5, "lon": 2.5, "alt": 3}}}, (1.5, 2.5)),
        ({"position": {"latitude": -1.5, "longitude": 2.5}}, (-1.5, 2.5)),
        ({"lla": {"lat": 91.0, "lon": 2.5}}, None),
        ({"lla": {"lat": "1.5", "lon": "2.5"}}, None),
        ({"name": "no position"}, None),
    ],
)
def test_find_lla(node, lla):
    assert evacuate.find_lla(node) == lla


def test_at_path():
    s = {"pickupZone": {"centerPoint": {"lat": 1.0, "lon": 2.0}}}
    assert evacuate.at_path(s, "pickup_zone.center_point") == {"lat": 1.0, "lon": 2.0}
    assert evacuate.at_path(s, "pickupZone.centerPoint.lat") == 1.0
    assert evacuate.at_path(s, "pickup_zone.missing") is None
    assert evacuate.at_path(s, "pickup_zone.center_point.lat.deeper") is None
