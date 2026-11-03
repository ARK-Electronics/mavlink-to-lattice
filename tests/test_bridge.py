"""The whole bridge against the fake Lattice server and the simulated rover."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

import pytest
from anduril import ExecutionConstraints
from pymavlink.dialects.v20 import common as mavlink

from mavlink_lattice import geo
from mavlink_lattice.config import VehicleConfig
from mavlink_lattice.vehicle import Mode

from .conftest import MARSHAL, TRANSIT, point, wait_until

DONE = ["STATUS_CREATED", "STATUS_SENT", "STATUS_MACHINE_RECEIPT", "STATUS_WILCO", "STATUS_EXECUTING", "STATUS_DONE_OK"]


def ahead(h, north_m: float, east_m: float = 0.0) -> tuple[float, float]:
    return geo.offset(h.sim.lat, h.sim.lon, north_m, east_m)


async def test_entity(harness):
    e = harness.entity()
    assert harness.entity_id == "mavlink-00ab12cd34ef5678"
    assert e["ontology"] == {"template": "TEMPLATE_ASSET", "platformType": "UGV"}
    assert e["milView"]["environment"] == "ENVIRONMENT_LAND"
    assert e["aliases"]["alternateIds"] == [{"id": "00ab12cd34ef5678", "type": "ALT_ID_TYPE_SERIAL_NUMBER"}]
    assert e["location"]["position"]["altitudeHaeMeters"] == pytest.approx(488.0 + 47.0, abs=0.01)
    assert e["health"]["healthStatus"] == "HEALTH_STATUS_HEALTHY"
    assert e["status"]["platformActivity"] == "DISARMED"
    assert e["indicators"]["simulated"] is True
    assert e["powerState"]["sourceIdToState"]["battery"]["powerLevel"]["voltage"] == pytest.approx(16.8)
    assert harness.catalog() == [MARSHAL, TRANSIT]


async def test_marshal_to_point(harness):
    lat, lon = ahead(harness, 30)
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(lat, lon, "pickup")}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    await wait_until(lambda: "routeDetails" in harness.entity(), what="route details")
    assert harness.entity()["routeDetails"]["destinationName"] == "pickup"
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"
    assert harness.fake.statuses(task_id) == DONE
    assert geo.distance_m(harness.sim.lat, harness.sim.lon, lat, lon) < 2.5
    assert harness.sim.armed  # auto_arm
    await wait_until(lambda: "routeDetails" not in harness.entity(), what="route details cleared")


async def test_transit(harness):
    pts = [ahead(harness, 15), ahead(harness, 15, 15), ahead(harness, 0, 15)]
    path = [{"waypoint": {"llaPoint": point(*p)}} for p in pts]
    task_id = await harness.create(TRANSIT, {"plan": {"route": {"path": path}}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"
    assert harness.fake.statuses(task_id) == DONE
    uploaded = [(i.x / 1e7, i.y / 1e7) for i in harness.sim.mission]
    assert [c for p in uploaded for c in p] == pytest.approx([c for p in pts for c in p], abs=1e-6)
    assert geo.distance_m(harness.sim.lat, harness.sim.lon, *pts[-1]) < 1.0
    await wait_until(lambda: harness.sim.mode == Mode.HOLD, timeout=2, what="Hold after the route")


async def test_cancel(harness):
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 500))}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    await harness.operator.tasks.cancel_task(task_id)
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["code"] == "ERROR_CODE_CANCELLED"
    assert harness.fake.statuses(task_id)[-2:] == ["STATUS_CANCEL_REQUESTED", "STATUS_DONE_NOT_OK"]
    assert harness.sim.mode == Mode.HOLD and harness.sim.target is None


async def test_operator_complete(harness):
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 500))}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    harness.fake.complete(task_id)
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"
    assert harness.sim.target is None


@pytest.mark.parametrize("mode", [Mode.MANUAL, Mode.RTL])
async def test_takeover_fails_task(harness, mode):
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 500))}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    # Far enough from home that RTL doesn't arrive (and auto-disarm) before the bridge notices the mode.
    await wait_until(lambda: geo.distance_m(harness.sim.lat, harness.sim.lon, *harness.sim.home) > 10, what="driving")
    harness.sim.take_over(mode)
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id) == {"code": "ERROR_CODE_FAILED", "message": f"vehicle left HOLD for {mode.name}"}
    assert harness.sim.mode == mode  # the bridge doesn't fight whoever took over


async def test_disarm_fails_task(harness):
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 500))}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    harness.sim.disarm()
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["message"] == "vehicle disarmed"


async def test_vehicle_link_loss(harness):
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 500))}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    harness.sim.silent = True
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["message"] == "lost the link to the vehicle"
    await wait_until(lambda: harness.entity()["health"]["healthStatus"] == "HEALTH_STATUS_OFFLINE", what="entity offline")
    assert harness.catalog() == []
    harness.sim.silent = False
    await wait_until(lambda: harness.catalog() != [], what="catalog back")


async def test_new_task_supersedes(harness):
    first = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 500))}})
    await harness.wait_status(first, "STATUS_EXECUTING")
    second = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 0, 20))}})
    assert await harness.wait_terminal(first) == "STATUS_DONE_NOT_OK"
    assert harness.error(first) == {"code": "ERROR_CODE_CANCELLED", "message": f"superseded by task {second}"}
    assert await harness.wait_terminal(second) == "STATUS_DONE_OK"


async def test_stream_drop_mid_task(harness):
    """The fake redelivers open tasks on reconnect; the bridge keeps running the task it already has."""
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 40))}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    harness.fake.drop_streams()
    await wait_until(lambda: harness.fake.connections(harness.entity_id) == 0, what="dropped")
    await wait_until(lambda: harness.fake.connections(harness.entity_id) == 1, what="reconnected")
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"
    assert harness.fake.statuses(task_id) == DONE


async def test_stream_stall_reconnects(harness):
    harness.fake.stall_streams()
    await wait_until(lambda: harness.fake.connections(harness.entity_id) == 1, timeout=10, what="reconnect after stall")
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 10))}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"


async def test_bridge_stop_mid_task(harness):
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 500))}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    await harness.stop_bridge()
    assert harness.status(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["message"] == "bridge stopped"
    assert harness.sim.target is None


async def test_bridge_crash_mid_task(harness, tmp_path):
    """A task left executing by a bridge that died without cleanup is failed by the next run."""
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 500))}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    state = (tmp_path / "open_tasks.json").read_text()
    assert task_id in state
    manager = harness.bridge.manager
    manager.active, manager._open = None, {}  # skip the clean shutdown
    await harness.stop_bridge()
    (tmp_path / "open_tasks.json").write_text(state)
    assert harness.sim.target is not None  # still driving
    harness.fake.redeliver_on_connect = False
    await harness.start_bridge()
    await harness.wait_terminal(task_id, timeout=5)
    assert harness.error(task_id)["message"] == "bridge restarted while running the task"
    assert harness.sim.target is None


async def test_command_retry(harness):
    harness.sim.ignore[mavlink.MAV_CMD_DO_REPOSITION] = 2
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 10))}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"


async def test_command_denied(harness):
    harness.sim.refuse[mavlink.MAV_CMD_DO_REPOSITION] = mavlink.MAV_RESULT_DENIED
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 10))}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id) == {"code": "ERROR_CODE_FAILED", "message": "MAV_CMD_DO_REPOSITION: MAV_RESULT_DENIED"}
    assert "STATUS_EXECUTING" not in harness.fake.statuses(task_id)


async def test_complete_before_passes(harness):
    deadline = datetime.now(timezone.utc) + timedelta(seconds=2)
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 500))}}, execution_constraints=ExecutionConstraints(complete_before=deadline))
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["code"] == "ERROR_CODE_TIMEOUT"
    assert harness.sim.target is None


async def test_bad_spec_rejected(harness):
    task_id = await harness.create(MARSHAL, {"objective": {"point": {"lla": {"lat": 95.0, "lon": 8.5}}}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id) == {"code": "ERROR_CODE_REJECTED", "message": "objective point has no valid lat/lon"}
    assert harness.fake.statuses(task_id)[-2:] == ["STATUS_MACHINE_RECEIPT", "STATUS_DONE_NOT_OK"]


async def test_unsupported_type_rejected(harness):
    harness.fake.require_catalog = False
    task_id = await harness.create("type.googleapis.com/anduril.tasks.v2.Monitor", {"objective": {"point": point(*ahead(harness, 10))}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["message"] == "unsupported task type type.googleapis.com/anduril.tasks.v2.Monitor"


@pytest.mark.parametrize("vehicle_cfg", [VehicleConfig(link_timeout_s=1.5, auto_arm=False)])
async def test_disarmed_rejected(harness):
    task_id = await harness.create(MARSHAL, {"objective": {"point": point(*ahead(harness, 10))}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id) == {"code": "ERROR_CODE_REJECTED", "message": "vehicle is disarmed"}
    assert not harness.sim.armed


async def _publish_target(harness, entity_id: str, lat: float, lon: float, expiry_s: float = 60.0) -> None:
    from anduril import Aliases, Location, MilView, Ontology, Position, Provenance

    now = datetime.now(timezone.utc)
    await harness.operator.entities.publish_entity(
        entity_id=entity_id,
        is_live=True,
        expiry_time=now + timedelta(seconds=expiry_s),
        aliases=Aliases(name="casualty"),
        ontology=Ontology(template="TEMPLATE_TRACK"),
        mil_view=MilView(disposition="DISPOSITION_FRIENDLY", environment="ENVIRONMENT_LAND"),
        location=Location(position=Position(latitude_degrees=lat, longitude_degrees=lon)),
        provenance=Provenance(integration_name="test", data_type="test", source_update_time=now),
    )


async def test_marshal_to_entity_that_moves(harness):
    await _publish_target(harness, "casualty-1", *ahead(harness, 40))
    task_id = await harness.create(MARSHAL, {"objective": {"entityId": "casualty-1"}})
    await harness.wait_status(task_id, "STATUS_EXECUTING")
    await wait_until(lambda: harness.entity().get("routeDetails", {}).get("destinationName") == "casualty", what="route details")
    moved = ahead(harness, 0, 25)
    await _publish_target(harness, "casualty-1", *moved)
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_OK"
    assert geo.distance_m(harness.sim.lat, harness.sim.lon, *moved) < 2.5


async def test_marshal_to_missing_entity(harness):
    task_id = await harness.create(MARSHAL, {"objective": {"entityId": "nobody"}})
    assert await harness.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert harness.error(task_id)["message"] == "objective entity nobody not found"
