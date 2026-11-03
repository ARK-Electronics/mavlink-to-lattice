from __future__ import annotations

import math

import pytest
from anduril import AsyncLattice, GoogleProtobufAny, Principal, Relations, System, TaskStatus
from anduril.core.api_error import ApiError

from mavlink_lattice import config, geo
from mavlink_lattice.vehicle.state import Mode

from .conftest import TOKEN

S = math.sqrt(0.5)


@pytest.mark.parametrize(
    "q_ned, q_enu",
    [
        ((1.0, 0.0, 0.0, 0.0), (S, 0.0, 0.0, S)),  # facing north: ENU yaw +90 degrees
        ((S, 0.0, 0.0, S), (1.0, 0.0, 0.0, 0.0)),  # facing east: ENU identity
        ((S, 0.0, 0.0, -S), (0.0, 0.0, 0.0, 1.0)),  # facing west: ENU yaw 180 degrees
    ],
)
def test_attitude_ned_to_enu(q_ned, q_enu):
    q = geo.attitude_ned_to_enu(q_ned)
    assert abs(sum(a * b for a, b in zip(q, q_enu))) == pytest.approx(1.0)  # q and -q are the same rotation


def test_attitude_keeps_body_axes():
    # Pitched nose-up 10 degrees facing north: forward stays north and points up in ENU.
    p = math.radians(10)
    q = geo.attitude_ned_to_enu((math.cos(p / 2), 0.0, math.sin(p / 2), 0.0))
    w, x, y, z = q
    forward = (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y))
    assert forward == pytest.approx((0.0, math.cos(p), math.sin(p)), abs=1e-9)


def test_offset_and_distance():
    lat, lon = geo.offset(47.0, 8.0, 100.0, 50.0)
    assert geo.distance_m(47.0, 8.0, lat, lon) == pytest.approx(math.hypot(100, 50), rel=1e-3)


def test_px4_modes():
    assert Mode.from_custom_mode((4 << 16) | (3 << 24)) == Mode.HOLD
    assert Mode.from_custom_mode((4 << 16) | (4 << 24)) == Mode.MISSION
    assert Mode.from_custom_mode(1 << 16) == Mode.MANUAL
    assert Mode.from_custom_mode((9 << 16)) == Mode.UNKNOWN


def test_config(tmp_path):
    path = tmp_path / "c.toml"
    path.write_text('[lattice]\nurl = "https://x"\n[vehicle]\nauto_arm = true\n[entity]\nname = "Rover 1"\n')
    cfg = config.load(path, env={"LATTICE_TOKEN": "secret"})
    assert cfg.lattice.token == "secret" and cfg.vehicle.auto_arm and cfg.entity.name == "Rover 1"
    path.write_text('[lattice]\nurl = "https://x"\ntokn = "typo"\n')
    with pytest.raises(ValueError, match="tokn"):
        config.load(path, env={})
    path.write_text('[lattice]\nurl = "https://x"\n')
    with pytest.raises(ValueError, match="token"):
        config.load(path, env={})
    assert config.load(path, env={"LATTICE_TOKEN": "t"}).vehicle.mavlink == "udpin:127.0.0.1:14540"


def test_tasks_config(tmp_path):
    path = tmp_path / "c.toml"
    base = '[lattice]\nurl = "https://x"\ntoken = "t"\n'
    path.write_text(base + '[tasks]\nenabled = ["evacuate"]\n[tasks.evacuate]\ntype_url = "type.googleapis.com/a.v1.Evacuate"\npickup = "pickup.point"\n')
    cfg = config.load(path, env={})
    assert cfg.tasks.enabled == ["evacuate"]
    assert cfg.tasks.evacuate.type_url == "type.googleapis.com/a.v1.Evacuate" and cfg.tasks.evacuate.pickup == "pickup.point"
    assert cfg.tasks.evacuate.dropoff == "dropoff"
    path.write_text(base + '[tasks]\nenabled = ["evacuate"]\n')
    with pytest.raises(ValueError, match="type_url"):
        config.load(path, env={})
    path.write_text(base + '[tasks]\nenabled = ["evacuate"]\n[tasks.evacuate]\ntype_url = "type.googleapis.com/anduril.tasks.v2.Marshal"\n')
    with pytest.raises(ValueError, match="built-in"):
        config.load(path, env={})
    path.write_text(base + '[tasks]\nenabled = ["evacuate"]\n[tasks.evacuate]\ntype_url = "type.googleapis.com/a.v1.Evacuate"\narrival_radius_m = 0\n')
    with pytest.raises(ValueError, match="arrival_radius_m"):
        config.load(path, env={})
    path.write_text(base + '[tasks]\nenabled = ["marshall"]\n')
    with pytest.raises(ValueError, match="marshall"):
        config.load(path, env={})
    path.write_text(base + '[tasks.evacuate]\npickpu = "x"\n')
    with pytest.raises(ValueError, match="pickpu"):
        config.load(path, env={})


async def _task(fake, client):
    fake.require_catalog = False
    return await client.tasks.create_task(
        specification=GoogleProtobufAny.model_validate({"@type": "type.googleapis.com/anduril.tasks.v2.Marshal"}),
        relations=Relations(assignee=Principal(system=System(entity_id="agent"))),
    )


async def test_fake_status_versions(fake):
    client = AsyncLattice(base_url=fake.url, token=TOKEN)
    task = await _task(fake, client)
    tid = task.version.task_id
    author = Principal(system=System(entity_id="agent"))
    stale = await client.tasks.update_task_status(tid, status_version=1, new_status=TaskStatus(status="STATUS_WILCO"), author=author)
    assert stale.status.status == "STATUS_CREATED" and fake.ignored_updates
    ok = await client.tasks.update_task_status(tid, status_version=2, new_status=TaskStatus(status="STATUS_DONE_OK"), author=author)
    assert ok.version.status_version == 2 and ok.status.status == "STATUS_DONE_OK"
    with pytest.raises(ApiError) as exc:
        await client.tasks.update_task_status(tid, status_version=3, new_status=TaskStatus(status="STATUS_EXECUTING"), author=author)
    assert exc.value.status_code == 400
    fake.violations.clear()  # the update after terminal was deliberate


async def test_fake_rejects_bad_enum_and_unknown_fields(fake):
    client = AsyncLattice(base_url=fake.url, token=TOKEN)
    task = await _task(fake, client)
    with pytest.raises(ApiError) as exc:
        await client.tasks.update_task_status(task.version.task_id, status_version=2, new_status=TaskStatus(status="DONE"))
    assert exc.value.status_code == 400 and "DONE" in str(exc.value.body)
    fake.violations.clear()


async def test_fake_cancel_before_delivery(fake):
    client = AsyncLattice(base_url=fake.url, token=TOKEN)
    task = await _task(fake, client)
    cancelled = await client.tasks.cancel_task(task.version.task_id)
    assert cancelled.status.status == "STATUS_DONE_NOT_OK"
    assert cancelled.status.task_error.code == "ERROR_CODE_CANCELLED"


async def test_fake_auth(fake):
    with pytest.raises(ApiError) as exc:
        await AsyncLattice(base_url=fake.url, token="wrong").entities.get_entity("x")
    assert exc.value.status_code == 401
    fake.client_id, fake.client_secret = "id", "secret"
    with pytest.raises(ApiError) as exc:
        await AsyncLattice(base_url=fake.url, client_id="id", client_secret="secret").entities.get_entity("x")
    assert exc.value.status_code == 404  # authenticated with an OAuth token; the entity just doesn't exist
