"""The bridge against real PX4 (SIH rover simulation, no Gazebo) and the fake Lattice server.

Run: PX4_DIR=~/PX4-Autopilot pytest -m sitl   (needs `make px4_sitl_default` built in PX4_DIR)
PX4_INSTANCE picks the SITL instance (default 7; ports are offset by it, so it can run beside other sims).
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
from pymavlink import mavutil

from mavlink_lattice import geo
from mavlink_lattice.config import Config, EntityConfig, LatticeConfig, VehicleConfig
from mavlink_lattice.vehicle import Mode

from ..conftest import MARSHAL, TOKEN, TRANSIT, Harness, point, wait_until

PX4_DIR = os.environ.get("PX4_DIR")
INSTANCE = int(os.environ.get("PX4_INSTANCE", "7"))

pytestmark = [
    pytest.mark.sitl,
    pytest.mark.skipif(not PX4_DIR, reason="set PX4_DIR to a PX4-Autopilot checkout with px4_sitl_default built"),
]


@pytest.fixture(scope="module")
def px4(tmp_path_factory):
    build = Path(PX4_DIR).expanduser() / "build" / "px4_sitl_default"
    workdir = tmp_path_factory.mktemp(f"px4_{INSTANCE}")
    env = dict(os.environ, PX4_SIM_MODEL="sihsim_rover_ackermann", HEADLESS="1", PX4_PARAM_COM_DISARM_PRFLT="-1")
    log = open(workdir / "px4.log", "w")
    proc = subprocess.Popen(
        [str(build / "bin" / "px4"), "-i", str(INSTANCE), "-d", str(build / "etc"), "-w", str(workdir)],
        cwd=workdir,
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    yield workdir / "px4.log"
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(10)
    except subprocess.TimeoutExpired:
        proc.kill()
    log.close()


class Gcs:
    """A second MAVLink peer on PX4's GCS port, standing in for QGC or an RC takeover."""

    def __init__(self):
        self.conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{18570 + INSTANCE}", source_system=255)
        self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self.conn.wait_heartbeat(timeout=10)

    def set_mode(self, mode: Mode) -> None:
        self.conn.mav.command_long_send(self.conn.target_system, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode.main, mode.sub, 0, 0, 0, 0)

    def close(self) -> None:
        self.conn.close()


@pytest.fixture
async def sitl(px4, fake, tmp_path):
    cfg = Config(
        vehicle=VehicleConfig(mavlink=f"udpin:127.0.0.1:{14540 + INSTANCE}", auto_arm=True),
        entity=EntityConfig(id=f"px4-sih-{INSTANCE}", publish_period_s=0.5, simulated=True),
        lattice=LatticeConfig(url=fake.url, token=TOKEN, heartbeat_interval_s=1.0),
        state_dir=str(tmp_path),
    )
    h = Harness(fake, None, cfg)
    try:
        await h.start_bridge()
    except AssertionError:
        raise AssertionError(f"bridge didn't come up against PX4; see {px4}") from None
    s = h.bridge.vehicle.state
    await wait_until(lambda: s.home is not None and s.gps_fix >= 3 and (s.armed or s.prearm_ok), timeout=60, what="PX4 ready to arm")
    yield h
    await h.stop_bridge()
    assert fake.violations == []


def here(h) -> tuple[float, float]:
    s = h.bridge.vehicle.state
    return s.lat, s.lon


def ahead(h, north_m: float, east_m: float = 0.0) -> tuple[float, float]:
    return geo.offset(*here(h), north_m, east_m)


DONE = ["STATUS_CREATED", "STATUS_SENT", "STATUS_MACHINE_RECEIPT", "STATUS_WILCO", "STATUS_EXECUTING", "STATUS_DONE_OK"]


async def test_marshal(sitl):
    target = ahead(sitl, 20, 5)
    task_id = await sitl.create(MARSHAL, {"objective": {"point": point(*target, "pickup")}})
    assert await sitl.wait_terminal(task_id, timeout=60) == "STATUS_DONE_OK", sitl.error(task_id)
    assert sitl.fake.statuses(task_id) == DONE
    assert geo.distance_m(*here(sitl), *target) <= sitl.cfg.vehicle.arrival_radius_m + 0.5
    s = sitl.bridge.vehicle.state
    assert s.mode == Mode.HOLD and s.armed
    await wait_until(lambda: s.ground_speed_mps < 0.3, timeout=10, what="rover to stop")


async def test_transit(sitl):
    pts = [ahead(sitl, 15), ahead(sitl, 15, 15), ahead(sitl, 0, 15)]
    task_id = await sitl.create(TRANSIT, {"plan": {"route": {"path": [{"waypoint": {"llaPoint": point(*p)}} for p in pts]}}})
    assert await sitl.wait_terminal(task_id, timeout=90) == "STATUS_DONE_OK", sitl.error(task_id)
    assert sitl.fake.statuses(task_id) == DONE
    assert geo.distance_m(*here(sitl), *pts[-1]) < 4.0
    s = sitl.bridge.vehicle.state
    await wait_until(lambda: s.mode == Mode.HOLD, timeout=3, what="Hold after the route")


async def test_cancel_stops_the_rover(sitl):
    task_id = await sitl.create(MARSHAL, {"objective": {"point": point(*ahead(sitl, 200))}})
    await sitl.wait_status(task_id, "STATUS_EXECUTING")
    s = sitl.bridge.vehicle.state
    await wait_until(lambda: s.ground_speed_mps > 1.0, timeout=15, what="rover driving")
    await sitl.operator.tasks.cancel_task(task_id)
    assert await sitl.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    assert sitl.error(task_id)["code"] == "ERROR_CODE_CANCELLED"
    assert s.mode == Mode.HOLD
    await wait_until(lambda: s.ground_speed_mps < 0.3, timeout=10, what="rover to stop")


async def test_gcs_takeover_fails_the_task(sitl):
    task_id = await sitl.create(MARSHAL, {"objective": {"point": point(*ahead(sitl, 200))}})
    await sitl.wait_status(task_id, "STATUS_EXECUTING")
    s = sitl.bridge.vehicle.state
    await wait_until(lambda: s.ground_speed_mps > 1.0, timeout=15, what="rover driving")
    gcs = Gcs()
    try:
        gcs.set_mode(Mode.RTL)
        assert await sitl.wait_terminal(task_id) == "STATUS_DONE_NOT_OK"
    finally:
        gcs.close()
    assert sitl.error(task_id) == {"code": "ERROR_CODE_FAILED", "message": "vehicle left HOLD for RTL"}
    assert s.mode == Mode.RTL
    # Leave the rover parked for the next test.
    await sitl.bridge.vehicle.hold()
    time.sleep(0.5)
