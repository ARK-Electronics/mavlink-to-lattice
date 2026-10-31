"""Vehicle state to a Lattice asset entity. Pure: the same inputs always build the same entity."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta

from anduril import (
    Aliases,
    AlternateId,
    Enu,
    Health,
    Indicators,
    Location,
    MilStd2525C,
    MilView,
    Ontology,
    Position,
    PowerLevel,
    PowerSource,
    PowerState,
    Provenance,
    Quaternion,
    RouteDetails,
    Status,
    Symbology,
    TaskCatalog,
    TaskDefinition,
)
from pymavlink.dialects.v20 import common as mavlink

from .. import geo
from ..config import EntityConfig
from ..vehicle.state import Mode, VehicleState

INTEGRATION_NAME = "mavlink-lattice"

_AIR = {
    mavlink.MAV_TYPE_FIXED_WING,
    mavlink.MAV_TYPE_QUADROTOR,
    mavlink.MAV_TYPE_COAXIAL,
    mavlink.MAV_TYPE_HELICOPTER,
    mavlink.MAV_TYPE_HEXAROTOR,
    mavlink.MAV_TYPE_OCTOROTOR,
    mavlink.MAV_TYPE_TRICOPTER,
    mavlink.MAV_TYPE_DECAROTOR,
    mavlink.MAV_TYPE_DODECAROTOR,
    mavlink.MAV_TYPE_GENERIC_MULTIROTOR,
    mavlink.MAV_TYPE_VTOL_DUOROTOR,
    mavlink.MAV_TYPE_VTOL_QUADROTOR,
    mavlink.MAV_TYPE_VTOL_TILTROTOR,
    mavlink.MAV_TYPE_VTOL_GYRODYNE,
}


def platform(mav_type: int) -> tuple[str, str]:
    """Lattice platform type and military environment for a MAV_TYPE."""
    if mav_type == mavlink.MAV_TYPE_GROUND_ROVER:
        return "UGV", "ENVIRONMENT_LAND"
    if mav_type == mavlink.MAV_TYPE_SURFACE_BOAT:
        return "USV", "ENVIRONMENT_SURFACE"
    if mav_type == mavlink.MAV_TYPE_SUBMARINE:
        return "UUV", "ENVIRONMENT_SUB_SURFACE"
    if mav_type in _AIR:
        return "UAV", "ENVIRONMENT_AIR"
    return "Unmanned Vehicle", "ENVIRONMENT_UNKNOWN"


def _set(cls, **fields):
    """Build an SDK model with only the fields that have values; the SDK would send the rest as null."""
    return cls(**{k: v for k, v in fields.items() if v is not None})


def default_entity_id(state: VehicleState) -> str:
    if state.uid:
        return f"mavlink-{state.uid:016x}"
    return f"mavlink-sys{state.system_id}"


@dataclass
class Route:
    """What the active task is driving to, shown in the entity's route details."""

    destination: str
    lat: float
    lon: float


def health(state: VehicleState, link_ok: bool) -> str:
    if not link_ok:
        return "HEALTH_STATUS_OFFLINE"
    if state.system_status in (mavlink.MAV_STATE_CRITICAL, mavlink.MAV_STATE_EMERGENCY):
        return "HEALTH_STATUS_FAIL"
    if state.sensors_unhealthy or (state.battery.percent is not None and state.battery.percent < 15):
        return "HEALTH_STATUS_WARN"
    return "HEALTH_STATUS_HEALTHY"


def activity(state: VehicleState, link_ok: bool) -> str:
    if not link_ok:
        return "NO VEHICLE LINK"
    if not state.armed:
        return "DISARMED"
    return state.mode.name if state.mode is not Mode.UNKNOWN else "ARMED"


def build(
    cfg: EntityConfig,
    entity_id: str,
    state: VehicleState,
    *,
    link_ok: bool,
    now: datetime,
    created: datetime,
    task_types: list[str],
    route: Route | None = None,
) -> dict:
    """Keyword arguments for the SDK's publish_entity. Needs a position (an asset without one is rejected)."""
    assert state.has_position
    platform_type, environment = platform(state.mav_type)
    attitude = None
    if state.attitude:
        w, x, y, z = geo.attitude_ned_to_enu(state.attitude)
        attitude = Quaternion(w=w, x=x, y=y, z=z)
    location = _set(
        Location,
        position=_set(Position, latitude_degrees=state.lat, longitude_degrees=state.lon, altitude_hae_meters=state.alt_hae_m),
        velocity_enu=Enu(**dict(zip("enu", geo.velocity_ned_to_enu(*state.vel_ned)))),
        speed_mps=state.ground_speed_mps,
        attitude_enu=attitude,
    )

    alternate_ids = [AlternateId(id=f"{state.uid:016x}", type="ALT_ID_TYPE_SERIAL_NUMBER")] if state.uid else None
    kwargs: dict = dict(
        entity_id=entity_id,
        description=f"MAVLink vehicle, system ID {state.system_id}",
        is_live=True,
        created_time=created,
        expiry_time=now + timedelta(seconds=cfg.expiry_s),
        aliases=_set(Aliases, name=cfg.name or entity_id, alternate_ids=alternate_ids),
        ontology=_set(Ontology, template="TEMPLATE_ASSET", platform_type=cfg.platform_type or platform_type, specific_type=cfg.specific_type or None),
        mil_view=MilView(disposition="DISPOSITION_FRIENDLY", environment=environment),
        location=location,
        provenance=Provenance(integration_name=INTEGRATION_NAME, data_type="mavlink", source_update_time=now),
        health=Health(
            connection_status="CONNECTION_STATUS_ONLINE" if link_ok else "CONNECTION_STATUS_OFFLINE",
            health_status=health(state, link_ok),
            update_time=now,
        ),
        status=Status(platform_activity=activity(state, link_ok)),
        indicators=Indicators(simulated=cfg.simulated),
        task_catalog=TaskCatalog(task_definitions=[TaskDefinition(task_specification_url=url) for url in task_types]),
    )
    b = state.battery
    if b.percent is not None or b.voltage_v is not None:
        kwargs["power_state"] = PowerState(
            source_id_to_state={
                "battery": PowerSource(
                    power_status="POWER_STATUS_OPERATING",
                    power_type="POWER_TYPE_BATTERY",
                    power_level=_set(
                        PowerLevel,
                        percent_remaining=b.percent,
                        voltage=b.voltage_v,
                        current_amps=b.current_a,
                        run_time_to_empty_mins=b.time_remaining_s / 60 if b.time_remaining_s else None,
                    ),
                )
            }
        )
    if cfg.sidc:
        kwargs["symbology"] = Symbology(mil_std2525c=MilStd2525C(sidc=cfg.sidc))
    if route:
        details: dict = {"destination_name": route.destination}
        speed = state.ground_speed_mps
        if speed > 0.2:
            remaining = geo.distance_m(state.lat, state.lon, route.lat, route.lon)
            details["estimated_arrival_time"] = now + timedelta(seconds=remaining / speed)
        kwargs["route_details"] = RouteDetails(**details)
    return kwargs
