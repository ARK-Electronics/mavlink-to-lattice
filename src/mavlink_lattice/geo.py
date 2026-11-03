"""Geodesy and frame conversions between MAVLink (NED/FRD) and Lattice (ENU/FLU)."""

from __future__ import annotations

import math

EARTH_RADIUS_M = 6371008.8

Quat = tuple[float, float, float, float]  # (w, x, y, z)


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance; accurate to well under a metre at the ranges a ground vehicle drives."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(min(1.0, a)))


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Point displaced by a local north/east offset (flat-earth; fine for a few km)."""
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def quat_mul(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


_S = math.sqrt(0.5)
# Rotation taking NED vectors to ENU: swap north/east, negate down.
_Q_ENU_NED: Quat = (0.0, _S, _S, 0.0)
# Rotation taking FLU body vectors to FRD: negate y and z.
_Q_FRD_FLU: Quat = (0.0, 1.0, 0.0, 0.0)


def attitude_ned_to_enu(q_ned_frd: Quat) -> Quat:
    """MAVLink attitude (FRD body to NED) to Lattice attitude (FLU body to ENU)."""
    w, x, y, z = quat_mul(quat_mul(_Q_ENU_NED, q_ned_frd), _Q_FRD_FLU)
    if w < 0:  # keep a canonical sign so consumers comparing quaternions don't see flips
        w, x, y, z = -w, -x, -y, -z
    return (w, x, y, z)


def velocity_ned_to_enu(vn: float, ve: float, vd: float) -> tuple[float, float, float]:
    return (ve, vn, -vd)
