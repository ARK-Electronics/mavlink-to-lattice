"""anduril.tasks.v2.Transit: drive a route of waypoints as a PX4 mission."""

from __future__ import annotations

from ..lattice.entity import Route
from ..vehicle import Mode
from .base import TYPE_PREFIX, Executor, TaskContext, TaskRejected, lla_of

MAX_WAYPOINTS = 500


class Transit(Executor):
    type_url = TYPE_PREFIX + "anduril.tasks.v2.Transit"
    expected_mode = Mode.MISSION

    def __init__(self, ctx: TaskContext, spec: dict):
        super().__init__(ctx, spec)
        path = (((spec.get("plan") or {}).get("route") or {}).get("path")) or []
        if not path:
            raise TaskRejected("Transit route has no path segments")
        if len(path) > MAX_WAYPOINTS:
            raise TaskRejected(f"Transit route has {len(path)} segments; the limit is {MAX_WAYPOINTS}")
        self.points: list[tuple[float, float]] = []
        for i, seg in enumerate(path):
            if "waypoint" in seg:
                self.points.append(lla_of((seg["waypoint"] or {}).get("llaPoint"), f"segment {i + 1} waypoint"))
            elif "loiter" in seg:
                objective = (seg["loiter"] or {}).get("objective") or {}
                if "point" not in objective:
                    raise TaskRejected(f"segment {i + 1}: loiter around an entity isn't supported in a route")
                self.points.append(lla_of(objective["point"], f"segment {i + 1} loiter point"))
            else:
                raise TaskRejected(f"segment {i + 1} has no waypoint")
        self.last_seq = -1

    async def start(self) -> None:
        v = self.ctx.vehicle
        self.last_seq = await v.upload_route(self.points)
        self.route = Route(f"waypoint {len(self.points)} of {len(self.points)}", *self.points[-1])
        await v.start_mission()

    async def step(self) -> bool:
        return self.ctx.vehicle.state.mission_reached >= self.last_seq

    async def finish(self) -> None:
        await self.ctx.vehicle.stop()
