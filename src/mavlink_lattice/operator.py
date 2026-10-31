"""Send tasks the way a Lattice operator would, for trying the bridge by hand (against the fake or a real environment).

    export LATTICE_URL=http://127.0.0.1:8080 LATTICE_TOKEN=dev
    lattice-task marshal mavlink-sys1 47.3980 8.5457 --name pickup
    lattice-task transit mavlink-sys1 47.3980,8.5457 47.3982,8.5460
    lattice-task watch <task-id>
    lattice-task cancel <task-id>
    lattice-task complete <task-id>     # fake server only; on real Lattice an operator does this in the UI
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys

import httpx
from anduril import AsyncLattice, GoogleProtobufAny, Principal, Relations, System, User

PREFIX = "type.googleapis.com/anduril.tasks.v2."


def _point(lat: float, lon: float, name: str = "") -> dict:
    p: dict = {"lla": {"lat": lat, "lon": lon}}
    if name:
        p["referenceName"] = name
    return p


async def _create(client: AsyncLattice, assignee: str, type_name: str, spec: dict, description: str) -> None:
    task = await client.tasks.create_task(
        specification=GoogleProtobufAny.model_validate({"@type": PREFIX + type_name, **spec}),
        relations=Relations(assignee=Principal(system=System(entity_id=assignee))),
        author=Principal(user=User(user_id=os.environ.get("USER", "operator"))),
        description=description,
    )
    print(task.version.task_id)


async def _watch(client: AsyncLattice, task_id: str) -> None:
    last = None
    while True:
        task = await client.tasks.get_task(task_id)
        status = task.status.status
        if status != last:
            err = task.status.task_error
            print(status + (f" {err.code}: {err.message}" if err else ""), flush=True)
            last = status
        if status in ("STATUS_DONE_OK", "STATUS_DONE_NOT_OK"):
            return
        await asyncio.sleep(0.5)


async def _run(args) -> None:
    url, token = os.environ.get("LATTICE_URL"), os.environ.get("LATTICE_TOKEN")
    if not url or not token:
        sys.exit("set LATTICE_URL and LATTICE_TOKEN")
    headers = {"anduril-sandbox-authorization": f"Bearer {os.environ['LATTICE_SANDBOX_TOKEN']}"} if os.environ.get("LATTICE_SANDBOX_TOKEN") else None
    client = AsyncLattice(base_url=url.rstrip("/"), token=token, headers=headers)
    if args.cmd == "marshal":
        await _create(client, args.assignee, "Marshal", {"objective": {"point": _point(args.lat, args.lon, args.name)}}, f"go to {args.name or 'point'}")
    elif args.cmd == "marshal-entity":
        await _create(client, args.assignee, "Marshal", {"objective": {"entityId": args.entity}}, f"go to {args.entity}")
    elif args.cmd == "transit":
        pts = [tuple(float(v) for v in p.split(",")) for p in args.points]
        path = [{"waypoint": {"llaPoint": _point(lat, lon)}} for lat, lon in pts]
        await _create(client, args.assignee, "Transit", {"plan": {"route": {"path": path}}}, f"route of {len(pts)} waypoints")
    elif args.cmd == "cancel":
        await client.tasks.cancel_task(args.task_id, author=Principal(user=User(user_id=os.environ.get("USER", "operator"))))
    elif args.cmd == "complete":
        async with httpx.AsyncClient() as http:
            r = await http.post(f"{url.rstrip('/')}/_fake/tasks/{args.task_id}/complete")
            r.raise_for_status()
    elif args.cmd == "watch":
        await _watch(client, args.task_id)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="lattice-task", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("marshal", help="drive to a point")
    p.add_argument("assignee")
    p.add_argument("lat", type=float)
    p.add_argument("lon", type=float)
    p.add_argument("--name", default="")
    p = sub.add_parser("marshal-entity", help="drive to an entity")
    p.add_argument("assignee")
    p.add_argument("entity")
    p = sub.add_parser("transit", help="drive a route")
    p.add_argument("assignee")
    p.add_argument("points", nargs="+", metavar="LAT,LON")
    for name in ("cancel", "complete", "watch"):
        sub.add_parser(name).add_argument("task_id")
    asyncio.run(_run(parser.parse_args(argv)))


if __name__ == "__main__":
    main()
