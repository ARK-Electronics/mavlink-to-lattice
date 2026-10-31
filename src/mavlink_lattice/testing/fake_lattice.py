"""A fake Lattice REST server for development and tests, without an Anduril environment.

It implements the endpoints the bridge and an operator use: entity publish/get, task create/get/cancel,
status updates, the agent SSE stream and OAuth client credentials. Every request body is validated
against Anduril's published OpenAPI spec, with unknown fields rejected and enums enforced, so mistakes
the SDK lets through locally fail here the way they would on a real server.

The spec is Anduril's and isn't redistributed here: it's downloaded from developer.anduril.com on first
use and cached, or read from the file LATTICE_OPENAPI names.

Behaviour the spec documents is reproduced: status updates are ignored unless their status version is
higher than the server's, terminal statuses are final, cancel is routed to the agent once delivered, and
cancel/complete bump the status version. Behaviour the spec leaves open is a choice, listed in
`FakeLattice.__init__`. Anything an agent does that a real server might reject or that breaks the task
lifecycle is recorded in `violations`.

Run standalone: `fake-lattice --port 8080 --token dev` (then point the bridge at http://127.0.0.1:8080).
"""

from __future__ import annotations

import argparse
import asyncio
import copy
import itertools
import json
import logging
import os
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path

import httpx
from aiohttp import web
from jsonschema import Draft202012Validator
from referencing import Registry, Resource
from referencing.jsonschema import DRAFT202012

log = logging.getLogger(__name__)

SPEC_URL = "https://developer.anduril.com/openapi/rest.json"
_URI = "urn:lattice-rest"

TERMINAL = {"STATUS_DONE_OK", "STATUS_DONE_NOT_OK"}
# Statuses only the server sets.
SERVER_ONLY = {"STATUS_INVALID", "STATUS_CREATED", "STATUS_SCHEDULED_IN_MANAGER", "STATUS_SENT", "STATUS_CANCEL_REQUESTED", "STATUS_COMPLETE_REQUESTED", "STATUS_VERSION_REJECTED"}
_RANK = {"STATUS_MACHINE_RECEIPT": 1, "STATUS_ACK": 2, "STATUS_WILCO": 3, "STATUS_EXECUTING": 4, "STATUS_WAITING_FOR_UPDATE": 4, "STATUS_DONE_OK": 5, "STATUS_DONE_NOT_OK": 5}
# Proto wrapper types (FloatValue, Int64Value...) appear in the spec as strings titled with the proto type,
# but protojson accepts JSON numbers for them and the SDK sends numbers.
_NUMERIC_TITLES = {"float", "double", "int32", "uint32", "int64", "uint64", "sint32", "sint64", "fixed32", "fixed64"}


def load_spec() -> dict:
    if os.environ.get("LATTICE_OPENAPI"):
        return json.loads(Path(os.environ["LATTICE_OPENAPI"]).read_text())
    cache = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache") / "mavlink-lattice" / "lattice_rest_openapi.json"
    if not cache.exists():
        log.info("downloading the Lattice OpenAPI spec from %s", SPEC_URL)
        resp = httpx.get(SPEC_URL, timeout=30, follow_redirects=True)
        resp.raise_for_status()
        spec = resp.json()  # parse before caching, so a bad download isn't kept
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_text(json.dumps(spec))
    return json.loads(cache.read_text())


def _now() -> datetime:
    return datetime.now(timezone.utc)


def _ts(dt: datetime) -> str:
    return dt.isoformat().replace("+00:00", "Z")


def _parse_ts(s: str) -> datetime:
    return datetime.fromisoformat(s.replace("Z", "+00:00"))


class SpecValidator:
    def __init__(self, spec: dict):
        spec = copy.deepcopy(spec)
        schemas = spec["components"]["schemas"]
        schemas["GoogleProtobufAny"]["additionalProperties"] = True

        def walk(node):
            if isinstance(node, dict):
                if node.get("type") == "string" and node.get("title") in _NUMERIC_TITLES:
                    node["type"] = ["string", "number"]
                props = node.get("properties")
                if isinstance(props, dict) and not any(k in node for k in ("additionalProperties", "oneOf", "anyOf", "allOf")):
                    node["additionalProperties"] = False
                for v in node.values():
                    walk(v)
            elif isinstance(node, list):
                for v in node:
                    walk(v)

        walk(schemas)
        self.registry = Registry().with_resource(_URI, Resource.from_contents(spec, default_specification=DRAFT202012))
        self._validators: dict[str, Draft202012Validator] = {}

    def errors(self, schema: str, instance) -> list[str]:
        v = self._validators.get(schema)
        if v is None:
            v = self._validators[schema] = Draft202012Validator({"$ref": f"{_URI}#/components/schemas/{schema}"}, registry=self.registry)
        return [f"{'/'.join(map(str, e.absolute_path)) or '<root>'}: {e.message}" for e in v.iter_errors(instance)]


@dataclass
class FakeTask:
    task: dict
    assignee: str | None
    delivered: bool = False
    history: list[dict] = field(default_factory=list)

    @property
    def status(self) -> str:
        return self.task["status"]["status"]

    @property
    def version(self) -> int:
        return self.task["version"]["statusVersion"]


class _Stream:
    def __init__(self, entity_ids: list[str]):
        self.entity_ids = entity_ids
        self.queue: asyncio.Queue = asyncio.Queue()
        self.stalled = False


def _error(status: int, code: str, message: str) -> web.Response:
    return web.json_response({"code": code, "message": message}, status=status)


class FakeLattice:
    def __init__(
        self,
        *,
        token: str = "fake-token",
        client_id: str | None = None,
        client_secret: str | None = None,
        sandbox_token: str | None = None,
        # On a new agent connection, send execute requests again for tasks already delivered and not
        # terminal. Whether Lattice does this is undocumented; agents must cope either way.
        redeliver_on_connect: bool = True,
        # Refuse tasks whose type isn't in the assignee entity's task catalog.
        require_catalog: bool = True,
    ):
        self.tokens = {token} if token else set()
        self.client_id, self.client_secret = client_id, client_secret
        self.sandbox_token = sandbox_token
        self.redeliver_on_connect = redeliver_on_connect
        self.require_catalog = require_catalog
        self.validator = SpecValidator(load_spec())
        self.entities: dict[str, dict] = {}
        self.entity_publishes: dict[str, int] = {}
        self.tasks: dict[str, FakeTask] = {}
        self.violations: list[str] = []
        self.ignored_updates: list[dict] = []
        self._streams: list[_Stream] = []
        self._faults: list[dict] = []
        self._ids = itertools.count(1)
        self._runner: web.AppRunner | None = None
        self.url = ""

    # --- lifecycle ---

    def app(self) -> web.Application:
        app = web.Application(middlewares=[self._auth])
        r = app.router
        r.add_post("/api/v1/oauth/token", self._oauth)
        r.add_put("/api/v1/entities", self._publish_entity)
        r.add_get("/api/v1/entities/{entityId}", self._get_entity)
        r.add_post("/api/v1/tasks", self._create_task)
        r.add_get("/api/v1/tasks/{taskId}", self._get_task)
        r.add_put("/api/v1/tasks/{taskId}/status", self._update_status)
        r.add_put("/api/v1/tasks/{taskId}/cancel", self._cancel_task)
        r.add_post("/api/v1/agent/stream", self._agent_stream)
        r.add_get("/_fake/state", self._state)
        r.add_post("/_fake/tasks/{taskId}/complete", self._complete_http)
        r.add_post("/_fake/streams/drop", self._drop_http)
        r.add_post("/_fake/streams/stall", self._stall_http)
        return app

    async def start(self, host: str = "127.0.0.1", port: int = 0) -> str:
        self._runner = web.AppRunner(self.app(), shutdown_timeout=0.5)
        await self._runner.setup()
        site = web.TCPSite(self._runner, host, port)
        await site.start()
        port = site._server.sockets[0].getsockname()[1]
        self.url = f"http://{host}:{port}"
        return self.url

    async def stop(self) -> None:
        self.drop_streams()
        if self._runner:
            await self._runner.cleanup()

    # --- test controls ---

    def complete(self, task_id: str) -> None:
        """What an operator marking the task done in the Lattice UI does."""
        t = self.tasks[task_id]
        if t.status in TERMINAL:
            raise ValueError("task is terminal")
        self._server_status(t, "STATUS_COMPLETE_REQUESTED")
        self._push(t.assignee, {"event": "agent_request", "completeRequest": {"taskId": task_id}})

    def drop_streams(self) -> None:
        for s in list(self._streams):
            s.queue.put_nowait(None)

    def stall_streams(self) -> None:
        """Keep current agent connections open but silent: no heartbeats, no requests."""
        for s in self._streams:
            s.stalled = True

    def fail(self, method: str, path_contains: str, status: int = 503, count: int | None = None, seconds: float | None = None, delay_s: float = 0.0, body_contains: str = "") -> None:
        """Answer matching requests with an error, `count` times or for `seconds` (or until clear_faults).
        With `delay_s`, hold each matching request that long and then answer it normally instead."""
        until = asyncio.get_running_loop().time() + seconds if seconds else None
        self._faults.append({"method": method, "path": path_contains, "status": status, "count": count, "until": until, "delay": delay_s, "body": body_contains})

    def clear_faults(self) -> None:
        self._faults.clear()

    def edit(self, task_id: str, specification: dict) -> None:
        """What editing a task in Lattice does: new definition version, redelivered to the agent."""
        t = self.tasks[task_id]
        t.task["specification"] = specification
        t.task["version"]["definitionVersion"] += 1
        self._push(t.assignee, {"event": "agent_request", "executeRequest": {"task": copy.deepcopy(t.task)}})

    def connections(self, entity_id: str) -> int:
        return sum(1 for s in self._streams if entity_id in s.entity_ids and not s.stalled)

    def statuses(self, task_id: str) -> list[str]:
        return [h["status"] for h in self.tasks[task_id].history]

    # --- helpers ---

    def _violation(self, msg: str) -> None:
        log.warning("violation: %s", msg)
        self.violations.append(msg)

    def _invalid(self, schema: str, body) -> web.Response | None:
        errs = self.validator.errors(schema, body)
        if errs:
            self._violation(f"{schema}: " + "; ".join(errs[:5]))
            return _error(400, "invalid_argument", f"{schema}: " + "; ".join(errs[:5]))
        return None

    def _server_status(self, t: FakeTask, status: str, error: dict | None = None) -> None:
        now = _ts(_now())
        new = {"status": status, "lastUpdateTime": now}
        if error:
            new["taskError"] = error
        if status in TERMINAL:
            new["endTime"] = now
        t.task["status"] = new
        t.task["version"]["statusVersion"] += 1
        t.task["lastUpdateTime"] = now
        t.history.append({"status": status, "statusVersion": t.version, "by": "server", "error": error})

    def _push(self, entity_id: str | None, event: dict) -> bool:
        sent = False
        for s in self._streams:
            if entity_id in s.entity_ids and not s.stalled:
                s.queue.put_nowait(event)
                sent = True
        return sent

    def _deliver(self, t: FakeTask) -> None:
        if self._push(t.assignee, {"event": "agent_request", "executeRequest": {"task": copy.deepcopy(t.task)}}) and not t.delivered:
            t.delivered = True
            self._server_status(t, "STATUS_SENT")

    @web.middleware
    async def _auth(self, request: web.Request, handler):
        if request.path.startswith("/_fake") or request.path == "/api/v1/oauth/token":
            return await handler(request)
        now = asyncio.get_running_loop().time()
        for f in list(self._faults):
            if f["until"] is not None and now > f["until"]:
                self._faults.remove(f)
            elif request.method == f["method"] and f["path"] in request.path and (not f["body"] or f["body"] in await request.text()):
                if f["count"] is not None:
                    f["count"] -= 1
                    if f["count"] <= 0:
                        self._faults.remove(f)
                if f["delay"]:
                    await asyncio.sleep(f["delay"])
                    break
                return _error(f["status"], "unavailable", "injected fault")
        auth = request.headers.get("Authorization", "")
        if not auth.startswith("Bearer ") or auth[7:] not in self.tokens:
            return _error(401, "unauthenticated", "missing or unknown bearer token")
        if self.sandbox_token and request.headers.get("anduril-sandbox-authorization") != f"Bearer {self.sandbox_token}":
            return _error(401, "unauthenticated", "missing sandbox token")
        return await handler(request)

    async def _json(self, request: web.Request):
        try:
            return await request.json()
        except (json.JSONDecodeError, ValueError):
            return None

    # --- OAuth ---

    async def _oauth(self, request: web.Request) -> web.Response:
        if request.content_type == "application/json":
            body = await self._json(request) or {}
        else:
            body = dict(await request.post())
        if body.get("grant_type") != "client_credentials":
            return _error(400, "unsupported_grant_type", "grant_type must be client_credentials")
        if not self.client_id or (body.get("client_id"), body.get("client_secret")) != (self.client_id, self.client_secret):
            return _error(401, "invalid_client", "bad client credentials")
        token = f"fake-oauth-{next(self._ids)}"
        self.tokens.add(token)
        return web.json_response({"access_token": token, "token_type": "Bearer", "expires_in": 1800})

    # --- entities ---

    async def _publish_entity(self, request: web.Request) -> web.Response:
        body = await self._json(request)
        if (resp := self._invalid("Entity", body)) is not None:
            return resp
        problems = []
        if not body.get("entityId"):
            problems.append("entityId is required")
        if "isLive" not in body:
            problems.append("isLive is required")
        prov = body.get("provenance") or {}
        for key in ("integrationName", "dataType", "sourceUpdateTime"):
            if not prov.get(key):
                problems.append(f"provenance.{key} is required")
        if not (body.get("aliases") or {}).get("name"):
            problems.append("aliases.name is required")
        if not body.get("noExpiry"):
            if not body.get("expiryTime"):
                problems.append("expiryTime is required unless noExpiry is set")
            else:
                expiry = _parse_ts(body["expiryTime"])
                if expiry <= _now():
                    problems.append("expiryTime is in the past")
                elif expiry > _now() + timedelta(days=30):
                    problems.append("expiryTime is more than 30 days out")
        if (body.get("ontology") or {}).get("template") == "TEMPLATE_ASSET":
            for key in ("location", "milView"):
                if key not in body:
                    problems.append(f"an asset needs {key}")
        for td in (body.get("taskCatalog") or {}).get("taskDefinitions", []):
            if not td.get("taskSpecificationUrl", "").startswith("type.googleapis.com/"):
                problems.append("taskSpecificationUrl must start with type.googleapis.com/")
        if problems:
            self._violation("entity: " + "; ".join(problems))
            return _error(400, "invalid_argument", "; ".join(problems))
        eid = body["entityId"]
        old = self.entities.get(eid)
        if old and _parse_ts(old["provenance"]["sourceUpdateTime"]) >= _parse_ts(prov["sourceUpdateTime"]):
            return web.json_response(old)  # stale: Lattice keeps the newer data
        self.entities[eid] = body
        self.entity_publishes[eid] = self.entity_publishes.get(eid, 0) + 1
        return web.json_response(body)

    async def _get_entity(self, request: web.Request) -> web.Response:
        e = self.entities.get(request.match_info["entityId"])
        if not e:
            return _error(404, "not_found", "entity not found")
        e = copy.deepcopy(e)
        if e.get("expiryTime") and _parse_ts(e["expiryTime"]) <= _now():
            e["isLive"] = False
        return web.json_response(e)

    # --- tasks ---

    async def _create_task(self, request: web.Request) -> web.Response:
        body = await self._json(request)
        if (resp := self._invalid("TaskCreation", body)) is not None:
            return resp
        spec_type = (body.get("specification") or {}).get("@type", "")
        if not spec_type.startswith("type.googleapis.com/"):
            return _error(400, "invalid_argument", "specification @type must start with type.googleapis.com/")
        assignee = (((body.get("relations") or {}).get("assignee") or {}).get("system") or {}).get("entityId")
        if self.require_catalog and assignee:
            entity = self.entities.get(assignee) or {}
            catalog = [d.get("taskSpecificationUrl") for d in (entity.get("taskCatalog") or {}).get("taskDefinitions", [])]
            if spec_type not in catalog:
                return _error(400, "failed_precondition", f"{assignee} doesn't offer {spec_type}")
        task_id = body.get("taskId") or f"task-{next(self._ids)}"
        if task_id in self.tasks:
            return _error(409, "already_exists", "task exists")
        now = _ts(_now())
        task = {
            "version": {"taskId": task_id, "definitionVersion": 1, "statusVersion": 1},
            "specification": body["specification"],
            "status": {"status": "STATUS_CREATED", "lastUpdateTime": now},
            "createTime": now,
            "lastUpdateTime": now,
        }
        for key in ("displayName", "description", "relations", "initialEntities", "executionConstraints"):
            if key in body:
                task[key] = body[key]
        if "author" in body:
            task["createdBy"] = task["lastUpdatedBy"] = body["author"]
        if (resp := self._invalid("Task", task)) is not None:
            return resp
        t = FakeTask(task, assignee, history=[{"status": "STATUS_CREATED", "statusVersion": 1, "by": "server", "error": None}])
        self.tasks[task_id] = t
        self._deliver(t)
        return web.json_response(t.task, status=201)

    async def _get_task(self, request: web.Request) -> web.Response:
        t = self.tasks.get(request.match_info["taskId"])
        if not t:
            return _error(404, "not_found", "task not found")
        return web.json_response(t.task)

    async def _update_status(self, request: web.Request) -> web.Response:
        task_id = request.match_info["taskId"]
        body = await self._json(request)
        if (resp := self._invalid("TaskStatusUpdate", body)) is not None:
            return resp
        t = self.tasks.get(task_id)
        if not t:
            return _error(404, "not_found", "task not found")
        if t.status in TERMINAL:
            self._violation(f"{task_id}: status update after terminal {t.status}")
            return _error(400, "failed_precondition", "task is terminal")
        version = body.get("statusVersion") or 0
        new = body.get("newStatus") or {}
        status = new.get("status")
        if version <= t.version:
            self.ignored_updates.append({"taskId": task_id, "statusVersion": version, "status": status, "serverVersion": t.version})
            return web.json_response(t.task)
        author = ((body.get("author") or {}).get("system") or {}).get("entityId")
        if author != t.assignee:
            self._violation(f"{task_id}: status author {author!r} isn't the assignee {t.assignee!r}")
        if status in SERVER_ONLY or status not in _RANK:
            self._violation(f"{task_id}: agent sent {status}")
        else:
            prev = [h["status"] for h in t.history if h["by"] == "agent"]
            if prev and _RANK[status] < _RANK[prev[-1]]:
                self._violation(f"{task_id}: {status} after {prev[-1]}")
        if status == "STATUS_DONE_NOT_OK" and not (new.get("taskError") or {}).get("code"):
            self._violation(f"{task_id}: DONE_NOT_OK without an error code")
        new = dict(new)
        new["lastUpdateTime"] = _ts(_now())
        t.task["status"] = new
        t.task["version"]["statusVersion"] = version
        t.task["lastUpdateTime"] = new["lastUpdateTime"]
        t.history.append({"status": status, "statusVersion": version, "by": "agent", "error": new.get("taskError")})
        return web.json_response(t.task)

    async def _cancel_task(self, request: web.Request) -> web.Response:
        task_id = request.match_info["taskId"]
        body = await self._json(request) or {}
        if (resp := self._invalid("TaskCancellation", body)) is not None:
            return resp
        t = self.tasks.get(task_id)
        if not t:
            return _error(404, "not_found", "task not found")
        if t.status in TERMINAL:
            return _error(400, "failed_precondition", "task is terminal")
        if not t.delivered:
            self._server_status(t, "STATUS_DONE_NOT_OK", {"code": "ERROR_CODE_CANCELLED", "message": "cancelled before delivery"})
        else:
            self._server_status(t, "STATUS_CANCEL_REQUESTED")
            req = {"taskId": task_id}
            if "author" in body:
                req["author"] = body["author"]
            if t.assignee:
                req["assignee"] = {"system": {"entityId": t.assignee}}
            self._push(t.assignee, {"event": "agent_request", "cancelRequest": req})
        return web.json_response(t.task)

    # --- agent stream ---

    async def _agent_stream(self, request: web.Request) -> web.StreamResponse:
        body = await self._json(request) or {}
        if (resp := self._invalid("AgentStreamRequest", body)) is not None:
            return resp
        ids = (body.get("agentSelector") or {}).get("entityIds") or []
        heartbeat_s = (body.get("heartbeatIntervalMs") or 30000) / 1000
        resp = web.StreamResponse(headers={"Content-Type": "text/event-stream", "Cache-Control": "no-cache"})
        await resp.prepare(request)
        stream = _Stream(ids)
        self._streams.append(stream)
        for t in self.tasks.values():
            if t.assignee in ids and t.status not in TERMINAL and (not t.delivered or self.redeliver_on_connect):
                self._deliver(t)
        try:
            while True:
                try:
                    event = await asyncio.wait_for(stream.queue.get(), heartbeat_s)
                except asyncio.TimeoutError:
                    event = {"event": "heartbeat", "timestamp": _ts(_now())}
                if event is None:
                    break
                if stream.stalled:
                    continue
                await resp.write(f"data: {json.dumps(event)}\n\n".encode())
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            self._streams.remove(stream)
        return resp

    # --- HTTP controls for out-of-process use ---

    async def _state(self, request: web.Request) -> web.Response:
        return web.json_response(
            {
                "entities": self.entities,
                "tasks": {k: {"task": t.task, "history": t.history} for k, t in self.tasks.items()},
                "violations": self.violations,
                "ignoredUpdates": self.ignored_updates,
            }
        )

    async def _complete_http(self, request: web.Request) -> web.Response:
        try:
            self.complete(request.match_info["taskId"])
        except (KeyError, ValueError) as exc:
            return _error(400, "failed_precondition", str(exc))
        return web.json_response(self.tasks[request.match_info["taskId"]].task)

    async def _drop_http(self, request: web.Request) -> web.Response:
        self.drop_streams()
        return web.json_response({})

    async def _stall_http(self, request: web.Request) -> web.Response:
        self.stall_streams()
        return web.json_response({})


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="fake-lattice", description="Fake Lattice REST server for development")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--token", default="dev", help="bearer token clients must send")
    parser.add_argument("--no-redeliver", action="store_true", help="don't resend open tasks when an agent reconnects")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    fake = FakeLattice(token=args.token, redeliver_on_connect=not args.no_redeliver)
    web.run_app(fake.app(), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
