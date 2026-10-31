"""The agent stream: Lattice pushes execute, cancel and complete requests for our entity over SSE."""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable

import httpx

from .client import ApiError, LatticeApi

log = logging.getLogger(__name__)


class AgentStream:
    def __init__(self, api: LatticeApi, on_request: Callable[[object], None], on_connect: Callable[[], None] = lambda: None):
        self.api = api
        self.on_request = on_request
        # Called on each (re)connection. Requests sent while the stream was down may never be delivered,
        # so the agent re-reads what it's running.
        self.on_connect = on_connect
        self.connected = False
        # Delivery isn't acknowledged, so a stream that stops sending heartbeats is treated as dead.
        self.stall_timeout_s = max(3 * api.heartbeat_interval_ms / 1000, 2.0)

    async def run(self) -> None:
        backoff = 1.0
        while True:
            stream = self.api.stream_as_agent()
            nxt: asyncio.Future | None = None
            try:
                while True:
                    # asyncio.wait rather than wait_for, which can swallow our own cancellation before Python 3.12.
                    nxt = asyncio.ensure_future(stream.__anext__())
                    done, _ = await asyncio.wait({nxt}, timeout=self.stall_timeout_s)
                    if not done:
                        raise asyncio.TimeoutError
                    event = nxt.result()
                    if not self.connected:
                        self.connected = True
                        log.info("agent stream connected")
                        self.on_connect()
                    backoff = 1.0
                    if getattr(event, "event", None) == "agent_request":
                        self.on_request(event)
            except StopAsyncIteration:
                log.warning("agent stream closed by Lattice")
            except asyncio.TimeoutError:
                log.warning("agent stream: no heartbeat for %.0f s", self.stall_timeout_s)
            except (ApiError, httpx.HTTPError) as exc:
                log.warning("agent stream: %s", getattr(exc, "body", None) or exc)
            except Exception:
                log.exception("agent stream: unexpected error")
            finally:
                self.connected = False
                if nxt and not nxt.done():
                    nxt.cancel()  # the generator can't be closed while a read is in flight
                    await asyncio.gather(nxt, return_exceptions=True)
                await stream.aclose()
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30.0)
