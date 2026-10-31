"""MAVLink 2 over UDP on asyncio, using pymavlink only for encoding and parsing."""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable

from pymavlink.dialects.v20 import common as mavlink

log = logging.getLogger(__name__)


def parse_address(address: str) -> tuple[str, str, int]:
    mode, host, port = address.split(":")
    if mode not in ("udpin", "udpout"):
        raise ValueError(f"unsupported MAVLink address {address!r}")
    return mode, host, int(port)


class _Protocol(asyncio.DatagramProtocol):
    def __init__(self, link: "MavlinkLink"):
        self._link = link
        self.closed = asyncio.get_running_loop().create_future()

    def connection_lost(self, exc) -> None:
        if not self.closed.done():
            self.closed.set_result(None)

    def datagram_received(self, data: bytes, addr) -> None:
        self._link._received(data, addr)

    def error_received(self, exc: Exception) -> None:
        log.debug("MAVLink socket error: %s", exc)


class MavlinkLink:
    def __init__(self, address: str, source_system: int, source_component: int):
        self.mode, self.host, self.port = parse_address(address)
        self._mav = mavlink.MAVLink(None, srcSystem=source_system, srcComponent=source_component)
        self._mav.robust_parsing = True
        self._transport: asyncio.DatagramTransport | None = None
        self._protocol: _Protocol | None = None
        self._peer: tuple[str, int] | None = (self.host, self.port) if self.mode == "udpout" else None
        self._handlers: list[Callable[[object], None]] = []

    @property
    def mav(self) -> mavlink.MAVLink:
        return self._mav

    def add_handler(self, handler: Callable[[object], None]) -> None:
        self._handlers.append(handler)

    async def open(self) -> None:
        loop = asyncio.get_running_loop()
        local = (self.host, self.port) if self.mode == "udpin" else ("0.0.0.0", 0)
        self._transport, self._protocol = await loop.create_datagram_endpoint(lambda: _Protocol(self), local_addr=local)

    async def close(self) -> None:
        if self._transport:
            self._transport.close()
            self._transport = None
            # The socket closes on a later loop iteration; wait so the port can be bound again right away.
            await self._protocol.closed

    def send(self, msg) -> None:
        if not self._transport or not self._peer:
            return
        self._transport.sendto(msg.pack(self._mav), self._peer)

    def _received(self, data: bytes, addr) -> None:
        if self.mode == "udpin":
            self._peer = addr
        try:
            msgs = self._mav.parse_buffer(data) or []
        except mavlink.MAVError as exc:
            log.debug("dropping bad MAVLink packet: %s", exc)
            return
        for msg in msgs:
            if msg.get_type() == "BAD_DATA":
                continue
            for handler in self._handlers:
                handler(msg)
