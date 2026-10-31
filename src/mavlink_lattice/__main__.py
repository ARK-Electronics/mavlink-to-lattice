from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import signal

from . import config
from .bridge import Bridge


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="mavlink-lattice", description=__doc__)
    parser.add_argument("-c", "--config", help="TOML config file (LATTICE_* environment variables override it)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    if not args.verbose:
        logging.getLogger("httpx").setLevel(logging.WARNING)
    cfg = config.load(args.config)

    async def run() -> None:
        task = asyncio.create_task(Bridge(cfg).run())
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, task.cancel)
        with contextlib.suppress(asyncio.CancelledError):
            await task

    asyncio.run(run())


if __name__ == "__main__":
    main()
