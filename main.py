import asyncio
from telemetry_stream import stream_position
from lattice_publisher import start_publisher

async def main():
    queue = asyncio.Queue()
    await asyncio.gather(
        stream_position(queue),
        start_publisher(queue)
    )

if __name__ == "__main__":
    asyncio.run(main())
