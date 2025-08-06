# telemetry_stream.py
import asyncio
from datetime import datetime, timezone

from mavsdk import System


class ConnectionManager:
    def __init__(self):
        self.is_connected = False
        self.drone = None

telemetry_data = {
    "position": None,
    "velocity": None,
    "altitude": None,
    "odometry": None,
}

async def stream_position(queue, retry_delay=1.0):
    """Stream telemetry data from drone to queue with connection management."""
    conn = ConnectionManager()

    while True:  # Infinite retry loop
        try:
            conn.drone = System()
            print("Attempting to connect to drone...")
            await conn.drone.connect(system_address="udp://:14540")

            # Connection state handler
            async def handle_connection():
                previous_state = None
                async for state in conn.drone.core.connection_state():
                    if state.is_connected != previous_state:
                        conn.is_connected = state.is_connected
                        if conn.is_connected:
                            print("Connection established!")
                        else:
                            print("Connection lost!")
                            break
                        previous_state = state.is_connected

            # Start connection monitoring
            connection_task = asyncio.create_task(handle_connection())

            # Set telemetry rates
            await asyncio.gather(
                conn.drone.telemetry.set_rate_position(10.0),
                conn.drone.telemetry.set_rate_velocity_ned(10.0),
                conn.drone.telemetry.set_rate_altitude(10.0),
                conn.drone.telemetry.set_rate_odometry(10.0),
                conn.drone.telemetry.set_rate_attitude_quaternion(10.0)
            )

            # Start telemetry consumers
            tasks = [
                connection_task,
                asyncio.create_task(consume_position(conn)),
                asyncio.create_task(consume_velocity(conn)),
                asyncio.create_task(consume_altitude(conn)),
                asyncio.create_task(consume_odometry(conn)),
                asyncio.create_task(publish_at_interval(queue, conn)),
            ]

            # Wait until connection is lost
            while conn.is_connected:
                await asyncio.sleep(1)

        except Exception as e:
            print(f"Connection error: {str(e)}")
        finally:
            # Cleanup
            conn.is_connected = False
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            print(f"Reconnecting in {retry_delay} seconds...")
            await asyncio.sleep(retry_delay)

async def consume_position(conn):
    """Consume position telemetry data."""
    async for position in conn.drone.telemetry.position():
        if conn.is_connected and position.latitude_deg:
            telemetry_data["position"] = position


async def consume_velocity(conn):
    """Consume velocity telemetry data."""
    async for velocity in conn.drone.telemetry.position_velocity_ned():
        if conn.is_connected:
            telemetry_data["velocity"] = velocity


async def consume_altitude(conn):
    """Consume altitude telemetry data."""
    async for altitude in conn.drone.telemetry.altitude():
        if conn.is_connected:
            telemetry_data["altitude"] = altitude


async def consume_odometry(conn):
    """Consume odometry telemetry data."""
    async for odometry in conn.drone.telemetry.odometry():
        if conn.is_connected:
            telemetry_data["odometry"] = odometry


async def publish_at_interval(queue, conn, interval_sec=1.0):
    """Publish telemetry data to queue at regular intervals."""
    while True:
        if conn.is_connected and all(telemetry_data.values()):
            await queue.put(
                (
                    telemetry_data["position"],
                    telemetry_data["velocity"],
                    telemetry_data["altitude"],
                    telemetry_data["odometry"],
                )
            )
        await asyncio.sleep(interval_sec)
        