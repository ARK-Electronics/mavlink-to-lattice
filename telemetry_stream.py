# telemetry_stream.py
import asyncio
from mavsdk import System

# Shared state to hold the latest telemetry values
telemetry_data = {
    "position": None,
    "velocity": None,
    "altitude": None,
    "odometry": None
}

async def stream_position(queue):
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Connecting to vehicle...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to the drone!")
            break

    print("🔄 Consuming all telemetry at full rate, publishing at 1Hz...")

    # Set update rates high so we get fast data
    await drone.telemetry.set_rate_position(10.0)
    await drone.telemetry.set_rate_position_velocity_ned(10.0)
    await drone.telemetry.set_rate_altitude(10.0)
    await drone.telemetry.set_rate_odometry(10.0)

    # Start tasks to continuously update telemetry state
    await asyncio.gather(
        consume_position(drone),
        consume_velocity(drone),
        consume_altitude(drone),
        consume_odometry(drone),
        publish_at_interval(queue)
    )

async def consume_position(drone):
    async for position in drone.telemetry.position():
        if position.latitude_deg:  # basic validity check
            telemetry_data["position"] = position

async def consume_velocity(drone):
    async for velocity in drone.telemetry.position_velocity_ned():
        telemetry_data["velocity"] = velocity

async def consume_altitude(drone):
    async for altitude in drone.telemetry.altitude():
        telemetry_data["altitude"] = altitude

async def consume_odometry(drone):
    async for odometry in drone.telemetry.odometry():
        telemetry_data["odometry"] = odometry

async def publish_at_interval(queue, interval_sec=1.0):
    while True:
        if all(telemetry_data.values()):
            await queue.put((
                telemetry_data["position"],
                telemetry_data["velocity"],
                telemetry_data["altitude"],
                telemetry_data["odometry"]
            ))
        await asyncio.sleep(interval_sec)

