#!/usr/bin/env python3
import asyncio
import logging
import math
import os
import sys
from datetime import datetime, timezone, timedelta
from uuid import uuid4

import numpy as np
from anduril import (
    Aliases,
    Enu,
    Health,
    Lattice,
    Location,
    MilView,
    Ontology,
    Position,
    Provenance,
    Quaternion,
    TaskCatalog,
    TaskDefinition,
)
from scipy.spatial.transform import Rotation as R

# Import your telemetry streamer
from telemetry_stream import stream_position

lattice_endpoint = os.getenv('LATTICE_ENDPOINT')
environment_token = os.getenv('ENVIRONMENT_TOKEN')
sandboxes_token = os.getenv('SANDBOXES_TOKEN')

if not all([environment_token, lattice_endpoint, sandboxes_token]):
    print("Missing required environment variables.")
    sys.exit(1)

# === Client Setup ===
client = Lattice(
    base_url=f"https://{lattice_endpoint}",
    token=environment_token,
    headers={"anduril-sandbox-authorization": f"Bearer {sandboxes_token}"}
)

# === Logging Setup ===
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

DRONE_ID = "drone-1"

async def start_publisher(queue, max_telemetry_gap=120):
    """Publish telemetry to Lattice at fixed intervals."""
    creation_time = datetime.now(timezone.utc)

    while True:
        try:
            current_time = datetime.now(timezone.utc)
            expiry_time = current_time + timedelta(minutes=5)  # Always 5 mins in future
            position, velocity, altitude, odometry = await queue.get()

            # Validate all required data is present
            if None in [position, velocity, altitude, odometry]:
                logging.warning("Incomplete telemetry data, skipping publish")
                continue

            # Process and validate values
            latest_timestamp = datetime.now(timezone.utc)
            agl = altitude.altitude_terrain_m
            if agl is None or math.isnan(agl) or math.isinf(agl):
                logging.warning(
                    "AGL altitude missing or invalid, falling back to Local Altitude"
                )
                agl_altitude = altitude.altitude_local_m
            else:
                agl_altitude = agl

            # Convert NED quaternion to ENU
            ned_q = [
                odometry.q.x,
                odometry.q.y,
                odometry.q.z,
                odometry.q.w,
            ]
            r_ned = R.from_quat(ned_q)
            r_ned_to_enu = R.from_euler('xyz', [np.pi, 0, np.pi / 2])
            r_enu = r_ned_to_enu * r_ned
            x, y, z, w = r_enu.as_quat()

            # Log processed data
            logging.info("Publishing Entity:")
            logging.info(f"  Position: ({position.latitude_deg:.6f}, {position.longitude_deg:.6f})")
            logging.info(f"  Altitude: HAE={position.absolute_altitude_m:.2f}m, AGL={agl_altitude:.2f}m")
            logging.info(f"  Velocity: N={velocity.velocity.north_m_s:.2f}, E={velocity.velocity.east_m_s:.2f}")
            logging.info(f"  Quaternion: w={w:.4f}, x={x:.4f}, y={y:.4f}, z={z:.4f}")

            # Publish to Lattice
            try:
                client.entities.publish_entity(
                    entity_id=DRONE_ID,
                    description="Friendly drone asset",
                    aliases=Aliases(name="ARK Drone"),
                    is_live=True,
                    created_time=creation_time,
                    expiry_time=expiry_time + timedelta(minutes=5),
                    ontology=Ontology(
                        template="TEMPLATE_ASSET",
                        platform_type="UAV"
                    ),
                    mil_view=MilView(
                        disposition="DISPOSITION_FRIENDLY",
                        environment="ENVIRONMENT_AIR"
                    ),
                    location=Location(
                        position=Position(
                            latitude_degrees=position.latitude_deg,
                            longitude_degrees=position.longitude_deg,
                            altitude_hae_meters=position.absolute_altitude_m,
                            altitude_agl_meters=agl_altitude,
                        ),
                        velocity_enu=Enu(
                            e=velocity.velocity.east_m_s,
                            n=velocity.velocity.north_m_s,
                            u=-velocity.velocity.down_m_s
                        ),
                        attitude_enu=Quaternion(w=w, x=x, y=y, z=z)
                    ),
                    provenance=Provenance(
                        integration_name="mavsdk_integration",
                        data_type="telemetry",
                        source_update_time=latest_timestamp
                    ),
                    health=Health(
                        connection_status="CONNECTION_STATUS_ONLINE",
                        health_status="HEALTH_STATUS_HEALTHY",
                        update_time=latest_timestamp
                    ),
                    task_catalog=TaskCatalog(
                        task_definitions=[
                            TaskDefinition(task_specification_url="type.googleapis.com/anduril.tasks.v2.VisualId"),
                            TaskDefinition(task_specification_url="type.googleapis.com/anduril.tasks.v2.Monitor"),
                            TaskDefinition(task_specification_url="type.googleapis.com/anduril.tasks.v2.Investigate")
                        ]
                    ),
                )
            except Exception as publish_error:
                logging.error(f"Publish failed: {str(publish_error)}")
                continue

            await asyncio.sleep(1.0)  # Maintain 1Hz publish rate

        except asyncio.CancelledError:
            logging.info("Publisher task cancelled")
            break
        except Exception as error:
            logging.error(f"Unexpected error: {str(error)}", exc_info=True)
            await asyncio.sleep(1)


async def main():
    """Main async entry point."""
    queue = asyncio.Queue(maxsize=10)  # Prevent unbounded queue growth

    try:
        # Start both tasks
        await asyncio.gather(
            stream_position(queue),  # Your telemetry producer
            start_publisher(queue),  # The publisher
        )
    except KeyboardInterrupt:
        logging.info("Shutting down...")
    except Exception as error:
        logging.error(f"Fatal error: {str(error)}", exc_info=True)
    finally:
        # Cleanup tasks
        tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

if __name__ == "__main__":
    asyncio.run(main())
    