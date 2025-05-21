import asyncio
from datetime import datetime, timezone, timedelta
from uuid import uuid4
from grpclib.client import Channel
from anduril.entitymanager.v1 import (
    EntityManagerApiStub, PublishEntityRequest, Aliases, Entity,
    MilView, Location, Position, Ontology, Template, Provenance, 
)
from anduril.ontology.v1 import Disposition
from anduril.tasks.v2 import TaskCatalog, TaskDefinition
from anduril.type import Quaternion, Enu
import os
import sys
from scipy.spatial.transform import Rotation as R
import numpy as np
import logging

lattice_url = os.getenv('LATTICE_URL')
sandboxes_token = os.getenv('SANDBOXES_TOKEN')
environment_token = os.getenv('ENVIRONMENT_TOKEN')

# Setup logging configuration once at the top level
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

if not lattice_url or not sandboxes_token or not environment_token:
    print("Missing environment variables.")
    sys.exit(1)

metadata = {
    'authorization': f"Bearer {environment_token}",
    'anduril-sandbox-authorization': f"Bearer {sandboxes_token}"
}

async def publish_entity(entity):
    channel = Channel(host=lattice_url, port=443, ssl=True)
    stub = EntityManagerApiStub(channel)
    await stub.publish_entity(PublishEntityRequest(entity=entity), metadata=metadata)
    channel.close()

async def start_publisher(queue):
    entity_id = str(uuid4())
    creation_time = datetime.now(timezone.utc)

    while True:
        position, velocity, distance, quaternion = await queue.get()
        now = datetime.now(timezone.utc)

        # Full conversion from NED to ENU
        ned_q = [quaternion.q.x, quaternion.q.y, quaternion.q.z, quaternion.q.w]
        r_ned = R.from_quat(ned_q)

        # Transform: 90° around Z (to swap N/E) and 180° around X (to flip Down to Up)
        r_ned_to_enu = R.from_euler('xyz', [np.pi, 0, np.pi/2])
        r_enu = r_ned_to_enu * r_ned

        x, y, z, w = r_enu.as_quat()


        entity = Entity(
            entity_id=entity_id,
            created_time=creation_time,
            aliases=Aliases(name="ARK Simulated UAV"),
            expiry_time=now + timedelta(minutes=10),
            mil_view=MilView(disposition=Disposition.FRIENDLY),
            location=Location(
                # Figure out the altitude properly
                position=Position(
                    latitude_degrees=position.latitude_deg,
                    longitude_degrees=position.longitude_deg,
                    altitude_hae_meters=position.absolute_altitude_m,
                    altitude_agl_meters=distance.altitude_terrain_m,
                ),
                velocity_enu = Enu(
                    e=velocity.velocity.east_m_s,
                    n=velocity.velocity.north_m_s,
                    u=-velocity.velocity.down_m_s
                ),
                attitude_enu = Quaternion(w=w, x=x, y=y, z=z),
            ),
            ontology=Ontology(template=Template.ASSET, platform_type="UAV"),
            task_catalog=TaskCatalog(
                task_definitions=[
                    TaskDefinition(task_specification_url="type.googleapis.com/anduril.tasks.v2.VisualId")
                ]
            ),
            provenance=Provenance(
                integration_name="mavsdk_integration",
                data_type="telemetry",
                source_update_time=now
            ),
            is_live=True
        )

        # Structured and detailed log output
        logging.info("Publishing Entity:")
        logging.info(f"  Time: {now.isoformat()}")
        logging.info(f"  Position (lat, lon, HAE): ({position.latitude_deg:.6f}, {position.longitude_deg:.6f}, {position.absolute_altitude_m:.2f} m)")
        logging.info(f"  AGL: {distance.altitude_terrain_m:.2f} m")
        logging.info(f"  Velocity (NED): north={velocity.velocity.north_m_s:.2f}, east={velocity.velocity.east_m_s:.2f}, down={velocity.velocity.down_m_s:.2f}")
        logging.info(f"  Attitude (ENU Quaternion): x={x:.4f}, y={y:.4f}, z={z:.4f}, w={w:.4f}")
        logging.info(f"  Raw NED Quaternion: w={quaternion.q.w:.4f}, x={quaternion.q.x:.4f}, y={quaternion.q.y:.4f}, z={quaternion.q.z:.4f}")

        await publish_entity(entity)
