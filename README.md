## Lattice client for MAVLink
This application integrates with your UAV over MAVLink using the [Lattice SDK](https://www.anduril.com/lattice-sdk/). It is designed to work within **Lattice Sandboxes** for simulation or testing purposes.

To obtain access tokens and credentials, please reach out to **Anduril**.

### Behavior
Once a MAVSDK connection is established, the application initializes a Lattice Client instance and runs asynchronously, continuously publishing UAV telemetry data to the Lattice system.

### Instructions
Install the SDK for REST in Python
```
pip install anduril-lattice-sdk
```

You will need the following:
- **SANDBOXES_TOKEN** – Bearer token for sandbox authentication
- **LATTICE_ENDPOINT** – Endpoint URL of your Lattice environment
- **ENVIRONMENT_TOKEN** – Token scoped to your Lattice environment


Place them in the setup.sh
Run
```
source setup.sh
```
You should have a drone running.
Then you should be able to run the script using
```
python3 lattice_publisher.py 
```
