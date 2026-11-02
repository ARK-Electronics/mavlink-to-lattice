# mavlink-lattice

Bridges a PX4 vehicle over MAVLink to [Anduril Lattice](https://developer.anduril.com): publishes it as a taskable asset entity and executes the Lattice tasks assigned to it.

## Tasks

| Lattice task | On the vehicle | Ends |
|---|---|---|
| `anduril.tasks.v2.Marshal`, objective point | `DO_REPOSITION` to the point (PX4 drives there in Hold) | `DONE_OK` within `arrival_radius_m` |
| `anduril.tasks.v2.Marshal`, objective entity | same, re-targeted when the entity moves more than 5 m | `DONE_OK` on arrival; `FAILED` if the entity disappears |
| `anduril.tasks.v2.Transit` | route uploaded as a mission, then Mission mode | `DONE_OK` on the last waypoint, then holds |
| NGC2 Evacuate (`tasks.evacuate`) | nothing: the operator drives (see below) | `DONE_OK` once stopped within `arrival_radius_m` of the last position |

`tasks.enabled` picks which are advertised. Only ground and surface vehicles advertise tasks. The catalog is empty while the vehicle link is down or there's no position.

Status: `MACHINE_RECEIPT`, then `WILCO` or `DONE_NOT_OK`/`REJECTED`, `EXECUTING` once PX4 accepted the commands, then `DONE_OK` or `DONE_NOT_OK`. A running Marshal or Transit fails with `FAILED` if the vehicle disarms, loses its link or leaves the task's mode, as in an RC or GCS takeover or a failsafe. Any task times out with `TIMEOUT` at `completeBefore`. A new task supersedes the running one (`CANCELLED`).

Stopping:
- Every task end stops the vehicle where it is, except a takeover or disarm, where someone else has it, and an Evacuate task, where the operator has it.
- The stop is `DO_REPOSITION` with NaN lat/lon/alt (PX4's pause), sent twice. Switching to Hold isn't enough: after a reposition PX4 is already in Hold, and re-entering it keeps driving to the old target.
- A stop that gets no acknowledgement (link down) is retried until it does, unless the vehicle is disarmed or someone switches it out of Hold/Mission first.

Every task received is tracked until Lattice holds a terminal status for it, and unsent statuses are retried. A cancel sent while the agent stream was down is found by re-reading the running task on reconnect. With `state_dir` set, a restarted bridge fails the tasks the previous run left open, and stops the vehicle if one of them drove it. Without it, a redelivered task already past `MACHINE_RECEIPT` is failed rather than rerun.

## Evacuate

The bridge stages the task's pickup and dropoff for the ground station operator, who plans the route and drives. For this task it never arms, changes mode, repositions or stops the vehicle, whether the task ends, is cancelled, times out or is superseded, or the bridge restarts. Mode changes and disarming don't fail it, so the operator can stop and disarm to load a casualty.

- The bridge uploads the next position to PX4 as a one-waypoint mission. The pickup goes first, so the staged mission run as-is stops there. QGC doesn't reload a mission another component uploaded, so the operator loads it with Plan view's Download.
- Uploading waits while the vehicle is in Mission mode, armed or not. PX4 starts a mission uploaded in Mission mode, or on its next arming.
- After the pickup, uploading also waits while the vehicle's mission is one the operator planned and hasn't finished, such as a route through the pickup to the dropoff. Clearing that mission lets the bridge upload the dropoff. PX4 versions that don't report the mission's state count only a mission the size of the bridge's as its own.
- The bridge moves on from a position once the vehicle has stood within `arrival_radius_m` of it for 3 s, stopped or disarmed. How it got there doesn't matter.
- Each step is announced in a STATUSTEXT. QGC only sees them if mavlink-router or PX4's forwarding passes the bridge's messages to the GCS link. PX4 forwards when `MAV_<n>_FORWARD` is on for both links.
- NGC2's task definitions aren't public, so the type URL and the field paths to the two positions are set in `[tasks.evacuate]`. A task may carry either position or both. One with neither is rejected and logged in full, to show where its positions are.

## Run

```sh
pip install .
mavlink-lattice -c config.toml        # see config.example.toml
```

PX4 must be reachable over UDP (a mavlink-router endpoint, or SITL's port 14540 + instance). Python 3.10+.

## Develop without Lattice

`fake-lattice` is a local Lattice REST server that validates every request against Anduril's OpenAPI spec and records lifecycle violations at `/_fake/state`. The spec isn't in this repo: it's downloaded from developer.anduril.com on first use into `~/.cache/mavlink-lattice/`, or read from the file `LATTICE_OPENAPI` names. `lattice-task` sends tasks to the fake, or to a real environment.

```sh
fake-lattice --port 8080 --token dev &
LATTICE_URL=http://127.0.0.1:8080 LATTICE_TOKEN=dev mavlink-lattice -c config.toml &
export LATTICE_URL=http://127.0.0.1:8080 LATTICE_TOKEN=dev
lattice-task marshal <entity-id> 47.3980 8.5457 --name pickup
lattice-task watch <task-id>
```

## Test

```sh
pip install -e '.[test]'
pytest                                          # fake Lattice + simulated rover, ~2.5 min
PX4_DIR=~/PX4-Autopilot pytest -m sitl          # real PX4 SIH rover; needs px4_sitl_default built
```

On a machine with ROS sourced, run pytest with `env -u PYTHONPATH`: ROS's pytest plugins break collection.

## Add a task type

Subclass `tasks.base.Executor` (parse the spec in `__init__`, command the vehicle in `start`, report arrival from `step`, set `expected_mode`) and add it to `EXECUTORS` in `tasks/manager.py`. It's advertised in the catalog automatically. A custom type needs its proto in the Lattice Schema Registry before an environment will accept it.
