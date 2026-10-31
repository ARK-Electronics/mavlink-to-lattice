# mavlink-lattice

Bridges a PX4 vehicle over MAVLink to [Anduril Lattice](https://developer.anduril.com): publishes it as a taskable asset entity and executes the Lattice tasks assigned to it.

## Tasks

| Lattice task | On the vehicle | Ends |
|---|---|---|
| `anduril.tasks.v2.Marshal`, objective point | `DO_REPOSITION` to the point (PX4 drives there in Hold) | `DONE_OK` within `arrival_radius_m` |
| `anduril.tasks.v2.Marshal`, objective entity | same, re-targeted when the entity moves more than 5 m | `DONE_OK` on arrival; `FAILED` if the entity disappears |
| `anduril.tasks.v2.Transit` | route uploaded as a mission, then Mission mode | `DONE_OK` on the last waypoint, then holds |

Only ground and surface vehicles advertise tasks. The catalog is empty while the vehicle link is down or there's no position.

Status: `MACHINE_RECEIPT`, then `WILCO` or `DONE_NOT_OK`/`REJECTED`, `EXECUTING` once PX4 accepted the commands, then `DONE_OK` or `DONE_NOT_OK`. A running task fails (`FAILED`) if the vehicle leaves the task's mode (RC or GCS takeover, failsafe), disarms or loses its link, and times out (`TIMEOUT`) at `completeBefore`. A new task supersedes the running one (`CANCELLED`).

Stopping:
- Every task end stops the vehicle where it is, except a takeover or disarm, where someone else has it.
- The stop is `DO_REPOSITION` with NaN lat/lon/alt (PX4's pause), sent twice. Switching to Hold isn't enough: after a reposition PX4 is already in Hold, and re-entering it keeps driving to the old target.
- A stop that gets no acknowledgement (link down) is retried until it does, unless the vehicle is disarmed or someone switches it out of Hold/Mission first.

Every task received is tracked until Lattice holds a terminal status for it, and unsent statuses are retried. A cancel sent while the agent stream was down is found by re-reading the running task on reconnect. With `state_dir` set, a restarted bridge stops the vehicle and fails the tasks the previous run left open. Without it, a redelivered task already past `MACHINE_RECEIPT` is failed rather than rerun.

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
