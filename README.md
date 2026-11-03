# mavlink-lattice

mavlink-lattice connects a PX4 ground or surface vehicle to [Anduril Lattice](https://developer.anduril.com). It runs on the vehicle's companion computer and talks MAVLink to PX4. It publishes the vehicle to Lattice as an asset and runs the Lattice tasks assigned to it.

Marshal and Transit tasks drive the vehicle from Lattice. NGC2's Evacuate task leaves the driving to a ground station operator. For Evacuate, the bridge gets the task's positions to the operator's QGC and reports progress back to Lattice.

## How it works

```
            Lattice REST API                    MAVLink over UDP
Lattice  <-------------------->  mavlink-lattice  <-------->  mavlink-router  <---->  PX4
          entity, task requests,                                    |
          task status                                               +------------->  QGC
```

The bridge does four things:

- It reads PX4's telemetry and keeps the vehicle's position, velocity, attitude, battery, health, armed state and mode.
- It publishes that state to Lattice once a second as an asset entity. The entity's task catalog lists the tasks the vehicle takes. Lattice drops the entity 15 s after the bridge stops publishing.
- It keeps Lattice's agent stream open. The stream delivers requests to execute, cancel or complete the tasks assigned to the entity.
- It runs one task at a time and reports its status to Lattice.

A task's status goes from `MACHINE_RECEIPT` to `WILCO` once the bridge accepts it. It goes to `EXECUTING` once PX4 has accepted the commands, and ends as `DONE_OK` or `DONE_NOT_OK`. A task the bridge can't run ends as `DONE_NOT_OK` with `REJECTED`. Any task times out at its `completeBefore`. A new task supersedes the running one, which ends as `CANCELLED`.

## Tasks

| Lattice task | What the bridge does | Done when |
|---|---|---|
| `anduril.tasks.v2.Marshal` to a point | Sends PX4 a `DO_REPOSITION` to the point. PX4 drives there in Hold. | The vehicle is within `vehicle.arrival_radius_m` |
| `anduril.tasks.v2.Marshal` to an entity | The same, re-sent whenever the entity moves more than 5 m | The vehicle arrives. The task fails if the entity disappears. |
| `anduril.tasks.v2.Transit` | Uploads the route as a mission and switches to Mission mode | The vehicle reaches the last waypoint and holds there |
| NGC2 Evacuate | Uploads the pickup, then the dropoff, as a mission for the operator to load | The vehicle has stopped within `tasks.evacuate.arrival_radius_m` of the last position |

`tasks.enabled` picks the tasks to advertise. Only ground and surface vehicles advertise any. The catalog is empty while the bridge has no link to PX4 or no position.

## Design choices

**PX4 does the driving.** The bridge gives PX4 a destination or a mission. It never streams setpoints or uses Offboard mode. PX4's navigation and failsafes stay in charge. If the bridge or Lattice goes away, the vehicle is left in a mode PX4 handles on its own.

**A driving task leaves the vehicle stopped.** Every end of a Marshal or Transit task stops the vehicle where it is. A takeover or a disarm is the exception, because someone else has the vehicle then. A takeover is a switch out of the task's mode by RC, the GCS or a failsafe. A takeover, a disarm or a lost link to PX4 fails the task. The stop is a `DO_REPOSITION` with NaN latitude, longitude and altitude, which is PX4's pause. Switching to Hold isn't enough. After a reposition PX4 is already in Hold, and re-entering Hold keeps it driving to the old target. An unacknowledged stop is retried until it gets through. The retries end early if the vehicle disarms or someone switches it out of Hold and Mission.

**No task is left open in Lattice.** The bridge tracks every task it receives until Lattice holds a terminal status for it. Statuses that didn't get through are retried. The agent stream doesn't guarantee delivery, so the bridge re-reads the running task on reconnect to catch a missed cancel. With `state_dir` set, a restarted bridge fails the tasks the previous run left open. It stops the vehicle only if one of them was driving it. Without `state_dir`, a redelivered task already past `MACHINE_RECEIPT` is failed rather than rerun.

**Lattice's REST API, not gRPC.** Anduril publishes an OpenAPI spec for the REST API. The tests run against a local fake server that checks every request against that spec. REST also delivers Schema Registry task types, such as NGC2's, as JSON with their `@type`.

**Standard MAVLink only.** The bridge uses MAVLink's common messages. Evacuate's positions reach QGC through the mission protocol, which PX4 and QGC already speak. A custom message needs its dialect built into the bridge and the ground station. It also needs it in PX4 if it passes through the autopilot. Each upgrade of any of them means merging the dialect again. A custom message is a last resort, for data no standard message can carry. A QGC fork that should load the staged mission without the Download step can reload it when `mission_id` in `MISSION_CURRENT` changes.

**The operator drives Evacuate.** The bridge never arms, changes mode, repositions or stops the vehicle for an Evacuate task. That holds however the task ends, and through a supersede, a bridge restart or a shutdown. Mode changes and disarming don't fail it, so the operator can stop and disarm at the pickup. With only `evacuate` enabled, nobody in Lattice can drive the vehicle.

**Unknown task definitions are settings.** NGC2's task definitions aren't public. The Evacuate type URL and the field paths to its pickup and dropoff are set in `[tasks.evacuate]`. A task that doesn't match is rejected and logged in full. The log shows where the task's positions are.

## Evacuate

1. Lattice delivers the task. The bridge sends QGC a STATUSTEXT with its positions.
2. The bridge uploads the pickup to PX4 as a one-waypoint mission. It waits while PX4 is in Mission mode, armed or not. PX4 starts a mission uploaded in Mission mode as soon as the vehicle is armed.
3. The operator loads the mission in QGC with Plan view's Download. QGC doesn't reload a mission that another component uploaded. The operator plans the route from it and starts it, or drives by hand or FPV.
4. The vehicle is at the pickup once it has stood still or disarmed within the arrival radius for 3 s. The bridge then uploads the dropoff. It waits while the operator's own mission is unfinished, so a route through the pickup to the dropoff survives. Clearing that mission lets the dropoff through.
5. Arrival at the dropoff completes the task in Lattice.

A task may carry the pickup, the dropoff or both. An edit to the running task keeps the positions already reached and a mission already staged. PX4 versions that don't report a mission's state count only a mission the size of the bridge's as its own.

PX4 logs a STATUSTEXT from the companion computer but doesn't pass it to the GCS link. QGC sees the bridge's messages only if mavlink-router routes them to QGC, or if PX4 forwards them with `MAV_<n>_FORWARD` on for both links.

## Install and run

The bridge needs Python 3.10 or later.

```sh
pip install .
cp config.example.toml config.toml    # then edit it
mavlink-lattice -c config.toml
```

`config.example.toml` documents every setting. These are the ones to set:

| Setting | Purpose |
|---|---|
| `lattice.url` | The Lattice environment |
| `lattice.token`, or `lattice.client_id` and `lattice.client_secret` | Credentials. `LATTICE_URL`, `LATTICE_TOKEN`, `LATTICE_CLIENT_ID` and `LATTICE_CLIENT_SECRET` in the environment override the file. A Lattice Sandboxes environment also needs `LATTICE_SANDBOX_TOKEN`. |
| `vehicle.mavlink` | The bridge's MAVLink endpoint. The default `udpin:127.0.0.1:14540` listens for a mavlink-router endpoint on the same computer. PX4 SITL sends to port 14540 plus its instance number. |
| `tasks.enabled` | Any of `marshal`, `transit` and `evacuate` |
| `tasks.evacuate.type_url`, `pickup`, `dropoff` | The Evacuate type URL and the field paths to its positions |
| `state_dir` | Where open tasks are saved across restarts |

The entity ID comes from the autopilot's hardware UID unless `entity.id` sets one. The bridge takes PX4's MAVLink system ID. Marshal and Transit sent to a disarmed vehicle are rejected unless `vehicle.auto_arm` is on.

## Send tasks by hand

`lattice-task` creates and follows tasks in a Lattice environment or the fake server. It reads `LATTICE_URL`, `LATTICE_TOKEN` and `LATTICE_SANDBOX_TOKEN` from the environment.

```sh
lattice-task marshal <entity-id> 47.3980 8.5457 --name pickup
lattice-task marshal-entity <entity-id> <objective-entity-id>
lattice-task transit <entity-id> 47.3980,8.5457 47.3982,8.5460
lattice-task evacuate <entity-id> <type-url> --pickup 47.3980 8.5457 --dropoff 47.3982 8.5460
lattice-task watch <task-id>
lattice-task cancel <task-id>
```

`evacuate` sends a stand-in task with `pickup` and `dropoff` fields. Those match the default field paths. `lattice-task complete <task-id>` works only against the fake server. In real Lattice an operator completes a task in the UI.

## Develop without Lattice

`fake-lattice` is a local Lattice REST server. It validates every request against Anduril's OpenAPI spec and records lifecycle violations at `/_fake/state`. Anduril's SDK licence doesn't allow redistributing the spec, so it isn't in this repo. It's downloaded from developer.anduril.com on first use into `~/.cache/mavlink-lattice/`, or read from the file `LATTICE_OPENAPI` names.

```sh
fake-lattice --port 8080 --token dev &
LATTICE_URL=http://127.0.0.1:8080 LATTICE_TOKEN=dev mavlink-lattice -c config.toml &
export LATTICE_URL=http://127.0.0.1:8080 LATTICE_TOKEN=dev
lattice-task marshal <entity-id> 47.3980 8.5457
```

## Test

```sh
pip install -e '.[test]'
pytest                                          # fake Lattice and a simulated rover, about 2.5 min
PX4_DIR=~/PX4-Autopilot pytest -m sitl          # PX4's SIH rover, needs px4_sitl_default built
```

ROS's pytest plugins break test collection. On a machine with ROS sourced, run pytest with `env -u PYTHONPATH`.

## Add a task type

Subclass `tasks.base.Executor`. Parse the task in `__init__`, command the vehicle in `start`, report arrival from `step` and set `expected_mode`. A task the operator drives sets `drives = False` and needs no `expected_mode`. Add the class to `TASKS` in `tasks/manager.py` and its name to `TASK_NAMES` in `config.py`. A custom task type needs its proto in Lattice's Schema Registry before an environment accepts it.

## Status

The bridge has been tested against the fake server, a simulated rover and PX4's SIH rover. It hasn't run against a real Lattice environment, NGC2's Evacuate type or a real vehicle. Scheduled tasks with `startAfter`, air vehicles and packaging as a service aren't done.
