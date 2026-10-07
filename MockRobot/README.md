# MockRobot

Standalone Compose Desktop virtual FTC Robot Controller for AzConductor development.

## Run

From the FTC repository root:

    gradlew.bat :MockRobot:run

The simulator listens on port 8888. Point AzConductor at:

    127.0.0.1

For network-only testing without the GUI:

    gradlew.bat :MockRobot:run --args="--headless"

## What it simulates

- Network V2 health/session ownership.
- Real HTTP and Server-Sent Events on port 8888.
- Autonomous OpMode list and FTC-style INIT -> START -> STOP lifecycle.
- External lifecycle changes from the MockRobot GUI, equivalent to another control surface.
- 60 Hz pose publication, configurable down to 1 Hz.
- Demo routes with cubic Hermite interpolation.
- Automatic route following.
- Continuous correlated position/heading noise.
- Manual robot dragging and exact pose entry.
- Temporary disturbance injection and gradual tracking correction.
- Execution state, route revisions, ETag/If-Match behavior, command revision events, and heartbeat.
- Configurable artificial HTTP latency and explicit session disconnect.

## Shared robot network stack

The MockRobot module compiles the same TeamCode protocol/server sources used on the real
Robot Controller, including RobotHttpServer, RobotApiRouter, RobotEventStream,
SessionLease, runtime/execution stores, and OpModeLifecycleService.

Only platform-specific pieces are substituted:

- Android SharedPreferences route storage -> desktop MockRouteStore.
- Android Log and SystemClock -> small JVM compatibility shims.
- FTC SDK OpMode backend -> MockOpModeBackend.

The GUI does not contain an AzConductor-only network shortcut.

## Data

Simulator routes and settings live outside the Git checkout under:

    ~/.azconductor/mockrobot/

The GUI can restore the built-in demo routes at any time.
