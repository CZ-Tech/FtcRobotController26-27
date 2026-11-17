package org.firstinspires.ftc.teamcode.opmode.auto;


import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.command.auto.RouteCommandDispatcher;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTracker;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.RobotNetworkService;
import org.firstinspires.ftc.teamcode.common.network.RobotNetworkV2;
import org.firstinspires.ftc.teamcode.common.network.RouteRepository;
import org.firstinspires.ftc.teamcode.common.network.SubsystemCommandBinding;
import org.firstinspires.ftc.teamcode.common.opmode.RobotLinearOpMode;

/**
 * One persisted AzConductor route exposed as one real FTC Autonomous OpMode.
 *
 * <p>The FTC SDK registers one instance of this class per route name. Selecting that
 * OpMode therefore selects exactly one route; pressing START immediately starts that
 * route. There is intentionally no independent "execute path" control plane.</p>
 *
 * <p>Each waypoint's {@code commands} list runs sequentially on one managed task thread.
 * Different waypoints may overlap while the path keeps moving. Legacy single
 * {@code command}/{@code commandParams} is still accepted. Dispatch and worker failures
 * abort the route; successful path completion waits for all started commands.</p>
 */
public final class RouteAutoOpMode extends RobotLinearOpMode {
    private static final String TAG = "RouteAutoOpMode";

    private final String routeName;

    public RouteAutoOpMode(String routeName) {
        if (routeName == null || routeName.trim().isEmpty()) {
            throw new IllegalArgumentException("routeName is empty");
        }
        this.routeName = routeName;
    }

    @Override
    protected void runRobotOpMode() throws InterruptedException {
        Globals.isPoseInitialized = true;

        RobotNetworkV2 network = RobotNetworkService.get();
        SplineTracker tracker = new SplineTracker(robot);
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(tracker);
        // One waypoint list runs sequentially on one managed task; later points may overlap.
        // The catalog is the same one served to remote command clients.
        RouteCommandDispatcher commands = RouteCommandDispatcher.forBatch(
                new SubsystemCommandBinding(robot.subsystem)::submitAll,
                network.commands);
        RouteRepository.Entry route = network.routes.get(routeName);

        if (route == null) {
            telemetry.addData("Route", routeName);
            telemetry.addData("Error", "route no longer exists");
            telemetry.update();
            requestOpModeStop();
            return;
        }

        telemetry.addData("Route", routeName);
        telemetry.addData("Status", "initialized; waiting for start");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            publishInactive(network, robot);
            return;
        }

        SplineTrajectoryLoader.ExecutionResult started = loader.start(route.json);
        if (started.terminal()) {
            // A route that cannot be parsed or is empty is an error, not "nothing to do".
            Log.e(TAG, "Route '" + routeName + "' did not start: " + started);
            telemetry.addData("Error", started.status + ": " + started.message);
            telemetry.update();
            requestOpModeStop();
            publishInactive(network, robot);
            return;
        }
        network.execution.publish(
                ExecutionStateStore.State.RUNNING,
                0,
                routeName);

        try {
            runWhileActive(() -> {
                if (loader.isRunning()) {
                    loader.setFailurePolicy(Globals.Auto.FAILURE_POLICY);
                    loader.update();
                    SplineTrajectoryLoader.TrajectoryEvent event;
                    while ((event = loader.pollEvent()) != null) {
                        RouteCommandDispatcher.Result result = commands.dispatch(event);
                        if (RouteCommandDispatcher.fatal(result)) {
                            // Any dispatch failure terminates the whole route: no skip,
                            // no retry, no silent default.
                            throw new RouteAborted("frame " + event.frameIndex
                                    + " commands " + event.commands + " -> " + result);
                        }
                    }
                    if (!loader.isRunning()) {
                        if (loader.getStatus() != SplineTrajectoryLoader.ExecutionStatus.COMPLETED) {
                            Log.e(TAG, "Route '" + routeName + "' ended: "
                                    + loader.getLastResult());
                            network.execution.publish(ExecutionStateStore.State.NOT_READY, 0, routeName);
                            requestOpModeStop();
                            return;
                        }
                        // The loader has stopped the drivetrain. Keep ticking until all
                        // previously started waypoint tasks finish, including final-point waits.
                        robot.subsystem.finishCommands();
                    }
                }
                if (loader.getStatus() == SplineTrajectoryLoader.ExecutionStatus.COMPLETED
                        && robot.subsystem.commandsIdle()) {
                    robot.subsystem.checkCommandFailure();
                    network.execution.publish(ExecutionStateStore.State.IDLE, 0, routeName);
                    requestOpModeStop();
                    return;
                }

                robot.odo.update();
                Pose2D pose = robot.odo.getPosition();
                network.runtime.publish(
                        true,
                        routeName,
                        pose.getX(DistanceUnit.INCH),
                        pose.getY(DistanceUnit.INCH),
                        pose.getHeading(AngleUnit.DEGREES));

                telemetry.addData("Route", routeName);
                telemetry.addData("Execution", network.execution.snapshot().state);
                telemetry.addData("Frame", loader.getCurrentFrameIndex());
                telemetry.update();
            });
        } catch (RouteAborted aborted) {
            Log.e(TAG, "Route '" + routeName + "' aborted: " + aborted.getMessage());
            telemetry.addData("Aborted", aborted.getMessage());
            telemetry.update();
            requestOpModeStop();
        } finally {
            loader.cancel();
            tracker.stopMotor();
            network.execution.publish(ExecutionStateStore.State.NOT_READY, 0, null);
            publishInactive(network, robot);
        }
    }

    private static void publishInactive(RobotNetworkV2 network, Robot robot) {
        try {
            robot.odo.update();
            Pose2D pose = robot.odo.getPosition();
            network.runtime.publish(
                    false,
                    null,
                    pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
        } catch (Exception ignored) {
            network.runtime.publish(false, null, 0, 0, 0);
        }
    }

    /** Fatal waypoint-command failure; unwinds {@code runWhileActive} immediately. */
    private static final class RouteAborted extends RuntimeException {
        RouteAborted(String message) {
            super(message);
        }
    }
}
