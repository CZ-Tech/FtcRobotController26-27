package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTracker;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.RobotNetworkService;
import org.firstinspires.ftc.teamcode.common.network.RobotNetworkV2;
import org.firstinspires.ftc.teamcode.common.network.RouteRepository;

/**
 * One persisted AzConductor route exposed as one real FTC Autonomous OpMode.
 *
 * <p>The FTC SDK registers one instance of this class per route name. Selecting that
 * OpMode therefore selects exactly one route; pressing START immediately starts that
 * route. There is intentionally no independent "execute path" control plane.</p>
 */
public final class RouteAutoOpMode extends LinearOpMode {
    private final String routeName;

    public RouteAutoOpMode(String routeName) {
        if (routeName == null || routeName.trim().isEmpty()) {
            throw new IllegalArgumentException("routeName is empty");
        }
        this.routeName = routeName;
    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(this);
        MixedOdo.isPoseInitialized = true;

        RobotNetworkV2 network = RobotNetworkService.get();
        SplineTracker tracker = new SplineTracker(robot);
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(tracker);
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
        network.execution.publish(
                started.running()
                        ? ExecutionStateStore.State.RUNNING
                        : ExecutionStateStore.State.IDLE,
                0,
                routeName);

        try {
            while (opModeIsActive()) {
                if (loader.isRunning()) {
                    loader.update();
                    while (loader.pollEvent() != null) {
                        // Command/marker dispatch is intentionally handled separately.
                    }
                    if (!loader.isRunning()) {
                        network.execution.publish(
                                ExecutionStateStore.State.IDLE,
                                0,
                                routeName);
                        requestOpModeStop();
                        break;
                    }
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
                idle();
            }
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
}
