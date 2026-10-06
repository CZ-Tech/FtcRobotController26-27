package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTracker;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.network.ControlRequest;
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.RobotNetworkService;
import org.firstinspires.ftc.teamcode.common.network.RobotNetworkV2;
import org.firstinspires.ftc.teamcode.common.network.RouteStore;

/**
 * Long-running TeleOp variant of the V2 network path executor for field debugging.
 *
 * <p>Like HttpAuto, all hardware access happens on this OpMode thread and path control
 * advances by one tick per loop.</p>
 */
@TeleOp(name = "AzConductor调试", group = "调试")
public class AzConductorDebug extends LinearOpMode {
    private static final String OPMODE_NAME = "AzConductorDebug";

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(this);
        MixedOdo.isPoseInitialized = true;

        RobotNetworkV2 network = RobotNetworkService.get();
        SplineTracker tracker = new SplineTracker(robot);
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(tracker);

        telemetry.addData("Status", "initialized; waiting for start");
        telemetry.addData("Network", "V2 :8888");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            publishInactive(network, robot);
            return;
        }

        network.control.activate();
        network.execution.publish(ExecutionStateStore.State.IDLE, 0, null);

        long activeRequestId = 0;
        String activeSubject = null;

        try {
            while (opModeIsActive()) {
                ControlRequest request = network.control.pollLatest();
                if (request != null) {
                    switch (request.type) {
                        case EXECUTE_SAVED_PATH: {
                            RouteStore.Entry route = network.routes.get(request.pathName);
                            if (route != null) {
                                SplineTrajectoryLoader.ExecutionResult started =
                                        loader.start(route.json);
                                activeRequestId = request.id;
                                activeSubject = request.pathName;
                                network.execution.publish(
                                        started.running()
                                                ? ExecutionStateStore.State.RUNNING
                                                : ExecutionStateStore.State.IDLE,
                                        activeRequestId,
                                        activeSubject);
                            } else {
                                network.execution.publish(
                                        ExecutionStateStore.State.IDLE,
                                        request.id,
                                        request.pathName);
                            }
                            break;
                        }

                        case EXECUTE_INLINE_PATH: {
                            SplineTrajectoryLoader.ExecutionResult started =
                                    loader.start(request.inlineJson);
                            activeRequestId = request.id;
                            activeSubject = "inline";
                            network.execution.publish(
                                    started.running()
                                            ? ExecutionStateStore.State.RUNNING
                                            : ExecutionStateStore.State.IDLE,
                                    activeRequestId,
                                    activeSubject);
                            break;
                        }

                    }
                }

                if (loader.isRunning()) {
                    loader.update();
                    while (loader.pollEvent() != null) {
                        // Command/marker dispatch is intentionally deferred.
                    }
                    if (!loader.isRunning()) {
                        network.execution.publish(
                                ExecutionStateStore.State.IDLE,
                                activeRequestId,
                                activeSubject);
                    }
                }

                publishRuntime(network, robot, true);

                telemetry.addData("Network", "V2 active");
                telemetry.addData("Execution", network.execution.snapshot().state);
                telemetry.addData("Frame", loader.getCurrentFrameIndex());
                telemetry.update();
                idle();
            }
        } finally {
            loader.cancel();
            tracker.stopMotor();
            network.control.deactivate();
            network.execution.publish(ExecutionStateStore.State.NOT_READY, 0, null);
            publishRuntime(network, robot, false);
        }
    }

    private static void publishRuntime(
            RobotNetworkV2 network,
            Robot robot,
            boolean active) {
        try {
            robot.odo.update();
            Pose2D pose = robot.odo.getPosition();
            network.runtime.publish(
                    active,
                    active ? OPMODE_NAME : null,
                    pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
        } catch (Exception ignored) {
            network.runtime.publish(active, active ? OPMODE_NAME : null, 0, 0, 0);
        }
    }

    private static void publishInactive(RobotNetworkV2 network, Robot robot) {
        publishRuntime(network, robot, false);
    }
}
