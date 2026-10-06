package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
 * Network-controlled autonomous OpMode.
 *
 * <p>All hardware access remains on this OpMode thread. Network workers can only replace
 * the latest data-only {@link ControlRequest}. Trajectory execution is cooperative:
 * exactly one loader/tracker tick is performed per OpMode loop.</p>
 */
@Autonomous(name = "HttpAuto", group = "HTTP")
public class HttpAuto extends LinearOpMode {

    private static final String OPMODE_NAME = "HttpAuto";

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

                        case EXECUTE_INLINE_PATH:
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

                        case RUN_COMMAND:
                            // Intentionally not wired yet. Existing @AutoTask methods still
                            // contain blocking waits/loops and therefore are not safe to invoke
                            // from the cooperative OpMode loop.
                            network.execution.publish(
                                    ExecutionStateStore.State.IDLE,
                                    request.id,
                                    request.commandName);
                            break;
                    }
                }

                if (loader.isRunning()) {
                    loader.update();

                    // Consume trajectory events immediately so they can never accumulate.
                    // Command/marker hardware dispatch will be wired after commands themselves
                    // are converted to non-blocking state machines.
                    while (loader.pollEvent() != null) {
                        // no-op by design for this phase
                    }

                    if (!loader.isRunning()) {
                        network.execution.publish(
                                ExecutionStateStore.State.IDLE,
                                activeRequestId,
                                activeSubject);
                    }
                }

                robot.odo.update();
                Pose2D pose = robot.odo.getPosition();
                network.runtime.publish(
                        true,
                        OPMODE_NAME,
                        pose.getX(DistanceUnit.INCH),
                        pose.getY(DistanceUnit.INCH),
                        pose.getHeading(AngleUnit.DEGREES));

                telemetry.addData("HTTP", "V2 active");
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
