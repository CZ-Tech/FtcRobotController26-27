package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore;
import org.firstinspires.ftc.teamcode.common.network.RobotNetworkService;
import org.firstinspires.ftc.teamcode.common.network.RobotNetworkV2;
import org.firstinspires.ftc.teamcode.common.network.SubsystemCommandBinding;
import org.firstinspires.ftc.teamcode.common.opmode.RobotLinearOpMode;

/**
 * Gamepad-driven chassis that also accepts subsystem commands over the network.
 *
 * <p>{@link RobotLinearOpMode} already initializes every subsystem. The network may
 * start and stop this OpMode like any other; remote commands are only accepted between
 * START and STOP, and each one is a normal subsystem schedule executed by the
 * scheduler tick in {@code runWhileActive}.</p>
 */
@TeleOp(name = "RemoteCommand Test", group = "Test")
public class RemoteCommandTest extends RobotLinearOpMode {
    private static final String OPMODE_NAME = "RemoteCommand Test";

    @Override
    protected void runRobotOpMode() {
        RobotNetworkV2 network = RobotNetworkService.get();
        SubsystemCommandBinding binding = new SubsystemCommandBinding(robot.subsystem);

        try {
            telemetry.addLine("Remote command test ready");
            telemetry.addLine("Left stick: field translation");
            telemetry.addLine("Right stick X: turn");
            telemetry.addLine("Subsystem commands: accepted over the network while running");
            telemetry.update();

            robot.odo.resetPosAndIMU();

            waitForStart();
            if (!opModeIsActive()) return;

            network.execution.publish(ExecutionStateStore.State.IDLE, 0, null);
            network.commandDispatcher.attach(binding);

            runWhileActive(() -> {
                robot.odo.update();

                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                robot.odoDrivetrain.driveRobotFieldCentric(axial, lateral, yaw);

                Pose2D pose = robot.odo.getPosition();
                network.runtime.publish(
                        true,
                        OPMODE_NAME,
                        pose.getX(DistanceUnit.INCH),
                        pose.getY(DistanceUnit.INCH),
                        pose.getHeading(AngleUnit.DEGREES));

                telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
                telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
                telemetry.addData("Heading", "%.1f°", pose.getHeading(AngleUnit.DEGREES));
                telemetry.update();
            });
        } finally {
            // Detach before RobotLinearOpMode shuts the subsystems down so no command is
            // accepted into a scheduler that is about to be cancelled.
            network.commandDispatcher.detach(binding);
            network.execution.publish(ExecutionStateStore.State.NOT_READY, 0, null);
            if (robot.odoDrivetrain != null) {
                robot.odoDrivetrain.stopMotor();
            }
            publishInactive(network);
        }
    }

    private void publishInactive(RobotNetworkV2 network) {
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
