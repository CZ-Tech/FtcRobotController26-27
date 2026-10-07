package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.drive.OdoDrivetrain;

/**
 * Minimal field-centric drivetrain test.
 *
 * <p>Only initializes MixedOdo and OdoDrivetrain. Vision, subsystems, commands,
 * and the rest of Robot.init() are intentionally not initialized.</p>
 */
@TeleOp(name = "DrivetrainTest Test", group = "Test")
public class DrivetrainTest extends LinearOpMode {
    private static final double WHEEL_TEST_POWER = 0.25;

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.opMode = this;
        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;

        try {
            robot.odo = new MixedOdo(robot, false);
            robot.odoDrivetrain = new OdoDrivetrain(robot);

            telemetry.addLine("Headless Drive ready");
            telemetry.addLine("Left stick: field translation");
            telemetry.addLine("Right stick X: turn");
            telemetry.addLine("Wheel test: LB=forward, RB=reverse");
            telemetry.addLine("X=LF  Y=RF  A=LB  B=RB");
            telemetry.update();

            robot.odo.resetPosAndIMU();

            waitForStart();

            while (opModeIsActive()) {
                robot.odo.update();

                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                robot.odoDrivetrain.driveRobotFieldCentric(
                        axial,
                        lateral,
                        yaw
                );

                Pose2D pose = robot.odo.getPosition();
                telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
                telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
                telemetry.addData("Heading", "%.1f°", pose.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Axial", "%.2f", axial);
                telemetry.addData("Lateral", "%.2f", lateral);
                telemetry.addData("Yaw", "%.2f", yaw);
                telemetry.update();
            }
        } finally {
            if (robot.odoDrivetrain != null) {
                robot.odoDrivetrain.stopMotor();
            }
        }
    }
}
