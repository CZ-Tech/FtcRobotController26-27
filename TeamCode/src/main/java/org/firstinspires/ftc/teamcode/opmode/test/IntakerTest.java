package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.opmode.RobotLinearOpMode;

/** Field-centric drivetrain and intake scheduler test. */
@TeleOp(name = "IntakerTest Test", group = "Test")
public class IntakerTest extends RobotLinearOpMode {

    @Override
    protected void runRobotOpMode() {
        try {
            telemetry.addLine("Headless Drive ready");
            telemetry.addLine("Left stick: field translation");
            telemetry.addLine("Right stick X: turn");
            telemetry.addLine("Wheel test: LB=forward, RB=reverse");
            telemetry.addLine("X=LF  Y=RF  A=LB  B=RB");
            telemetry.update();

            robot.odo.resetPosAndIMU();

            waitForStart();

            runWhileActive(() -> {
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

                if (gamepad1.leftBumperWasPressed()) {
                    robot.subsystem.intaker.schedule().intakerIn(1).execute();
                } else if (gamepad1.leftBumperWasReleased()) {
                    robot.subsystem.intaker.schedule().intakerArmStop().execute();
                }
            });
        } finally {
            if (robot.odoDrivetrain != null) {
                robot.odoDrivetrain.stopMotor();
            }
        }
    }
}
