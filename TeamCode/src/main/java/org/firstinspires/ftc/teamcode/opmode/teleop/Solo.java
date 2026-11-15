package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.drive.OdoDrivetrain;
import org.firstinspires.ftc.teamcode.common.opmode.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.common.subsystem.Puller;
import org.firstinspires.ftc.teamcode.common.subsystem.Shooter;

/**
 * Minimal field-centric drivetrain test.
 *
 * <p>Only initializes MixedOdo and OdoDrivetrain. Vision, subsystems, commands,
 * and the rest of Robot.init() are intentionally not initialized.</p>
 */
@TeleOp(name = "Solo", group = "TeleOp")
public class Solo extends RobotLinearOpMode {
    private static final double WHEEL_TEST_POWER = 0.25;

    private boolean ShooterOn = false;

    @Override
    protected void runRobotOpMode() {
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
            //shooter保持开启

            runWhileActive(() -> {
                //Puller与Intake控制
                int current = robot.subsystem.puller.getPullerState();
                int next = current;

                if (gamepad1.dpadUpWasPressed() && current < Puller.STATE_STORE) {
                    next = current + 1;
                }
                if (gamepad1.dpadDownWasPressed() && current > Puller.STATE_DOWN) {
                    next = current - 1;
                }

                if (next != current) {
                    robot.subsystem.puller.setState(next);
                }

                if (gamepad1.rightBumperWasPressed()){
                    robot.subsystem.intaker.intakerIn();
                }
                if (gamepad1.rightBumperWasReleased()) {
                    robot.subsystem.intaker.intakerArmStop();
                }

                //Shooter控制
                if (gamepad1.leftTriggerWasPressed()){
                    if (ShooterOn) {
                        robot.subsystem.shooter.stopShooter();
                        ShooterOn = false;
                    }
                    else {
                        robot.subsystem.shooter.setTargetVelocity(6000);
                        ShooterOn = true;
                    }
                }
                if (gamepad1.leftBumperWasPressed()) robot.subsystem.shooter.setValveOpen();
                if (gamepad1.leftBumperWasReleased()) robot.subsystem.shooter.setValveClose();

                //移动部分
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

            });
        } finally {
            if (robot.odoDrivetrain != null) {
                robot.odoDrivetrain.stopMotor();
            }
        }
    }
}
