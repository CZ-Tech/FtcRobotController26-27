package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.opmode.RobotLinearOpMode;

@TeleOp(name = "ResetOdo", group = "Test")
public class ResetOdo extends RobotLinearOpMode {
    @Override
    protected void runRobotOpMode() {
        robot.odo.reCollaborate();
        telemetry.addLine("Odo has been reset! Next init needs AprilTag!");
        while (!Globals.isPoseInitialized && !isStarted()) {
            robot.odo.update();
            telemetry.update();
        };
        waitForStart();
        runWhileActive(() -> {
            robot.odo.update();
            telemetry.update();
        });
    }

}

