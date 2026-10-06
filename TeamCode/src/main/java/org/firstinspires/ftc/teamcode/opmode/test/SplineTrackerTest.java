package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTracker;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTrajectoryLoader;

/** Manual field test for the non-blocking spline follower. */
@Autonomous(name = "SplineTrackerTest", group = "Test")
public class SplineTrackerTest extends LinearOpMode {

    private static final String TEST_PATH =
            "["
                    + "{\"x\":0,\"y\":0,\"dx\":1,\"dy\":0,\"heading\":0,\"marker\":\"start\"},"
                    + "{\"x\":24,\"y\":0,\"dx\":1,\"dy\":0,\"heading\":0,\"marker\":\"A\"},"
                    + "{\"x\":24,\"y\":24,\"dx\":0,\"dy\":1,\"heading\":0,\"marker\":\"B\"},"
                    + "{\"x\":24,\"y\":24,\"dx\":0,\"dy\":0,\"heading\":90,\"marker\":\"C\"},"
                    + "{\"x\":0,\"y\":24,\"dx\":-1,\"dy\":0,\"heading\":90,\"marker\":\"D\"},"
                    + "{\"x\":0,\"y\":0,\"dx\":0,\"dy\":-1,\"heading\":0,\"marker\":\"E\"}"
                    + "]";

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(this);

        SplineTracker tracker = new SplineTracker(robot);
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(tracker);

        telemetry.addData("Status", "Ready. DEBUG=" + Globals.DEBUG);
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        loader.start(TEST_PATH);

        while (opModeIsActive() && loader.isRunning()) {
            loader.update();

            SplineTrajectoryLoader.TrajectoryEvent event;
            while ((event = loader.pollEvent()) != null) {
                if (event.marker != null) {
                    telemetry.addData("Waypoint", event.marker);
                }
            }

            telemetry.addData("State", loader.getStatus());
            telemetry.addData("Frame", loader.getCurrentFrameIndex());
            telemetry.update();
            idle();
        }

        loader.cancel();
        robot.odoDrivetrain.stopMotor();
    }
}
