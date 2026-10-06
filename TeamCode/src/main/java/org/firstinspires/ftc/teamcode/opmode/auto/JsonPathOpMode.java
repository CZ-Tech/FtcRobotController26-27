package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTracker;
import org.firstinspires.ftc.teamcode.common.command.auto.SplineTrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.util.HttpJsonService;

/**
 * A dynamically registered OpMode that executes a saved JSON trajectory from
 * {@link HttpJsonService}. One instance is created per saved path name; the
 * instance is reused across OpMode runs (LinearOpMode supports this).
 *
 * <p>Registered via {@code HttpJsonService}'s {@code InstanceOpModeRegistrar}
 * so that adding/removing paths via the HTTP API immediately updates the
 * Driver Station OpMode list.</p>
 */
public class JsonPathOpMode extends LinearOpMode {

    private final String pathName;

    /**
     * @param pathName the key used to look up the saved JSON in HttpJsonService
     */
    public JsonPathOpMode(String pathName) {
        this.pathName = pathName;
    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(this);

        MixedOdo.isPoseInitialized = true;

        SplineTracker tracker = new SplineTracker(robot);
        SplineTrajectoryLoader loader = new SplineTrajectoryLoader(tracker);

        waitForStart();

        if (opModeIsActive()) {
            String json = HttpJsonService.getSavedJson(pathName);
            if (json != null && !json.isEmpty()) {
                loader.start(json);
                while (opModeIsActive() && loader.isRunning()) {
                    loader.update();
                    while (loader.pollEvent() != null) {
                        // This OpMode follows geometry only; marker/command events are ignored.
                    }
                    idle();
                }
            } else {
                robot.telemetry.addData("JsonPathOpMode", "No JSON found for path: " + pathName);
                robot.telemetry.update();
            }

            robot.odoDrivetrain.driveRobotFieldCentric(0,0,0);

        }
    }
}
