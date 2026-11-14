package org.firstinspires.ftc.teamcode.common.opmode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.network.RobotNetworkService;
import org.firstinspires.ftc.teamcode.common.network.RobotNetworkV2;
import org.firstinspires.ftc.teamcode.common.telemetry.ApiTelemetry;
import org.firstinspires.ftc.teamcode.common.telemetry.MultipleTelemetry;

/** Common Robot initialization and cooperative subsystem scheduling for LinearOpModes. */
public abstract class RobotLinearOpMode extends LinearOpMode {
    private static final String TAG = "RobotLinearOpMode";

    protected final Robot robot = new Robot();

    @Override
    public final void runOpMode() throws InterruptedException {
        // Everything written to telemetry (by the OpMode and by Robot code through
        // robot.telemetry) goes to the Driver Station and the network API alike.
        Telemetry officialTelemetry = telemetry;
        telemetry = fuseWithApi(officialTelemetry);
        try {
            robot.init(this);
            runRobotOpMode();
        } finally {
            try {
                if (robot.subsystem != null) {
                    robot.subsystem.shutdown();
                }
            } finally {
                telemetry = officialTelemetry;
            }
        }
    }

    /** The official telemetry plus the API one; the official one alone if there is no network. */
    private static Telemetry fuseWithApi(Telemetry official) {
        try {
            RobotNetworkV2 network = RobotNetworkService.get();
            return new MultipleTelemetry(official, new ApiTelemetry(network.telemetry));
        } catch (IllegalStateException e) {
            Log.w(TAG, "Network is not running; telemetry goes to the Driver Station only");
            return official;
        }
    }

    /** OpMode-specific initialization, waitForStart and control flow. */
    protected abstract void runRobotOpMode() throws InterruptedException;

    /** Run one application iteration and one subsystem scheduler tick until stopped. */
    protected final void runWhileActive(Runnable iteration) {
        while (opModeIsActive()) {
            iteration.run();
            robot.subsystem.update();
            idle();
        }
    }
}
