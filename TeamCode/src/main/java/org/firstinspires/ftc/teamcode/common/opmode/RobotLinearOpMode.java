package org.firstinspires.ftc.teamcode.common.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

/** Common Robot initialization and cooperative subsystem scheduling for LinearOpModes. */
public abstract class RobotLinearOpMode extends LinearOpMode {
    protected final Robot robot = new Robot();

    @Override
    public final void runOpMode() throws InterruptedException {
        try {
            beforeRobotInit();
            robot.init(this);
            runRobotOpMode();
        } finally {
            if (robot.subsystem != null) {
                robot.subsystem.cancelAllSchedules();
            }
        }
    }

    /** Optional initialization preparation, before Robot.init() allocates hardware. */
    protected void beforeRobotInit() {}

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
