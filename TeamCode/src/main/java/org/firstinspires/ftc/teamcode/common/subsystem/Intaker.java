package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.network.RemoteCommand;
import org.firstinspires.ftc.teamcode.common.util.Param;

import java.util.Objects;

/** Intake hardware with ordinary synchronous methods; command entry owns the worker. */
public class Intaker implements SubsystemLifecycle {
    private final DcMotorEx intakerItself;
    private boolean isRunning = false;
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    public Intaker(Robot robot) {
        intakerItself = Objects.requireNonNull(
                robot.hardwareMap.get(DcMotorEx.class, Globals.intaker),
                "Intaker motor is missing: " + Globals.intaker);
        setIntakePower(0);
        intakerItself.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @RemoteCommand
    public void setIntakePower(@Param("power") double power) {
        CommandTasks.checkCancelled();
        intakerItself.setPower(power);
    }

    @RemoteCommand
    public void setTargetVelocity(@Param("power")double intakerRPM) {
        CommandTasks.checkCancelled();
        isRunning = true;
        intakerItself.setVelocity(intakerRPM / 60.0 * COUNTS_PER_MOTOR_REV);
    }
    /** Blocks only its caller; direct aggregate calls naturally wait for completion. */
    @RemoteCommand
    public void setIntakerPower(@Param("power") double power, @Param("time") long time)
            throws InterruptedException {
        if (time < 0) throw new IllegalArgumentException("time must be non-negative");
        setIntakePower(power);
        try {
            Thread.sleep(time);
        } finally {
            // A cancelled worker must not race the lifecycle's final hardware stop.
            if (!CommandTasks.isCancelled()) intakerArmStop();
        }
    }

    @RemoteCommand
    public void intakerIn(@Param("power") double power) { setIntakePower(power); }

    @RemoteCommand
    public void intakerIn() { intakerIn(1); }

    @RemoteCommand
    public void intakerArmStop() { setIntakePower(0); }

    @Override
    public void onShutdown() { setIntakePower(0); }
}
