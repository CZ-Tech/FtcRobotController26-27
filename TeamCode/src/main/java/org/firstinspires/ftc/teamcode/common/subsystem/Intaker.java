package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.network.RemoteCommand;
import org.firstinspires.ftc.teamcode.common.util.Param;

import java.util.Objects;
import java.util.function.Consumer;

/** Intake hardware. Methods are called directly on the control thread. */
public class Intaker implements SubsystemLifecycle {
    private final DcMotorEx intakerItself;
    private final Consumer<ActionSequence> startSequence;

    public Intaker(Robot robot, Consumer<ActionSequence> startSequence) {
        this.startSequence = Objects.requireNonNull(startSequence, "startSequence");
        intakerItself = Objects.requireNonNull(
                robot.hardwareMap.get(DcMotorEx.class, Globals.intaker),
                "Intaker motor is missing: " + Globals.intaker);
        setIntakePower(0);
        intakerItself.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @RemoteCommand
    public void setIntakePower(@Param("power") double power) {
        intakerItself.setPower(power);
    }

    /** Timed intake: explicit callbacks around a non-blocking wait. */
    @RemoteCommand
    public void setIntakerPower(@Param("power") double power, @Param("time") long time) {
        ActionSequence finish = ActionSequence.create().waitMillis(time).run(this::intakerArmStop);
        setIntakePower(power);
        startSequence.accept(finish);
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
