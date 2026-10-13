package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

@GenerateSchedule
public class Intaker implements ScheduleLifecycle {

    private final DcMotorEx intakerItself;

    public Intaker(Robot robot) {
        intakerItself = java.util.Objects.requireNonNull(
                robot.hardwareMap.get(DcMotorEx.class, Globals.intaker),
                "Intaker motor is missing: " + Globals.intaker);

        this.setArmPower(0);
    }

    private Intaker setArmPower(double power){
        intakerItself.setPower(power);
        return this;
    }

    public void intakerIn(double power){
        this.setArmPower(power);
    }

    public void intakerIn(){
        this.intakerIn(1);
    }

    public void intakerArmStop(){
        this.setArmPower(0);
    }

    @Override
    public void onScheduleCancelled() {
        setArmPower(0);
    }

    /** Called by the subsystem owner on OpMode shutdown, even without an active schedule. */
    public void onShutdown() {
        setArmPower(0);
    }
}

