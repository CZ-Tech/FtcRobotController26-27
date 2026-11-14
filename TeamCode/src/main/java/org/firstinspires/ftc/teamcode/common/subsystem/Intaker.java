package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

@GenerateSchedule
public class Intaker implements ScheduleLifecycle {

    private final DcMotorEx intakerItself;


    public Intaker(Robot robot) {
        intakerItself = java.util.Objects.requireNonNull(
                robot.hardwareMap.get(DcMotorEx.class, Globals.intaker),
                "Intaker motor is missing: " + Globals.intaker);
        this.setIntakePower(0);
        intakerItself.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setIntakePower(double power){
        intakerItself.setPower(power);
    }

    public void setIntakerPower(double power, long time){
        intakerItself.setPower(power);
        waitMillis(time);
        intakerItself.setPower(0.0);
    }

    public void intakerIn(double power){
        this.setIntakePower(power);
    }

    public void intakerIn(){
        this.intakerIn(1);
    }

    public void intakerArmStop(){
        this.setIntakePower(0);
    }

    @Override
    public void onScheduleCancelled() {
        setIntakePower(0);
    }

    /** Called by the subsystem owner on OpMode shutdown, even without an active schedule. */
    public void onShutdown() {
        setIntakePower(0);
    }
}

