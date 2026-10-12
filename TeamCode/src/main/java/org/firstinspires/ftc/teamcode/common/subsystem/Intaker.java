package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

public class Intaker extends ScheduledSubsystem<Intaker.Schedule> {
    public interface Schedule extends ScheduleApi<Schedule> {
        Schedule intakerIn(double power);
        Schedule intakerIn();
        Schedule intakerArmStop();
    }

    private final DcMotorEx intakerItself;

    public Intaker(Robot robot) {
        super(Schedule.class);
        intakerItself = java.util.Objects.requireNonNull(
                robot.hardwareMap.get(DcMotorEx.class, Globals.intaker),
                "Intaker motor is missing: " + Globals.intaker);

        this.setArmPower(0);
    }

    private Intaker setArmPower(double power){
        intakerItself.setPower(power);
        return this;
    }

    public Intaker intakerIn(double power){
        return this.setArmPower(power);
    }

    public Intaker intakerIn(){
        return this.intakerIn(1);
    }

    public Intaker intakerArmStop(){
        return this.setArmPower(0);
    }
}
