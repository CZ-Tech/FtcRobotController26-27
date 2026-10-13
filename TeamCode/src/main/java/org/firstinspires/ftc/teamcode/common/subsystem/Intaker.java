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

    private Intaker intakerIn(double power){
        return this.setArmPower(power);
    }

    private Intaker intakerIn(){
        return this.intakerIn(1);
    }

    private Intaker intakerArmStop(){
        return this.setArmPower(0);
    }

    @Override
    protected void onScheduleCancelled() {
        setArmPower(0);
    }

    /** Called by the subsystem owner on OpMode shutdown, even without an active schedule. */
    void stopForShutdown() {
        setArmPower(0);
    }
}
