package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

public class Belt extends ScheduledSubsystem<Belt.Schedule> {
    public interface Schedule extends ScheduleApi<Schedule> {
        Schedule up(double power);
        Schedule up();
        Schedule down();
        Schedule stop();
    }

    private final DcMotorEx beltItself;

    public Belt(Robot robot){
        super(Schedule.class);
        beltItself = robot.hardwareMap.get(DcMotorEx.class, Globals.belt);
    }

    private Belt beltPower(double power){
        beltItself.setPower(power);
        return this;
    }

    private Belt up(double power){
        return beltPower(Math.abs(power));
    }

    private Belt up(){
        return this.up(-1);
    }

    private Belt down(){
        return beltPower(-0.8);
    }

    private Belt stop(){
        return this.beltPower(0);
    }
}
