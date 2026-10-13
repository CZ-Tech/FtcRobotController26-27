package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

public class Door extends ScheduledSubsystem<Door.Schedule> {
    public interface Schedule extends ScheduleApi<Schedule> {
        Schedule spin(double power);
        Schedule spin();
        Schedule stop();
        Schedule standBy();
        Schedule end();
    }

    private final CRServoImplEx doorWheel;

    private static final double DOOR_END = 1200;    //在上面
    private static final double DOOR_STANDBY = 1980;//推出去

    public Door(Robot robot){
        super(Schedule.class);
        doorWheel = robot.hardwareMap.get(CRServoImplEx.class, Globals.doorWheel);

        doorWheel.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    private Door spin(double power){
        doorWheel.setPower(power);
        return this;
    }

    private Door spin(){
        return this.spin(1);
    }

    private Door stop(){
        return this.spin(0);
    }

    private Door standBy(){
//        doorController.setPosition((DOOR_STANDBY - 500.0) / 2000.0);
        return this;
    }

    private Door end(){
//        doorController.setPosition((DOOR_END - 500.0) / 2000.0);
        return this;
    }
}
