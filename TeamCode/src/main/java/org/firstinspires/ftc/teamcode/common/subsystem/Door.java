package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

@GenerateSchedule
public class Door {

    private final CRServoImplEx doorWheel;

    private static final double DOOR_END = 1200;    //在上面
    private static final double DOOR_STANDBY = 1980;//推出去

    public Door(Robot robot){
        doorWheel = robot.hardwareMap.get(CRServoImplEx.class, Globals.doorWheel);

        doorWheel.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void spin(double power){
        doorWheel.setPower(power);
    }

    public void spin(){
        this.spin(1);
    }

    public void stop(){
        this.spin(0);
    }

    public void standBy(){
//        doorController.setPosition((DOOR_STANDBY - 500.0) / 2000.0);
    }

    public void end(){
//        doorController.setPosition((DOOR_END - 500.0) / 2000.0);
    }
}

