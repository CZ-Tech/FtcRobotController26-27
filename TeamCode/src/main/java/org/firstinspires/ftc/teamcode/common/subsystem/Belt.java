package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

@GenerateSchedule
public class Belt {

    private final DcMotorEx beltItself;

    public Belt(Robot robot){
        beltItself = robot.hardwareMap.get(DcMotorEx.class, Globals.belt);
    }

    private Belt beltPower(double power){
        beltItself.setPower(power);
        return this;
    }

    public void up(double power){
        beltPower(Math.abs(power));
    }

    public void up(){
        this.up(-1);
    }

    public void down(){
        beltPower(-0.8);
    }

    public void stop(){
        this.beltPower(0);
    }
}

