package org.firstinspires.ftc.teamcode.common.subsystem;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

@GenerateSchedule
public class Puller implements ScheduleLifecycle{
    private int pullerState = 0;
    private boolean isRunning = false;
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    public double r = 0;
    public ServoImplEx leftKickerServo;
    public ServoImplEx rightKickerServo;
    public static double leftIN = 1656, leftOUT = 2450, rightIN =114514, rightOUT =114514;//right的数字需要更改！

    public Puller(@NonNull Robot robot){
        leftKickerServo = robot.hardwareMap.get(ServoImplEx.class, Globals.leftPullerServo);
        rightKickerServo = robot.hardwareMap.get(ServoImplEx.class, Globals.rightPullerServo);

    }

    //放下铲子
    public void state0(){
        leftKickerServo.setPosition((leftIN - 500.0) / 2500.0);
        rightKickerServo.setPosition((leftIN - 500.0) / 2500.0);
    }

    public void state1(){}
    //抬起铲子
    public void state2(){}

    public void state3(){
        leftKickerServo.setPosition((leftOUT - 500.0) / 2500.0);
        rightKickerServo.setPosition((rightOUT - 500.0) / 2500);
    }

    public int getPullerState(){
        return pullerState;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public void setRunning(boolean running) {
        isRunning = running;
    }
}


