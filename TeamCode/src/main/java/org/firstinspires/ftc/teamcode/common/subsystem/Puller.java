package org.firstinspires.ftc.teamcode.common.subsystem;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

@GenerateSchedule
public class Puller implements ScheduleLifecycle{
    private int pullerState = 3;
    private boolean isRunning = false;
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    public double r = 0;
    public ServoImplEx leftKickerServo;
    public ServoImplEx rightKickerServo;
    //setPosition里面需要填写的计算公式：(目标位置 - 最小值) / 最大值),最小值500，最大值2500
    public Puller(@NonNull Robot robot){
        leftKickerServo = robot.hardwareMap.get(ServoImplEx.class, Globals.leftPullerServo);
        rightKickerServo = robot.hardwareMap.get(ServoImplEx.class, Globals.rightPullerServo);

    }

    //放下铲子
    public void state0(){//最低角度状态，需要设置两个舵机的的数值
        leftKickerServo.setPosition();
        rightKickerServo.setPosition();
    }

    public void state1(){}//中间位置1
    //抬起铲子
    public void state2(){}//中间位置2

    public void state3(){//最大位置
//
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


