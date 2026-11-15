package org.firstinspires.ftc.teamcode.common.subsystem;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

@GenerateSchedule
public class Puller implements ScheduleLifecycle{
    public ServoImplEx leftKickerServo;
    public ServoImplEx rightKickerServo;

    public static int pullerState = 2;

    public static double leftDown = (549 - 500)/2500.0;
    public static double leftUp = (1534 - 500)/2500.0;
    public static double leftStore = (2038 - 500)/2500.0;
    public static double rightDown = (2340 - 500)/2500.0;
    public static double rightStore = (915 - 500)/2500.0;
    public static double rightUp = (1345 - 500)/2500.0;

    //setPosition里面需要填写的计算公式：(目标位置 - 最小值) / 最大值),最小值500，最大值2500
    private int caculate(int degree){
        return (degree - 500) / 2500;
    }
    public Puller(@NonNull Robot robot){
        leftKickerServo = robot.hardwareMap.get(ServoImplEx.class, Globals.leftPullerServo);
        rightKickerServo = robot.hardwareMap.get(ServoImplEx.class, Globals.rightPullerServo);

    }

    //放下铲子
    public void down(){//最低角度状态，需要设置两个舵机的的数值
        leftKickerServo.setPosition(0.0196);
        rightKickerServo.setPosition(0.736);
        pullerState = 0;
    }

    //抬起铲子
    public void up(){
        leftKickerServo.setPosition(0.4136);
        rightKickerServo.setPosition(0.338);
        pullerState = 1;
    }

    // 收起至机器内（会挡住发射）
    public void store(){
        leftKickerServo.setPosition(0.1652);
        rightKickerServo.setPosition(0.166);
        pullerState = 2;
    }

    public int getPullerState() {
        return pullerState;
    }
}


