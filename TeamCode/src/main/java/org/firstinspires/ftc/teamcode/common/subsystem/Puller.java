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

    public static int leftDown = 549;
    public static int leftUp = 1534;
    public static int leftStore = 2038;
    public static int rightDown = 2340;
    public static int rightStore = 915;
    public static int rightUp = 1345;

    //setPosition里面需要填写的计算公式：(目标位置 - 最小值) / 最大值),最小值500，最大值2500
    public Puller(@NonNull Robot robot){
        leftKickerServo = robot.hardwareMap.get(ServoImplEx.class, Globals.leftPullerServo);
        rightKickerServo = robot.hardwareMap.get(ServoImplEx.class, Globals.rightPullerServo);

    }

    //放下铲子
    public void down(){//最低角度状态，需要设置两个舵机的的数值
        leftKickerServo.setPosition(Robot.NormalizePWM(leftDown));
        rightKickerServo.setPosition(Robot.NormalizePWM(rightDown));
        pullerState = 0;
    }

    //抬起铲子
    public void up(){
        leftKickerServo.setPosition(Robot.NormalizePWM(leftUp));
        rightKickerServo.setPosition(Robot.NormalizePWM(rightUp));
        pullerState = 1;
    }

    // 收起至机器内（会挡住发射）
    public void store(){
        leftKickerServo.setPosition(Robot.NormalizePWM(leftStore));
        rightKickerServo.setPosition(Robot.NormalizePWM(rightStore));
        pullerState = 2;
    }

    public int getPullerState() {
        return pullerState;
    }
}


