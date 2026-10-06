package org.firstinspires.ftc.teamcode.common.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

public class Classifier {
    public Robot robot;
    public ServoImplEx classifierServo;
    public static double IN = 1656, OUT = 2450;

    public Classifier(@NonNull Robot robot){
        this.robot = robot;
        classifierServo = robot.hardwareMap.get(ServoImplEx.class, Globals.classifierServo);
    }

    public Classifier in(){
        classifierServo.setPosition((IN - 500.0) / 2000.0);
        return this;
    }

    public Classifier out(){
        classifierServo.setPosition((OUT - 500.0) / 2000.0);
        return this;
    }
}
