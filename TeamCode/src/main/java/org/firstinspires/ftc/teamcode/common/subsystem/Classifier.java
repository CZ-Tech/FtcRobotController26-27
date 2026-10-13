package org.firstinspires.ftc.teamcode.common.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

public class Classifier extends ScheduledSubsystem<Classifier.Schedule> {
    public interface Schedule extends ScheduleApi<Schedule> {
        Schedule in();
        Schedule out();
    }

    private final ServoImplEx classifierServo;
    private static final double IN = 1656, OUT = 2450;

    public Classifier(@NonNull Robot robot){
        super(Schedule.class);
        classifierServo = robot.hardwareMap.get(ServoImplEx.class, Globals.classifierServo);
    }

    private Classifier in(){
        classifierServo.setPosition((IN - 500.0) / 2000.0);
        return this;
    }

    private Classifier out(){
        classifierServo.setPosition((OUT - 500.0) / 2000.0);
        return this;
    }
}
