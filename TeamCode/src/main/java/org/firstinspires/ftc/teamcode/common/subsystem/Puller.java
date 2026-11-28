package org.firstinspires.ftc.teamcode.common.subsystem;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.network.RemoteCommand;
import org.firstinspires.ftc.teamcode.common.util.Param;

public class Puller implements SubsystemLifecycle {
    public static final int STATE_DOWN = 0;
    public static final int STATE_UP = 2;
    public static final int STATE_STORE = 1;

    private static final double SERVO_MIN = 500.0;
    private static final double SERVO_MAX = 2500.0;

    private static final double LEFT_DOWN = 0.0196;
    private static final double RIGHT_DOWN = 0.736;
    private static final double LEFT_UP = 0.4136;
    private static final double RIGHT_UP = 0.338;
    private static final double LEFT_STORE = 1;
    private static final double RIGHT_STORE = 0;

    private final ServoImplEx leftKickerServo;
    private final ServoImplEx rightKickerServo;
    private volatile int pullerState = STATE_STORE;

    private double calculate(double degree) {
        return (degree - SERVO_MIN) / (SERVO_MAX - SERVO_MIN);
    }

    public Puller(@NonNull Robot robot) {
        leftKickerServo = robot.hardwareMap.get(ServoImplEx.class, Globals.leftPullerServo);
        rightKickerServo = robot.hardwareMap.get(ServoImplEx.class, Globals.rightPullerServo);
    }

    @RemoteCommand
    public void down() {
        CommandTasks.checkCancelled();
        leftKickerServo.setPosition(LEFT_DOWN);
        rightKickerServo.setPosition(RIGHT_DOWN);
        pullerState = STATE_DOWN;
    }

    @RemoteCommand
    public void up() {
        CommandTasks.checkCancelled();
        leftKickerServo.setPosition(LEFT_UP);
        rightKickerServo.setPosition(RIGHT_UP);
        pullerState = STATE_UP;
    }

    @RemoteCommand
    public void store() {
        CommandTasks.checkCancelled();
        leftKickerServo.setPosition(LEFT_STORE);
        rightKickerServo.setPosition(RIGHT_STORE);
        pullerState = STATE_STORE;
    }

    public int getPullerState() {
        CommandTasks.checkCancelled();
        return pullerState;
    }

    @RemoteCommand
    public void setState(@Param("state") int state) {
        switch (state) {
            case STATE_DOWN:
                down();
                break;
            case STATE_UP:
                up();
                break;
            case STATE_STORE:
                store();
                break;
            default:
                throw new IllegalArgumentException("Unknown puller state: " + state);
        }
    }
}


