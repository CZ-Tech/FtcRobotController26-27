package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

@GenerateSchedule
public class Shooter implements ScheduleLifecycle{
    private final DcMotorEx shooterItself;
    public Shooter(Robot robot) {
        shooterItself = java.util.Objects.requireNonNull(
                robot.hardwareMap.get(DcMotorEx.class, Globals.Shooter),
                "Shooter motor is missing: " + Globals.Shooter);

        this.setPower(0);
        shooterItself.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private Shooter setPower(double power){
        shooterItself.setPower(power);
        return this;
    }

    public void shooterIn(double power){
        this.setPower(power);
    }

    public void shooterIn(){
        this.shooterIn(1);
    }

    public void shooterArmStop(){
        this.setPower(0);
    }

    @Override
    public void onScheduleCancelled() {
        setPower(0);
    }

    /** Called by the subsystem owner on OpMode shutdown, even without an active schedule. */
    public void onShutdown() {
        setPower(0);
    }
}