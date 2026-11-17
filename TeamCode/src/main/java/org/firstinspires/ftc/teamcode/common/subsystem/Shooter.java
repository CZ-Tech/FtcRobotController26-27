package org.firstinspires.ftc.teamcode.common.subsystem;

import static org.firstinspires.ftc.teamcode.common.Globals.ShooterD;
import static org.firstinspires.ftc.teamcode.common.Globals.ShooterF;
import static org.firstinspires.ftc.teamcode.common.Globals.ShooterI;
import static org.firstinspires.ftc.teamcode.common.Globals.ShooterP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.network.RemoteCommand;
import org.firstinspires.ftc.teamcode.common.util.Param;

public class Shooter implements SubsystemLifecycle {

    private final DcMotorEx shooterItself;
    private final ServoImplEx valveItself;
    private boolean isRunning = false;
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    public double r = 0;
    public double valveOpenDegree = 0.736;//计算公式：(目标位置 - 最小值) / 最大值),最小值500，最大值2500。下一行的同理。分别为挡板打开（发射），与关闭（阻止发射）
    public double valveCloseDegree = 0.166;

    public Shooter(Robot robot) {
        shooterItself = java.util.Objects.requireNonNull(
                robot.hardwareMap.get(DcMotorEx.class, Globals.Shooter),
                "Shooter motor is missing: " + Globals.Shooter);

        shooterItself.setPower(0);
        shooterItself.setDirection(DcMotorSimple.Direction.FORWARD);
        updatePIDF();

        valveItself = java.util.Objects.requireNonNull(
                robot.hardwareMap.get(ServoImplEx.class, Globals.valve),
                "Shooter motor is missing: " + Globals.valve);

        shooterItself.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterItself.setMotorEnable();
        shooterItself.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @RemoteCommand
    public void updatePIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(ShooterP, ShooterI, ShooterD, ShooterF);
        shooterItself.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
    @RemoteCommand
    public void setPower(@Param("power") double power){
        shooterItself.setPower(power);
    }

    @RemoteCommand
    public void setTargetVelocity(@Param("shooterRPM") double shooterRPM) {
        isRunning = true;
        shooterItself.setVelocity(shooterRPM / 60.0 * COUNTS_PER_MOTOR_REV);
    }

    /** Valve and motor calls are independent ordinary control-thread operations. */
    @RemoteCommand
    public void setValvePos(@Param("n") double n) {
        valveItself.setPosition(n);
    }

    @RemoteCommand
    public void setValveOpen() {
        valveItself.setPosition(valveOpenDegree);
    }

    @RemoteCommand
    public void setValveClose() {
        valveItself.setPosition(valveCloseDegree);
    }

    @RemoteCommand
    public void stopShooter(){
        setPower(0);
    }

    /**
     * Direct method (OpMode thread only). Publish it with
     * {@code telemetry.addData("shooter rpm", "%.0f rpm", shooter.getMotorRPM())}.
     */
    public double getMotorRPM() {
        return shooterItself.getVelocity() * 60.0 / COUNTS_PER_MOTOR_REV;
    }



    /** Always stop the motor on OpMode shutdown, including after plain direct calls. */
    @Override
    public void onShutdown() {
        setPower(0);
    }
}