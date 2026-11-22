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
    private final DcMotorEx transferItself;
    private final ServoImplEx valveItself;
    private boolean isRunning = false;
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    public double r = 0;
    public volatile double valveOpenDegree = 0.736;//计算公式：(目标位置 - 最小值) / 最大值),最小值500，最大值2500。下一行的同理。分别为挡板打开（发射），与关闭（阻止发射）
    public volatile double valveCloseDegree = 0.166;

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

        transferItself = robot.hardwareMap.get(DcMotorEx.class,Globals.transfer);

        shooterItself.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterItself.setMotorEnable();
        shooterItself.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @RemoteCommand
    public void updatePIDF() {
        CommandTasks.checkCancelled();
        PIDFCoefficients pidf = new PIDFCoefficients(ShooterP, ShooterI, ShooterD, ShooterF);
        shooterItself.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
    @RemoteCommand
    public void setPower(@Param("power") double power){
        CommandTasks.checkCancelled();
        shooterItself.setPower(power);
    }

    @RemoteCommand
    public void setTargetVelocity(@Param("shooterRPM") double shooterRPM) {
        CommandTasks.checkCancelled();
        isRunning = true;
        shooterItself.setVelocity(shooterRPM / 60.0 * COUNTS_PER_MOTOR_REV);
    }

    /** Valve and motor calls are independent ordinary Java operations. */
    @RemoteCommand
    public void setValvePos(@Param("n") double n) {
        CommandTasks.checkCancelled();
        valveItself.setPosition(n);
    }

    @RemoteCommand
    public void setValveOpen() {
        CommandTasks.checkCancelled();
        valveItself.setPosition(valveOpenDegree);
    }

    @RemoteCommand
    public void setValveClose() {
        CommandTasks.checkCancelled();
        valveItself.setPosition(valveCloseDegree);
    }

    @RemoteCommand
    public void stopShooter(){
        setPower(0);
    }

    /**
     * Direct status read, including from a command's polling loop. Publish it with
     * {@code telemetry.addData("shooter rpm", "%.0f rpm", shooter.getMotorRPM())}.
     */
    public double getMotorRPM() {
        CommandTasks.checkCancelled();
        return shooterItself.getVelocity() * 60.0 / COUNTS_PER_MOTOR_REV;
    }

    public void startTransfer(){
        transferItself.setPower(1);
    }

    public void stopTransfer(){
        transferItself.setPower(0);
    }

    /** Always stop the motor on OpMode shutdown, including after plain direct calls. */
    @Override
    public void onShutdown() {
        try { setPower(0); }
        finally { setValveClose(); }
    }
}