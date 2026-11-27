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

import java.util.Objects;

public class Shooter implements SubsystemLifecycle {

    private final DcMotorEx leftShooterItself;
    private final DcMotorEx rightShooterItself;
    private final DcMotorEx transferItself;
    private final ServoImplEx valveItself;
    private boolean isRunning = false;

    private boolean transferIsRunning = false;
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    public double r = 0;
    public volatile double valveOpenDegree = 0.736;//计算公式：(目标位置 - 最小值) / 最大值),最小值500，最大值2500。下一行的同理。分别为挡板打开（发射），与关闭（阻止发射）
    public volatile double valveCloseDegree = 0.166;

    public Shooter(Robot robot) {
        leftShooterItself = Objects.requireNonNull(
                robot.hardwareMap.get(DcMotorEx.class, Globals.leftShooter),
                "LeftShooter motor is missing: " + Globals.leftShooter);

        leftShooterItself.setPower(0);
        leftShooterItself.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShooterItself = Objects.requireNonNull(
                robot.hardwareMap.get(DcMotorEx.class, Globals.rightShooter),
                "RightShooter motor is missing: " + Globals.rightShooter);
        rightShooterItself.setPower(0);
        rightShooterItself.setDirection(DcMotorSimple.Direction.FORWARD);

        updatePIDF();

        valveItself = Objects.requireNonNull(
                robot.hardwareMap.get(ServoImplEx.class, Globals.valve),
                "Shooter motor is missing: " + Globals.valve);

        transferItself = robot.hardwareMap.get(DcMotorEx.class,Globals.transfer);

        leftShooterItself.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooterItself.setMotorEnable();
        leftShooterItself.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightShooterItself.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterItself.setMotorEnable();
        rightShooterItself.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @RemoteCommand
    public void updatePIDF() {
        CommandTasks.checkCancelled();
        PIDFCoefficients pidf = new PIDFCoefficients(ShooterP, ShooterI, ShooterD, ShooterF);
        leftShooterItself.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightShooterItself.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidf);
    }
    @RemoteCommand
    public void setPower(@Param("power") double power){
        CommandTasks.checkCancelled();
        leftShooterItself.setPower(power);
        rightShooterItself.setPower(power);
    }

    @RemoteCommand
    public void setTargetVelocity(@Param("shooterRPM") double shooterRPM) {
        CommandTasks.checkCancelled();
        isRunning = true;
        leftShooterItself.setVelocity(shooterRPM / 60.0 * COUNTS_PER_MOTOR_REV);
        rightShooterItself.setVelocity(shooterRPM / 60.0 * COUNTS_PER_MOTOR_REV);
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
        return leftShooterItself.getVelocity() * 60.0 / COUNTS_PER_MOTOR_REV;
    }
    public double getLeftMotorRPM() {
        CommandTasks.checkCancelled();
        return leftShooterItself.getVelocity() * 60.0 / COUNTS_PER_MOTOR_REV;
    }
    public double getRightMotorRPM() {
        CommandTasks.checkCancelled();
        return rightShooterItself.getVelocity() * 60.0 / COUNTS_PER_MOTOR_REV;
    }


    public void startTransfer(){
        transferItself.setPower(1);
    }

    public void stopTransfer(){
        transferItself.setPower(0);
    }

    public void setTransferVelocity(@Param("power")double transferRPM) {
        CommandTasks.checkCancelled();
        transferIsRunning = true;
        transferItself.setVelocity(transferRPM / 60.0 * COUNTS_PER_MOTOR_REV);
    }
    /** Always stop the motor on OpMode shutdown, including after plain direct calls. */
    @Override
    public void onShutdown() {
        try { setPower(0); }
        finally { setValveClose(); }
    }
}