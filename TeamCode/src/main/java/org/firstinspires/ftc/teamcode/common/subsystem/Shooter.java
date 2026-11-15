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

@GenerateSchedule
public class Shooter implements ScheduleLifecycle{

    private final DcMotorEx shooterItself;
    private final ServoImplEx valveItself;
    private boolean isRunning = false;
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    public double r = 0;
    public double valveOpenDegree = 0.736;//这个数据需要更改！具体数据计算公式：(目标位置 - 最小值) / 最大值),最小值500，最大值2500。下一行的同理。分别为挡板打开（发射），与关闭（阻止发射）
    public double valveCloseDegree = 0.166;//这个数据需要更改！

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

    public void updatePIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(ShooterP, ShooterI, ShooterD, ShooterF);
        shooterItself.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
    public void setPower(double power){
        shooterItself.setPower(power);
    }

    public void setTargetVelocity(double shooterRPM) {
        isRunning = true;
        shooterItself.setVelocity(shooterRPM / 60.0 * COUNTS_PER_MOTOR_REV);
    }

    /**
     * 阀是瞬时动作，必须直接执行，不能进调度邮箱。
     *
     * <p>它和 {@link #setTargetVelocity(double)} 共用 shooter 的 capacity-1 邮箱，若也走调度，
     * 同一圈里先后提交的指令会互相顶掉（先提交、还没变成 active 的那条既不执行也不触发
     * {@link #onScheduleCancelled()}），而且提交阀门指令会取消正在跑的转速调度并把马达停掉。
     * 因此这三个方法标 {@link DirectAction}：只在 OpMode/控制线程里当场写舵机。</p>
     */
    @DirectAction
    public void setValvePos(double n) {
        valveItself.setPosition(n);
    }

    @DirectAction
    public void setValveOpen() {
        valveItself.setPosition(valveOpenDegree);
    }

    @DirectAction
    public void setValveClose() {
        valveItself.setPosition(valveCloseDegree);
    }

//    public void setTatgetVelocity(double shooterRPM , long time){
//        isRunning = true;
//        shooterItself.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterItself.setMotorEnable();
//        valveItself.setPosition(1);//这里以及及接下来有关valve的参数需要更改！
//        shooterItself.setPower(1.0);
//        shooterItself.setVelocity(shooterRPM / 60.0 * COUNTS_PER_MOTOR_REV);
//        waitMillis(time);
//        valveItself.setPosition(1);
//        shooterItself.setPower(0.0);
//    }

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



    @Override
    public void onScheduleCancelled() {
        setPower(0);
    }

    /** Called by the subsystem owner on OpMode shutdown, even without an active schedule. */
    public void onShutdown() {
        setPower(0);
    }
}