package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Param;

/**
 * 没有硬件的“组合指令”类：通过其他子系统的调度器驱动它们。
 *
 * <p>在 {@link Subsystem} 里注册为 {@code auto} 字段后，本类每个 public、void 的方法
 * 自动成为远程指令 {@code auto.<方法名>}（见 {@code GET /api/v2/commands}）。</p>
 *
 * <h2>写组合指令的规则</h2>
 * <ul>
 *   <li><b>只通过其他子系统的调度器写硬件</b>（{@code shooter.xxx()}、
 *       {@code intaker.schedule()...execute()}），它们只往对方邮箱投递，线程安全。
 *       直接方法（有返回值的，如 {@code getMotorRPM()}）只读，可在 {@code waitUntil}
 *       条件里用，条件在控制线程求值。</li>
 *   <li><b>方法体里的代码总是立刻执行完。</b>要“等条件成立后再做某事”，必须把等待放进
 *       <b>被延后的那个子系统自己的调度</b>：
 *       {@code intaker.schedule().waitUntil(cond).intakerIn(1).execute()}。
 *       不能在本方法里写 {@code waitUntil(..)} 后再接着写代码——后面的代码不会被延后。</li>
 *   <li>调用其他子系统会<b>顶掉该子系统当前的调度</b>（后提交的覆盖先提交的），
 *       但不会顶掉本类（{@code auto}）自己的调度；本类自己的调度只会被后发的
 *       {@code auto.xxx} 顶掉。</li>
 *   <li>方法本身很快返回，调度随即结束，已提交给其他子系统的调度会继续运行；
 *       要停下它们请调用 {@link #stopAll()}。</li>
 *   <li>参数类型限于基本类型、包装类和 String；不要写泛型或 throws。</li>
 *   <li>刚提交、还没被 {@code update()} 取走的调度，{@code hasActiveSchedule()} 返回
 *       false，不要用它判断“对方是否忙”。</li>
 * </ul>
 */
@GenerateSchedule
public class AutoCommands {
    /** 转速达到目标的这个比例后才开始送球。 */
    private static final double READY_RATIO = 0.95;

    private final IntakerScheduler intaker;
    private final ShooterScheduler shooter;
    private final PullerScheduler puller;

    /** 只保存引用，不碰硬件，所以不会增加 INIT 失败风险。 */
    public AutoCommands(IntakerScheduler intaker, ShooterScheduler shooter , PullerScheduler puller) {
        this.intaker = java.util.Objects.requireNonNull(intaker, "intaker");
        this.shooter = java.util.Objects.requireNonNull(shooter, "shooter");
        this.puller  = java.util.Objects.requireNonNull(puller,"puller");
    }

    /**
     * 射球：shooter 升到 {@code rpm}，转速达到 95% 后 intaker 开始送球。
     * 送球的等待放在 intaker 自己的调度里，不阻塞 OpMode 循环。
     */
    /*
    public void shootBalls(@Param(value = "rpm", defaultValue = "3000") double rpm) {
     */
    /*
        shooter.setTargetVelocity(rpm);
        intaker.schedule()
                .waitUntil(() -> )
                .intakerIn(1)
                .execute();
    }
    **/
    /** 停止 shooter 和 intaker；同时顶掉它们尚未完成的调度（包括 shootBalls 的等待）。 */
    public void stopAll() {
        intaker.intakerArmStop();
        shooter.stopShooter();
    }

//    public void intake_flower(double power , long time){
//        puller.out();
//        intaker.schedule().setIntakePower(power).waitMillis(500);
//        puller.in();
//    }
}
