package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.Robot;
import java.util.ArrayList;
import java.util.List;

/**
 * 机器人所有子系统的集合，每个子系统对应一个 {@link Scheduler}（调度器）。
 *
 * <h2>如何新建子系统</h2>
 * <ol>
 *   <li>在 {@code Globals} 里加硬件配置名，并在机器配置里加上对应设备。</li>
 *   <li>写子系统类，必须放在本包（{@code common.subsystem}）里，且是顶层类
 *       （生成的代码按简单名引用它，不支持嵌套类）。类上标 {@link GenerateSchedule}，
 *       可选地实现 {@link ScheduleLifecycle}。</li>
 *   <li>在本类里同时写两处：{@code public final XxxScheduler xxx;} 字段，以及构造函数里的
 *       {@code this.xxx = add(new XxxScheduler(new Xxx(robot)));}。</li>
 * </ol>
 *
 * <pre>{@code
 * // 类上加 GenerateSchedule，可选实现 ScheduleLifecycle
 * public class Shooter implements ScheduleLifecycle {
 *     private final DcMotorEx motor;
 *
 *     public Shooter(Robot robot) {
 *         motor = robot.hardwareMap.get(DcMotorEx.class, Globals.shooter);
 *         motor.setPower(0);                        // 构造时就处于安全状态
 *     }
 *
 *     public void spin(@Param(value = "power", defaultValue = "1") double power) { ... }
 *     public void stop() { ... }
 *     public double getRpm() { ... }                 // 有返回值：直接方法
 *
 *     public void onScheduleCancelled() { motor.setPower(0); }
 *     public void onShutdown() { motor.setPower(0); }
 * }
 * }</pre>
 *
 * <h2>方法的分类规则（由注解处理器 ScheduleProcessor 决定）</h2>
 * <ul>
 *   <li>public、非 static、返回 {@code void}：进入可链式调用的 {@code XxxSchedule} 接口。</li>
 *   <li>public、非 static、有返回值：直接方法，在调用线程上当场执行，不能排进调度，
 *       也不会作为远程指令暴露。只能在 OpMode 线程里调用，因为它直接碰硬件。</li>
 *   <li>加 {@link ScheduledAction}：强制进调度接口（返回值被丢弃）；
 *       加 {@link DirectAction}：强制作直接方法；两者不能同时用。</li>
 *   <li>private 或 static 方法被忽略。</li>
 *   <li>方法名不能是保留名：schedule、execute、cancel、waitMillis、waitUntil、isCancelled、
 *       isExecuted、update、shutdown、cancelSchedule、hasActiveSchedule、periodic、
 *       onScheduleCancelled、onShutdown。保留名的方法被忽略，标了上面两个注解则编译报错。</li>
 *   <li>方法不能带泛型参数或 {@code throws}，否则编译报错。</li>
 * </ul>
 *
 * <h2>会自动生成什么</h2>
 * 编译时每个 {@link GenerateSchedule} 类生成两个文件（在 {@code build/generated} 下，
 * 不进仓库；IDE 找不到时先 Build 一次）：
 * <ul>
 *   <li>{@code XxxSchedule}：继承 {@link ScheduleApi} 的接口。每个 void 方法对应一个返回自身的
 *       链式方法，参数带上 {@link org.firstinspires.ftc.teamcode.common.util.Param}
 *       的名称和默认值；另有 waitMillis、waitUntil、execute、cancel、isCancelled、isExecuted。</li>
 *   <li>{@code XxxScheduler}：继承 {@link Scheduler} 的类。直接方法原样转发；
 *       每个链式方法还有一个同签名的快捷方法，
 *       {@code xxx.spin(1)} 等价于 {@code xxx.schedule().spin(1).execute()}。</li>
 * </ul>
 * 注册进本类之后还会自动得到：
 * <ul>
 *   <li>远程指令：{@code GET /api/v2/commands} 里出现 {@code 字段名.方法名}（如
 *       {@code intaker.intakerIn}）。只暴露链式方法，参数类型限于基本类型、包装类和
 *       String，否则该方法不发布；带默认值的末尾参数可省略；同一指令的不同重载，
 *       可接受的参数个数范围不能重叠，否则该指令整体不发布。</li>
 *   <li>启动检查：调度器构造时会校验接口里每个方法在目标类里都存在。</li>
 *   <li>OpMode 结束清理：{@link #shutdown()} 取消所有调度并调用各子系统的
 *       {@link ScheduleLifecycle#onShutdown()}。</li>
 * </ul>
 *
 * <h2>如何使用</h2>
 * <pre>{@code
 * robot.subsystem.shooter.spin(0.8);                       // 快捷方式：提交一步调度
 * robot.subsystem.shooter.schedule()                       // 多步：按顺序执行，等待不阻塞
 *         .spin(1).waitMillis(500).waitUntil(() -> ready).stop().execute();
 * double rpm = robot.subsystem.shooter.getRpm();           // 直接方法：当场执行
 * robot.subsystem.shooter.cancelSchedule();                // 取消该子系统的调度
 * }</pre>
 * <ul>
 *   <li>{@code schedule()} 创建的草稿互不影响，也不会打断正在运行的调度；
 *       只有 {@code execute()} 才会提交，忘了写就永远不会执行。</li>
 *   <li>同一子系统后提交的覆盖先提交的（邮箱容量为 1），旧调度被取消并触发
 *       {@link ScheduleLifecycle#onScheduleCancelled()}；不同子系统互不影响。</li>
 *   <li>调度正常结束不会触发取消钩子，硬件会保持最后的输出。例如 intakerIn(1)
 *       执行完后马达仍在转，要自己调用停止方法。</li>
 *   <li>调度只在 {@link #update()} 里推进，每次最多推进 32 步。
 *       用 {@code RobotLinearOpMode.runWhileActive} 时每圈自动调用；自己写循环就要自己调用。</li>
 *   <li>{@code schedule()} 和 {@code execute()} 可在任意线程调用（只写邮箱），
 *       硬件只在 {@link #update()} 里被碰，所以远程指令可以安全接入。</li>
 *   <li>调度步骤里抛出的异常会从 {@link #update()} 传出，使 OpMode 循环崩溃。</li>
 * </ul>
 *
 * <h2>初始化时的注意事项</h2>
 * <ul>
 *   <li><b>所有用 {@code RobotLinearOpMode} 的 OpMode 都会创建本类</b>：它的
 *       {@code runOpMode()} 是 final，无条件调用 {@code robot.init}，而后者会 new 本类。
 *       新子系统在机器配置里缺设备，构造函数抛异常，所有 OpMode 的 INIT 都会失败，
 *       包括路径 OpMode 和各个测试 OpMode。</li>
 *   <li><b>构造函数里能用的对象有限</b>：{@code Robot.init} 的顺序是
 *       hardwareMap/telemetry → odo → 本类 → gamepad1/gamepad2 → odoDrivetrain。
 *       构造函数里可以用 {@code robot.hardwareMap}、{@code telemetry}、{@code opMode}、
 *       {@code odo}；不能用 {@code gamepad1}、{@code gamepad2}、{@code odoDrivetrain}
 *       （此时为 null），{@code vision}、{@code limelight} 等也是 null。</li>
 *   <li><b>忘了 {@code add()} 会静默失效</b>：只有通过 {@code add()} 注册的调度器才会被
 *       {@link #update()}、{@link #cancelAllSchedules()}、{@link #shutdown()} 处理。
 *       没注册的调度永远不会推进，OpMode 结束时也不会被关闭。字段必须是
 *       {@code public final}，否则远程指令目录里看不到它。</li>
 *   <li><b>构造失败时不会执行 shutdown</b>：{@code robot.subsystem} 要等本类构造完整返回
 *       才赋值。构造到一半抛异常，{@code RobotLinearOpMode} 的 finally 会跳过
 *       {@code shutdown()}，已创建的子系统不会被关闭，所以每个构造函数都应在返回前
 *       把输出置于安全状态；某个子系统出错也会阻止排在它后面的子系统创建。</li>
 *   <li><b>构造函数不要等待</b>：构造发生在 INIT 阶段，{@code opModeIsActive()} 为假，
 *       {@code Robot.waitFor()} 的循环会立即返回，等于没等。需要等待请用调度的
 *       {@code waitMillis}/{@code waitUntil}，或用 {@code opModeInInit()} 自己写循环。</li>
 *   <li><b>初始状态要安全</b>：构造函数里设好方向、模式并停止输出。舵机在构造函数里
 *       设位置，会在按下 INIT 时就动起来。</li>
 *   <li><b>一个设备只归一个子系统</b>：{@code hardwareMap.get} 返回同一个对象。
 *       两个子系统共用一个马达，它们的调度互不知情，不会按“后者覆盖前者”协调。
 *       底盘马达归 {@code OdoDrivetrain}，不要再给子系统。</li>
 *   <li><b>每个 OpMode 都是全新实例</b>：每次 {@code robot.init} 都新建本类，
 *       字段状态不会跨 OpMode 保留，需要保留的只能放静态状态（如 {@code Globals}）。</li>
 *   <li><b>远程指令目录与构造无关</b>：目录在启动时只靠类元数据扫描本类生成，
 *       不会运行构造函数。目录里有某条指令，不代表当前 OpMode 里该子系统已构造成功。</li>
 *   <li><b>可测试性</b>：构造函数要读硬件，单元测试里无法创建真实子系统。
 *       测试调度行为请用不依赖硬件的假目标类，见 {@code GeneratedSchedulerTest}。</li>
 * </ul>
 *
 * <h2>新增子系统检查清单</h2>
 * <ol>
 *   <li>在 {@code Globals} 加配置名，并在机器配置里加设备。</li>
 *   <li>类放在本包，顶层类，标 {@link GenerateSchedule}。</li>
 *   <li>构造函数只用 hardwareMap、telemetry、odo，把输出置于安全状态，不阻塞等待。</li>
 *   <li>在本类里同时写 {@code public final XxxScheduler xxx;} 和
 *       {@code this.xxx = add(new XxxScheduler(new Xxx(robot)));}。</li>
 *   <li>用 {@code onScheduleCancelled()} 和 {@code onShutdown()} 让设备在被取消或
 *       OpMode 结束时停下来。</li>
 * </ol>
 */
public class Subsystem {
    private final Robot robot;

    private final List<Scheduler<?>> all = new ArrayList<>();
    public final IntakerScheduler intaker;

    private <T extends Scheduler<?>> T add(T instance) {
        all.add(instance);
        return instance;
    }

    public Subsystem(Robot robot) {
        this.robot = robot;
        this.intaker = add(new IntakerScheduler(new Intaker(robot)));
    }

    /** Advance every subsystem scheduler once from the OpMode/control thread. */
    public void update() {
        for (Scheduler<?> s : all) s.update();
    }

    /** Invalidate all draft, pending and active subsystem schedules. */
    public void cancelAllSchedules() {
        for (Scheduler<?> s : all) s.cancelSchedule();
    }

    /** Cancel work and stop the intake even when its last action has already finished. */
    public void shutdown() {
        for (Scheduler<?> s : all) s.shutdown();
    }
}
