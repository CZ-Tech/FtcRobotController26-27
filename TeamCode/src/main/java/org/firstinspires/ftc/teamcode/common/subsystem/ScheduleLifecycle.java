package org.firstinspires.ftc.teamcode.common.subsystem;

import java.util.function.BooleanSupplier;

/**
 * Hooks always invoked on the subsystem control thread.
 *
 * <h2>在子系统方法里等待</h2>
 * {@link #waitMillis} / {@link #waitUntil} 与外面链式调用的 {@code .waitMillis()} /
 * {@code .waitUntil()} 等价：在当前调度里，紧跟在这个方法之后插入一个等待步骤。
 * 不 sleep、不阻塞，等待期间 OpMode 循环照常运行。
 * <pre>{@code
 * // 外面：shooter.schedule().kick().waitMillis(500).retract().execute();
 * // 里面：kick() 自带等待，之后只需
 * //       shooter.schedule().kick().retract().execute();   // retract 在 kick 之后 500ms 才执行
 * public void kick() {
 *     pusher.setPosition(OUT);
 *     waitMillis(500);
 * }
 * }</pre>
 * 注意：
 * <ul>
 *   <li>延后的是<b>这个方法之后的调度步骤</b>，不是方法体里 {@code waitMillis} 后面的代码——
 *       那些代码仍会立刻执行（方法本身不阻塞）。</li>
 *   <li>同一个方法里多次调用，按调用顺序累加。</li>
 *   <li>等待是调度里的一个步骤，随调度一起被覆盖/取消。</li>
 *   <li>只能在“被自己的调度执行的子系统方法”里调用；在 {@code periodic()}、生命周期钩子、
 *       其他线程、其他子系统的方法里调用会抛 {@link IllegalStateException}。</li>
 * </ul>
 */
public interface ScheduleLifecycle {
    default void periodic() {}
    default void onScheduleCancelled() {}
    default void onShutdown() {}

    /** 当前调度里，在本方法之后插入 {@code millis} 毫秒的等待。不阻塞。 */
    default void waitMillis(long millis) {
        Scheduler.insertWaitMillis(this, millis);
    }

    /** 当前调度里，在本方法之后插入“等条件成立”的等待。不阻塞。 */
    default void waitUntil(BooleanSupplier condition) {
        Scheduler.insertWaitUntil(this, condition);
    }
}
