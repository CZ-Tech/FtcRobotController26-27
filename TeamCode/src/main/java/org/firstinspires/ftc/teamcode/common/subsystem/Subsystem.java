package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.Robot;

/**
 * Plain hardware-owning objects, not scheduler wrappers.
 *
 * <p>Call subsystem methods directly on the OpMode/control thread. Only remote calls
 * need {@link #enqueue(Runnable)}; they run FIFO during {@link #update()} and never
 * supersede one another. Mark published methods with
 * {@link org.firstinspires.ftc.teamcode.common.network.RemoteCommand}.</p>
 *
 * <p>For non-blocking multi-step work, optionally use:</p>
 * <pre>{@code
 * robot.subsystem.start(ActionSequence.create()
 *     .run(() -> robot.subsystem.intaker.intakerIn())
 *     .waitMillis(500)
 *     .run(() -> robot.subsystem.intaker.intakerArmStop()));
 * }</pre>
 * <p>Flows are independent; no ownership, priority or automatic conflict handling.
 * Callers are responsible for the order of writes to a shared device. Do not sleep
 * in actions. Waits belong between callbacks, never inside a hardware method.</p>
 *
 * <p>New hardware subsystem: implement optional {@link SubsystemLifecycle} hooks,
 * add a public final field and construct it with {@code loop.add(new Xxx(robot))}.
 * Constructors run in INIT, must be non-blocking and leave devices safe. At that
 * point Robot gamepads/drivetrain may not exist yet. No annotations or generated
 * classes are required for local use.</p>
 */
public final class Subsystem {
    private final SubsystemLoop loop;
    public final Intaker intaker;
    public final Shooter shooter;
    public final Puller puller;
    public final AutoCommands auto;

    public Subsystem(Robot robot) {
        loop = new SubsystemLoop(System::nanoTime, () -> !robot.opMode.isStopRequested());
        try {
            intaker = loop.add(new Intaker(robot, this::start));
            shooter = loop.add(new Shooter(robot));
            puller = loop.add(new Puller(robot));
            auto = new AutoCommands(intaker, shooter, this::clearFlows);
        } catch (RuntimeException | Error e) {
            try { loop.shutdown(); }
            catch (RuntimeException | Error cleanup) {
                if (cleanup != e) e.addSuppressed(cleanup);
            }
            throw e;
        }
    }

    /** Thread-safe ingress for remote calls. Rejected after shutdown. */
    public void enqueue(Runnable action) { loop.enqueue(action); }

    /** Submit a single-use optional flow. Does not replace any previous work. */
    public void start(ActionSequence sequence) { loop.start(sequence); }

    /** Run FIFO calls, optional flows and periodic hooks once on the control thread. */
    public void update() { loop.update(); }

    /** Explicitly stop flows, without changing hardware outputs. Control thread only. */
    public void clearFlows() { loop.clearFlows(); }

    /** Discard queued work and flows, then safely stop every registered device. */
    public void shutdown() { loop.shutdown(); }
}
