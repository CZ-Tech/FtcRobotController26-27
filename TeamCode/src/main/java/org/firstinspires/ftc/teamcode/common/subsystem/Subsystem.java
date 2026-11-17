package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.Robot;

/**
 * Plain hardware-owning objects. Published commands run on managed task threads;
 * direct calls remain synchronous on their caller. No device arbitration is implied.
 * Aggregate commands are ordinary Java methods and may sleep or poll interruptibly.
 * Optional legacy ActionSequences and periodic hooks still advance in update().
 */
public final class Subsystem {
    private final SubsystemLoop loop;
    private final CommandTasks tasks;
    public final Intaker intaker;
    public final Shooter shooter;
    public final Puller puller;
    public final AutoCommands auto;

    public Subsystem(Robot robot) {
        tasks = new CommandTasks(() -> !robot.opMode.isStopRequested());
        loop = new SubsystemLoop(System::nanoTime, () -> !robot.opMode.isStopRequested());
        try {
            intaker = loop.add(new Intaker(robot));
            shooter = loop.add(new Shooter(robot));
            puller = loop.add(new Puller(robot));
            auto = new AutoCommands(intaker, shooter, this::cancelWork);
        } catch (RuntimeException | Error e) {
            try { shutdown(); }
            catch (RuntimeException | Error cleanup) {
                if (cleanup != e) e.addSuppressed(cleanup);
            }
            throw e;
        }
    }

    /** One external command or waypoint group becomes one managed task thread. */
    public void execute(Runnable action) {
        CommandTasks.checkCancelled();
        tasks.execute(action);
    }

    /** Compatibility ingress; unlike the old FIFO this now starts a managed task. */
    public void enqueue(Runnable action) { execute(action); }

    /** Optional legacy non-blocking flow; new aggregate commands need only direct calls. */
    public void start(ActionSequence sequence) {
        CommandTasks.checkCancelled();
        loop.start(sequence);
    }

    public void checkCommandFailure() { tasks.throwIfFailed(); }

    /** Seal external ingress at path completion while existing tasks finish normally. */
    public void finishCommands() { tasks.seal(); }

    public boolean commandsIdle() { return tasks.isIdle() && loop.isIdle(); }

    public void update() {
        tasks.throwIfFailed();
        loop.update();
        tasks.throwIfFailed();
    }

    /** Discard only legacy flows, not ordinary task threads or hardware output. */
    public void clearFlows() { loop.clearFlows(); }

    /** Used by stopAll: cancel sibling command tasks and legacy flows, not this caller. */
    private void cancelWork() {
        tasks.cancelOthers();
        loop.clearFlows();
    }

    /** Join cancelled tasks before stopping devices; always attempt every shutdown hook. */
    public void shutdown() {
        Throwable failure = null;
        try { tasks.shutdown(); }
        catch (RuntimeException | Error e) { failure = e; }
        try { loop.shutdown(); }
        catch (RuntimeException | Error e) {
            if (failure == null) failure = e;
            else if (failure != e) failure.addSuppressed(e);
        }
        if (failure instanceof RuntimeException) throw (RuntimeException) failure;
        if (failure instanceof Error) throw (Error) failure;
    }
}
