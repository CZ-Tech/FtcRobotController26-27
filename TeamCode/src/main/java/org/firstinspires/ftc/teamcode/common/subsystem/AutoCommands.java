package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.network.RemoteCommand;
import org.firstinspires.ftc.teamcode.common.util.Param;
import java.util.Objects;

/** Aggregate commands are ordinary Java: direct subsystem calls, sleep and polling. */
public final class AutoCommands {
    private final Intaker intaker;
    private final Shooter shooter;
    private final Runnable cancelWork;

    public AutoCommands(Intaker intaker, Shooter shooter, Runnable cancelWork) {
        this.intaker = Objects.requireNonNull(intaker, "intaker");
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.cancelWork = Objects.requireNonNull(cancelWork, "cancelWork");
    }

    /** An ordinary command, useful between calls in a waypoint list; milliseconds. */
    @RemoteCommand
    public void waitMillis(@Param("millis") long millis) throws InterruptedException {
        if (millis < 0) throw new IllegalArgumentException("millis must be non-negative");
        CommandTasks.checkCancelled();
        Thread.sleep(millis);
    }

    /** Example aggregate: spin up, feed for a bounded time, then close and stop. */
    @RemoteCommand
    public void shoot(@Param("rpm") double rpm,
                      @Param("spinupMillis") long spinupMillis,
                      @Param("feedMillis") long feedMillis) throws InterruptedException {
        if (!Double.isFinite(rpm) || rpm < 0 || spinupMillis < 0 || feedMillis < 0) {
            throw new IllegalArgumentException("rpm and durations must be finite/non-negative");
        }
        shooter.setValveClose();
        try {
            shooter.setTargetVelocity(rpm);
            Thread.sleep(spinupMillis);
            shooter.setValveOpen();
            Thread.sleep(feedMillis);
        } finally {
            if (!CommandTasks.isCancelled()) {
                try { shooter.setValveClose(); }
                finally { shooter.stopShooter(); }
            }
        }
    }

    /** Cancel sibling tasks/legacy flows before stopping intake and shooter, not the route. */
    @RemoteCommand
    public void stopAll() {
        CommandTasks.checkCancelled();
        cancelWork.run();
        try { intaker.intakerArmStop(); }
        finally {
            try { shooter.setValveClose(); }
            finally { shooter.stopShooter(); }
        }
    }
}
