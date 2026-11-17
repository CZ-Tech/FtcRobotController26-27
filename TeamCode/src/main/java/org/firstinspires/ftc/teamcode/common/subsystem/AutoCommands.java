package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.network.RemoteCommand;
import java.util.Objects;

/** Ordinary cross-subsystem operations; no generated wrapper or implicit scheduling. */
public final class AutoCommands {
    private final Intaker intaker;
    private final Shooter shooter;
    private final Runnable clearFlows;

    public AutoCommands(Intaker intaker, Shooter shooter, Runnable clearFlows) {
        this.intaker = Objects.requireNonNull(intaker, "intaker");
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.clearFlows = Objects.requireNonNull(clearFlows, "clearFlows");
    }

    /** Stop optional flows first so a delayed callback cannot restart the devices. */
    @RemoteCommand
    public void stopAll() {
        clearFlows.run();
        try { intaker.intakerArmStop(); }
        finally { shooter.stopShooter(); }
    }
}
