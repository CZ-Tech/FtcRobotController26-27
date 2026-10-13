package org.firstinspires.ftc.teamcode.common.subsystem;

/** Hooks always invoked on the subsystem control thread. */
public interface ScheduleLifecycle {
    default void periodic() {}
    default void onScheduleCancelled() {}
    default void onShutdown() {}
}
