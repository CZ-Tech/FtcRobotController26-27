package org.firstinspires.ftc.teamcode.common.subsystem;

/** Optional hooks, called only on the OpMode/control thread. */
public interface SubsystemLifecycle {
    default void periodic() {}
    default void onShutdown() {}
}
