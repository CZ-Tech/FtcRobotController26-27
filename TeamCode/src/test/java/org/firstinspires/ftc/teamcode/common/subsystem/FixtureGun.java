package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.util.Param;

/**
 * Hardware-free subsystem used to test the code {@code ScheduleProcessor} generates.
 * Must be a top-level class: the generated sources refer to it by simple name.
 */
@GenerateSchedule
public class FixtureGun implements ScheduleLifecycle {
    int count;
    double power;
    int boosted;
    int cancelledCount;

    public void shoot(int count, @Param(defaultValue = "0.8") double power) {
        this.count = count;
        this.power = power;
    }

    public void stop() {
        count = 0;
    }

    /** Non-void, so a direct method: runs immediately on the calling thread. */
    public double rpm() {
        return 42;
    }

    /** Non-void but forced into the schedule chain. */
    @ScheduledAction
    public int boost(int amount) {
        boosted += amount;
        return boosted;
    }

    @Override
    public void onScheduleCancelled() {
        cancelledCount++;
    }
}
