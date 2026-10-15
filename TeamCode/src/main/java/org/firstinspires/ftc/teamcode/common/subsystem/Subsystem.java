package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.Robot;
import java.util.ArrayList;
import java.util.List;

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
