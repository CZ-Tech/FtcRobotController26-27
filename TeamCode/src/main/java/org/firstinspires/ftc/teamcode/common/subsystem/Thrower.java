package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.Robot;

public class Thrower extends ScheduledSubsystem<Thrower.Schedule> {
    public interface Schedule extends ScheduleApi<Schedule> {}

    private final Robot robot;

    public Thrower(Robot robot) {
        super(Schedule.class);
        this.robot = robot;
    }
}
