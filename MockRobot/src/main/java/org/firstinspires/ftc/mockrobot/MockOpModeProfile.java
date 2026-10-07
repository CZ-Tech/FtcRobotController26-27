package org.firstinspires.ftc.mockrobot;

/** One simulated Autonomous entry exposed through the real /api/v2/opmodes endpoint. */
public final class MockOpModeProfile {
    public final String name;
    public final String group;
    private volatile String routeName;
    private volatile boolean autoStop;

    public MockOpModeProfile(String name, String group, String routeName, boolean autoStop) {
        this.name = name;
        this.group = group;
        this.routeName = routeName;
        this.autoStop = autoStop;
    }

    public String routeName() { return routeName; }
    public boolean autoStop() { return autoStop; }
    public void setRouteName(String value) { routeName = value; }
    public void setAutoStop(boolean value) { autoStop = value; }

    @Override
    public String toString() {
        return name;
    }
}
