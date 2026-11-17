package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.junit.Test;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;

/** Actual intake methods with a fake SDK motor; no Control Hub is required. */
public class IntakerFlowTest {
    private static Intaker intake(SubsystemLoop loop, List<Double> powers) {
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPower")) powers.add((Double) args[0]);
                    if (method.getName().equals("hashCode")) return System.identityHashCode(proxy);
                    if (method.getName().equals("equals")) return proxy == args[0];
                    Class<?> type = method.getReturnType();
                    if (type == boolean.class) return false;
                    if (type == double.class) return 0.0;
                    if (type == float.class) return 0.0f;
                    if (type == long.class) return 0L;
                    if (type == int.class) return 0;
                    return null;
                });
        Robot robot = new Robot();
        robot.hardwareMap = new HardwareMap(null, null) {
            @Override public <T> T get(Class<? extends T> type, String name) {
                if (!Globals.intaker.equals(name)) throw new IllegalArgumentException(name);
                return type.cast(motor);
            }
        };
        Intaker intake = new Intaker(robot, loop::start);
        powers.clear(); // Constructor explicitly stops the motor.
        return intake;
    }

    @Test public void ordinaryMethodWritesImmediatelyWithoutAnUpdate() {
        SubsystemLoop loop = new SubsystemLoop();
        List<Double> powers = new ArrayList<>();
        Intaker intake = intake(loop, powers);
        intake.intakerIn(0.7);
        assertEquals(Arrays.asList(0.7), powers);
    }

    @Test public void timedMethodKeepsPowerUntilItsDelayExpires() {
        AtomicLong clock = new AtomicLong();
        SubsystemLoop loop = new SubsystemLoop(clock::get);
        List<Double> powers = new ArrayList<>();
        Intaker intake = intake(loop, powers);
        intake.setIntakerPower(0.8, 100);
        assertEquals(Arrays.asList(0.8), powers);
        loop.update();
        clock.set(99_999_999L);
        loop.update();
        assertEquals(Arrays.asList(0.8), powers);
        clock.set(100_000_000L);
        loop.update();
        assertEquals(Arrays.asList(0.8, 0.0), powers);
    }

    @Test public void queuedTimedStartThenStopDoesNotRestartTheMotor() {
        AtomicLong clock = new AtomicLong();
        SubsystemLoop loop = new SubsystemLoop(clock::get);
        List<Double> powers = new ArrayList<>();
        Intaker intake = intake(loop, powers);
        loop.enqueue(() -> intake.setIntakerPower(1, 500));
        loop.enqueue(intake::intakerArmStop);
        loop.update();
        assertEquals(Arrays.asList(1.0, 0.0), powers);
        clock.set(500_000_000L);
        loop.update();
        assertEquals(Arrays.asList(1.0, 0.0, 0.0), powers);
    }

    @Test public void badDurationDoesNotTouchMotor() {
        SubsystemLoop loop = new SubsystemLoop();
        List<Double> powers = new ArrayList<>();
        Intaker intake = intake(loop, powers);
        assertThrows(IllegalArgumentException.class, () -> intake.setIntakerPower(1, -1));
        assertTrue(powers.isEmpty());
    }
}
