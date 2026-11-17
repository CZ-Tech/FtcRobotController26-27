package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.junit.Test;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import static org.junit.Assert.*;

/** Actual synchronous intake methods with a fake SDK motor; no Control Hub required. */
public class IntakerFlowTest {
    private static Intaker intake(List<Double> powers, CountDownLatch powered) {
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPower")) {
                        powers.add((Double) args[0]);
                        if ((Double) args[0] != 0) powered.countDown();
                    }
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
        Intaker intake = new Intaker(robot);
        powers.clear();
        return intake;
    }

    @Test public void ordinaryMethodWritesImmediatelyWithoutAnUpdate() {
        List<Double> powers = new CopyOnWriteArrayList<>();
        Intaker intake = intake(powers, new CountDownLatch(1));
        intake.intakerIn(0.7);
        assertEquals(Arrays.asList(0.7), powers);
    }

    @Test public void timedMethodCompletesItsStopBeforeReturning() throws Exception {
        List<Double> powers = new CopyOnWriteArrayList<>();
        Intaker intake = intake(powers, new CountDownLatch(1));
        intake.setIntakerPower(0.8, 0);
        assertEquals(Arrays.asList(0.8, 0.0), powers);
    }

    @Test(timeout = 5000) public void cancellationInterruptsWaitAndLifecycleOwnsFinalStop() throws Exception {
        List<Double> powers = new CopyOnWriteArrayList<>();
        CountDownLatch powered = new CountDownLatch(1);
        Intaker intake = intake(powers, powered);
        CommandTasks tasks = new CommandTasks(() -> true);
        AtomicBoolean tail = new AtomicBoolean();
        AtomicBoolean interrupted = new AtomicBoolean();
        try {
            tasks.execute(() -> {
                try {
                    intake.setIntakerPower(0.8, 60_000);
                    tail.set(true);
                } catch (InterruptedException e) {
                    interrupted.set(true);
                    Thread.currentThread().interrupt();
                }
            });
            assertTrue(powered.await(2, TimeUnit.SECONDS));
            assertFalse(tasks.isIdle());
            tasks.shutdown();
            assertTrue(interrupted.get());
            assertFalse(tail.get());
            assertEquals(Arrays.asList(0.8), powers);
            intake.onShutdown();
            assertEquals(Arrays.asList(0.8, 0.0), powers);
            tasks.throwIfFailed();
        } finally {
            tasks.shutdown();
            intake.onShutdown();
        }
    }

    @Test public void badDurationDoesNotTouchMotor() {
        List<Double> powers = new CopyOnWriteArrayList<>();
        Intaker intake = intake(powers, new CountDownLatch(1));
        assertThrows(IllegalArgumentException.class, () -> intake.setIntakerPower(1, -1));
        assertTrue(powers.isEmpty());
    }
}
