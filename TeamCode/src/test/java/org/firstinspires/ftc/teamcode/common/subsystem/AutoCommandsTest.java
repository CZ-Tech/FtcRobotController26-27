package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.Assert.*;

/** Real aggregate/subsystem methods with fake SDK devices; no control loop or hub required. */
public class AutoCommandsTest {
    private static final long TEST_TIMEOUT_MILLIS = 8000;
    private static final double OPEN = 0.75;
    private static final double CLOSED = 0.25;

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void ordinaryShootRunsSynchronouslyInOrderAndClosesValveAndStopsMotor() throws Exception {
        Fixture fixture = new Fixture();
        try {
            fixture.auto.shoot(6000, 0, 0);
            assertEquals(Arrays.asList("valve:0.25", "shooter.velocity:2800.0", "valve:0.75",
                    "valve:0.25", "shooter.power:0.0"), fixture.events);
            for (Thread writer : fixture.writers) assertSame(Thread.currentThread(), writer);
            assertEquals(CLOSED, fixture.valvePosition, 0);
            assertEquals(0, fixture.shooterPower, 0);
        } finally {
            fixture.close();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void invalidShootParametersDoNotTouchHardware() {
        Fixture fixture = new Fixture();
        try {
            assertThrows(IllegalArgumentException.class, () -> fixture.auto.shoot(-1, 0, 0));
            assertThrows(IllegalArgumentException.class, () -> fixture.auto.shoot(Double.NaN, 0, 0));
            assertThrows(IllegalArgumentException.class,
                    () -> fixture.auto.shoot(Double.POSITIVE_INFINITY, 0, 0));
            assertThrows(IllegalArgumentException.class,
                    () -> fixture.auto.shoot(Double.NEGATIVE_INFINITY, 0, 0));
            assertThrows(IllegalArgumentException.class, () -> fixture.auto.shoot(6000, -1, 0));
            assertThrows(IllegalArgumentException.class, () -> fixture.auto.shoot(6000, 0, -1));
            assertTrue(fixture.events.isEmpty());
            assertTrue(fixture.writers.isEmpty());
        } finally {
            fixture.close();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void waitMillisRejectsNegativeAndZeroReturnsWithoutHardwareWork() throws Exception {
        Fixture fixture = new Fixture();
        try {
            assertThrows(IllegalArgumentException.class, () -> fixture.auto.waitMillis(-1));
            fixture.auto.waitMillis(0);
            assertTrue(fixture.events.isEmpty());
            assertTrue(fixture.writers.isEmpty());
        } finally {
            fixture.close();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void stopAllCancelsFirstThenStopsIntakeClosesValveAndStopsShooter() {
        Fixture fixture = new Fixture();
        try {
            fixture.primeHardware();
            fixture.auto.stopAll();
            assertEquals(Arrays.asList("cancel", "intaker.power:0.0", "valve:0.25",
                    "shooter.power:0.0"), fixture.events);
            for (Thread writer : fixture.writers) assertSame(Thread.currentThread(), writer);
            assertEquals(0, fixture.intakePower, 0);
            assertEquals(0, fixture.shooterPower, 0);
            assertEquals(CLOSED, fixture.valvePosition, 0);
        } finally {
            fixture.close();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void stopAllStillClosesValveAndStopsShooterWhenIntakeStopFails() {
        Fixture fixture = new Fixture();
        RuntimeException failure = new RuntimeException("intake stop failed");
        try {
            fixture.primeHardware();
            fixture.intakeStopFailure = failure;
            assertSame(failure, assertThrows(RuntimeException.class, fixture.auto::stopAll));
            assertEquals(Arrays.asList("cancel", "intaker.power:0.0", "valve:0.25",
                    "shooter.power:0.0"), fixture.events);
            assertEquals(CLOSED, fixture.valvePosition, 0);
            assertEquals(0, fixture.shooterPower, 0);
        } finally {
            fixture.close();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void stopAllStillStopsShooterWhenClosingValveFails() {
        Fixture fixture = new Fixture();
        RuntimeException failure = new RuntimeException("valve close failed");
        try {
            fixture.primeHardware();
            fixture.valveCloseFailure = failure;
            assertSame(failure, assertThrows(RuntimeException.class, fixture.auto::stopAll));
            assertEquals(Arrays.asList("cancel", "intaker.power:0.0", "valve:0.25",
                    "shooter.power:0.0"), fixture.events);
            assertEquals(0, fixture.intakePower, 0);
            assertEquals(0, fixture.shooterPower, 0);
        } finally {
            fixture.close();
        }
    }

    @Test(timeout = TEST_TIMEOUT_MILLIS)
    public void stopAllCancelsOpenValveShootWithoutWorkerTailAndOwnerCanFinallyStop() throws Exception {
        CommandTasks tasks = new CommandTasks(() -> true);
        Fixture fixture = new Fixture(tasks::cancelOthers);
        AtomicBoolean tailRan = new AtomicBoolean();
        AtomicBoolean interrupted = new AtomicBoolean();
        try {
            tasks.execute(() -> {
                try {
                    fixture.auto.shoot(6000, 0, 60_000);
                    tailRan.set(true);
                } catch (InterruptedException cancelled) {
                    interrupted.set(true);
                    Thread.currentThread().interrupt();
                }
            });
            assertTrue("shoot must open the valve before cancellation",
                    fixture.valveOpened.await(2, TimeUnit.SECONDS));
            assertFalse(tasks.isIdle());
            fixture.auto.stopAll();
            tasks.shutdown();
            assertTrue(interrupted.get());
            assertFalse(tailRan.get());
            assertTrue(tasks.isIdle());
            tasks.throwIfFailed();
            assertEquals(Arrays.asList("valve:0.25", "shooter.velocity:2800.0", "valve:0.75",
                    "cancel", "intaker.power:0.0", "valve:0.25", "shooter.power:0.0"),
                    fixture.events);
            assertEquals(CLOSED, fixture.valvePosition, 0);
            assertEquals(0, fixture.intakePower, 0);
            assertEquals(0, fixture.shooterPower, 0);
            for (int i = 0; i < 3; i++) assertNotSame(Thread.currentThread(), fixture.writers.get(i));
            for (int i = 3; i < fixture.writers.size(); i++) {
                assertSame(Thread.currentThread(), fixture.writers.get(i));
            }
            // Lifecycle cleanup is an ordinary owner-thread call, even after command cancellation.
            fixture.shooter.onShutdown();
            assertEquals(Arrays.asList("valve:0.25", "shooter.velocity:2800.0", "valve:0.75",
                    "cancel", "intaker.power:0.0", "valve:0.25", "shooter.power:0.0",
                    "shooter.power:0.0", "valve:0.25"), fixture.events);
        } finally {
            try {
                tasks.shutdown();
            } finally {
                fixture.close();
            }
        }
    }

    private static final class Fixture {
        final List<String> events = new CopyOnWriteArrayList<>();
        final List<Thread> writers = new CopyOnWriteArrayList<>();
        final CountDownLatch valveOpened = new CountDownLatch(1);
        final Intaker intaker;
        final Shooter shooter;
        final AutoCommands auto;
        volatile double intakePower;
        volatile double shooterPower;
        volatile double valvePosition;
        volatile RuntimeException intakeStopFailure;
        volatile RuntimeException valveCloseFailure;

        Fixture() { this(() -> {}); }

        Fixture(Runnable cancelWork) {
            DcMotorEx intakeMotor = motor("intaker");
            DcMotorEx shooterMotor = motor("shooter");
            ServoControllerEx controller = (ServoControllerEx) Proxy.newProxyInstance(
                    ServoControllerEx.class.getClassLoader(), new Class<?>[]{ServoControllerEx.class},
                    (proxy, method, args) -> {
                        if (method.getName().equals("setServoPosition")) {
                            double position = (Double) args[1];
                            record("valve:" + position);
                            if (position == CLOSED && valveCloseFailure != null) throw valveCloseFailure;
                            valvePosition = position;
                            if (position == OPEN) valveOpened.countDown();
                        }
                        if (method.getName().equals("getServoPosition")) return valvePosition;
                        return defaultValue(proxy, method, args);
                    });
            // SDK 12 requires an explicit type; the fake controller does not use its PWM metadata.
            ServoImplEx valve = new ServoImplEx(controller, 0, new ServoConfigurationType());
            Robot robot = new Robot();
            robot.hardwareMap = new HardwareMap(null, null) {
                @Override public <T> T get(Class<? extends T> type, String name) {
                    if (Globals.intaker.equals(name)) return type.cast(intakeMotor);
                    if (Globals.Shooter.equals(name)) return type.cast(shooterMotor);
                    if (Globals.valve.equals(name)) return type.cast(valve);
                    throw new IllegalArgumentException("unexpected hardware: " + name);
                }
            };
            intaker = new Intaker(robot);
            shooter = new Shooter(robot);
            shooter.valveOpenDegree = OPEN;
            shooter.valveCloseDegree = CLOSED;
            auto = new AutoCommands(intaker, shooter, () -> {
                record("cancel");
                cancelWork.run();
            });
            clearEvents();
        }

        private DcMotorEx motor(String name) {
            return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                        if (method.getName().equals("setPower")) {
                            double power = (Double) args[0];
                            record(name + ".power:" + power);
                            if (name.equals("intaker")) {
                                if (power == 0 && intakeStopFailure != null) throw intakeStopFailure;
                                intakePower = power;
                            } else {
                                shooterPower = power;
                            }
                        }
                        if (method.getName().equals("setVelocity")) {
                            record(name + ".velocity:" + args[0]);
                        }
                        return defaultValue(proxy, method, args);
                    });
        }

        private void record(String event) {
            events.add(event);
            writers.add(Thread.currentThread());
        }

        private void clearEvents() {
            events.clear();
            writers.clear();
        }

        void primeHardware() {
            intaker.setIntakePower(0.7);
            shooter.setPower(0.8);
            shooter.setValveOpen();
            clearEvents();
        }

        void close() {
            intakeStopFailure = null;
            valveCloseFailure = null;
            try {
                intaker.onShutdown();
            } finally {
                shooter.onShutdown();
            }
        }
    }

    private static Object defaultValue(Object proxy, Method method, Object[] args) {
        if (method.getName().equals("hashCode")) return System.identityHashCode(proxy);
        if (method.getName().equals("equals")) return proxy == args[0];
        if (method.getName().equals("toString")) return "fake SDK device";
        Class<?> type = method.getReturnType();
        if (type == boolean.class) return false;
        if (type == double.class) return 0.0;
        if (type == float.class) return 0.0f;
        if (type == long.class) return 0L;
        if (type == int.class) return 0;
        if (type == short.class) return (short) 0;
        if (type == byte.class) return (byte) 0;
        if (type == char.class) return '\0';
        return null;
    }
}
