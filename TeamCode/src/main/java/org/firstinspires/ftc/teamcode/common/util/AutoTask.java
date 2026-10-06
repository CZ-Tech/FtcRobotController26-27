package org.firstinspires.ftc.teamcode.common.util;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks a method or class as a legacy autonomous JSON task.
 * <p>
 * <b>On a method:</b> Registers that method as a task with the given name (or the method name
 * if {@link #value()} is empty). The method may accept any combination of primitive types and
 * String as parameters.
 * <p>
 * <b>On a class:</b> All public methods of the class are registered as tasks.
 *
 * <p>This annotation is no longer exposed through HTTP. It is retained only for the
 * current season's hard-coded JSON autonomous paths.</p>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.METHOD, ElementType.TYPE})
public @interface AutoTask {
    /** Custom task name. If empty, the method name is used. */
    String value() default "";
}
