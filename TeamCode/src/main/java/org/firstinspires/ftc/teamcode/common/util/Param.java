package org.firstinspires.ftc.teamcode.common.util;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * 为方法参数指定名称和可选的默认值，供运行时反射获取。
 * <p>
 * Control Hub (Java 8 / Android API &lt; 26) 不支持
 * {@code java.lang.reflect.Parameter.getName()}，
 * 使用此注解可替代编译期 {@code -parameters} 标志。
 *
 * <p>标在 {@code @GenerateSchedule} 子系统的 public 方法参数上时，注解处理器会把名称和默认值
 * 复制到生成的 {@code *Schedule} 接口（没有标注的参数也会带上 Java 参数名），
 * 远程指令目录从那里读取。{@link #value()} 省略时使用 Java 参数名。
 *
 * <pre>{@code
 * public void intakerIn(@Param(value = "power", defaultValue = "1") double power) { ... }
 * public void shoot(int count, @Param(defaultValue = "0.8") double power) { ... }
 * }</pre>
 *
 * <p>带默认值的参数是可选的：远程调用可以省略它们，由默认值补上。参数按位置传递，
 * 所以带默认值的参数必须是<b>末尾连续</b>的几个，否则编译期报错。默认值写成字符串，
 * 按参数类型用与远程调用相同的规则转换（例如 {@code double} 写 {@code "0.8"}，
 * {@code boolean} 只能是 {@code "true"}/{@code "false"}）。
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.PARAMETER)
public @interface Param {
    /** {@link #defaultValue()} 的缺省值，表示该参数没有默认值（必填）。 */
    String NO_DEFAULT = "\u0000";

    /** 参数名称；省略时使用 Java 参数名。 */
    String value() default "";

    /** 参数的默认值（字符串形式）；缺省表示必填。空字符串是合法的默认值。 */
    String defaultValue() default NO_DEFAULT;
}
