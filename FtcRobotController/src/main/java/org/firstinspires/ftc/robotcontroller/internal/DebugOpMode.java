package org.firstinspires.ftc.robotcontroller.internal;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * <p>This annotation can be used to describe an OpMode as optional for normal match conditions.</p>
 * <p>OpModes using this annotation can be quickly enabled/disabled inside {@link org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister FtcOpModeRegister}.</p>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface DebugOpMode {
    String name() default "";

    String group() default OpModeMeta.DefaultGroup;

    OpModeMeta.Flavor type() default OpModeMeta.Flavor.AUTONOMOUS;

    OpModeMeta.Source source() default OpModeMeta.Source.ANDROID_STUDIO;
}
