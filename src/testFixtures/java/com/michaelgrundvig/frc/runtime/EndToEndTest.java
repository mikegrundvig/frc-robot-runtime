package com.michaelgrundvig.frc.runtime;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import org.junit.jupiter.api.Tag;

/**
 * Marks a test class that runs the robot program as a program, in its own process, as {@code
 * ./gradlew run} would. {@code ./gradlew test -Pquick} leaves these out unless {@code --tests}
 * names them; {@code ./gradlew ci} and {@code ./gradlew test} run them.
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
@Tag("end-to-end")
public @interface EndToEndTest {}
