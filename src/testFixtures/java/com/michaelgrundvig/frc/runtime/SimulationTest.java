package com.michaelgrundvig.frc.runtime;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import org.junit.jupiter.api.Tag;

/**
 * Marks a test class that runs the simulator for seconds of robot time: the whole robot, or a
 * mechanism's physics. {@code ./gradlew test -Pquick} leaves these out, for a fast edit-test loop,
 * unless {@code --tests} names them; {@code ./gradlew ci} and {@code ./gradlew test} run them.
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
@Tag("simulation")
public @interface SimulationTest {}
