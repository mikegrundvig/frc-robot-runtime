package com.michaelgrundvig.frc.simulation;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import org.junit.jupiter.api.Tag;

/**
 * Marks a check in the truth suite that today's simulator passes by construction: it compares the
 * simulator with itself, not with an answer worked out apart from it, so it can't show today's
 * simulator wrong, and the reason is known (its value). It stays, to catch a change that breaks the
 * agreement, and as a truth check for an engine that can fail it, where its value says one can.
 * {@code docs/simulator.md} lists it as consistency, not truth.
 */
@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
@Tag("consistency")
public @interface Consistency {
  /** Why today's simulator can't fail it. */
  String value();
}
