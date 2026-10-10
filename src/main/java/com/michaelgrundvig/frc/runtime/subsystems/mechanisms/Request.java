package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

/** What a joint is asked to do in a loop, by its commands or what's beneath them. */
enum Request {
  /** Nothing: no command asked, so its idle decides; or, applied, nothing drove it. */
  NONE,
  /** Go to a position, along its profile. */
  POSITION,
  /** Run at a speed, on its motor controller's velocity loop. */
  VELOCITY,
  /** A voltage across its motors. */
  VOLTAGE,
  /** A share of the battery's voltage. */
  DUTY_CYCLE,
  /** Neutral: its motors brake or coast, as its control says. */
  NEUTRAL,
  /** Homing's push, applied: toward its stop or its switch. */
  HOMING
}
