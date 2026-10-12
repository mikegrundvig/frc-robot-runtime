package com.michaelgrundvig.frc.runtime.subsystems.drive;

/**
 * The clauses of the contract a swerve module's driver keeps ({@link ModuleIO}), by the numbers its
 * Javadoc gives them. The conformance suite ({@link ModuleIOConformance}) checks the ones about
 * units and signs against every driver. The contract's words are {@link ModuleIO}'s; what is here
 * is a short name for each clause, for a test's message. A test keeps the two lists the same
 * ({@code HardwareContractsTest}).
 */
public enum ModuleClause {
  M1("the wheel reads in radians at the wheel, forward positive"),
  M2("the wheel's reading leaves out what steering turns it"),
  M3("the module's angle is its absolute encoder's, within half a turn either way"),
  M4("the module's turning speed has the angle's sign"),
  M5("a wheel speed runs the drive's velocity loop with the volts added"),
  M6("an angle runs the steering's loop the short way round"),
  M7("stop drives neither motor"),
  M8("the applied voltages are signed"),
  M9("the drive's stator current is never negative"),
  M10("the drive's supply current is never negative"),
  M11("the steering's current is the most since the last loop"),
  M12("odometry readings come in step, oldest first, timed on the robot's clock"),
  M13("configure puts gains and limits in force, waiting only at startup"),
  M14("each motor reads as silent after half a second of silence"),
  M15("readings stand in a loop a frame did not come"),
  M16("a restarted controller has its tuning again, and a restarted drive says so in one loop"),
  M17("unconfigured is empty when every device took its settings"),
  M18("the drive says when its current limit held it, where the device says");

  private final String says;

  ModuleClause(String says) {
    this.says = says;
  }

  /** The clause in a few words, for a test's message. */
  public String says() {
    return says;
  }

  @Override
  public String toString() {
    return name() + " (" + says + ")";
  }
}
