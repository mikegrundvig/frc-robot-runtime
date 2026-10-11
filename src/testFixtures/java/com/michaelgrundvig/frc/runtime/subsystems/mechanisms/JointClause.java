package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

/**
 * The clauses of the contract a joint's driver keeps ({@link JointIO}), by the numbers its Javadoc
 * gives them. The conformance suite ({@link JointIOConformance}) checks each against every driver,
 * and a driver that is known to differ from one says so by its name here ({@link
 * JointBench#knownDifference}).
 *
 * <p>The contract's words are {@link JointIO}'s. What is here is a short name for each clause, for
 * a test's message. A test keeps the two lists the same.
 */
public enum JointClause {
  J1("positions and velocities are in the joint's units"),
  J2("forward is the joint's forward, however the motor is mounted"),
  J3("a request holds until the next"),
  J4("neutral until first asked"),
  J5("a position runs the position law on the controller's loop"),
  J6("a velocity runs the velocity law on the controller's loop"),
  J7("kI is the position loop's alone"),
  J8("a voltage goes across the motors as asked"),
  J9("a duty cycle is that share of the battery's voltage"),
  J10("neutral is no voltage, braking or coasting as the control says"),
  J11("a homing push ignores the soft limits"),
  J12("told where the joint is, it reads there and counts on"),
  J13("no request but a homing push drives past a soft limit"),
  J14("the soft limits are back at the first request after a homing push"),
  J15("a tripped limit switch that stops the motor stops every request, a homing push too"),
  J16("a limit input reads tripped by its switch's polarity"),
  J17("a limit read elsewhere stops the motor and is not reported"),
  J18("a retune puts its gains and current limits in force, a zero too"),
  J19("a retune changes nothing but gains and current limits"),
  J20("a joint with no absolute encoder starts where its definition rests it"),
  J21("the applied voltage is signed, positive forward"),
  J22("the stator current is never negative"),
  J23("the supply current is never negative"),
  J24("unconfigured is empty when every device took its settings"),
  J25("faults and followers are listed one per device"),
  J26("connected is false after half a second of silence, and true again at the first frame"),
  J27("connected stays true through less than half a second of silence"),
  J28("readings stand while frames do not come"),
  J29("fresh is false in a loop the leader's frame did not come"),
  J30("the first request after a silence is in force"),
  J31("restarted is true in exactly one loop, and the count starts again"),
  J32("after a restart the tuning and the soft limits are in force again"),
  J33("a follower drives the joint the way its leader does"),
  J34("a follower's restart is said once, and it follows again"),
  J35("a follower's silence shows after half a second, the leader's not with it"),
  J36("the absolute encoder beside the motor's count is reported each loop"),
  J37("the encoder beside the count seeds it at startup and after a restart"),
  J38("an encoder the controller's loop runs on is where the joint reads"),
  J39("a joint that turns freely reads on past a turn"),
  J40("a position is taken as its number says, not as the nearest turn");

  private final String says;

  JointClause(String says) {
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
