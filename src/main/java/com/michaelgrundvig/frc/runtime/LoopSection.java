package com.michaelgrundvig.frc.runtime;

/**
 * The sections of a robot loop, in the order they run. Each runs under a guard of its own ({@link
 * LoopHealth#threw}): on a robot, and in a replay of one, a section that throws is reported and the
 * sections after it still run. In the simulator and in tests a throw ends the program at once.
 *
 * <p>What a throw does to the rest of the loop depends on what the section is for ({@link Part}).
 */
enum LoopSection {
  /** The Pitwall page's requests: tuning, actions, experiments, devices, status lines. */
  PITWALL("Pitwall", Part.ASIDE),
  /** The coprocessors, through their Spotter agents, and Visor's installs on them. */
  COPROCESSORS("Coprocessors", Part.ASIDE),
  /** The health of the CANivores and the coprocessors, in the log and on Pitwall. */
  HEALTH("Health", Part.ASIDE),
  /** The venue and the auto the pit chose. */
  VENUES("Venues", Part.ASIDE),
  /**
   * The match as the Driver Station says it, where the drive team placed the robot, the simulated
   * world's step, and the auto's start and end.
   */
  MATCH("Match", Part.ASIDE),
  /** The drivetrain's sensors. */
  DRIVE("Drive", Part.READS),
  /** The joints' sensors, the switches and rangefinders, and the PDH. */
  JOINTS("Joints", Part.READS),
  /** The cameras' frames, and localization's estimate from them and the drivetrain's sensors. */
  VISION_AND_LOCALIZATION("Vision and localization", Part.READS),
  /** The field's checks of where the robot is: its placement, its zones. */
  FIELD("Field", Part.ASIDE),
  /** The drive team's dashboard. */
  DASHBOARD("Dashboard", Part.ASIDE),
  /** The command scheduler: the commands, the bindings, and canceling them all at a disable. */
  COMMANDS("Commands", Part.DRIVES),
  /** What the drivetrain and the joints are told: each joint's output, on its own. */
  OUTPUTS("Outputs", Part.DRIVES);

  /** What a section is for, which decides what its throw does to the rest of the loop. */
  enum Part {
    /**
     * It reads sensors. After it throws, this loop's readings can't be trusted: some are this
     * loop's and some the last one's, and one may be half read. So the commands don't run this
     * loop, the drivetrain is told to stand still, and each joint does what it does idle. The loop
     * has failed.
     */
    READS,
    /**
     * It decides what the robot does, or tells the motors. The loop has failed: a command that
     * throws is ended by the scheduler, and a joint whose write throws keeps what it was last told
     * until the next loop.
     */
    DRIVES,
    /**
     * It serves the pit or the drive team, or the field around the robot. The robot drives the same
     * without it, so its throw is reported and the loop has not failed. Ending the program over a
     * page's or a dashboard's bug would turn it into a robot that is dead while it restarts.
     */
    ASIDE
  }

  /** The section's name, as a person reads it in the alert. */
  private final String label;

  private final Part part;

  LoopSection(String label, Part part) {
    this.label = label;
    this.part = part;
  }

  String label() {
    return label;
  }

  Part part() {
    return part;
  }
}
