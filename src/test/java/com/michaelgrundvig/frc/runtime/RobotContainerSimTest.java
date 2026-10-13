package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Outcome;
import com.michaelgrundvig.frc.runtime.behavior.Binding;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimulation;
import com.michaelgrundvig.frc.runtime.sim.SensorSimulation;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.config.Finding.Scope;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Placement;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.Command;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.simulation.PDPSim;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.AlertDataJNI;

/**
 * The whole robot as the simulator runs it: the real composition root, the real controller
 * bindings, the real field, a simulated gamepad, and the simulated drivetrain. If this passes, a
 * driver in the simulator GUI gets the same behavior.
 */
@SimulationTest
class RobotContainerSimTest {
  private static final FieldModel FIELD = RobotSim.FIELD;

  /** What a robot whose configuration has no drivetrain wiring says. */
  private static final String NO_DRIVETRAIN =
      "This robot's configuration has no drivetrain wiring (CAN IDs and encoder offsets), so the"
          + " drivetrain stays still. Add it to the robot's entry in Robots.";

  @RegisterExtension final RobotSim sim = new RobotSim();

  /** Turns the simulated robot by hand, where it stands: its gyro feels it. */
  private void turnByHand(double degrees) {
    DriveSimulation simulation = sim.simulation();
    Pose2d truth = simulation.truePose();
    simulation.place(
        new Pose2d(
            truth.getTranslation(), truth.getRotation().plus(Rotation2d.fromDegrees(degrees))));
  }

  private void pressBack() {
    sim.driver().setBackButton(true);
    sim.driver().notifyNewData();
    sim.loops(1);
    sim.driver().setBackButton(false);
    sim.driver().notifyNewData();
  }

  private double headingDegrees() {
    return sim.robot().drive().getPose().getRotation().getDegrees();
  }

  private void pushLeftStickForward() {
    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-1.0); // gamepad Y is negative when pushed forward
    sim.driver().notifyNewData();
  }

  private void releaseStickAndDisable() {
    sim.driver().setLeftY(0.0);
    sim.driver().notifyNewData();
    RobotSim.setEnabled(false);
  }

  private static Pose2d start(String name, Alliance alliance) {
    return FIELD.starts().stream()
        .filter(start -> start.name().equals(name))
        .findFirst()
        .orElseThrow()
        .on(FIELD, alliance);
  }

  @Test
  void waitsOnItsStartWhileDisabled() {
    sim.run(1.0);

    // Placed on its start; the cameras then refine that to within their noise.
    Pose2d pose = sim.robot().drive().getPose();
    Pose2d center = start("Center", Alliance.BLUE);
    assertThat(pose.getTranslation().getDistance(center.getTranslation())).isLessThan(0.03);
    assertThat(pose.getRotation().minus(center.getRotation()).getDegrees())
        .isCloseTo(0.0, within(0.5));
    assertThat(sim.robot().matchField().allianceStation()).isEqualTo("Blue 2");
    assertThat(sim.robot().matchField().placement().status()).isEqualTo(Placement.Status.READY);
    assertThat(activeAlerts())
        .as("the simulator is the drivetrain's hardware")
        .doesNotContain(NO_DRIVETRAIN);
  }

  /**
   * Pressed while the robot is disabled, as a drive team presses it before a match: the heading
   * reset is the one binding that starts while the robot is disabled.
   */
  @Test
  void theBackButtonSaysTheRobotFacesDownfieldForTheDriversAlliance() {
    sim.build(RobotSim.WITHOUT_CAMERAS, SensorErrors.NONE);
    RobotSim.station(AllianceStationID.RED_2);
    sim.loops(2); // placed on red's center start, facing blue's wall
    turnByHand(30);
    sim.loops(1);
    double turned = headingDegrees();

    pressBack();

    assertThat(turned).isCloseTo(-150, within(0.5));
    assertThat(Math.abs(headingDegrees())).as("red faces -x").isCloseTo(180, within(1e-6));
  }

  /**
   * The driver pressed Back with the robot turned 30 degrees: the cameras turn the estimate back.
   */
  @Test
  void theCamerasTurnBackAWrongHeadingReset() {
    RobotSim.station(AllianceStationID.RED_2);
    sim.run(1.0);
    turnByHand(30);
    sim.run(0.5);

    pressBack();
    double wrong = headingDegrees();
    sim.run(1.0);

    assertThat(Math.abs(wrong)).isCloseTo(180, within(0.5));
    assertThat(headingDegrees()).isCloseTo(-150, within(1.0));
    assertThat(
            sim.robot()
                .drive()
                .getPose()
                .getTranslation()
                .getDistance(sim.simulation().truePose().getTranslation()))
        .isLessThan(0.05);
  }

  @Test
  void itSaysWhenItIsReadyToScoreAndWhyNot() {
    String before = sim.robot().scoring();
    sim.run(1.0);
    String settled = sim.robot().scoring();
    boolean ready = sim.robot().readyToScore();
    sim.simulation().tilt(Math.toRadians(10), 0);
    sim.loops(2);

    assertThat(before).isEqualTo("Not ready: not started");
    assertThat(settled).isEqualTo("Ready to score");
    assertThat(ready).isTrue();
    assertThat(sim.robot().scoring()).isEqualTo("Not ready: the robot is tilted 10 degrees");
  }

  @Test
  void autonomousStartingHoldsTheCamerasOnlyWhenAsked() {
    var hold = sim.pitwall().tunables().requestChanges(Map.of("Vision/autoStartHoldSeconds", 2.0));
    sim.run(1.0);
    assertThat(hold.getNow(null)).isEqualTo(new Outcome(Outcome.Result.APPLIED, ""));
    int before = sim.robot().estimate().health().accepted();

    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    sim.run(1.0);
    int held = sim.robot().estimate().health().accepted() - before;
    sim.run(1.5);

    assertThat(held).as("held for 2 s").isZero();
    assertThat(sim.robot().estimate().health().accepted() - before).as("then used").isPositive();
  }

  @Test
  void theDriverStationsLocationChoosesTheStart() {
    RobotSim.station(AllianceStationID.RED_1);

    sim.loops(1);

    assertEquals(start("Left", Alliance.RED), sim.robot().drive().getPose());
    Placement placement = sim.robot().matchField().placement();
    assertThat(placement.status()).isEqualTo(Placement.Status.READY);
    assertThat(placement.start()).isEqualTo("Left");
    assertThat(placement.distanceMeters()).as("off its start by").isCloseTo(0, within(0.005));
    assertThat(placement.headingErrorDegrees()).as("turned off by").isCloseTo(0, within(0.5));
  }

  @Test
  void theLeftStickDrivesAwayFromTheBlueDrivers() {
    Pose2d start = start("Center", Alliance.BLUE);
    pushLeftStickForward();

    sim.run(1.5);

    Pose2d pose = sim.robot().drive().getPose();
    assertTrue(pose.getX() > start.getX() + 2.0, "toward red's wall: now at x=" + pose.getX());
    assertEquals(start.getY(), pose.getY(), 0.05, "straight ahead");
  }

  /** The red drivers stand at the other end, so their forward is the field's -x. */
  @Test
  void theLeftStickDrivesAwayFromTheRedDrivers() {
    RobotSim.station(AllianceStationID.RED_2);
    Pose2d start = start("Center", Alliance.RED);
    pushLeftStickForward();

    sim.run(1.5);

    Pose2d pose = sim.robot().drive().getPose();
    assertTrue(pose.getX() < start.getX() - 2.0, "toward blue's wall: now at x=" + pose.getX());
    assertEquals(start.getY(), pose.getY(), 0.05, "straight ahead");
  }

  @Test
  void afterDrivingThePlacementCheckSaysHowFarAwayTheStartIs() {
    pushLeftStickForward();
    sim.run(1.0);
    releaseStickAndDisable();

    sim.loops(1);

    assertThat(sim.robot().matchField().placement().status())
        .isEqualTo(Placement.Status.NOT_ON_A_START);
    assertThat(sim.robot().matchField().placement().describe())
        .startsWith("Not on a start: Center is ");
  }

  /** In the pit or the simulator, the robot goes on the start chosen while it's disabled. */
  @Test
  void changingTheStationWhileDisabledMovesTheRobotButNeverWhileItDrives() {
    pushLeftStickForward();
    sim.loops(5);
    RobotSim.station(AllianceStationID.BLUE_3);
    sim.loops(5);
    assertThat(sim.robot().drive().getPose().getY())
        .as("still driving from the center start")
        .isCloseTo(start("Center", Alliance.BLUE).getY(), within(0.05));

    releaseStickAndDisable();
    sim.run(1.0); // coasts to a stop, then the drive team moves it
    RobotSim.station(AllianceStationID.BLUE_1);
    sim.loops(1);

    assertThat(
            sim.robot()
                .drive()
                .getPose()
                .minus(start("Left", Alliance.BLUE))
                .getTranslation()
                .getNorm())
        .isLessThan(0.001);
  }

  @Test
  void zonesFollowTheRobot() {
    sim.loops(1);

    assertThat(sim.robot().matchField().inZone(Rebuilt.ALLIANCE_ZONE))
        .as("on a start, in its zone")
        .isTrue();
    assertThat(sim.robot().matchField().inZone(Rebuilt.TRENCH)).isFalse();
  }

  /** From the left start, straight ahead passes under the left trench. */
  @Test
  void theTrenchTriggerTurnsOnBeforeTheRobotReachesIt() {
    RobotSim.station(AllianceStationID.BLUE_1);
    sim.loops(1);
    pushLeftStickForward();

    boolean onBeforeArriving = false;
    for (int i = 0; i < RobotLoop.loops(2.0); i++) {
      sim.loops(1);
      boolean under =
          Rebuilt.TRENCH.contains(
              FIELD, Alliance.BLUE, sim.robot().drive().getPose().getTranslation());
      if (sim.robot().matchField().inZone(Rebuilt.TRENCH) && !under) {
        onBeforeArriving = true;
      }
      if (under) {
        break;
      }
    }

    assertThat(onBeforeArriving).isTrue();
    assertThat(sim.robot().matchField().inZone(Rebuilt.TRENCH))
        .as("and on while under it")
        .isTrue();
  }

  @Test
  void untilTheAllianceIsKnownTheRobotWaitsMidfieldAndSaysSo() {
    RobotSim.station(AllianceStationID.UNKNOWN);

    sim.loops(1);

    assertThat(sim.robot().drive().getPose().getX()).isCloseTo(FIELD.length() / 2, within(1e-9));
    assertThat(sim.robot().matchField().allianceStation()).isEqualTo("Unknown");
    assertThat(sim.robot().matchField().placement().status())
        .isEqualTo(Placement.Status.ALLIANCE_UNKNOWN);
  }

  @Test
  void theFieldViewShowsTheRobotItsStartAndTheTargets() {
    pushLeftStickForward();

    sim.run(1.0);

    assertEquals(sim.robot().drive().getPose(), shown("/Field/Robot")[0]);
    assertEquals(start("Center", Alliance.BLUE), shown("/Field/Start")[0]);
    assertEquals(Rebuilt.HUB.on(FIELD, Alliance.BLUE), shown("/Field/Hub")[0]);
  }

  /**
   * The simulator's robot has a season's wear on its wheels and gyro, as {@code ./gradlew run}
   * builds it, and four cameras whose views PhotonVision renders from where the robot really is:
   * the estimate stays with the truth, and the field view shows both.
   */
  @Test
  void theCamerasKeepTheEstimateWithTheTruth() {
    sim.build(SensorErrors.TYPICAL);
    sim.loops(1);
    pushLeftStickForward();

    sim.run(1.5);

    DriveSimulation simulation = sim.simulation();
    Pose2d truth = simulation.truePose();
    Pose2d believed = sim.robot().drive().getPose();
    assertThat(believed.getTranslation().getDistance(truth.getTranslation()))
        .as("6 m driven on worn wheels")
        .isLessThan(0.10);
    assertThat(sim.robot().estimate().health().accepted()).as("camera measurements").isPositive();
    assertEquals(truth, shown("/Field/Truth")[0]);
  }

  /**
   * The template's own binding is in the robot's record, as a controller map reads it, and its
   * press is logged; a second command on its button the same way is said with the robot's doubts.
   */
  @Test
  void theRobotsBindingsAreRecordedAndADuplicateIsDoubted() {
    RobotContainer robot = sim.robot();
    sim.loops(1);
    List<String> recorded = robot.bindings().all().stream().map(Binding::toString).toList();

    pressBack();
    List<Binding> fired = robot.bindings().fired();
    robot.driver().back().onTrue(Command.noRequirements(co -> {}).named("Zero arm"));

    assertThat(recorded).containsExactly("driver back: on press: Reset heading");
    assertThat(fired).containsExactlyElementsOf(robot.bindings().all().subList(0, 1));
    assertThat(
            Doubts.lines(Doubts.of(ExampleRobot.SIM.definition(), robot.bindings()), Scope.ROBOT))
        .contains(
            "driver back starts Reset heading and Zero arm on press: one press runs them together."
                + " If that's meant, make them one command; if not, bind one to another button");
  }

  @Test
  void replayRunsWithoutTheSimulator() {
    sim.build(RunMode.REPLAY, ExampleRobot.SIM, SensorErrors.NONE);

    sim.loops(5);

    assertEquals(
        start("Center", Alliance.BLUE),
        sim.robot().drive().getPose(),
        "on its start, as the log's Driver Station says; nothing moves");
    assertThat(activeAlerts()).as("replay's hardware is the log").doesNotContain(NO_DRIVETRAIN);
    assertThat(sim.robot().simulation()).as("no physics in a replay").isEmpty();
  }

  /**
   * Until a team writes the motor-controller code for its modules, a real robot holds still and
   * says why, rather than guessing at motors.
   */
  @Test
  void aRealRobotWithoutWiringHoldsStillAndSaysSo() {
    sim.build(RunMode.REAL, ExampleRobot.SIM, SensorErrors.NONE);
    sim.loops(1);
    pushLeftStickForward();

    sim.run(0.5);

    assertThat(activeAlerts()).contains(NO_DRIVETRAIN);
    assertEquals(start("Center", Alliance.BLUE), sim.robot().drive().getPose(), "nothing moved");
  }

  /** A sensor with no wiring on a robot is said once, by its own alert, not again as lost. */
  @Test
  void aRealRobotsUnwiredSensorIsSaidOnceNotAsLost() {
    Sensor.Switch bumper = Sensor.Switch.contact("bumper", Part.CHASSIS);
    sim.build(
        RunMode.REAL,
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .sensors(List.of(bumper))
                    .build())
            .build(),
        SensorErrors.NONE);

    sim.run(0.2);

    assertThat(activeAlerts())
        .contains(
            "No wiring for bumper, so nothing waiting on them holds. Add it to the robot's entry"
                + " in Robots.")
        .noneMatch(text -> text.contains("bumper is lost"));
  }

  /**
   * A wired sensor that stops answering is said as lost, by the joints' own alert, until it answers
   * again.
   */
  @Test
  void aSensorThatStopsAnsweringIsSaidAsLost() {
    Sensor.Switch bumper =
        Sensor.Switch.contact("bumper", Part.CHASSIS).on(CANDI.can(30).s1().normallyOpen());
    sim.build(
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .sensors(List.of(bumper))
                    .build())
            .build(),
        SensorErrors.NONE);
    SensorSimulation sensors = Objects.requireNonNull(sim.robot().world().orElseThrow().sensors());
    sim.run(0.1);
    assertThat(activeAlerts()).noneMatch(text -> text.contains("bumper is lost"));

    sensors.silence(bumper, true);
    sim.run(0.1);
    assertThat(activeAlerts())
        .as("a tenth of a second of silence is a glitch")
        .noneMatch(text -> text.contains("bumper is lost"));
    sim.run(0.5);

    assertThat(activeAlerts())
        .contains("bumper is lost: nothing waiting on it holds. Check the wiring and the CAN bus.");

    sensors.silence(bumper, false);
    sim.run(0.1);

    assertThat(activeAlerts()).noneMatch(text -> text.contains("bumper is lost"));
  }

  /**
   * A robot whose configuration has its wiring drives CTRE hardware: here Phoenix's simulated
   * devices, which answer like real ones.
   */
  @Test
  void aRealRobotWithWiringReadsItsCtreHardware() {
    SwerveWiring.ModuleWiring[] modules = new SwerveWiring.ModuleWiring[4];
    for (int i = 0; i < 4; i++) {
      modules[i] =
          SwerveWiring.ModuleWiring.builder()
              .driveId(40 + 3 * i)
              .steerId(41 + 3 * i)
              .encoderOffset(Rotations.of(0))
              .driveInverted(false)
              .steerInverted(true)
              .encoderInverted(false)
              .encoderId(42 + 3 * i)
              .build();
    }
    RobotConfig wired =
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .drivetrain(
                        ExampleRobot.SIM
                            .definition()
                            .drivetrain()
                            .wired(
                                SwerveWiring.builder()
                                    .canBus(CanBus.canivore("drivetrain"))
                                    .driveController(MechanismWiring.Controller.TALON_FX)
                                    .steerController(MechanismWiring.Controller.TALON_FX)
                                    .gyroId(52)
                                    .gyroYawScale(1.0)
                                    .modules(List.of(modules))
                                    .build()))
                    .build())
            .build();
    sim.build(RunMode.REAL, wired, SensorErrors.NONE);

    sim.run(0.5);

    assertThat(activeAlerts()).doesNotContain(NO_DRIVETRAIN);
    assertThat(sim.robot().estimate().health().gyroConnected()).as("the Pigeon answers").isTrue();
  }

  /**
   * A real robot reads the PDH its definition declares, and says when it doesn't answer: here
   * WPILib's simulated one, its battery cut.
   */
  @Test
  void aRealRobotReadsItsPdhAndSaysWhenItDoesntAnswer() {
    RobotConfig powered =
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .pdh(Pdh.can(1))
                    .build())
            .build();
    PDPSim pdh = new PDPSim(1);
    pdh.resetData();
    pdh.setVoltage(12.2);
    sim.build(RunMode.REAL, powered, SensorErrors.NONE);

    sim.run(0.5);

    assertThat(sim.robot().joints().power().reads()).isTrue();
    assertThat(activeAlerts()).noneMatch(text -> text.startsWith("The PDH isn't answering"));

    pdh.setVoltage(0);
    sim.run(1);

    assertThat(activeAlerts())
        .contains(
            "The PDH isn't answering at CAN ID 1, so no channel's current is read. Check its CAN"
                + " wiring, and its ID in REV's Hardware Client.");
  }

  /**
   * A motor controller drawing nothing on its PDH channel while its joint works is an alert, said
   * with what to check: here a follower's channel zeroed in the simulator while its arm pushes.
   */
  @Test
  void aControllerDrawingNothingWhileItsJointWorksIsAnAlert() {
    Output push = Output.volts("push", -3);
    Joint.Revolute arm =
        Joint.pivot("arm", Part.CHASSIS, new Translation3d(0.2, 0, 0.4), Joint.PITCH_UP)
            .range(Range.degrees(0, 90))
            .rest(Degrees.of(0))
            .outputs(push)
            .motors(
                Motor.KRAKEN_X60
                    .through(Gearing.of(50))
                    .on(TALON_FX.can(11).pdh(4, Amps.of(40)), TALON_FX.can(12).pdh(5, Amps.of(40))))
            .control(
                MechanismControl.position(30)
                    .withKg(0)
                    .withHoming(MechanismControl.Homing.assumeAtRest()));
    sim.build(
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .pdh(Pdh.can(1))
                    .joints(List.of(arm))
                    .build())
            .build(),
        SensorErrors.NONE);
    RobotSim.setEnabled(true);
    sim.scheduler().schedule(sim.robot().joints().of(arm).run(push));
    sim.run(0.2);

    Objects.requireNonNull(sim.robot().world().orElseThrow().pdh()).zero(5, true);
    sim.run(0.6);

    assertThat(activeAlerts())
        .contains(
            "arm motor 12 draws nothing on channel 5 while arm's motors work: its breaker tripped,"
                + " its power is unplugged, or its block names the wrong channel");
  }

  /**
   * A robot closed leaves none of its tuning published, its joints' included: the robot built in
   * its place offers only its own on the page.
   */
  @Test
  void aRobotClosedLeavesNoneOfItsTuningPublished() {
    Joint.Revolute arm =
        Joint.pivot("arm", Part.CHASSIS, new Translation3d(0.2, 0, 0.4), Joint.PITCH_UP)
            .range(Range.degrees(0, 90))
            .rest(Degrees.of(0))
            .motors(Motor.KRAKEN_X60.through(Gearing.of(50)))
            .control(
                MechanismControl.position(30)
                    .withKg(0)
                    .withHoming(MechanismControl.Homing.assumeAtRest()));
    sim.build(
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .joints(List.of(arm))
                    .build())
            .build(),
        SensorErrors.NONE);
    MockTunableBackend first = new MockTunableBackend(); // takes over what the robot published
    TunableRegistry.registerBackend("", first);
    assertThat(first.getTunable("Joints/arm/config/gains/kP")).isNotNull();

    sim.build(SensorErrors.NONE);
    MockTunableBackend second = new MockTunableBackend();
    TunableRegistry.registerBackend("", second);

    assertThat(second.getTunable("Drive/config/gains/steerKp")).isNotNull();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> second.getTunable("Joints/arm/config/gains/kP"))
        .withMessageStartingWith("No such tunable");
  }

  /** The alerts a dashboard shows now. */
  private static List<String> activeAlerts() {
    List<String> active = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      if (alert.activeStartTime != 0) {
        active.add(alert.text);
      }
    }
    return active;
  }

  private Pose2d[] shown(String path) {
    return (Pose2d[])
        sim.telemetry().getLastValue(path, MockTelemetryBackend.LogStructArrayValue.class).value();
  }

  /** The drive check runs from the Pitwall page, on the robot's own scheduler. */
  @Test
  void theDriveCheckIsAnActionOnThePage() {
    RobotSim.enable(RobotMode.UTILITY, true);
    var started = sim.pitwall().actions().requestStart(RobotContainer.DRIVE_CHECK);
    sim.loops(25);

    assertThat(started.getNow(null)).isEqualTo(new Outcome(Outcome.Result.APPLIED, ""));
    assertThat(sim.pitwall().actions().view())
        .filteredOn(action -> action.name().equals(RobotContainer.DRIVE_CHECK))
        .singleElement()
        .extracting(Actions.State::phase)
        .isEqualTo(Actions.Phase.RUNNING);
    assertThat(sim.robot().drive().getRobotRelativeVelocities().vx)
        .as("driving forward slowly")
        .isBetween(0.4, 0.6);
  }
}
