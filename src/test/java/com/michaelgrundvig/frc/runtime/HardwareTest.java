package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANRANGE;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.SignalLogger;
import com.michaelgrundvig.frc.runtime.coprocessors.Coprocessors;
import com.michaelgrundvig.frc.runtime.coprocessors.VisorTuning;
import com.michaelgrundvig.frc.runtime.hardware.Hardware;
import com.michaelgrundvig.frc.runtime.hardware.RobotHardware;
import com.michaelgrundvig.frc.runtime.hardware.RunsOn;
import com.michaelgrundvig.frc.runtime.hardware.World;
import com.michaelgrundvig.frc.runtime.sim.SimulatedHardware;
import com.michaelgrundvig.frc.runtime.sim.SimulatedWorld;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIOPort;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIOSpark;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIOTalon;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIORev;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIOCanRange;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIOCandi;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIOPort;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIOPhotonVision;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIOVisor;
import com.michaelgrundvig.frc.runtime.subsystems.vision.VisorBoards;
import com.michaelgrundvig.frc.runtime.subsystems.vision.VisorPorts;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.visor.CameraToTag;
import com.michaelgrundvig.frc.visor.VisorFrame;
import com.michaelgrundvig.frc.visor.VisorWire;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Driven.LimitSwitch;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RevoluteJoint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.system.RobotController;
import org.wpilib.util.AlertDataJNI;

/**
 * The robot's hardware in each mode the program runs in: on a robot, the devices its definition's
 * wiring names, and nothing, said by an alert, for a part with none; in the simulator, the
 * simulated world's; in a replay, nothing, since the log has every reading.
 */
class HardwareTest {
  /** A pivot on a TalonFX, its encoder and lower limit switch on Systemcore ports. */
  private static final RevoluteJoint PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, Joint.PITCH_DOWN)
          .at(Meters.of(0.25), Meters.of(0), Meters.of(0.32))
          .range(Range.degrees(0, 135))
          .rest(Degrees.of(0))
          .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(11)))
          .absoluteEncoder(MechanismWiring.PortEncoder.dutyCycle(2).offset(Rotations.of(0.3)))
          .limitSwitch(
              LimitSwitch.lower().at(Degrees.of(0)).on(SensorWiring.port(1).normallyClosed()))
          .control(MechanismControl.position(30).withKg(0));

  /** An arm on a SPARK, its lower limit switch on the SPARK's own input. */
  private static final RevoluteJoint ARM =
      Joint.pivot("arm", Part.CHASSIS, Joint.PITCH_UP)
          .at(Meters.of(-0.2), Meters.of(0), Meters.of(0.4))
          .range(Range.degrees(0, 90))
          .rest(Degrees.of(0))
          .motors(Motor.NEO.through(Gearing.of(50)).on(SPARK_MAX.can(12)))
          .limitSwitch(
              LimitSwitch.lower()
                  .at(Degrees.of(0))
                  .on(SensorWiring.Direction.REVERSE.normallyClosed()))
          .control(
              MechanismControl.position(30)
                  .withKg(0)
                  .withHoming(MechanismControl.Homing.assumeAtRest()));

  /** A roller whose motor has no controller named: it runs in the simulator alone. */
  private static final RevoluteJoint ROLLER =
      Joint.roller("roller", PIVOT)
          .at(Meters.of(0), Meters.of(0), Meters.of(0.35))
          .outputs(Output.volts("intaking", 6))
          .motors(Motor.KRAKEN_X60.through(Gearing.of(3)))
          .control(MechanismControl.velocity(0.02));

  private static final Sensor.Switch BEAM =
      Sensor.Switch.beam("claw beam", PIVOT).on(CANDI.can(30).s1().normallyOpen());
  private static final Sensor.Rangefinder HOPPER =
      Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))
          .on(CANRANGE.can(31));

  /** A switch with no wiring. */
  private static final Sensor.Switch BUMPER = Sensor.Switch.contact("bumper", Part.CHASSIS);

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
          .name("hardware")
          .pdh(Pdh.can(1))
          .joints(List.of(PIVOT, ARM, ROLLER))
          .sensors(List.of(BEAM, HOPPER, BUMPER))
          .build();

  @RegisterExtension final RobotSim sim = new RobotSim();

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false);
  }

  /** Each of the robot's switches and rangefinders, its joints' limit switches among them. */
  private static Map<String, Sensor.Placed> sensors() {
    Map<String, Sensor.Placed> sensors = new LinkedHashMap<>();
    for (Sensor sensor : ROBOT.math().sensors()) {
      if (sensor instanceof Sensor.Placed placed) {
        sensors.put(placed.label(), placed);
      }
    }
    return sensors;
  }

  /**
   * On a robot, each part on the device its wiring names: the encoder and the limit switch on
   * Systemcore ports (each cast to its kind, to close it). A switch on a SPARK's input has no
   * device of its own: its joint reads it from the SPARK, and nothing asks for one.
   */
  @Test
  void aRobotsHardwareIsTheDevicesItsWiringNames() {
    Map<String, Sensor.Placed> sensors = sensors();
    try (Hardware hardware =
            RobotContainer.hardware(
                RunMode.REAL,
                RunsOn.SYSTEMCORE,
                ROBOT,
                Optional.empty(),
                VisorBoards.none(),
                VisorPorts.ANY_FREE);
        AbsoluteEncoderIOPort encoder = (AbsoluteEncoderIOPort) hardware.absoluteEncoder(PIVOT);
        SensorIOPort lower =
            (SensorIOPort)
                hardware.sensor(Objects.requireNonNull(sensors.get("pivot lower limit")))) {
      assertThat(hardware).isInstanceOf(RobotHardware.class);
      assertThat(hardware.joint(PIVOT)).isInstanceOf(JointIOTalon.class);
      assertThat(hardware.joint(ARM)).isInstanceOf(JointIOSpark.class);
      assertThatIllegalArgumentException()
          .isThrownBy(() -> hardware.sensor(Objects.requireNonNull(sensors.get("arm lower limit"))))
          .withMessage(
              "arm lower limit is on arm's controller's limit input: arm reads it from its"
                  + " controller");
      assertThat(hardware.sensor(BEAM)).isInstanceOf(SensorIOCandi.class);
      assertThat(hardware.sensor(HOPPER)).isInstanceOf(SensorIOCanRange.class);
      assertThat(hardware.pdh()).isInstanceOf(PdhIORev.class);
      List<CameraIO> cameras = hardware.cameras();
      assertThat(cameras)
          .hasSize(ROBOT.cameras().size())
          .allMatch(camera -> camera instanceof CameraIOPhotonVision);
      cameras.forEach(CameraIO::close);
    }
  }

  /**
   * On a robot, a part with no wiring gets nothing, and stays still rather than guess, and an alert
   * says which: the drivetrain, a joint, a sensor.
   */
  @Test
  void aRobotsPartWithNoWiringGetsNothingAndAnAlertSaysSo() {
    try (Hardware hardware =
        RobotContainer.hardware(
            RunMode.REAL,
            RunsOn.SYSTEMCORE,
            ROBOT,
            Optional.empty(),
            VisorBoards.none(),
            VisorPorts.ANY_FREE)) {
      assertThat(hardware.drive().modules())
          .allMatch(module -> module.getClass().isAnonymousClass());
      assertThat(hardware.joint(ROLLER).getClass().isAnonymousClass()).isTrue();
      assertThat(hardware.sensor(BUMPER).getClass().isAnonymousClass()).isTrue();

      assertThat(activeAlerts())
          .contains(
              "This robot's configuration has no drivetrain wiring (CAN IDs and encoder offsets),"
                  + " so the drivetrain stays still. Add it to the robot's entry in Robots.",
              "No wiring for roller, so they stay still. Add it to the robot's entry in Robots.",
              "No wiring for bumper, so nothing waiting on them holds. Add it to the robot's entry"
                  + " in Robots.");
    }
    assertThat(activeAlerts())
        .as("closed with it")
        .noneMatch(text -> text.startsWith("No wiring for "));
  }

  /**
   * On a laptop driving real devices (CTRE's hardware-attached simulation), only what's on its
   * CANivore is opened. Systemcore's own CAN port, its I/O ports and the PDH aren't there, so a
   * part on them gets nothing, as an unwired part does, and one alert names them all; a SPARK is on
   * Systemcore's port too, and REV has no way to hardware from a laptop anyway. The same joint and
   * sensor on a CANivore open as on a robot.
   */
  @Test
  void aLaptopOpensOnlyWhatsOnItsCanivore() {
    Map<String, Sensor.Placed> sensors = sensors();
    try (Hardware hardware =
        RobotContainer.hardware(
            RunMode.REAL,
            RunsOn.DESKTOP,
            ROBOT,
            Optional.empty(),
            VisorBoards.none(),
            VisorPorts.ANY_FREE)) {
      assertThat(hardware).isInstanceOf(RobotHardware.class);
      List<Object> made = new ArrayList<>();
      made.add(hardware.joint(PIVOT));
      made.add(hardware.joint(ARM));
      made.add(hardware.absoluteEncoder(PIVOT));
      made.add(hardware.sensor(Objects.requireNonNull(sensors.get("pivot lower limit"))));
      made.add(hardware.sensor(BEAM));
      made.add(hardware.sensor(HOPPER));
      made.add(hardware.pdh());
      assertThat(made).allMatch(io -> io.getClass().isAnonymousClass());
      List<CameraIO> cameras = hardware.cameras();
      assertThat(cameras).allMatch(camera -> camera instanceof CameraIOPhotonVision);
      cameras.forEach(CameraIO::close);

      assertThat(activeAlerts())
          .contains(
              "Not on this laptop's CANivore, so not opened: pivot, arm, pivot's absolute encoder,"
                  + " pivot lower limit, claw beam, hopper, the PDH. Only a robot reaches them.");
    }
    assertThat(activeAlerts())
        .as("closed with it")
        .noneMatch(text -> text.startsWith("Not on this laptop's CANivore"));

    CanBus canivore = CanBus.canivore("bench");
    RevoluteJoint onTheCanivore =
        Joint.pivot("pivot", Part.CHASSIS, Joint.PITCH_DOWN)
            .at(Meters.of(0.25), Meters.of(0), Meters.of(0.32))
            .range(Range.degrees(0, 135))
            .rest(Degrees.of(0))
            .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(canivore, 11)))
            .control(
                MechanismControl.position(30)
                    .withKg(0)
                    .withHoming(MechanismControl.Homing.assumeAtRest()));
    Sensor.Rangefinder hopper =
        Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))
            .on(CANRANGE.can(canivore, 31));
    RobotDefinition benched =
        RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
            .name("benched")
            .joints(List.of(onTheCanivore))
            .sensors(List.of(hopper))
            .build();
    try (Hardware hardware =
        RobotContainer.hardware(
            RunMode.REAL,
            RunsOn.DESKTOP,
            benched,
            Optional.empty(),
            VisorBoards.none(),
            VisorPorts.ANY_FREE)) {
      assertThat(hardware.joint(onTheCanivore)).isInstanceOf(JointIOTalon.class);
      assertThat(hardware.sensor(hopper)).isInstanceOf(SensorIOCanRange.class);
      assertThat(activeAlerts())
          .noneMatch(text -> text.startsWith("Not on this laptop's CANivore"));
    }
  }

  /**
   * A camera whose definition says Visor is read through Visor, by its place in the list; the rest
   * through PhotonVision. Closing the hardware closes Visor's port and its clock responder.
   */
  @Test
  void aCameraOnVisorIsReadThroughVisor() {
    try (Hardware hardware =
        RobotContainer.hardware(
            RunMode.REAL,
            RunsOn.DESKTOP,
            onVisor(),
            Optional.empty(),
            VisorBoards.none(),
            VisorPorts.ANY_FREE)) {
      List<CameraIO> made = hardware.cameras();
      assertThat(made.get(0)).isInstanceOf(CameraIOVisor.class);
      assertThat(made.subList(1, made.size()))
          .allMatch(camera -> camera instanceof CameraIOPhotonVision);
      made.forEach(CameraIO::close);
    }
  }

  /** The example robot with its first camera on Visor. */
  private static RobotDefinition onVisor() {
    List<CameraConfig> cameras = new ArrayList<>(ExampleRobot.SIM.definition().cameras());
    cameras.set(0, cameras.get(0).withSoftware(CameraConfig.Software.VISOR));
    return RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
        .name("on Visor")
        .cameras(cameras)
        .build();
  }

  /** A robot's coprocessors as its configuration lists them, and what a frame from here meets. */
  private record Listed(String name, List<String> boards, boolean taken) {}

  /**
   * On a robot, the boards the coprocessors list reach Visor's frames, as the robot program wires
   * them: with boards listed, a frame from a sender that isn't one is refused, and with none listed
   * it is taken. Each row's frame comes from 127.0.0.1 to the port the robot listens on for Visor,
   * one the system picked, so no other program on this computer can take the frame.
   */
  @Test
  void aVisorFramesSenderIsCheckedAgainstTheListedCoprocessors(@TempDir Path deploy)
      throws Exception {
    List<Listed> rows =
        List.of(
            new Listed(
                "a board listed elsewhere: refused", List.of("127.0.0.2:" + unusedPort()), false),
            new Listed("no boards listed: taken", List.of(), true));
    RobotDefinition robot = onVisor();
    for (Listed row : rows) {
      // No team number, so nothing is installed: the boards are only listed and watched.
      try (Coprocessors coprocessors =
              Coprocessors.on(
                  row.boards(),
                  VisorTuning.live(robot.cameras(), Map.of()),
                  deploy,
                  0,
                  Rebuilt.GAME.officialField().tagSizeMeters(),
                  deploy.resolve("kept"),
                  () -> false,
                  () -> false,
                  () -> 0L);
          Hardware hardware =
              RobotContainer.hardware(
                  RunMode.REAL,
                  RunsOn.DESKTOP,
                  robot,
                  Optional.empty(),
                  coprocessors.visorBoards(robot.cameras()),
                  VisorPorts.ANY_FREE);
          DatagramChannel visor = DatagramChannel.open()) {
        List<CameraIO> made = hardware.cameras();
        CameraIO camera = made.get(0);
        int listening = ((RobotHardware) hardware).visorFramesPort().orElseThrow();
        visor.send(frame(robot.cameras().get(0)), new InetSocketAddress("127.0.0.1", listening));
        CameraIO.CameraIOInputs inputs = new CameraIO.CameraIOInputs();
        String name = row.name();
        Waits.until(
            name + ": the frame taken or refused",
            () -> camera.updateInputs(inputs),
            () -> inputs.frames.length > 0 || !inputs.error.isEmpty());
        if (row.taken()) {
          assertThat(inputs.frames).as(name).hasSize(1);
          assertThat(inputs.error).as(name).isEmpty();
        } else {
          assertThat(inputs.frames).as(name).isEmpty();
          assertThat(inputs.error).as(name).contains("none of the robot's coprocessors has");
        }
        made.forEach(CameraIO::close);
      }
    }
  }

  /** A frame from the first camera, just taken, at its configured size, with one tag solved. */
  private static ByteBuffer frame(CameraConfig camera) {
    VisorFrame frame = new VisorFrame();
    frame.setHeader(
        0,
        camera.sensor().widthPixels(),
        camera.sensor().heightPixels(),
        RobotController.getTime(),
        0,
        true,
        false);
    int solved = frame.addTag(7, 0, 80, new float[] {10, 20, 30, 20, 30, 40, 10, 40});
    CameraToTag ahead = new CameraToTag();
    ahead.set(3, 0.5, 0.25, 0, 0, 0, 1);
    frame.setPoses(solved, ahead, 0.1f, ahead, 0.5f);
    ByteBuffer bytes = ByteBuffer.allocate(VisorWire.MAX_BYTES);
    VisorWire.encode(frame, bytes);
    return bytes.flip();
  }

  /** A port nothing listens on: a listed board that never answers. */
  private static int unusedPort() throws IOException {
    try (ServerSocket socket = new ServerSocket(0)) {
      return socket.getLocalPort();
    }
  }

  /**
   * In the simulator, each part is the simulated world's, wired or not, built from the definition
   * with the world: the joints, their encoders and the sensors whatever order they're asked for in.
   */
  @Test
  void theSimulatorsHardwareIsTheWorlds() {
    try (SimulatedWorld world =
            new SimulatedWorld(RobotSim.FIELD, ROBOT, RobotSim.massOf(ROBOT), SensorErrors.NONE);
        Hardware hardware =
            RobotContainer.hardware(
                RunMode.SIM,
                RunsOn.DESKTOP,
                ROBOT,
                Optional.<World>of(world),
                VisorBoards.none(),
                VisorPorts.ANY_FREE)) {
      assertThat(hardware).isInstanceOf(SimulatedHardware.class);
      assertThat(hardware.drive().modules())
          .containsExactlyElementsOf(world.drivetrainHardware().modules());
      assertThat(hardware.sensor(BUMPER)).isSameAs(world.sensors().io(BUMPER));
      assertThat(hardware.absoluteEncoder(PIVOT)).isSameAs(world.joints().absoluteEncoder(PIVOT));
      assertThat(hardware.joint(ROLLER)).isSameAs(world.joints().io(ROLLER));
      assertThat(world.joints().position(ROLLER)).as("the roller, unwired, simulated").isZero();
      assertThatIllegalArgumentException()
          .isThrownBy(
              () -> hardware.sensor(Objects.requireNonNull(sensors().get("arm lower limit"))))
          .withMessage(
              "arm lower limit is on arm's controller's limit input: arm reads it from its"
                  + " controller");
      assertThat(hardware.pdh()).isSameAs(world.pdh());
      assertThat(hardware.cameras()).containsExactly(world.cameras());
    }
  }

  /** In a replay, nothing: every reading comes from the log. */
  @Test
  void aReplaysHardwareIsNothing() {
    try (Hardware hardware =
        RobotContainer.hardware(
            RunMode.REPLAY,
            RunsOn.DESKTOP,
            ROBOT,
            Optional.empty(),
            VisorBoards.none(),
            VisorPorts.ANY_FREE)) {
      List<Object> made = new ArrayList<>(hardware.drive().modules());
      made.add(hardware.drive().gyro());
      made.add(hardware.joint(PIVOT));
      made.add(hardware.absoluteEncoder(PIVOT));
      made.add(hardware.sensor(BEAM));
      made.add(hardware.pdh());
      made.addAll(hardware.cameras());

      assertThat(made).allMatch(io -> io.getClass().isAnonymousClass());
      assertThat(hardware.cameras()).hasSize(ROBOT.cameras().size());
      assertThat(activeAlerts()).noneMatch(text -> text.startsWith("No wiring for "));
    }
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
}
