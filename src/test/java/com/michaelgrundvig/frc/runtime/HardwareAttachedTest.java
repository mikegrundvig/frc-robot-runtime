package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.SignalLogger;
import com.michaelgrundvig.frc.pitwall.RobotKind;
import com.michaelgrundvig.frc.runtime.hardware.PhoenixBuses;
import com.michaelgrundvig.frc.runtime.hardware.RunsOn;
import com.michaelgrundvig.frc.runtime.venues.VenueStore;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Translation3d;

/**
 * What the program decides for each place it runs: the simulator on a laptop, real devices driven
 * from a laptop through CTRE's hardware-attached simulation ({@code ./gradlew run -PhwSim}), and a
 * robot. The laptop driving real devices runs as a robot does (real hardware, no simulated world,
 * Pitwall's page only showing) with a laptop's folders, and says so in a banner as it starts.
 * Pitwall's side of each is {@code PitwallRobotTest}'s; which devices a laptop opens is {@code
 * HardwareTest}'s.
 */
class HardwareAttachedTest {
  /** The CANivore the test robot's drivetrain is on. */
  private static final CanBus.Canivore CANIVORE = new CanBus.Canivore("bench");

  /** A pivot on Systemcore's own CAN port, which a laptop can't reach. */
  private static final Joint.Revolute PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
          .range(Range.degrees(0, 135))
          .rest(Degrees.of(0))
          .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(11)))
          .control(
              MechanismControl.position(30)
                  .withKg(0)
                  .withHoming(MechanismControl.Homing.assumeAtRest()));

  /** The guide's simulated robot, its drivetrain wired on a CANivore, with a PDH and the pivot. */
  private static final RobotConfig ROBOT =
      RobotConfigBuilder.builder(ExampleRobot.SIM)
          .definition(
              RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                  .name("bench")
                  .drivetrain(
                      ExampleRobot.SIM.definition().drivetrain().wired(drivetrainOn(CANIVORE)))
                  .pdh(Pdh.can(1))
                  .joints(List.of(PIVOT))
                  .build())
          .build();

  @RegisterExtension final RobotSim sim = new RobotSim();

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false);
  }

  /**
   * One place the program runs.
   *
   * @param where how the row reads
   * @param attached whether it's real devices driven from a laptop, and prints the banner
   * @param world whether a simulated world is built
   * @param venues where venues are kept
   * @param pitwall what Pitwall's page is told the program drives
   */
  private record Place(
      String where,
      RunMode mode,
      RunsOn runsOn,
      Attached attached,
      World world,
      Path venues,
      RobotKind pitwall) {}

  /** Whether the program drives real devices from a laptop. */
  private enum Attached {
    YES,
    NO
  }

  /** Whether the program builds a simulated world. */
  private enum World {
    SIMULATED,
    NONE
  }

  private static final Path LAPTOPS_VENUES = Path.of("build", "venues");

  private static final List<Place> PLACES =
      List.of(
          new Place(
              "the simulator, on a laptop",
              RunMode.SIM,
              RunsOn.DESKTOP,
              Attached.NO,
              World.SIMULATED,
              LAPTOPS_VENUES,
              RobotKind.SIMULATED),
          new Place(
              "real devices driven from a laptop (-PhwSim)",
              RunMode.REAL,
              RunsOn.DESKTOP,
              Attached.YES,
              World.NONE,
              LAPTOPS_VENUES,
              RobotKind.REAL),
          new Place(
              "a robot",
              RunMode.REAL,
              RunsOn.SYSTEMCORE,
              Attached.NO,
              World.NONE,
              VenueStore.ROBOT_FOLDER,
              RobotKind.REAL));

  /**
   * Each place builds what it should: the simulator its simulated world, and real devices none,
   * whether on a robot or a laptop. A laptop keeps venues in its own folder, a robot on its disk;
   * Pitwall's page is told real motors for both real ones, so it never enables them.
   */
  @Test
  void eachPlaceBuildsItsOwnWorldFoldersAndPage() {
    for (Place place : PLACES) {
      RobotContainer robot = sim.build(place.mode(), place.runsOn(), ROBOT, SensorErrors.NONE);

      assertThat(HardwareAttached.applies(place.mode(), place.runsOn()))
          .as(place.where())
          .isEqualTo(place.attached() == Attached.YES);
      assertThat(robot.world().isPresent())
          .as(place.where() + ": a simulated world")
          .isEqualTo(place.world() == World.SIMULATED);
      assertThat(robot.simulation().isPresent())
          .as(place.where() + ": the drivetrain's physics")
          .isEqualTo(place.world() == World.SIMULATED);
      assertThat(Robot.venueStore(place.mode(), place.runsOn())).as(place.where()).isPresent();
      Path stored = Robot.venueFolder(place.runsOn());
      assertThat(stored.endsWith(place.venues()))
          .as(place.where() + ": venues in " + stored)
          .isTrue();
      assertThat(HardwareAttached.pitwallKind(place.mode()))
          .as(place.where())
          .isEqualTo(place.pitwall());
    }
    assertThat(Robot.venueStore(RunMode.REPLAY, RunsOn.DESKTOP))
        .as("a replay keeps none")
        .isEmpty();
    assertThat(HardwareAttached.applies(RunMode.REPLAY, RunsOn.DESKTOP)).isFalse();
  }

  /**
   * The banner says what this is before anything else: real motors will move. It lists the devices
   * the laptop will open, those on the CANivore, and those only a robot reaches, here the pivot's
   * motor and the PDH on Systemcore's CAN port, and how the robot is enabled and stopped.
   */
  @Test
  void theBannerWarnsAndListsTheDevicesItOpens() {
    List<String> banner = HardwareAttached.banner(ROBOT.definition());

    assertThat(banner.get(1)).isEqualTo(HardwareAttached.WARNING);
    assertThat(HardwareAttached.WARNING)
        .isEqualTo("HARDWARE-ATTACHED: real motors will move when enabled");
    List<String> opened = between(banner, "It opens these devices:", "Not opened");
    assertThat(opened)
        .hasSize(13)
        .contains(
            "  the gyro: Pigeon 2 1 on the CANivore bench",
            "  front-left drive: TalonFX 10 on the CANivore bench",
            "  back-right CANcoder: CANcoder 21 on the CANivore bench");
    assertThat(between(banner, "Not opened, since only a robot reaches them:", "Nor is anything"))
        .containsExactly(
            "  the PDH: PDH 1 on Systemcore's CAN_S0 port",
            "  pivot motor 11: TalonFX 11 on Systemcore's CAN_S0 port");
    assertThat(banner).anyMatch(line -> line.contains("WPILib's simulator GUI"));
    assertThat(banner).anyMatch(line -> line.startsWith("To stop at once:"));
  }

  /** The CANivores the devices are on, each once; and what Phoenix says of one. */
  @Test
  void eachCanivoreIsAskedAfterOnce() {
    assertThat(HardwareAttached.canivores(ROBOT.definition())).containsExactly(CANIVORE);
    assertThat(HardwareAttached.canivores(ExampleRobot.SIM.definition())).isEmpty();
    assertThat(PhoenixBuses.status(CANIVORE)).startsWith("the CANivore bench: ");
  }

  /** The banner's lines after one that starts so, up to one that starts with another. */
  private static List<String> between(List<String> banner, String after, String before) {
    List<String> lines = new ArrayList<>();
    boolean in = false;
    for (String line : banner) {
      if (line.startsWith(before)) {
        in = false;
      }
      if (in) {
        lines.add(line);
      }
      if (line.startsWith(after)) {
        in = true;
      }
    }
    return lines;
  }

  /** A drivetrain's wiring on a CANivore: the gyro at 1, each module's devices from ID 10 up. */
  private static SwerveWiring drivetrainOn(CanBus canivore) {
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      modules.add(
          SwerveWiring.ModuleWiring.builder()
              .driveId(10 + 3 * i)
              .steerId(11 + 3 * i)
              .encoderOffset(Rotations.of(0))
              .driveInverted(false)
              .steerInverted(false)
              .encoderInverted(false)
              .encoderId(12 + 3 * i)
              .build());
    }
    return SwerveWiring.builder()
        .canBus(canivore)
        .driveController(MechanismWiring.Controller.TALON_FX)
        .steerController(MechanismWiring.Controller.TALON_FX)
        .gyroId(1)
        .gyroYawScale(1.0)
        .modules(modules)
        .build();
  }
}
