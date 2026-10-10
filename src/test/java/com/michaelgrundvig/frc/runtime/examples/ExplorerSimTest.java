package com.michaelgrundvig.frc.runtime.examples;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.SensorSimulation;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Random;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * The template's robot with a switch on its front bumper, exploring a walled room in the simulator:
 * the field's perimeter, which the test plays, solid, tripping the bumper as it meets a wall.
 */
class ExplorerSimTest {
  /** The bumper switch: a contact switch on the chassis, on Systemcore's port 0. */
  static final Sensor.Switch FRONT_BUMPER =
      Sensor.Switch.contact("front bumper", Part.CHASSIS).on(SensorWiring.port(0).normallyOpen());

  /** The template's robot with the bumper switch. */
  static final RobotConfig ROBOT =
      RobotConfigBuilder.builder(ExampleRobot.SIM)
          .definition(
              RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                  .sensors(List.of(FRONT_BUMPER))
                  .build())
          .build();

  /** Half the bumpers' length and width, and the room's size. */
  private static final double REACH =
      ExampleRobot.SIM.definition().body().bumperLength().baseUnitMagnitude() / 2;

  private static final double SIDE =
      ExampleRobot.SIM.definition().body().bumperWidth().baseUnitMagnitude() / 2;

  private static final double LENGTH = RobotSim.FIELD.length();
  private static final double WIDTH = RobotSim.FIELD.width();

  @RegisterExtension final RobotSim sim = new RobotSim();

  private SensorSimulation sensors;
  private Command explore;

  @BeforeEach
  void build() {
    sim.build(ROBOT, SensorErrors.NONE);
    sensors = Objects.requireNonNull(sim.world().sensors());
    explore = new Explorer(sim.drive(), sim.joints(), FRONT_BUMPER, new Random(1)).explore();
    RobotSim.setEnabled(true);
    sim.scheduler().schedule(explore);
  }

  /**
   * In 40 s it drives into a wall, turns, and drives on, at least three times across a 16.5 m room:
   * every run square to the walls, since each turn is to the room's next quarter.
   */
  @Test
  void itDrivesTurnsAtEachWallAndDrivesOn() {
    List<Double> headingsAtBumps = new ArrayList<>();
    boolean wasTouching = false;
    for (int i = 0; i < RobotLoop.loops(40); i++) {
      // A bump is a wall it drives into; brushing one as it turns beside it isn't.
      boolean driving = sim.drive().getRobotRelativeVelocities().vx > 0.5;
      boolean touching = loopInTheRoom();
      if (touching && !wasTouching && driving) {
        headingsAtBumps.add(sim.world().truePose().getRotation().getDegrees());
      }
      wasTouching = touching;
    }

    assertThat(headingsAtBumps).as("it bumped, turned and drove on").hasSizeGreaterThanOrEqualTo(3);
    for (double heading : headingsAtBumps) {
      assertThat(Math.abs(Math.IEEEremainder(heading, 90))).isCloseTo(0, within(2.0));
    }
    assertThat(sim.scheduler().isRunning(explore)).isTrue();
  }

  /**
   * With its bumper switch lost, it reaches the first wall and pushes against it; its move forward
   * times out at 15 s, and it stops, rather than pushing the wall forever.
   */
  @Test
  void aLostBumperStopsIt() {
    sensors.silence(FRONT_BUMPER, true);
    for (int i = 0; i < RobotLoop.loops(16); i++) {
      loopInTheRoom();
    }

    assertThat(sim.scheduler().isRunning(explore)).as("it stopped").isFalse();
    assertThat(sim.drive().getRobotRelativeVelocities().vx).isCloseTo(0, within(0.05));
  }

  /**
   * One robot loop in the room: a robot whose bumpers have crossed a wall is put back against it,
   * and the bumper switch is pressed wherever the front bumper, one rigid bar, meets a wall.
   * Whether it's pressed.
   */
  private boolean loopInTheRoom() {
    Pose2d robot = sim.world().truePose();
    double pastX = 0;
    double pastY = 0;
    for (double[] corner :
        new double[][] {{REACH, SIDE}, {REACH, -SIDE}, {-REACH, SIDE}, {-REACH, -SIDE}}) {
      Pose2d at = robot.transformBy(new Transform2d(corner[0], corner[1], Rotation2d.ZERO));
      pastX = Math.abs(pastX) > Math.abs(over(at.getX(), LENGTH)) ? pastX : over(at.getX(), LENGTH);
      pastY = Math.abs(pastY) > Math.abs(over(at.getY(), WIDTH)) ? pastY : over(at.getY(), WIDTH);
    }
    if (pastX != 0 || pastY != 0) {
      robot = new Pose2d(robot.getX() - pastX, robot.getY() - pastY, robot.getRotation());
      sim.world().drivetrain().place(robot);
    }
    boolean touching = false;
    for (double across : new double[] {SIDE, 0, -SIDE}) {
      Pose2d at = robot.transformBy(new Transform2d(REACH, across, Rotation2d.ZERO));
      touching |=
          at.getX() <= 0.01
              || at.getX() >= LENGTH - 0.01
              || at.getY() <= 0.01
              || at.getY() >= WIDTH - 0.01;
    }
    sensors.trip(FRONT_BUMPER, touching);
    sim.loop();
    return touching;
  }

  /** How far a point is past a wall at 0 or at {@code far}: negative past 0, 0 between. */
  private static double over(double at, double far) {
    return at < 0 ? at : at > far ? at - far : 0;
  }
}
