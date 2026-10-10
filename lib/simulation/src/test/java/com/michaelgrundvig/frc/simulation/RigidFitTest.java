package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

/** The best rigid motion through the wheels' travel: exact when they agree, split when not. */
class RigidFitTest {
  private static final Translation2d[] SQUARE = {
    new Translation2d(0.3, 0.3),
    new Translation2d(0.3, -0.3),
    new Translation2d(-0.3, 0.3),
    new Translation2d(-0.3, -0.3)
  };

  private final RigidFit fit = new RigidFit(SQUARE);
  private final double[] motion = new double[3];

  private static final double[] NONE = {0, 0, 0, 0};

  @Test
  void wheelsRollingTogetherMoveTheRobotThatFar() {
    double angle = Math.toRadians(30);
    double[] angles = {angle, angle, angle, angle};
    fit.motion(NONE, angles, new double[] {0.1, 0.1, 0.1, 0.1}, angles, motion);

    assertThat(motion[0]).isCloseTo(0.1 * Math.cos(angle), within(1e-12));
    assertThat(motion[1]).isCloseTo(0.1 * Math.sin(angle), within(1e-12));
    assertThat(motion[2]).isCloseTo(0, within(1e-12));
  }

  @Test
  void wheelsRollingAcrossTheirArmsTurnTheRobot() {
    double[] angles = new double[4];
    double[] rolled = new double[4];
    for (int i = 0; i < 4; i++) {
      angles[i] = SQUARE[i].getAngle().orElseThrow().getRadians() + Math.PI / 2;
      rolled[i] = 0.01 * SQUARE[i].getNorm();
    }
    fit.motion(NONE, angles, rolled, angles, motion);

    assertThat(motion[0]).isCloseTo(0, within(1e-12));
    assertThat(motion[1]).isCloseTo(0, within(1e-12));
    assertThat(motion[2]).isCloseTo(0.01, within(1e-12));
  }

  /**
   * One wheel of four rolling alone moves the robot a quarter of it: the fit splits the difference.
   */
  @Test
  void oneWheelRollingAloneMovesTheRobotAQuarterAsFar() {
    fit.motion(NONE, NONE, new double[] {0.04, 0, 0, 0}, NONE, motion);

    assertThat(motion[0]).isCloseTo(0.01, within(1e-12));
  }

  /** A wheel that steered on the way is taken at the angle halfway, the short way round. */
  @Test
  void aWheelThatSteeredIsTakenHalfway() {
    double[] from = {Math.PI - 0.1, Math.PI - 0.1, Math.PI - 0.1, Math.PI - 0.1};
    double[] to = {-Math.PI + 0.1, -Math.PI + 0.1, -Math.PI + 0.1, -Math.PI + 0.1};
    fit.motion(NONE, from, new double[] {0.1, 0.1, 0.1, 0.1}, to, motion);

    assertThat(motion[0]).isCloseTo(-0.1, within(1e-12));
    assertThat(motion[1]).isCloseTo(0, within(1e-12));
  }

  @Test
  void wheelsAllAtOnePointFitNoMotion() {
    Translation2d point = new Translation2d(0.2, 0.1);
    assertThatThrownBy(() -> new RigidFit(point, point, point))
        .hasMessage("the wheels must not all sit at one point");
  }
}
