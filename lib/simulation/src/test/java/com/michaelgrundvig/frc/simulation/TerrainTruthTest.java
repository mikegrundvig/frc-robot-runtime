package com.michaelgrundvig.frc.simulation;

import static com.michaelgrundvig.frc.simulation.KnownGap.Check.stillOpen;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.Terrain;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The truth suite's terrain: which wheels touch the ground as the robot crawls over a bump, against
 * geometry. A rigid frame on four wheels rests on three of them where the ground under the four
 * isn't one plane: the two of the higher diagonal and one more, the fourth hanging where the plane
 * through the three puts it. How far it hangs is the ground's twist under the wheels, (front left +
 * back right) − (front right + back left), whichever of the lower pair it is (the center of gravity
 * lies on the higher diagonal, so geometry alone can't choose).
 *
 * <p>The robot is placed along the crawl a centimeter at a time: a crawl at 0.1 m/s is
 * quasi-static, so this is the ground it would stand on. The frame itself is left out: the robot's
 * definition doesn't say how high it rides, so a frame that would meet the crest isn't checked.
 */
class TerrainTruthTest {
  /** A bump about as tall as REBUILT's, with a long crest along the field's y axis, at x = 5. */
  private static final Terrain BUMP =
      new Terrain(
          List.of(
              new Bump("A bump", new Translation2d(5, 1), new Translation2d(5, 7), 0.165, 1.2)));

  /** A wheel this close to the ground is on it: far below any hang geometry predicts here. */
  private static final double ON_THE_GROUND = 0.001;

  private final KinematicPhysics physics = TestRobot.physics();

  /**
   * Crossing square to the crest, the wheels on each axle stand at one height, so the ground under
   * them is always a plane: all four touch, everywhere along the crawl.
   */
  @Test
  void crossingSquareAllFourWheelsTouch() {
    physics.apply(new Disturbance.Ground(BUMP));
    for (double x = 3; x <= 7; x += 0.01) {
      physics.apply(new Disturbance.Placed(new Pose2d(x, 4, Rotation2d.ZERO)));

      assertThat(twist()).as("at x = %.2f", x).isLessThan(1e-9);
      for (double gap : gaps()) {
        assertThat(Math.abs(gap)).as("at x = %.2f", x).isLessThan(ON_THE_GROUND);
      }
    }
  }

  /**
   * Crossing at 45°, the crest passes under the robot's corners one by one, and where it's between
   * them the ground twists: geometry says three wheels touch and one hangs by the twist, within 2
   * mm, in 99% of the places at least 1 cm into the twist. Today the simulator sets the robot on
   * the plane that best fits the ground under all four wheels: none hangs, and each sits a quarter
   * of the twist off the ground, two above it and two sunk into it.
   */
  @Test
  @KnownGap("the robot stands on the plane that best fits all four wheels' ground")
  void crossingAt45DegreesOneWheelHangsByTheTwist() {
    physics.apply(new Disturbance.Ground(BUMP));
    int twisted = 0;
    int asGeometrySays = 0;
    double worstQuarter = 0;
    double mostTwist = 0;
    for (double x = 3; x <= 7; x += 0.01) {
      physics.apply(new Disturbance.Placed(new Pose2d(x, 4, Rotation2d.fromDegrees(45))));
      double twist = twist();
      if (twist < 0.01) {
        continue;
      }
      twisted++;
      int touching = 0;
      int hanging = 0;
      for (double gap : gaps()) {
        touching += Math.abs(gap) < ON_THE_GROUND ? 1 : 0;
        hanging += Math.abs(gap - twist) < 0.002 ? 1 : 0;
        worstQuarter = Math.max(worstQuarter, Math.abs(Math.abs(gap) - twist / 4));
      }
      asGeometrySays += touching == 3 && hanging == 1 ? 1 : 0;
      mostTwist = Math.max(mostTwist, twist);
    }

    assertThat(twisted).as("places where the ground twists").isGreaterThan(50);
    stillOpen(
        "three wheels touch and one hangs by the twist (worst twist " + mostTwist + " m)",
        asGeometrySays >= 0.99 * twisted,
        asGeometrySays == 0 && worstQuarter < 1e-6,
        asGeometrySays
            + " of "
            + twisted
            + " places as geometry says; each wheel a quarter of the twist off the ground, within "
            + worstQuarter
            + " m");
  }

  /** Where each wheel's tread is over the field, flat. */
  private Translation2d under(int wheel) {
    return physics.wheel(wheel).contactPoint().toTranslation2d();
  }

  /** The ground's twist under the wheels, in meters: zero where it's a plane. */
  private double twist() {
    return Math.abs(
        BUMP.heightAt(under(0))
            + BUMP.heightAt(under(3))
            - BUMP.heightAt(under(1))
            - BUMP.heightAt(under(2)));
  }

  /**
   * How far above the ground under it each wheel's tread is, in meters (below it, negative): where
   * the physics says it meets the ground, against the terrain there. A wheel the physics says is
   * off the ground hangs, however close.
   */
  private double[] gaps() {
    double[] gaps = new double[4];
    for (int i = 0; i < 4; i++) {
      WheelTruth wheel = physics.wheel(i);
      Translation3d contact = wheel.contactPoint();
      double gap = contact.getZ() - BUMP.heightAt(contact.toTranslation2d());
      gaps[i] = wheel.inContact() || gap > ON_THE_GROUND ? gap : ON_THE_GROUND;
    }
    return gaps;
  }
}
