package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.MechanismControl;
import org.junit.jupiter.api.Test;

/**
 * A joint's motor controllers checked against their PDH channels, a loop at a time: the PDH reads
 * what a test says. The arm of {@link PowerSimTest}, on channels 4 and 5, flagged after half a
 * second under 0.5 A while its leader draws more than 2 A.
 */
class DeadChannelsTest {
  private static final long LOOP = 20_000_000L;

  /** The PDH's channels, as a test sets them. */
  private final double[] channels = new double[24];

  private boolean answers = true;

  private final Power power =
      new Power(
          PowerSimTest.ROBOT,
          new PdhIO() {
            @Override
            public void updateInputs(PdhIOInputs inputs) {
              inputs.connected = answers;
              inputs.channelCurrentAmps = channels.clone();
            }
          });

  private final DeadChannels arm =
      new DeadChannels(PowerSimTest.ARM, MechanismControl.DeadChannel.DEFAULTS);

  private long now;

  /** Checks it each loop for a number of loops, its leader drawing a current: their total. */
  private double loops(int count, double leaderAmps) {
    double together = Double.NaN;
    for (int i = 0; i < count; i++) {
      power.read();
      together = arm.check(now, power, true, leaderAmps);
      now += LOOP;
    }
    return together;
  }

  /** Both drawing, nothing's flagged, and their total is the two channels'. */
  @Test
  void bothDrawingNothingIsFlagged() {
    channels[4] = 4.4;
    channels[5] = 4.3;

    double together = loops(40, 4.4);

    assertThat(arm.any()).isTrue();
    assertThat(together).isEqualTo(8.7);
    assertThat(arm.flagged()).isEmpty();
  }

  /**
   * A follower whose channel reads nothing while its leader works is flagged after half a second.
   */
  @Test
  void aChannelDrawingNothingWhileTheJointWorksIsFlagged() {
    channels[4] = 4.4;

    loops(25, 4.4);
    boolean early = arm.flagged().isEmpty();
    loops(1, 4.4);

    assertThat(early).isTrue();
    assertThat(arm.flagged())
        .containsExactly(
            "arm motor 12 draws nothing on channel 5 while arm's motors work: its breaker tripped,"
                + " its power is unplugged, or its block names the wrong channel");
    assertThat(arm.logged()).containsExactly("", arm.flagged().get(0));
  }

  /**
   * Nothing's flagged while the joint draws nothing itself, or the PDH isn't read; and a flag
   * clears as the channel draws again.
   */
  @Test
  void nothingIsFlaggedWhileTheJointIsntWorking() {
    loops(40, 0.5);
    boolean idle = arm.flagged().isEmpty();
    answers = false;
    double unread = loops(40, 4.4);
    answers = true;
    loops(40, 4.4);
    boolean flagged = !arm.flagged().isEmpty();
    channels[4] = 4.4;
    channels[5] = 4.4;
    loops(1, 4.4);

    assertThat(idle).isTrue();
    assertThat(unread).isNaN();
    assertThat(flagged).isTrue();
    assertThat(arm.flagged()).isEmpty();
  }

  /** A joint whose block says no channel has none to check. */
  @Test
  void aJointWithNoChannelsHasNoneToCheck() {
    assertThat(new DeadChannels(JointFixtures.PIVOT, MechanismControl.DeadChannel.DEFAULTS).any())
        .isFalse();
  }
}
