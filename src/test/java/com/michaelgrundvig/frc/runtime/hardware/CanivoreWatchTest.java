package com.michaelgrundvig.frc.runtime.hardware;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.CanDevice;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.util.AlertDataJNI;

/**
 * A robot with its drivetrain's three devices on the CANivore {@code drive}, a pivot's motor on
 * {@code arm}, and its PDH on Systemcore's own port, through a session on Systemcore: Phoenix sees
 * both, then loses {@code drive}, then sees it again. Each row is a moment on the robot's clock,
 * what Phoenix says, and the alerts a dashboard shows then.
 */
class CanivoreWatchTest {
  private static final CanBus.Canivore DRIVE = new CanBus.Canivore("drive");
  private static final CanBus.Canivore ARM = new CanBus.Canivore("arm");
  private static final List<CanDevice> DEVICES =
      List.of(
          device("the gyro", CanDevice.Kind.PIGEON_2, DRIVE, 1),
          device("front-left drive", CanDevice.Kind.TALON_FX, DRIVE, 10),
          device("front-left CANcoder", CanDevice.Kind.CANCODER, DRIVE, 20),
          device("pivot motor", CanDevice.Kind.TALON_FX, ARM, 11),
          device("the PDH", CanDevice.Kind.PDH, CanBus.port(CanBus.Systemcore.CAN_S0), 1));
  private static final String UNPLUGGED = "CANNotFound: the CANivore isn't there";
  private static final String DRIVE_LOST =
      "HIGH canivore/drive The CANivore drive isn't seen, so its 3 devices don't answer: check its"
          + " USB cable and its CAN power, and that CTRE's canivore-usb package is installed"
          + " (docs/canivore.md) ("
          + UNPLUGGED
          + ")";

  private final Map<CanBus.Canivore, String> unseen = new HashMap<>();
  private final List<CanBus.Canivore> asked = new ArrayList<>();
  private final AtomicLong nanos = new AtomicLong();
  private CanivoreWatch watch = watch(RunsOn.SYSTEMCORE);

  @AfterEach
  void close() {
    watch.close();
  }

  /**
   * A moment: the robot's clock, what Phoenix doesn't see, whether it's asked, the alerts shown.
   */
  private record Moment(
      String what,
      double seconds,
      List<CanBus.Canivore> unplugged,
      boolean asks,
      List<String> shown) {}

  @Test
  void aCanivoreNotSeenIsSaidUntilItsSeenAgain() {
    List<Moment> session =
        List.of(
            new Moment("both seen", 0, List.of(), true, List.of()),
            new Moment("drive unplugged, not asked yet", 0.5, List.of(DRIVE), false, List.of()),
            new Moment("asked a second on", 1.0, List.of(DRIVE), true, List.of(DRIVE_LOST)),
            new Moment("still out", 2.0, List.of(DRIVE), true, List.of(DRIVE_LOST)),
            new Moment("plugged back in", 3.0, List.of(), true, List.of()));
    for (Moment moment : session) {
      unseen.clear();
      moment.unplugged().forEach(canivore -> unseen.put(canivore, UNPLUGGED));
      asked.clear();
      nanos.set((long) (moment.seconds() * 1e9));
      watch.periodic();
      assertThat(asked)
          .as(moment.what() + ": asked")
          .isEqualTo(moment.asks() ? List.of(DRIVE, ARM) : List.of());
      assertThat(shown()).as(moment.what()).isEqualTo(moment.shown());
    }
  }

  @Test
  void onALaptopItSaysToCheckWithTunerX() {
    watch.close();
    watch = watch(RunsOn.DESKTOP);
    unseen.put(ARM, UNPLUGGED);
    watch.periodic();
    assertThat(shown())
        .containsExactly(
            "HIGH canivore/arm The CANivore arm isn't seen, so its device doesn't answer: check its"
                + " USB cable and its CAN power, and that Phoenix Tuner X sees it from this"
                + " computer ("
                + UNPLUGGED
                + ")");
  }

  @Test
  void aRobotWithNoCanivoreWatchesNone() {
    watch.close();
    watch =
        new CanivoreWatch(List.of(DEVICES.get(4)), RunsOn.SYSTEMCORE, this::phoenix, nanos::get);
    watch.periodic();
    assertThat(asked).isEmpty();
  }

  private CanivoreWatch watch(RunsOn runsOn) {
    return new CanivoreWatch(DEVICES, runsOn, this::phoenix, nanos::get);
  }

  /** Phoenix, as the test says: why it doesn't see a CANivore, or nothing. */
  private Optional<String> phoenix(CanBus.Canivore canivore) {
    asked.add(canivore);
    return Optional.ofNullable(unseen.get(canivore));
  }

  private static CanDevice device(String name, CanDevice.Kind kind, CanBus bus, int id) {
    return new CanDevice(name, kind, bus, id, Optional.empty());
  }

  /** The robot's CAN alerts a dashboard shows now: level, key and text. */
  private static List<String> shown() {
    List<String> active = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      if (alert.group.equals("CAN") && alert.activeStartTime != 0) {
        String level = alert.level == AlertDataJNI.LEVEL_HIGH ? "HIGH" : "MEDIUM";
        active.add(level + " " + alert.id + " " + alert.text);
      }
    }
    return active;
  }
}
