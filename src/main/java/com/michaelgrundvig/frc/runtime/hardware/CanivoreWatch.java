package com.michaelgrundvig.frc.runtime.hardware;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.LongSupplier;
import org.wpilib.util.Alert;

/**
 * Each CANivore the robot's devices are on, asked of Phoenix once a second: one it doesn't see is a
 * high alert naming it, Phoenix's reason, and how many devices it takes with it, so an unplugged
 * CANivore, or one Systemcore has no driver for, shows as that rather than as every device on it
 * failing to answer. The alert clears once Phoenix sees it again (docs/canivore.md).
 */
public final class CanivoreWatch implements AutoCloseable {
  private static final long EVERY_NANOS = 1_000_000_000L;

  private final Function<CanBus.Canivore, Optional<String>> notSeen;
  private final LongSupplier nanos;
  private final List<Watched> watched = new ArrayList<>();
  private long next = Long.MIN_VALUE;

  /** One CANivore: what its alert says while it isn't seen, and the alert. */
  private record Watched(CanBus.Canivore canivore, String text, Alert alert) {}

  /**
   * @param devices the robot's CAN devices ({@link RobotDefinition#devices}), each naming its bus
   * @param runsOn the computer: on Systemcore, the alert says the CANivore needs CTRE's package
   * @param notSeen why Phoenix doesn't see a CANivore, or empty while it does ({@link
   *     PhoenixBuses#notSeen})
   * @param nanos a clock, for once a second
   */
  public CanivoreWatch(
      List<CanDevice> devices,
      RunsOn runsOn,
      Function<CanBus.Canivore, Optional<String>> notSeen,
      LongSupplier nanos) {
    this.notSeen = notSeen;
    this.nanos = nanos;
    Map<CanBus.Canivore, Integer> onEach = new LinkedHashMap<>();
    for (CanDevice device : devices) {
      if (device.bus() instanceof CanBus.Canivore canivore) {
        onEach.merge(canivore, 1, Integer::sum);
      }
    }
    onEach.forEach(
        (canivore, count) -> {
          String text = text(canivore, runsOn, count);
          Alert alert = new Alert("CAN", "canivore/" + canivore.name(), text, Alert.Level.HIGH);
          watched.add(new Watched(canivore, text, alert));
        });
  }

  /** A robot's: Phoenix asked of the robot's own CANivores. */
  public static CanivoreWatch on(RobotDefinition definition, RunsOn runsOn) {
    return new CanivoreWatch(definition.devices(), runsOn, PhoenixBuses::notSeen, System::nanoTime);
  }

  /** Once each loop: Phoenix is asked once a second. */
  public void periodic() {
    long now = nanos.getAsLong();
    if (now < next) {
      return;
    }
    next = now + EVERY_NANOS;
    for (Watched each : watched) {
      Optional<String> why = notSeen.apply(each.canivore());
      why.ifPresent(reason -> each.alert().setText(each.text() + " (" + reason + ")"));
      each.alert().set(why.isPresent());
    }
  }

  private static String text(CanBus.Canivore canivore, RunsOn runsOn, int devices) {
    String said =
        "The CANivore "
            + canivore.name()
            + " isn't seen, so its "
            + (devices == 1 ? "device doesn't" : devices + " devices don't")
            + " answer: check its USB cable and its CAN power";
    if (runsOn == RunsOn.SYSTEMCORE) {
      return said + ", and that CTRE's canivore-usb package is installed (docs/canivore.md)";
    }
    return said + ", and that Phoenix Tuner X sees it from this computer";
  }

  @Override
  public void close() {
    watched.forEach(each -> each.alert().close());
  }
}
