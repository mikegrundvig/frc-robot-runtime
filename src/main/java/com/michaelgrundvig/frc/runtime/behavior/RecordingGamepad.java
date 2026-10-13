package com.michaelgrundvig.frc.runtime.behavior;

import com.michaelgrundvig.frc.world.record.BindingFired;
import com.michaelgrundvig.frc.world.record.Identity;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.event.EventLoop;

/**
 * A drive team's gamepad. It is WPILib's {@link CommandGamepad} with one difference: its buttons
 * record what is bound to them ({@link RecordingTrigger}, {@link Bindings}), so the robot knows
 * which button runs which command. A team binds exactly as on WPILib's gamepad, {@code
 * driver.back().onTrue(...)} or {@code operator.rightTrigger().whileTrue(...)}, and its sticks read
 * as WPILib's do. {@link Bindings#gamepad} makes one.
 */
public final class RecordingGamepad extends CommandGamepad {
  /** WPILib's own threshold for the trigger axes: past half-way counts as pressed. */
  private static final double HALF_WAY = 0.5;

  private final Scheduler scheduler;

  /** Its Driver Station port, which is how the log's facts name it. */
  private final int port;

  private final String name;
  private final Bindings bindings;

  /** Each button's trigger. A button has one for each event loop, made once, as WPILib's are. */
  private final Map<EventLoop, Map<String, RecordingTrigger>> triggers = new HashMap<>();

  RecordingGamepad(Scheduler scheduler, int port, String name, Bindings bindings) {
    super(scheduler, port);
    this.scheduler = scheduler;
    this.port = port;
    this.name = name;
    this.bindings = bindings;
  }

  /** How its bindings name it: "driver". */
  public String name() {
    return name;
  }

  @Override
  public RecordingTrigger button(int button) {
    return button(button, scheduler.getDefaultEventLoop());
  }

  /**
   * A button, by its index. Each of the gamepad's named buttons ({@code faceDown()}, {@code back()}
   * and the rest) comes through here, and is named as WPILib's {@link Gamepad.Button} names it:
   * "face down".
   */
  @Override
  public RecordingTrigger button(int button, EventLoop loop) {
    RecordingTrigger.Control on =
        new RecordingTrigger.Control(port, BindingFired.Control.BUTTON, button);
    return trigger(loop, buttonName(button), on, () -> getHID().getHID().getRawButton(button));
  }

  @Override
  public RecordingTrigger leftTrigger(double threshold, EventLoop loop) {
    return axis(
        loop,
        "left trigger",
        BindingFired.Control.LEFT_TRIGGER,
        Gamepad.Axis.LEFT_TRIGGER,
        threshold);
  }

  @Override
  public RecordingTrigger rightTrigger(double threshold, EventLoop loop) {
    return axis(
        loop,
        "right trigger",
        BindingFired.Control.RIGHT_TRIGGER,
        Gamepad.Axis.RIGHT_TRIGGER,
        threshold);
  }

  /**
   * A trigger axis pushed past a threshold. At WPILib's usual threshold it is named for the axis
   * alone, "right trigger". At any other it says so: "right trigger past 0.2".
   */
  private RecordingTrigger axis(
      EventLoop loop,
      String axis,
      BindingFired.Control which,
      Gamepad.Axis read,
      double threshold) {
    String name = threshold == HALF_WAY ? axis : axis + " past " + threshold;
    RecordingTrigger.Control on = new RecordingTrigger.Control(port, which, Identity.NONE);
    return trigger(loop, name, on, () -> getHID().getRawAxis(read.value) > threshold);
  }

  private RecordingTrigger trigger(
      EventLoop loop, String button, RecordingTrigger.Control on, BooleanSupplier pressed) {
    return triggers
        .computeIfAbsent(loop, each -> new HashMap<>())
        .computeIfAbsent(
            button,
            each -> new RecordingTrigger(scheduler, loop, name, button, on, pressed, bindings));
  }

  /** A button's name, from WPILib's name for it: FACE_DOWN is "face down". */
  private static String buttonName(int index) {
    for (Gamepad.Button button : Gamepad.Button.values()) {
      if (button.value == index) {
        return button.name().toLowerCase(Locale.ROOT).replace('_', ' ');
      }
    }
    return "button " + index;
  }
}
