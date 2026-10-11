package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;

/**
 * One of a joint's CTRE motor controllers: a TalonFX or a TalonFXS.
 *
 * <p>A TalonFX is the controller built into a Kraken or a Falcon motor. A TalonFXS is a separate
 * controller that drives another maker's motor, such as a NEO, through the motor's connector.
 * Phoenix has a class for each, and a configuration class for each, but both take the same control
 * requests and report the same signals. This type lets {@link JointIOTalon} treat the two alike:
 * its settings are worked out once, as a TalonFX's, and a TalonFXS is given them in its own terms
 * ({@link PhoenixMechanism#talonFxs}).
 *
 * <p>Every {@code apply} takes a timeout in seconds. With a timeout above zero the call waits that
 * long for the device to confirm, which is right at startup. With a timeout of zero it sends the
 * settings and returns at once, which is what the robot's loop needs.
 */
sealed interface PhoenixTalon {
  /** The controller as the type that takes control requests and reports signals. */
  CommonTalon device();

  /** The same controller as the type Phoenix's restart and bus functions take. */
  ParentDevice parent();

  /** Sends the controller its whole configuration, worked out as a TalonFX's. */
  StatusCode apply(TalonFXConfiguration settings, double timeoutSeconds);

  /** Sends the controller its position loop's gains alone. */
  StatusCode apply(Slot0Configs gains, double timeoutSeconds);

  /** Sends the controller its speed loop's gains alone. */
  StatusCode apply(Slot1Configs gains, double timeoutSeconds);

  /** Sends the controller its current limits alone. */
  StatusCode apply(CurrentLimitsConfigs limits, double timeoutSeconds);

  /**
   * Opens the controller at a CAN ID.
   *
   * @param kind which controller the joint's wiring names
   * @param motor the motor it drives, which a TalonFXS has to be told
   */
  static PhoenixTalon open(MechanismWiring.Controller kind, int id, CANBus bus, Motor motor) {
    if (kind == MechanismWiring.Controller.TALON_FXS) {
      return new Fxs(new TalonFXS(id, bus), motor);
    }
    return new Fx(new TalonFX(id, bus));
  }

  /** A TalonFX. */
  record Fx(TalonFX talon) implements PhoenixTalon {
    @Override
    public CommonTalon device() {
      return talon;
    }

    @Override
    public ParentDevice parent() {
      return talon;
    }

    @Override
    public StatusCode apply(TalonFXConfiguration settings, double timeoutSeconds) {
      return talon.getConfigurator().apply(settings, timeoutSeconds);
    }

    @Override
    public StatusCode apply(Slot0Configs gains, double timeoutSeconds) {
      return talon.getConfigurator().apply(gains, timeoutSeconds);
    }

    @Override
    public StatusCode apply(Slot1Configs gains, double timeoutSeconds) {
      return talon.getConfigurator().apply(gains, timeoutSeconds);
    }

    @Override
    public StatusCode apply(CurrentLimitsConfigs limits, double timeoutSeconds) {
      return talon.getConfigurator().apply(limits, timeoutSeconds);
    }
  }

  /**
   * A TalonFXS, and the motor plugged into it.
   *
   * @param motor the motor it drives: its whole configuration says which, so the controller knows
   *     how to commutate it
   */
  record Fxs(TalonFXS talon, Motor motor) implements PhoenixTalon {
    @Override
    public CommonTalon device() {
      return talon;
    }

    @Override
    public ParentDevice parent() {
      return talon;
    }

    @Override
    public StatusCode apply(TalonFXConfiguration settings, double timeoutSeconds) {
      return talon
          .getConfigurator()
          .apply(PhoenixMechanism.talonFxs(settings, motor), timeoutSeconds);
    }

    @Override
    public StatusCode apply(Slot0Configs gains, double timeoutSeconds) {
      return talon.getConfigurator().apply(gains, timeoutSeconds);
    }

    @Override
    public StatusCode apply(Slot1Configs gains, double timeoutSeconds) {
      return talon.getConfigurator().apply(gains, timeoutSeconds);
    }

    @Override
    public StatusCode apply(CurrentLimitsConfigs limits, double timeoutSeconds) {
      return talon.getConfigurator().apply(limits, timeoutSeconds);
    }
  }
}
