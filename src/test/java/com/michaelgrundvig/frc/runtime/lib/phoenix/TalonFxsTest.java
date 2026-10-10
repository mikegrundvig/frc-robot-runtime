package com.michaelgrundvig.frc.runtime.lib.phoenix;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_MOTOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.ControllerSettings;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/** A TalonFXS takes the settings worked out for a TalonFX, in its own terms. */
class TalonFxsTest {
  private static TalonFXConfiguration pivot() {
    return PhoenixMechanism.configure(
            ControllerPlan.of(
                ROBOT.math(),
                PIVOT,
                PIVOT_MOTOR,
                List.of(MechanismWiring.Controller.TALON_FXS.can(21).clockwise()),
                ControllerSettings.control(PIVOT)))
        .leader();
  }

  /**
   * A TalonFX's loop on a CANcoder, remote, fused or synced, is a TalonFXS's on its external
   * feedback the same way, the same CANcoder by its ID, with the same ratios.
   */
  @ParameterizedTest
  @EnumSource(
      value = FeedbackSensorSourceValue.class,
      names = {"RemoteCANcoder", "FusedCANcoder", "SyncCANcoder"})
  void aLoopOnACancoderIsItsExternalFeedbackTheSameWay(FeedbackSensorSourceValue source) {
    TalonFXConfiguration fx = pivot();
    fx.Feedback.FeedbackSensorSource = source;
    fx.Feedback.FeedbackRemoteSensorID = 34;
    fx.Feedback.RotorToSensorRatio = 25;
    fx.Feedback.SensorToMechanismRatio = 1;

    TalonFXSConfiguration fxs = PhoenixMechanism.talonFxs(fx, Motor.NEO);

    assertThat(fxs.ExternalFeedback.ExternalFeedbackSensorSource.name()).isEqualTo(source.name());
    assertThat(fxs.ExternalFeedback.FeedbackRemoteSensorID).isEqualTo(34);
    assertThat(fxs.ExternalFeedback.RotorToSensorRatio).isEqualTo(25);
    assertThat(fxs.ExternalFeedback.SensorToMechanismRatio).isEqualTo(1);
  }

  /**
   * The same slot, limits, direction and ratio; its rotor is its motor's hall sensor, and it's told
   * which motor is plugged in.
   */
  @Test
  void itTakesATalonFxsSettingsWithItsMotorsArrangement() {
    TalonFXConfiguration fx = pivot();

    TalonFXSConfiguration fxs = PhoenixMechanism.talonFxs(fx, Motor.NEO);

    assertThat(fxs.Commutation.MotorArrangement).isEqualTo(MotorArrangementValue.NEO_JST);
    assertThat(fxs.ExternalFeedback.ExternalFeedbackSensorSource)
        .isEqualTo(ExternalFeedbackSensorSourceValue.Commutation);
    assertThat(fxs.ExternalFeedback.SensorToMechanismRatio)
        .isEqualTo(fx.Feedback.SensorToMechanismRatio);
    assertThat(fxs.Slot0.kP).isEqualTo(fx.Slot0.kP);
    assertThat(fxs.Slot0.kG).isEqualTo(fx.Slot0.kG);
    assertThat(fxs.MotorOutput.Inverted).isEqualTo(fx.MotorOutput.Inverted);
    assertThat(fxs.CurrentLimits.StatorCurrentLimit).isEqualTo(fx.CurrentLimits.StatorCurrentLimit);
    assertThat(PhoenixMechanism.talonFxs(fx, Motor.MINION).Commutation.MotorArrangement)
        .isEqualTo(MotorArrangementValue.Minion_JST);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PhoenixMechanism.talonFxs(fx, Motor.KRAKEN_X60))
        .withMessage("a TalonFXS doesn't drive a Kraken X60");
  }
}
