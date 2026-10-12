package com.michaelgrundvig.frc.runtime.examples;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.WpilibLocalizer;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;

/** The localizers guide's examples (docs/localizers.md): a robot that chooses its localizer. */
public final class LocalizerExamples {
  private LocalizerExamples() {}

  /** The guide's robot on WPILib's pose estimator: its block, with the one line that chooses it. */
  public static final RobotConfig ON_WPILIB =
      RobotConfig.builder()
          .definition(ExampleRobot.SIM.definition())
          .vision(ExampleRobot.SIM.vision())
          .estimator(EstimatorConfig.DEFAULTS)
          .localizer(WpilibLocalizer::new)
          .build();

  /** The same, trusting WPILib's odometry to 3 cm and 1 degree between camera measurements. */
  public static final RobotConfig ON_WPILIB_TIGHTER =
      RobotConfig.builder()
          .definition(ExampleRobot.SIM.definition())
          .vision(ExampleRobot.SIM.vision())
          .estimator(EstimatorConfig.DEFAULTS)
          .localizer(WpilibLocalizer.with(0.03, Math.toRadians(1)))
          .build();
}
