package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.ARM;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.CLAW_BEAM;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.HOOD;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.HOPPER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.INDEXER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.INTAKE;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ROLLER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.SHOOTER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.STOWED;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * A definition's {@code with...} verbs go in any order: each part is checked on its own as it's
 * given, and one part against another once, on the finished robot, as {@code RobotConfig.of} takes
 * it. So the same robot written in any order is the same robot, and a mistake across its parts is
 * refused there, saying what it always said.
 */
class DefinitionOrderTest {
  /** The template's drivetrain, body and cameras, with nothing added yet. */
  private static final RobotDefinition START = ExampleRobot.SIM.definition().named("example");

  /** A camera that rides on the arm, so the robot must list the arm. */
  private static final CameraConfig ON_THE_ARM =
      CameraConfigBuilder.builder(ExampleRobot.BACK_LEFT)
          .name("on-arm")
          .forward(Inches.of(2))
          .left(Inches.of(0))
          .up(Inches.of(2))
          .tiltUp(Degrees.of(0))
          .yaw(Degrees.of(0))
          .joint(ARM)
          .build();

  /** The robot as the guide writes it: the PDH, then what it feeds, then what names those. */
  private static final RobotDefinition AS_THE_GUIDE_WRITES_IT =
      START
          .withPdh(Pdh.can(1))
          .withJoints(PIVOT, ROLLER, HOOD, ELEVATOR, ARM, FEEDER, INDEXER, SHOOTER)
          .withSensors(CLAW_BEAM, HOPPER)
          .withCameras(ON_THE_ARM)
          .withPresets(FEEDING, STOWED, INTAKE);

  static Stream<Arguments> orders() {
    return Stream.of(
        Arguments.of("as the guide writes it", AS_THE_GUIDE_WRITES_IT),
        Arguments.of(
            "what names a part before the part",
            START
                .withPresets(FEEDING, STOWED, INTAKE)
                .withCameras(ON_THE_ARM)
                .withSensors(CLAW_BEAM, HOPPER)
                .withJoints(PIVOT, ROLLER, HOOD, ELEVATOR, ARM, FEEDER, INDEXER, SHOOTER)
                .withPdh(Pdh.can(1))),
        Arguments.of(
            "the PDH last, the camera first",
            START
                .withCameras(ON_THE_ARM)
                .withJoints(PIVOT, ROLLER, HOOD, ELEVATOR, ARM, FEEDER, INDEXER, SHOOTER)
                .withPresets(FEEDING, STOWED, INTAKE)
                .withSensors(CLAW_BEAM, HOPPER)
                .withPdh(Pdh.can(1))),
        Arguments.of(
            "the joints in two lots, the presets between",
            START
                .withPdh(Pdh.can(1))
                .withJoints(PIVOT, ROLLER, HOOD, ELEVATOR)
                .withPresets(FEEDING, STOWED, INTAKE)
                .withJoints(ARM, FEEDER, INDEXER, SHOOTER)
                .withSensors(CLAW_BEAM, HOPPER)
                .withCameras(ON_THE_ARM)));
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("orders")
  void theSameRobotInAnyOrderIsTheSameRobotAndAccepted(String order, RobotDefinition written) {
    assertThat(written).isEqualTo(AS_THE_GUIDE_WRITES_IT);
    assertThat(RobotConfig.of(written).definition()).isEqualTo(AS_THE_GUIDE_WRITES_IT);
  }

  static Stream<Arguments> mistakes() {
    return Stream.of(
        Arguments.of(
            "a camera on a joint the robot hasn't",
            (Supplier<RobotDefinition>) () -> START.withCameras(ON_THE_ARM),
            "RobotDefinition[example].joints: camera on-arm's joint, arm, isn't one of the"
                + " robot's joints: list it"),
        Arguments.of(
            "a sensor on a joint the robot hasn't",
            (Supplier<RobotDefinition>) () -> START.withSensors(CLAW_BEAM),
            "RobotDefinition[example].joints: sensor claw beam's joint, pivot, isn't one of the"
                + " robot's joints: list it"),
        Arguments.of(
            "a preset of joints the robot hasn't",
            (Supplier<RobotDefinition>) () -> START.withPresets(INTAKE),
            "RobotDefinition[example].presets: preset intake's joint, pivot, isn't one of the"
                + " robot's joints: list it\nRobotDefinition[example].presets: preset intake's"
                + " joint, roller, isn't one of the robot's joints: list it"),
        Arguments.of(
            "motors fed from a PDH the robot hasn't",
            (Supplier<RobotDefinition>) () -> START.withJoints(PIVOT),
            "RobotDefinition[example].pdh: pivot motor 32 is fed from a PDH's channel, and the"
                + " robot declares no PDH: give its definition one, .withPdh(Pdh.can(1)) at its"
                + " CAN ID"),
        Arguments.of(
            "a second camera of a name the robot has",
            (Supplier<RobotDefinition>) () -> START.withCameras(ExampleRobot.BACK_LEFT),
            "RobotDefinition[example].cameras: two cameras are named back-left"),
        Arguments.of(
            "a camera outside the body",
            (Supplier<RobotDefinition>)
                () ->
                    START.withCameras(
                        CameraConfigBuilder.builder(ExampleRobot.BACK_LEFT)
                            .name("far")
                            .forward(Inches.of(40))
                            .build()),
            "RobotDefinition[example].cameras: camera far is outside the robot, at"));
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("mistakes")
  void aMistakeAcrossPartsIsRefusedOnceTheRobotIsHandedOver(
      String mistake, Supplier<RobotDefinition> written, String refusal) {
    RobotDefinition unfinished = written.get();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotConfig.of(unfinished))
        .withMessageStartingWith(refusal);
    assertThatIllegalArgumentException()
        .isThrownBy(unfinished::checked)
        .withMessageStartingWith(refusal);
  }
}
