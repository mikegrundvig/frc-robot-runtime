package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The catalog of what can go wrong: each part's faults named apart, each number starting inside its
 * range, and the numbers that depend on the world taken from it.
 */
class FaultCatalogTest {
  private static final SensorErrors.GyroErrors BUILT = SensorErrors.TYPICAL.gyro();

  /** Every fault of every part, as a world might have them. */
  private static List<List<FaultCatalog.Fault>> parts() {
    return List.of(
        FaultCatalog.Camera.ALL,
        FaultCatalog.Imu.all(BUILT),
        FaultCatalog.Module.all(0.5, 0.985),
        List.of(
            FaultCatalog.Robot.SHOVED,
            FaultCatalog.Robot.PUSHED,
            FaultCatalog.Robot.HIT,
            FaultCatalog.Robot.TILTED,
            FaultCatalog.Robot.SLIPPING,
            FaultCatalog.Robot.placed(16.5, 8.1),
            FaultCatalog.Robot.TO_A_BUMP),
        List.of(FaultCatalog.Field.tagMoved(1, 32, 26)));
  }

  @Test
  void eachPartsFaultsAreNamedApartAndSayWhatTheyDo() {
    for (List<FaultCatalog.Fault> part : parts()) {
      List<String> names = new ArrayList<>();
      for (FaultCatalog.Fault fault : part) {
        names.add(fault.name());
        assertThat(fault.does()).as(fault.name()).isNotBlank();
        assertThat(fault.undone()).as(fault.name()).isNotBlank();
        assertThat(fault.parameters().stream().map(FaultCatalog.Parameter::name))
            .as(fault.name())
            .doesNotHaveDuplicates();
      }
      assertThat(names).doesNotHaveDuplicates();
    }
  }

  /** Events can't be undone; everything else lasts until it is. */
  @Test
  void eventsHappenOnceAndTheRestLast() {
    assertThat(FaultCatalog.Robot.HIT.lasting()).isFalse();
    assertThat(FaultCatalog.Imu.RESET.lasting()).isFalse();
    assertThat(FaultCatalog.Camera.UNPLUGGED.lasting()).isTrue();
  }

  /** A part built off the usual range widens it, and starts there. */
  @Test
  void aNumberStartsWhereThePartWasBuiltHoweverFarOutThatIs() {
    SensorErrors.GyroErrors farOut =
        SensorErrors.GyroErrors.builder()
            .scale(1.8)
            .driftDegreesPerMinute(0)
            .noiseDegrees(0)
            .build();
    FaultCatalog.Parameter scale = FaultCatalog.Imu.errors(farOut).parameter(FaultCatalog.SCALE);

    assertThat(scale.min()).isEqualTo(0.5);
    assertThat(scale.max()).isEqualTo(1.8);
    assertThat(scale.initial()).isEqualTo(1.8);
    assertThat(FaultCatalog.Module.zeroedOff(0.5).parameter(FaultCatalog.OFFSET).initial())
        .isEqualTo(0.5);
  }

  @Test
  void theWorldsOwnNumbersComeFromIt() {
    FaultCatalog.Fault placed = FaultCatalog.Robot.placed(16.5, 8.1);
    assertThat(placed.parameter(FaultCatalog.X).max()).isEqualTo(16.5);
    assertThat(placed.parameter(FaultCatalog.Y).initial()).isEqualTo(4.05);
    assertThat(FaultCatalog.Field.tagMoved(1, 32, 26).parameter(FaultCatalog.TAG).initial())
        .isEqualTo(26);
  }

  @Test
  void partsAreNamedForThePage() {
    assertThat(FaultCatalog.camera("front-left")).isEqualTo("Camera front-left");
    assertThat(FaultCatalog.module(2)).isEqualTo("Back left module");
  }

  @Test
  void aNumberMustStartInsideItsRangeAndBeOneTheFaultTakes() {
    assertThatThrownBy(() -> new FaultCatalog.Parameter("Grip", "", 0, 1, 2))
        .hasMessage("Grip: 2.0 must be between 0.0 and 1.0");
    assertThatThrownBy(() -> FaultCatalog.Robot.HIT.parameter(FaultCatalog.GRIP))
        .hasMessage("Hit takes no Grip");
  }
}
