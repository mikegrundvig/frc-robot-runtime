package com.michaelgrundvig.frc.runtime.subsystems.drive;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import org.wpilib.util.Alert;

/**
 * What the drivetrain tells the drive team is wrong with its hardware: a module that has
 * disconnected, a module whose devices did not take their configuration, a module whose absolute
 * encoder says its magnet is out of place, and odometry readings that have stopped arriving.
 */
final class DriveAlerts implements AutoCloseable {
  /**
   * Wheels that report connected but send no high-rate readings for this long have lost whatever
   * reads them. On a robot that is the thread that records those readings.
   */
  private static final double STOPPED_SECONDS = 0.1;

  private final Alert[] moduleDisconnected = new Alert[Module.COUNT];
  private final Alert[] moduleUnconfigured = new Alert[Module.COUNT];
  private final Alert[] moduleMagnetRed = new Alert[Module.COUNT];
  private final Alert odometryStopped;

  /** How many loops in a row the wheels have reported connected but sent no high-rate readings. */
  private int loopsWithoutReadings;

  /**
   * @param group the name the alerts are grouped under: "Drive" on the robot
   */
  DriveAlerts(String group) {
    odometryStopped =
        new Alert(
            group,
            "odometryStopped",
            "Odometry stopped: the wheels report connected, but no readings have arrived for "
                + STOPPED_SECONDS
                + " s. Restart the robot code.",
            Alert.Level.HIGH);
    for (int i = 0; i < Module.COUNT; i++) {
      String module = Module.NAMES.get(i);
      moduleDisconnected[i] =
          new Alert(
              group,
              "module" + module + "Disconnected",
              module
                  + " swerve module disconnected: driving on the other three, odometry from them."
                  + " Check its motor controllers' CAN wiring and power",
              Alert.Level.HIGH);
      moduleUnconfigured[i] =
          new Alert(
              group,
              "module" + module + "Unconfigured",
              module + " swerve module not configured",
              Alert.Level.HIGH);
      moduleMagnetRed[i] =
          new Alert(
              group,
              "module" + module + "MagnetRed",
              module
                  + " swerve module's CANcoder says its magnet is too close or too far (red): its"
                  + " angle isn't to be trusted. Check the magnet is seated on the shaft and the"
                  + " CANcoder's spacing before zeroing the module again",
              Alert.Level.HIGH);
    }
  }

  /** Raises and clears the alerts for what was read this loop. */
  void check(Module[] modules, DriveReadings readings) {
    checkOdometryStillArrives(readings);
    for (int i = 0; i < Module.COUNT; i++) {
      moduleDisconnected[i].set(!modules[i].isConnected());
      checkConfigured(i, modules[i].unconfigured());
      moduleMagnetRed[i].set(modules[i].encoderMagnet() == ModuleIO.MagnetHealth.RED);
    }
  }

  /**
   * Odometry has stopped when every module is connected and yet no high-rate reading has come for a
   * while. A disconnected module explains missing readings by itself, and has its own alert.
   */
  private void checkOdometryStillArrives(DriveReadings readings) {
    boolean readingsAreMissing = readings.allModulesConnected() && !readings.wheelReadingsArrived();
    if (readingsAreMissing) {
      loopsWithoutReadings++;
    } else {
      loopsWithoutReadings = 0;
    }
    odometryStopped.set(loopsWithoutReadings > RobotLoop.loops(STOPPED_SECONDS));
  }

  /**
   * Raises a module's alert while any of its devices runs unconfigured. The alert's text names the
   * devices, and is written when the alert is raised, not again in every loop it stays up.
   *
   * @param unconfigured the module's devices that did not take their configuration, or empty
   */
  private void checkConfigured(int module, String unconfigured) {
    boolean runsUnconfigured = !unconfigured.isEmpty();
    boolean alreadyRaised = moduleUnconfigured[module].get();
    if (runsUnconfigured && !alreadyRaised) {
      moduleUnconfigured[module].setText(
          Module.NAMES.get(module)
              + " swerve module: "
              + unconfigured
              + " didn't take its configuration. The drivetrain holds still, and its moves wait,"
              + " until it does: it's sent its configuration again once it answers. Check its CAN"
              + " ID and wiring");
    }
    moduleUnconfigured[module].set(runsUnconfigured);
  }

  boolean isModuleUnconfiguredActive(int module) {
    return moduleUnconfigured[module].get();
  }

  boolean isModuleMagnetRedActive(int module) {
    return moduleMagnetRed[module].get();
  }

  boolean isOdometryStoppedActive() {
    return odometryStopped.get();
  }

  /** Releases the alerts, so a test can build another drivetrain. */
  @Override
  public void close() {
    for (Alert alert : moduleDisconnected) {
      alert.close();
    }
    for (Alert alert : moduleUnconfigured) {
      alert.close();
    }
    for (Alert alert : moduleMagnetRed) {
      alert.close();
    }
    odometryStopped.close();
  }
}
