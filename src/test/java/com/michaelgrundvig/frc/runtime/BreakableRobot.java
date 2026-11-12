package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.runtime.hardware.Hardware;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveHardware;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO.JointIOInputs;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Proxy;
import java.util.List;
import org.wpilib.telemetry.DiscardTelemetryBackend;
import org.wpilib.telemetry.TelemetryBackend;
import org.wpilib.telemetry.TelemetryEntry;
import org.wpilib.telemetry.TelemetryRegistry;

/**
 * A simulated robot that a test breaks on purpose, in one place at a time, to see what the loop
 * does when that place throws: the dashboard, one joint's reading, or one joint's output.
 */
final class BreakableRobot {
  /** Where the robot is broken. */
  enum Broken {
    NOWHERE,
    /** Every value the dashboard publishes throws. */
    THE_DASHBOARD,
    /** The joint's motor controllers throw as they are read. */
    A_JOINTS_READING,
    /** The joint's motor controllers throw as they are told anything. */
    A_JOINTS_OUTPUT
  }

  /** The competition robot's joints, without cameras, so its loops are quick. */
  static final RobotConfig JOINTS_WITHOUT_CAMERAS =
      RobotConfigBuilder.builder(ExampleRobot.COMPETITION)
          .definition(
              RobotDefinitionBuilder.builder(ExampleRobot.COMPETITION.definition())
                  .cameras(List.of())
                  .build())
          .build();

  /** What every throw here throws. */
  static final String THROWN = "broken on purpose";

  private Broken broken = Broken.NOWHERE;

  /** Breaks the robot there, from now on, and nowhere else. */
  void breakAt(Broken where) {
    broken = where;
  }

  /** Mends it. */
  void mend() {
    broken = Broken.NOWHERE;
  }

  /**
   * Puts the dashboard's values through this robot, so that it can break them. They go nowhere
   * while it isn't broken there.
   */
  void watchTheDashboard() {
    TelemetryRegistry.registerBackend("/Dashboard", new DashboardBackend());
  }

  /** The robot's hardware with one joint's motor controllers handed through this robot. */
  Hardware breakingAJoint(Hardware hardware, Joint breakable) {
    return new Hardware() {
      @Override
      public DriveHardware drive() {
        return hardware.drive();
      }

      @Override
      public JointIO joint(Joint joint) {
        JointIO io = hardware.joint(joint);
        if (!joint.equals(breakable)) {
          return io;
        }
        return new BreakableJoint(io);
      }

      @Override
      public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
        return hardware.absoluteEncoder(joint);
      }

      @Override
      public SensorIO sensor(Sensor.Placed sensor) {
        return hardware.sensor(sensor);
      }

      @Override
      public PdhIO pdh() {
        return hardware.pdh();
      }

      @Override
      public List<CameraIO> cameras() {
        return hardware.cameras();
      }

      @Override
      public void close() {
        hardware.close();
      }
    };
  }

  private void throwIfBrokenAt(Broken where) {
    if (broken == where) {
      throw new IllegalStateException(THROWN);
    }
  }

  /** A joint's motor controllers, which throw where this robot is broken. */
  private final class BreakableJoint implements JointIO {
    private final JointIO joint;

    BreakableJoint(JointIO joint) {
      this.joint = joint;
    }

    @Override
    public void updateInputs(JointIOInputs inputs) {
      throwIfBrokenAt(Broken.A_JOINTS_READING);
      joint.updateInputs(inputs);
    }

    @Override
    public void setPosition(double position, double velocity, double feedforwardVolts) {
      throwIfBrokenAt(Broken.A_JOINTS_OUTPUT);
      joint.setPosition(position, velocity, feedforwardVolts);
    }

    @Override
    public void setVelocity(double velocity) {
      throwIfBrokenAt(Broken.A_JOINTS_OUTPUT);
      joint.setVelocity(velocity);
    }

    @Override
    public void setVoltage(double volts) {
      throwIfBrokenAt(Broken.A_JOINTS_OUTPUT);
      joint.setVoltage(volts);
    }

    @Override
    public void setDutyCycle(double share) {
      throwIfBrokenAt(Broken.A_JOINTS_OUTPUT);
      joint.setDutyCycle(share);
    }

    @Override
    public void stop() {
      throwIfBrokenAt(Broken.A_JOINTS_OUTPUT);
      joint.stop();
    }

    @Override
    public void home(double volts) {
      throwIfBrokenAt(Broken.A_JOINTS_OUTPUT);
      joint.home(volts);
    }

    @Override
    public void resetPosition(double position) {
      joint.resetPosition(position);
    }

    @Override
    public void retune(MechanismControl control) {
      joint.retune(control);
    }
  }

  /**
   * Where the dashboard's values go: nowhere, or, while the robot is broken there, a throw. Its
   * entries are made through a proxy, because a telemetry entry has a method for every kind of
   * value.
   */
  private final class DashboardBackend implements TelemetryBackend {
    private final DiscardTelemetryBackend nowhere = new DiscardTelemetryBackend();

    @Override
    public TelemetryEntry getEntry(String path) {
      TelemetryEntry discarded = nowhere.getEntry(path);
      return (TelemetryEntry)
          Proxy.newProxyInstance(
              TelemetryEntry.class.getClassLoader(),
              new Class<?>[] {TelemetryEntry.class},
              (proxy, method, arguments) -> {
                // A discarding entry is never handed a value: this one says it keeps them.
                if (method.getName().equals("isDiscard")) {
                  return false;
                }
                if (method.getName().startsWith("log")) {
                  throwIfBrokenAt(Broken.THE_DASHBOARD);
                }
                try {
                  return method.invoke(discarded, arguments);
                } catch (InvocationTargetException e) {
                  throw e.getCause();
                }
              });
    }

    @Override
    public void close() {
      nowhere.close();
    }
  }
}
