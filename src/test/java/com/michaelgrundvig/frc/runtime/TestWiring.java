package com.michaelgrundvig.frc.runtime;

import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiringModuleWiringBuilder;
import java.util.ArrayList;
import java.util.List;

/**
 * Drivetrain wirings for the runtime's tests, written as a robot's block writes them: a gyro, then
 * each corner's drive, steering and encoder.
 */
public final class TestWiring {
  private TestWiring() {}

  /**
   * A module of TalonFXs on a bus, its drive, steering and CANcoder at these IDs, its CANcoder
   * offset 0, every device turning counterclockwise for positive.
   */
  public static SwerveWiring.ModuleWiring talons(CanBus bus, int drive, int steer, int cancoder) {
    return SwerveWiringModuleWiringBuilder.builder()
        .drive(MechanismWiring.Controller.TALON_FX.can(bus, drive))
        .steering(MechanismWiring.Controller.TALON_FX.can(bus, steer))
        .encoder(MechanismWiring.CANCODER.can(bus, cancoder).offset(Rotations.of(0)))
        .build();
  }

  /**
   * A drivetrain of TalonFXs on one bus, its gyro at an ID and its modules' devices three IDs each
   * from {@code first}: the front left's drive at {@code first}, its steering the next and its
   * CANcoder the one after, then the front right's, and on.
   */
  public static Drivetrain talons(Drivetrain drivetrain, CanBus bus, int gyro, int first) {
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      int drive = first + 3 * i;
      modules.add(talons(bus, drive, drive + 1, drive + 2));
    }
    return wired(drivetrain, SwerveWiring.PIGEON_2.can(bus, gyro), modules);
  }

  /**
   * A drivetrain wired with a gyro and four modules: front left, front right, back left, back
   * right.
   */
  public static Drivetrain wired(
      Drivetrain drivetrain, SwerveWiring.Gyro gyro, List<SwerveWiring.ModuleWiring> modules) {
    return drivetrain
        .gyro(gyro)
        .frontLeft(modules.get(0).drive(), modules.get(0).steering(), modules.get(0).encoder())
        .frontRight(modules.get(1).drive(), modules.get(1).steering(), modules.get(1).encoder())
        .backLeft(modules.get(2).drive(), modules.get(2).steering(), modules.get(2).encoder())
        .backRight(modules.get(3).drive(), modules.get(3).steering(), modules.get(3).encoder());
  }

  /**
   * A robot, its gyro mounted here: its drivetrain wired on Systemcore's CAN, as only a wired
   * drivetrain says where its gyro sits. The simulator runs it as it runs the robot unwired.
   */
  public static RobotDefinition gyroAt(RobotDefinition robot, ImuMount mount) {
    Drivetrain wired = talons(robot.drivetrain(), CanBus.SYSTEMCORE, 0, 1);
    SwerveWiring wiring = wired.wiring().orElseThrow();
    return RobotDefinitionBuilder.builder(robot)
        .drivetrain(wired(robot.drivetrain(), wiring.gyro().at(mount), wiring.modules()))
        .build();
  }
}
