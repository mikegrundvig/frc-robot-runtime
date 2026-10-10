package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.gearing;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.GoldenFile;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.simulation.SimDeviceSim;

/**
 * What a joint on a SPARK asks of it each loop, recorded: each request's setpoint, in the SPARK's
 * units, its slot, and the feedforward the robot works out and sends with it, in volts, as REV's
 * simulated SPARK holds them. The fixtures' pivot (its gravity along one cosine, where its weight
 * puts it, at a given angle, and half a turn back), its elevator and its roller, each at a point of
 * a profile and at a speed, from where it rests and while its motor turns; and the pivot with its
 * loop closed on the encoder on its data port. They must come out as recorded: a change to how the
 * robot works out a SPARK's requests that's meant to change nothing is shown to change nothing.
 * When a change is meant to move them, record them again with {@code GOLDEN_UPDATE=1} and say why
 * they moved (docs/testing.md).
 *
 * <p>Nothing here steps REV's simulation: its speeds carry a random noise, so what the SPARK does
 * with a request isn't the same twice. What it's asked is.
 */
@ExtendWith(RobotSim.class)
class SparkRequestsRecordedTest {
  private final List<String> rows =
      new ArrayList<>(List.of("case,step,setpoint,slot,feedforward volts"));

  @Test
  void theirRequestsAreAsRecorded() throws IOException {
    requests(
        "pivot seeded",
        unmotored(PIVOT)
            .motor(Motor.NEO, gearing(25), SPARK_MAX.can(55).clockwise())
            .retuned(control -> control.withKs(0.2)),
        25 / (2 * Math.PI));
    requests(
        "pivot at 30° back",
        unmotored(PIVOT)
            .motor(Motor.NEO, gearing(25), SPARK_MAX.can(56))
            .retuned(control -> control.withKs(0.1).withKv(0.6).withKg(0.4, Degrees.of(-30))),
        25 / (2 * Math.PI));
    requests(
        "pivot lowering",
        unmotored(PIVOT)
            .motor(Motor.NEO, gearing(25), SPARK_MAX.can(57))
            .retuned(control -> control.withKg(0.4, Degrees.of(180))),
        25 / (2 * Math.PI));
    requests(
        "elevator",
        unmotored(ELEVATOR)
            .motors(
                Motor.NEO_VORTEX,
                gearing(10),
                Inches.of(1),
                SPARK_FLEX.can(58),
                SPARK_FLEX.can(59).clockwise())
            .retuned(control -> control.withKs(0.15)),
        10 / (2 * Math.PI * 0.0254));
    requests(
        "roller",
        unmotored(ROLLER)
            .motor(Motor.NEO_550, gearing(3), SPARK_MAX.can(60))
            .retuned(control -> control.withKs(0.1).withKv(0.02)),
        3 / (2 * Math.PI));
    closedOnItsEncoder();

    GoldenFile.check(getClass(), "spark-requests.csv", rows.get(0), rows);
  }

  /**
   * One joint's requests: at a point of a profile, still and moving, from where it rests and while
   * its motor turns, and at a speed either way.
   *
   * @param rotorPerUnit its rotor's turns for each radian or meter of the joint
   */
  private void requests(String name, Joint joint, double rotorPerUnit) {
    JointIOSpark io = new JointIOSpark(ROBOT.with(joint).math(), joint, ControllerPins.NONE);
    SparkRelativeEncoderSim rotor =
        io.leader() instanceof SparkFlex flex
            ? new SparkRelativeEncoderSim(flex)
            : new SparkRelativeEncoderSim((SparkMax) io.leader());
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    boolean slides = joint instanceof Joint.Prismatic;

    io.updateInputs(inputs);
    io.setPosition(joint.restPosition(), 0, 0);
    asked(name, "hold where it rests", io);
    io.setPosition(slides ? 0.4 : Math.toRadians(45), slides ? 0.3 : 1.0, 0.6);
    asked(name, "to a point, rising", io);
    io.setPosition(slides ? 0.2 : Math.toRadians(100), slides ? -0.5 : -2.0, -0.3);
    asked(name, "to a point, falling", io);
    io.setVelocity(slides ? 0.6 : 3.0);
    asked(name, "at a speed", io);
    io.setVelocity(slides ? -0.4 : -2.0);
    asked(name, "at a speed back", io);
    // Its motor turning, 600 rpm, somewhere else: what the robot reads goes into its damping, and
    // gravity at a speed is where it is.
    rotor.setPosition((slides ? 0.7 : Math.toRadians(80)) * rotorPerUnit);
    rotor.setVelocity(600);
    io.updateInputs(inputs);
    io.setPosition(slides ? 0.5 : Math.toRadians(60), slides ? 0.2 : 0.5, 0);
    asked(name, "to a point, moving", io);
    io.setVelocity(slides ? 0.6 : 3.0);
    asked(name, "at a speed, moving", io);
  }

  /**
   * The pivot on a SPARK whose loop closes on the encoder on its data port: its setpoints in the
   * encoder's turns from the edge of its window, and the same feedforward.
   */
  private void closedOnItsEncoder() {
    Joint.Revolute pivot =
        unmotored(PIVOT)
            .motor(Motor.NEO, gearing(25), SPARK_MAX.can(61))
            .absoluteEncoder(DATA_PORT.offset(Rotations.of(0.1)).inverted().closesLoop())
            .retuned(
                control ->
                    MechanismControlBuilder.builder(control.withKs(0.2).withKg(0.5))
                        .homing(Optional.empty())
                        .build());
    JointIOSpark io = new JointIOSpark(ROBOT.with(pivot).math(), pivot, ControllerPins.NONE);
    SparkAbsoluteEncoderSim encoder = new SparkAbsoluteEncoderSim((SparkMax) io.leader());
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    // At 40°, a ninth of a turn past the pivot's zero, 0.3125 of a turn past the window's edge.
    encoder.setPosition(0.3125 + 40.0 / 360);
    io.updateInputs(inputs);
    io.updateInputs(inputs);

    io.setPosition(Math.toRadians(45), 1.0, 0.6);
    asked("pivot closed on its encoder", "to a point, rising", io);
    io.setVelocity(2.0);
    asked("pivot closed on its encoder", "at a speed", io);
  }

  /** What the SPARK was last asked, as REV's simulated SPARK holds it. */
  private void asked(String name, String step, JointIOSpark io) {
    SparkBase spark = io.leader();
    SimDeviceSim device =
        new SimDeviceSim(
            String.format(
                Locale.ROOT,
                "%s [%d,%d]",
                spark instanceof SparkFlex ? "SPARK Flex" : "SPARK MAX",
                spark.getCanPort().value,
                spark.getDeviceId()));
    rows.add(
        String.format(
            Locale.ROOT,
            "%s,%s,%.9f,%d,%.9f",
            name,
            step,
            device.getDouble("Setpoint").get(),
            device.getInt("Closed Loop Slot").get(),
            device.getDouble("Arbitrary Feedforward").get()));
  }
}
