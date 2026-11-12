# Phoenix Tuner X on the robot

Tuner X reads CTRE devices' IDs, plots their signals, and checks a CANcoder's magnet.
[bench.md](bench.md) covers it with the CANivore plugged into a laptop. This page covers it on the
robot, with the CANivore and Systemcore's own CAN ports on Systemcore. It also covers what the robot
program does to settings made in Tuner X.

Facts here are from CTRE's [Phoenix 6 documentation](https://v6.docs.ctr-electronics.com/) and
WPILib's [CTR-Phoenix page](https://github.com/wpilibsuite/SystemcoreTesting/blob/main/CTR-Phoenix.md)
for the alpha software the template is on. None of this is confirmed on a production Systemcore yet.

## Connecting

Tuner X never talks to devices directly. It talks to Phoenix's *diagnostic server*, which runs
inside the robot program once the program opens any CTRE device. CTRE gives the server's port as
1250.

1. **Deploy the robot program first.** On a roboRIO, Tuner X can start a temporary diagnostic
   server of its own. On Systemcore it can't: WPILib lists this as a known issue, and its
   workaround is a robot program that opens a Phoenix device. The template's program is one,
   whenever the robot's wiring names a CTRE device.
2. **Put the laptop on the robot's network**, on the radio's DS port or its Wi-Fi
   ([first-power-on.md](https://github.com/mikegrundvig/frc-robot-template/blob/reference/docs/first-power-on.md)).
3. **Point Tuner X at Systemcore.** Type `10.TE.AM.2` in the address box at the top left, or pick
   *Driver Station* there while the Driver Station is running. Leave the port at 1250.
4. **Check:** Tuner X lists every device the robot's definition names, on each bus. The bus is the
   CANivore's name, or one of Systemcore's ports. Not yet confirmed: whether Tuner X sees devices
   on Systemcore's own ports this way, and whether it can flash firmware through Systemcore. Until
   it's confirmed, flash firmware with the CANivore on a laptop ([bench.md](bench.md)).

On the bench, with the program running on the laptop (`-PhwSim`), pick *localhost* instead.

## What the robot program writes over

Each time the robot program starts, it sends every Talon FX, CANcoder, CANrange and the Pigeon 2
in the wiring its whole configuration, built from the robot's definition. That includes each
CANcoder's offset and direction, and the Pigeon's mount pose. Any of those settings changed in
Tuner X lasts only until the next restart. To change one, change the definition and redeploy. A
CANdi is sent only which state each switch on it counts as tripped, and its other settings stay as
they are.

The motors' slot gains are written over the same way. To keep a steering gain found in Tuner X,
give it in the drivetrain's control in Tuner X's own unit, per turn of the module:
`.withSteerKp(Volts.per(Rotation).of(100))` and `.withSteerKd(Volts.per(RotationsPerSecond).of(0.5))`
are Tuner X's stock steering gains, and the steering motor's slot gets 100 and 0.5 back. A
steering gain always names its unit: there is no plain `withSteerKp(100)`, which read per radian
would be 2π times as hot. A drive gain is per meter a second of the robot's
speed: the drive motor's slot holds it per turn a second of the wheel, so divide what Tuner X shows
by 2π times the wheel's radius in meters, or give it as `Volts.per(MetersPerSecond)`
([values.md](values.md#drivetraincontrolswervegains)). A drive kV copied from Tuner X's swerve
generator as a plain number is per turn a second of the motor's rotor, far smaller than the robot
expects. A kV that far from what the motors and gearing would give is doubted at startup, and the
doubt names that mix-up.

A swerve module's device that doesn't confirm its configuration (off the bus, at a wrong CAN ID, or
on a CANivore plugged in after the program started) raises the module's "not configured" alert,
which names it. The drivetrain then holds still, and its moves wait, because that device runs on
its defaults: a steering motor with no CANcoder to run on, a drive that counts the wheel several
times over, no current limits. Once the device answers, the program sends it its configuration
again, and the drivetrain drives as soon as it confirms. Fix its CAN ID or wiring; no restart is
needed. A joint's device that doesn't confirm faults its joint until the program restarts.

The Pigeon's mount pose is the orientation the definition gives it (`ImuMount.orientation`), the
same one the program turns its accelerometer by. So Tuner X's mount calibration lasts only until
the next restart. To use what it measured, put its yaw, pitch and roll, in degrees, into the
orientation: `new Rotation3d(Math.toRadians(roll), Math.toRadians(pitch), Math.toRadians(yaw))`.
Unconfirmed: that Phoenix reads a mount pose as the Pigeon's own turn on the robot, as the
definition's orientation is, and not the turn that undoes it. For a Pigeon mounted level with its x
axis forward the orientation is zero and it doesn't matter. For any other, [check the IMU reads
level](calibration.md#the-imu-reads-level) and that turning the robot counterclockwise makes
`Drive/Gyro/Yaw` grow.

## Zeroing a swerve module

This is the one procedure for a module's offset. The number to read is the robot's own:
`Drive/Module<name>/SteerAngleRotations`, the module's angle in rotations, from −0.5 to 0.5. It is
the same number Tuner X shows as the CANcoder's *Absolute Position*, which already includes the
offset the robot program wrote. (`SteerAngle` beside it is a `Rotation2d`, which AdvantageScope
shows in radians or degrees: don't read it for this.)

1. **Check the magnet first.** `Drive/Module<name>/EncoderMagnet` should read `GREEN` or `ORANGE`,
   as Tuner X's *Magnet Health* does. `RED` raises the module's magnet alert: the magnet is too
   close or too far, or has slipped on its shaft, and a zero taken now won't hold. Fix that first.
2. Square the wheel straight ahead against a straightedge, its bevel gear the way the robot's other
   modules have it.
3. Read `SteerAngleRotations` with the robot disabled.
4. The new offset is the definition's offset minus the reading, both in rotations:
   the module's `CANCODER.can(CANIVORE, 12).offset(Rotations.of(...))`, or a REV module's
   `DATA_PORT.offset(Rotations.of(...))`. If the result is past half a turn either way, add or
   subtract 1.
5. Put it in the definition, with the date, and redeploy. **Check:** with the wheel straight,
   `SteerAngleRotations` is now within ±0.005 of 0. Turn the module counterclockwise by hand, seen
   from above: the reading grows. If it shrinks, the module's CANcoder needs `.clockwise()` after
   its offset, or loses it (a REV module's data port, `.inverted()`).

For example: the definition says `Rotations.of(0.10)`, and with the wheel straight the reading is
0.25. The new offset is 0.10 − 0.25 = −0.15.

## A joint's encoder offset

A CANcoder's absolute position, as Tuner X shows it, already includes the offset the robot program
wrote. So an offset isn't the reading negated, unless the offset written was zero. It's the offset
the definition has now, minus what the encoder reads at the zero:

> new offset = the definition's offset − the reading, both in rotations

1. Put the joint at its zero, the pose its origin gives in CAD.
2. Read the CANcoder's absolute position in rotations, in Tuner X (the CANcoder's *Absolute
   Position*). For a swerve module, follow [zeroing a swerve module](#zeroing-a-swerve-module)
   instead.
3. Subtract it from the definition's offset (the encoder's `.offset(...)`). If the result is past half a turn either way, add or subtract 1.
4. Put the result in the definition, with the date, and redeploy. **Check:** at the zero, the
   reading is now within a few thousandths of a rotation of 0.

For example: the definition says `Rotations.of(0.10)`, and at the zero Tuner X reads 0.25. The new
offset is 0.10 − 0.25 = −0.15.

The same works on the bench, where the program writes its offsets too. A device Tuner X
factory-defaulted has offset 0 until the program starts, and then the reading negated is right.
