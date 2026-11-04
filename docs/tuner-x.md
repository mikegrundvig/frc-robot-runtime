# Phoenix Tuner X on the robot

Tuner X reads CTRE devices' IDs, plots their signals, and runs the Pigeon 2's mount calibration.
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

Each time the robot program starts, it sends every Talon FX, CANcoder, CANrange and CANdi in the
wiring its whole configuration, built from the robot's definition. That includes each CANcoder's
offset and direction. Any of those settings changed in Tuner X lasts only until the next restart.
To change one, change the definition and redeploy.

The Pigeon 2 is the exception. The program sends it no configuration, so the mount pose Tuner X's
mount calibration saves on it stays. Do that calibration on the robot, with the robot level, and
then [check the IMU reads level](calibration.md#the-imu-reads-level).

## Encoder offsets

A CANcoder's absolute position, as Tuner X shows it, already includes the offset the robot program
wrote. So an offset isn't the reading negated, unless the offset written was zero. It's the offset
the definition has now, minus what the encoder reads at the zero:

> new offset = the definition's offset − the reading, both in rotations

1. Put the mechanism at its zero. For a swerve module, square the wheel straight ahead against a
   straightedge, its bevel gear the way the robot's other modules have it. For a joint, set the
   pose its origin gives in CAD.
2. Read the CANcoder's absolute position in rotations. Use Tuner X (the CANcoder's *Absolute
   Position*), or, for a swerve module, the robot's log: `Drive/Module<name>/SteerAngle`, in
   degrees, divided by 360.
3. Subtract it from the definition's offset (`.encoderOffset(...)`, or the encoder's
   `.offset(...)`). If the result is past half a turn either way, add or subtract 1.
4. Put the result in the definition, with the date, and redeploy. **Check:** at the zero, the
   reading is now within a few thousandths of a rotation of 0.

For example: the definition says `Rotations.of(0.10)`, and with the wheel straight Tuner X reads
0.25. The new offset is 0.10 − 0.25 = −0.15.

The same works on the bench, where the program writes its offsets too. A device Tuner X
factory-defaulted has offset 0 until the program starts, and then the reading negated is right.
