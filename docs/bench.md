# Bench: drive real CTRE hardware from a laptop

Before the robot has its Systemcore, a laptop can run the robot program and drive the robot's real
CTRE devices: Talon FXs, CANcoders and a Pigeon 2 on a CANivore, with the CANivore's USB cable
plugged into the laptop. This is CTRE's *hardware-attached simulation*. The program is the one the
robot will run, with its real configuration, current limits and gains. Only the computer differs.

This page is for the mentor running the session. Read the safety checklist before anything moves.

## What you need

- **A Windows laptop** with WPILib 2027 alpha 7 (the version the template is on) and the repository
  checked out. Windows is the computer CTRE supports for this: the CANivore talks to it over USB
  with nothing else to install. Linux needs CTRE's `canivore-usb` kernel module, which CTRE builds
  for Ubuntu and Debian only. macOS can't do it at all: Phoenix has no hardware-attached library for
  it, and the build says so.
- **The CANivore**, moved from the robot's controller to the laptop's USB port. The robot's CAN
  wiring stays as it is: only the CANivore's USB cable moves.
- **Phoenix Tuner X Preview** (the 2027 one, from the Microsoft Store), to flash firmware and
  factory-default the devices.
- **The robot on blocks**, its wheels clear of the table, with its battery and main breaker as
  usual.
- A USB gamepad, if you have one. The keyboard works too.

## Once: firmware and factory default

Every device has to run 2027 firmware, **26.70.x**, to match the Phoenix version the template uses.
In Tuner X Preview, with the CANivore on the laptop, flash each device (the CANivore first, then
every Talon FX, CANcoder and the Pigeon 2) to the newest 26.70 firmware Tuner X offers.

Then **factory-default each device** in Tuner X. A device that has been on an FRC robot is
*FRC-locked*, and CTRE requires the lock cleared before a laptop can drive it; a factory default
clears it.

- **What the factory default wipes:** the settings stored in the device (current limits, which
  way it turns, the CANcoder's magnet offset). The robot program sends every setting it needs as it
  starts, so nothing it uses is lost. It should keep each device's CAN ID and name: check them in
  Tuner X afterwards. If last season's code relied on settings stored in the devices, write them
  down first.
- **It's reversible.** The stable Tuner X can flash 2026 firmware back.
- **It's needed anyway.** The robot will need 2027 firmware for Systemcore, so this is work done
  early, not extra.

Alpha firmware changes often. When the template moves to a newer Phoenix, flash again.

## Run it

```bash
./gradlew run -PhwSim -Probot=chassis
```

`-PhwSim` is the switch; `-Probot=` names the robot (`chassis`, or your own from `Robots.all()`).
Gradle then uses CTRE's hardware libraries instead of its simulated devices, and the program runs
as it does on a robot: real devices, nothing simulated. It prints a banner first:

```text
HARDWARE-ATTACHED: real motors will move when enabled
Robot "chassis", driven from this computer through CTRE's hardware-attached simulation.
It opens these devices:
  the gyro: Pigeon 2 1 on the CANivore canivore
  front-left drive: TalonFX 23 on the CANivore canivore
  ...
```

and once the devices are opened, whether the laptop sees the CANivore:

```text
Hardware-attached: the CANivore canivore: seen, 4% of the bus in use
```

Anything the laptop can't reach isn't opened, and an alert (`Robot/outOfReach`) names it: devices
on Systemcore's own CAN port, its I/O ports and its IMU, a PDH, and REV's SPARKs, which have no
laptop path to hardware. The log goes to `logs/` in the project folder, as the simulator's does.

`-PhwSim` can't be combined with `-Preplay` or `-Pscenario`, and `-PhwSim=false` is refused: Gradle
picks the hardware libraries whenever it sees `-PhwSim` at all, so leave it out to simulate.

## Enable and drive

**The robot starts disabled, and Pitwall can't enable it.** Pitwall's page
(http://localhost:5820) shows the robot as a real one, with the ROBOT badge: its status, the Robot
and World views, live values. It has no Sim view and no Enable, as on a robot.

**Enable from WPILib's simulator GUI**, which opens with `-PhwSim` (`-Pgui=false` leaves it out, for
a Driver Station instead; below):

1. In the GUI's **Robot State** panel, click **Teleoperated** to enable. Click **Disabled** to
   disable. That is the robot's enable: the devices follow it.
2. **Joysticks:** the template's GUI settings put **Keyboard 0** on joystick 0, the driver's
   gamepad. Its keys, as WPILib's GUI sets them: **W/S** and **A/D** are the left stick (drive),
   **E/R** is axis 2, the right stick's X (turn). Check them in the GUI's *DS → Keyboard 0
   Settings*. A **USB gamepad** is better for real motors: its sticks spring back to center. Drag
   it from *System Joysticks* onto *Joystick[0]*.
3. **Don't use the GUI's Timing panel** (Pause, Step) with real motors. Pausing stops the robot's
   loop, but Phoenix keeps sending the motors the last thing they were told for as long as the robot
   is enabled.

**A Driver Station** can connect instead (`-Pgui=false`): the program listens for one as the
simulator does. It hasn't been tried with the 2027 Driver Station; the GUI is the supported way.

**Tuning is off.** On a robot, and here, tuning starts switched off: live gains go to real motors,
so switch it on deliberately, in Pitwall's Tune view, with a mentor watching. It goes back off
each time the program starts.

## Safety checklist

Before the first enable, every session:

- [ ] **On blocks.** Every wheel clear of the table and of hands, cables and tools.
- [ ] **A hand on the main breaker** (or the battery's disconnect) whenever the robot is enabled.
  Ctrl+C in the terminal also stops it: the devices disable as the program ends.
- [ ] **Current limits low.** The test chassis's are named `BENCH_LIMITS` (20 A, 1.5 m/s). Raise
  them on purpose, in code, before driving on the floor.
- [ ] **Tuning off** until you mean to tune, and then one value at a time.
- [ ] **The banner lists what you expect**: the devices and their CAN IDs, on the right CANivore.
- [ ] **Enable at a crawl first.** Push the stick a little: every wheel should point the same way
  and roll forward. One rolling backward is a drive inverted wrongly; one pointing elsewhere is
  its module's zero.
- [ ] Students stand back from the chassis while it's enabled.

## Troubleshooting

| What you see | What it means, and what to do |
|---|---|
| `the CANivore canivore: not seen` | The laptop doesn't see the CANivore. Check its USB cable and its lights, that Tuner X lists it, and that its name in Tuner X is the name in the robot's definition (`CanBus.canivore("...")`). |
| Devices print `CAN frame not received`, the CANivore is seen | Wrong CAN IDs, a device unpowered, or a CAN wiring break. Compare the banner's IDs with Tuner X's device list. |
| `Device firmware could not be retrieved` or a firmware version error | The device isn't on 26.70.x firmware. Flash it in Tuner X Preview. |
| Enabled, but nothing moves; devices are seen | The FRC lock: factory-default the devices in Tuner X. |
| `UnlicensedFeatureInUse`, or "device may not behave as expected" | A Phoenix Pro feature without a Pro license: a motor written `_FOC`, or FUSED or SYNC CANcoder feedback. Use the plain motor (`Motor.KRAKEN_X60`) and leave the feedback as it is. |
| `-PhwSim needs Windows or Linux` | macOS has no hardware-attached library. Use a Windows laptop. |
| A wheel spins the wrong way, or points the wrong way at rest | Its drive inversion or its module's zero, in the robot's definition. The test chassis keeps both in its block of values to check. |
