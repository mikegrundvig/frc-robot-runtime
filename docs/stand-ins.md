# Stand-ins

Parts of this template exist only because the real 2027 version hasn't shipped yet. They are
throwaway: when the real one arrives, delete ours and use it. Add a row when you add a stand-in, and
delete the row when you replace it.

| Stand-in | Stands in for | Replace when |
|---|---|---|
| The swerve drivetrain, `runtime/src/main/java/com/michaelgrundvig/frc/runtime/subsystems/drive/`, including its CTRE hardware code (`PhoenixDrivetrain`: TalonFX drive and steering, CANcoders, a Pigeon 2, and the thread that reads them at 250 Hz) | AdvantageKit's swerve templates, which bring the same for CTRE and REV hardware. Only the drivetrain goes: where the robot is belongs to localization (`LocalizationService` in `lib/localization`: the estimator, choosing between the gyro and the backup IMU, trusting the wheels; fed and logged by `subsystems/localization`), and the simulated world to `com.michaelgrundvig.frc.runtime.sim`, and both stay. Theirs needs one addition, an adapter that reports `DrivetrainReadings` each loop (every wheel reading, both IMUs, and the velocity), and to take its pose from `Localization` rather than its own estimator. Keep what ours adds if theirs lacks it: the steer-drive coupling, the IMU's attitude and acceleration (localization checks the wheels against them for slip), and not a number for an acceleration an IMU can't read. | AdvantageKit publishes its 2027 swerve templates, planned for WPILib's 2027 beta (AdvantageKit 27.0.0-alpha-6 release notes) |
| PhotonLib, pinned to a development build (`vendordeps/photonlib.json`, dev-v2027.0.0-alpha-2-69-g71416112) that exists only in PhotonVision's snapshots repository, not in its releases. The dependency checksums (the robot template's `docs/upgrading.md`) flag it if it's replaced there | A PhotonLib release for WPILib alpha 7. Its one 2027 release (alpha-2, May 2026) targets alpha 5, and GradleRIO alpha 7 refuses it. The coprocessor must run the same PhotonVision version as the pinned library. | PhotonVision releases a build for the WPILib version in use |

## Alpha dependencies

Released, but alpha: each is tied to one WPILib alpha, and moves with it.

- **Phoenix 6 26.70.0-alpha-2** (`vendordeps/Phoenix6-26.70.0-alpha-2.json`, from WPILib's
  vendor-json-repo, `2027_alpha7`): the version CTRE pairs with WPILib alpha 7. Every CTRE device
  needs firmware 26.70 to talk to it (Phoenix Tuner X Preview flashes it), and a CANivore on
  Systemcore needs CTRE's `canivore-usb` package (WPILib's SystemcoreTesting repository,
  CTR-Phoenix). No Pro license is assumed: no FOC, and the steering reads its CANcoder as a remote
  sensor.

- **REVLib 2027.0.0-alpha-8** (`vendordeps/REVLib.json`, from REV's own `REVLib-2027.json`,
  `2027_alpha7`). Its units were measured in REV's simulation (2026-10-05); a SPARK's direction
  (counterclockwise positive, seen facing its shaft, as Phoenix's) is REV's convention and not yet
  confirmed on a robot. Neither is how a SPARK reads an encoder on its data port: its `zeroOffset`
  the raw reading where it reads zero, before inverting, and its reading from 0 to 1, as REV
  documents them; REV's simulation reports the reading a test gives it, applying neither. Nor, for
  a SPARK whose loop closes on that encoder, its soft limits on the encoder's turns, its speed in
  the encoder's RPM, or what it does when the encoder is unplugged: REV's simulation runs its loop
  on the motor's own encoder whatever it's told.

## Waiting on

Not built here at all. Each waited on a 2027 build of its library; add it when one is out.

- **A path tool's files:** PathPlannerLib (its build for WPILib alpha 7 is out, 2027.0.0-alpha-4.1, in
  WPILib's vendor-json-repo, `2027_alpha7`) and ChoreoLib (2027.0.0-alpha-3). Autos themselves are the
  template's now ([autos](autos.md)): declared in `Controls`, chosen in the pit, run fresh when
  autonomous starts, and checked before a match. What is waited on is reading a path tool's files,
  which the template doesn't do. A path read from a file could be handed to an auto as a command.

## The simulator's physics

The simulator's physics (`KinematicPhysics`, in the simulation library, `lib/simulation`) moves the
robot as its wheels roll, kinematically, and each driven joint in closed form (`JointPlant`): the
simulator tests the robot's code against simple plants, not the robot's traction or power. The
[truth suite](simulator.md#how-far-to-trust-it-the-truth-suite) says how far that falls short,
number by number. Game pieces are what the robot's sensors read of them, as a test sets them
(`world.sensors().trip(...)`, `see(...)`): no game piece, and no other robot, is a body the robot
touches. [The simulator's guide](simulator.md#how-its-built) says how it's built.

Power isn't simulated either: the simulator models neither the drive motors' supply current limit
nor Phoenix's step-down from it (`DrivetrainControl.Limits.driveSupplyLowerCurrent` and
`driveSupplyLowerTime`), and its battery holds 12 V, so the supply current a simulated module
reports is what its motor would draw with no supply limit. At the template's 30 A stator limit that
changes nothing: a drive motor draws under 30 A from the battery, below both limits. Raise the
stator limit past them, and the simulator lets a motor draw more from the battery than its motor
controller would.

What it does model: the robot's true pose apart from its sensors (`Physics`, `SensorErrors`), the
drive and steering motors held to their stator current limits, the steering's loop run as a
TalonFX's (every millisecond, on the angle its CANcoder sends every 10 ms), the robot riding over
the field's bumps (tilted and raised by the ground under its wheels), and faults made while it
runs, from the Pitwall page, by name (`com.michaelgrundvig.frc.runtime.sim.Scenarios`, `./gradlew run
-Pscenario=<name>`), or in a test (`FaultCatalog`, bound in `com.michaelgrundvig.frc.runtime.sim.Faults`, listed in
[simulator.md](simulator.md)): a slipping wheel, a push, a hit the IMU feels, a gyro unplugged,
frozen, or reset, a camera knocked off its mount, frozen, late, or with its clock off, a module's
encoders gone stale.
Wheels still never slip on their own, and no one wheel can leave the floor: `wheel-lifted` tilts
the robot by hand (which holds over any bump for the rest of the run) and scales every wheel's
grip alike, so its wheels never disagree as a lifted one's would, and the robot isn't raised. The
steering has no friction: a wheel's scrub on the carpet and its gearbox's drag need measuring on a
real module first (`CurrentLimitedMotor` says where they go). And it hears its CANcoder at the
rate the code asks for (`PhoenixDrivetrain.ENCODER_FREQUENCY_HZ`), which no robot has yet
confirmed.

The drive carries the robot's configured mass. Each drive motor turns its wheel's share of
`Body.mass` as inertia at the configured `DrivetrainGeometry.ModuleType.wheelRadius`, (m / 4) r²,
plus the wheel's own moving parts (`DrivetrainGeometry.ModuleType.driveInertia`, an estimate: about
2% more), so the robot speeds up as F = m a, and a heavier robot in `Robots.java` is a slower one
in the simulator. At the template's 30 A stator limit, it speeds up from rest at 4.35 m/s² and is
within 10% of full speed in under a second: 2.25% short of F = m a's 4.45, since the motor
controller's limit holds only at the start of each 4 ms step (a known gap). What that leaves out,
each a known gap in the truth suite, with its number:

- **Traction:** the robot follows its wheels only as fast as the tread's grip lets it change
  course (`wheelCoefficientOfFriction` times g, 11.8 m/s² at the template's 1.2): past that, the
  chassis keeps its momentum and the wheels skid, as a real robot's do when a module turns at
  speed. That's all of traction there is. Each wheel still turns as its own motor drives it,
  not as the floor drags it, so wheels that disagree with each other (each speeding up or
  slowing to its own new speed after a change of direction) stay apart longer than a real
  robot's would, and push and pull the chassis between them at up to the grip, shuddering at the
  steering's 100 Hz as the modules re-steer. And a robot whose motors push harder than the grip
  (a stator limit of 120 A, 17 m/s²) speeds up at 11.8 m/s² while its wheels spin up past it,
  all alike, where a real one's rear wheels grip under weight transfer and it speeds up at 11.3:
  a launch that slips, which only the accelerometer can see. No one wheel slips on
  its own: that's a fault to make by hand (a module Spinning).
  Its IMUs read as a Pigeon 2's do on the season-worn robot (`SensorErrors.AccelErrors`), so the
  slip check sees what a real accelerometer would: its noise, bias, range, and its filter and
  100 Hz sampling, which smooth that shudder before it's read once a loop.
- **Weight transfer:** every wheel carries a quarter of the mass however hard the robot speeds
  up, where a real one's front wheels lighten and slip first (`Robots` estimates where): the
  template's front wheels at a push of 125 N each (123 N at the tread, the rest spinning the wheel
  up), about 48 A, where the simulator slips all four at once at 209 N, about 81 A.
- **The battery:** it holds 12 V through a launch, as above.
- **Yaw inertia beyond the wheels':** the robot spins as if all its mass sat at its wheels, 11.1
  kg m² for the template's robot against the 6.7 its body is configured with
  (`Body.yawMomentOfInertia`, which the simulator doesn't read yet), so it spins up at 60% of the rate that
  body would.
- **Coupling between the modules:** each wheel moves its own share of the robot, and the robot
  follows the wheels' best fit, so a module pushing harder or weaker than the others, or pointed
  off, doesn't drag on them through the frame as on a robot: its wheel just disagrees with
  theirs. The robot still moves exactly as their best fit says, so the doubt the estimate takes
  from their disagreement through a hard change of direction (tens of centimeters, while the
  wheels speed up and slow down to their new speeds) is more than the simulated robot needs. On a
  robot, wheels set that far apart are scrubbing over the carpet. One module pointed 10° off the
  other three turns the simulated robot 4.8° a second, as their best fit. A real one turns too,
  since the three can't hold it straight (the other front wheel would need more grip across it
  than it has), at a rate statics only bounds: up to 16.4° a second at 1 m/s.
- **A wheel leaving the ground:** the robot stands on the plane that best fits the ground under
  its four wheels, so crossing a bump at 45°, where a rigid frame rests on three wheels and the
  fourth hangs, each wheel sits a quarter of the ground's twist off it instead.
- **Gravity along a slope:** nothing pulls the robot down a ramp, so braked on one with its wheels
  straight it stays put, where a real one creeps down as fast as its motors' back EMF lets it hold
  the slope (0.21 m/s on a 15° ramp).

The steering turns a fixed load, 0.004 kg m² (Phoenix Tuner X's generated simulation value, as
AdvantageKit's swerve templates use it; not measured). It assumes the module's turning parts and
its motor's rotor through the steering's gearing, the same whatever the robot weighs and however
hard the wheel is pressed to the carpet. It sets how fast a module turns at its current limit, so
the simulated steering's gains, and the rise, overshoot, and settling its experiment scores, hold
for a real module only as far as it does.

## Scenarios the simulator can't make yet

Faults a robot meets that the simulator can't make yet are listed in one place, [the simulator's
own guide](simulator.md#what-the-simulator-cant-make-fail-yet).
