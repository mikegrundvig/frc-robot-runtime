# Where every value comes from

The robot's code runs on numbers: a frame size, a current limit, how far to trust a camera at 4 m.
Some were measured, some were chosen, and some are still guesses. This register lists every value
in the robot's configuration: which kind it is, where the template's number came from, and how to
measure it or why it is what it is. The numbers themselves live in the code, never here: a robot's
in its entry in `Robots.java`, and a record's starting points in its defaults (`VisionConfig.DEFAULTS`),
where the record says what it is. A test keeps the list complete ([below](#keeping-it-complete)).

## Four kinds of value

| Kind | What it is | When it's measured | How |
|---|---|---|---|
| **calibrated** | A number that differs robot to robot: a wheel's radius, a camera's mount, a gyro's scale | On each robot, and again whenever something changes (tread wears, a camera gets bumped) | [docs/calibration.md](calibration.md), from the Pitwall page, or by hand as its row says |
| **characterized** | A number that belongs to a kind of part, or a design: how noisy a camera is at a distance, how late its results arrive, how far a gyro drifts, how a tread grips, a drivetrain design's feedforward | Once per model of part or design, on a bench, a practice floor, or one robot; robots built alike share it; again when the part, its software, or its settings change | [docs/characterization.md](characterization.md) |
| **chosen** | A design decision, not a measurement: a threshold, a margin, a tolerance | Never: it's justified, then checked against logs | Its row says why, and what in a log would make you change it ([below](#checking-a-chosen-value-against-logs)) |
| **as built** | A fact of how the robot is built: a frame size, a gear ratio, a CAN ID | Read off the CAD, a datasheet, or Phoenix Tuner X, once per robot | Its row says where |

## Where the template's numbers came from

The template ships a number for each value: its robot's, in `Robots.java`, or a record's
defaults. A row's **source** says where that number came from, never your robot's: once you measure
your own, it lives in `Robots.java` with its date (below). Each source starts with one of these
words, with any detail after it in parentheses.

| Source | Meaning |
|---|---|
| measured | Measured on a robot or a bench. |
| 2026 code | A team's 2026 competition code (the Beta robot's, itself built on AdvantageKit's template). |
| top team | A top team's published code. The row names the team. |
| datasheet | A vendor's datasheet, drawing, documentation, or a tool's default. |
| rules | The game manual (2026's, until the next season's kickoff replaces it). |
| math | Worked out from other values, or from statistics. The row says how. |
| simulation | Set until the template's simulator behaved. A real robot will differ. |
| guess | A number the template chose: an estimate or a judgment. Nobody measured it; the row's Why is its reason. |
| unknown | A number inherited from elsewhere (another team's code, a template) whose basis is lost. |
| none | No number to trace: the template's robot doesn't have one (it has no wiring), or it's a name or a seed. |

## The register describes values; `Robots.java` holds a robot's numbers

A row here describes a value: what kind it is and how to measure it. It never holds your robot's
number. Each robot's numbers live in its entry in `Robots.java`, and that's where a measurement is
recorded: the builder call that sets the value, with a comment saying when it was measured and how.
The calibration procedures print their results in exactly this form, ready to paste:

```java
.wheelRadius(Inches.of(1.974)) // measured YYYY-MM-DD
```

Do the same for what you measure by hand:

```java
.weight(Body.Weight.weighed(Pounds.of(131.2))) // measured YYYY-MM-DD: two scales, bumpers and battery on
```

A number measured for a part rather than a robot (a camera's latency, a tread's friction) goes where
every robot with that part reads it: a shared constant in `Robots.java`, built from the defaults
with only what you measured changed.

```java
private static final VisionConfig VISION =
    VisionConfigBuilder.builder(VisionConfig.DEFAULTS)
        .linearStdDevBaseline(0.012) // characterized YYYY-MM-DD: camera noise, our camera at 1280 x 800
        .distanceExponent(2.1) // characterized YYYY-MM-DD: camera noise
        .build();
```

The defaults in the code (`VisionConfig.DEFAULTS`, `EstimatorConfig.DEFAULTS`,
`LocalizationGate.SCORING`, `SensorErrors.TYPICAL`) are the template's starting points, and this
register says where each came from. When you change one in the code itself, change its row's
source too.

## Which records are listed

Every record marked `@ConfigRecord` is either configuration, with a section here, or listed under
[records that aren't configuration](#records-that-arent-configuration). A record is configuration
when its values describe the robot or how its code judges what it reads: numbers someone measured
or chose, fixed while the robot runs except as the pit tunes them. A record built from what the
robot reads or does (an IMU's readings, built every loop) isn't, and neither is one that hands a
mechanism its devices, nor a tool's options for one run (the camera study's). Where such a record
holds assumed numbers, they're listed in a table of their own.

In a configuration record, a value that is itself a configuration record (`RobotConfig.body`), or a
list or optional of them (`RobotConfig.cameras`), has a section of its own rather than a row. And
anything a robot's configuration holds, however deep (anything reached from `RobotConfig`), is
configuration: the test refuses to list it as not.

## Keeping it complete

`ValuesRegisterTest`, in the runtime's tests, reads this file and fails, naming exactly what to add or
remove, when:

- a configuration record has no section, or one of its values has no row;
- a section or a row names a record or a value the code no longer has;
- a row's kind isn't one of the four, or its source doesn't start with one of the words above, or
  the two legends above list other kinds or sources than the test accepts;
- a value a mechanism tunes live has no row under [tuning limits](#tuning-limits), or a row there
  names one it no longer tunes (the tunings come from `RobotContainer.tunings`, the one list of
  them);
- a switch the running robot publishes has no row under [switches](#switches), or a row there names
  one it doesn't (`ValuesRegisterSimTest`, which builds the robot).

So a new value starts in its record, and the test asks for its row. A section is a `###` heading
naming the record in backticks (`` ### `DrivetrainControl.Limits` ``, a nested record with its outer
one), followed by a table whose header starts with **Value** and has a **Kind** and a **Source**
column; a tuning's is headed `` ### `drivetrain` tuning ``, with a table whose header starts with
**Knob**. A table under such a heading must be one of those. Write a `|` inside a cell as `\|`. An
example table inside a code block or an HTML comment isn't read. The rest of each row is for
people.

## Checking a chosen value against logs

A chosen value makes decisions, and every decision lands in the log:

- **The cameras':** each image's outcome, with the number that decided it (`Vision/<camera>/Outcomes`:
  "too far: 6.3 m", "ambiguous single tag: 0.25 > 0.20", "turning too fast: 5.4 rad/s"), and the
  estimator's verdict on each measurement (`Vision/<camera>/Decisions`: "accepted", "gate: 20.1 >
  13.8", "too late: older than the history kept").
- **Localization's:** what the estimate rests on and how sure it is (`Localization/Mode`,
  `PositionStdDevMeters`, `HeadingStdDevDegrees`), how far the cameras disagree with it lately
  (`RecentDisagreementMeters`), the recoveries, the collisions, the tilt, the terrain, and whether
  it's good enough to score from (`ReadyToScore`, `NotReadyBecause`).
- **The drivetrain's:** each module's inputs (`Drive/Module<name>/`: currents, velocity, angle), what
  it was asked for (`Drive/SetpointsOptimized`), and the IMUs' (`Drive/Gyro/`, `Drive/BackupGyro/`).

Open a log in AdvantageScope, plot the decision beside what it judged, and ask two questions: how
often did it decide, and was it right when it did? To count, export the log (**File > Export
Data**) to a spreadsheet. A filter that throws out images the other
cameras agreed with is too tight; one that lets through images the estimator then rejects is too
loose. In the simulator, `Sim/PoseErrorMeters` says how far the estimate was from the truth, and a
scenario (`./gradlew run -Pscenario=collision`) shows what a value does when something goes wrong.

Then try another number on the same log. Change it where it's set (in `Robots.java`, or the
defaults) and replay the log through the code (`AKIT_LOG_PATH=logs/<log>.wpilog ./gradlew run
-Preplay`): the replay writes a `_sim` log of what the code decides now, from the same inputs
(the cameras' raw detections included), to compare with the original in AdvantageScope.

Change a chosen value when logs show it deciding wrong, more than once, for a reason you can name;
never from one run. Each row says what to look for.

## Where to start

The values that move the robot most, and are least known today:

1. **Camera latency and time sync** ([procedure](characterization.md#camera-latency-and-time-sync)),
   and how many cameras a coprocessor carries before they slow
   ([procedure](characterization.md#vision-coprocessor-capacity)). Every camera measurement is
   applied at its capture time. A capture time 10 ms off puts every measurement 3 cm off at 3 m/s,
   and nothing else in the code can notice.
2. **Camera noise** ([procedure](characterization.md#camera-noise)). The baseline and the distance
   exponent decide how far the estimate trusts every image, and the gate decides from them which
   images are wrong. The template's are the 2026 code's, for a different camera and coprocessor.
3. **Tread friction and the center of gravity's height**
   ([friction](characterization.md#tread-friction), [CG](calibration.md#the-robots-mass-center-of-gravity-and-inertia)).
   Together they set where the wheels slip, so the drive's current limit, and both are guesses.
   The grip is also what the drive's moves plan their acceleration on and the slip check judges
   by: left out, it's PathPlanner's 1.2, and the robot says so at startup.
4. **Gyro drift and noise** ([procedure](characterization.md#gyro-drift-and-noise)). The gyro owns
   the heading; the estimator should be exactly as sure of it as it deserves.
5. **Odometry error** ([procedure](characterization.md#odometry-error)). How fast the estimate's
   uncertainty grows between camera measurements, so how hard the cameras pull it back.

## The robot

### `RobotConfig`

`com.michaelgrundvig.frc.runtime.RobotConfig`: one robot, in `Robots`. Its values are records with sections of their own:
what the robot is (its definition, which names it, each joint's block saying its motors, wiring,
sensors and control), and how this program drives the drivetrain and judges what its sensors say.
Each joint's block says how it's driven and wired, and each sensor carries its wiring ([a sensor's
wiring](#a-sensors-wiring)): the program reads them from the definition, never written twice.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `localizer` | chosen | simulation (the estimator study; left out, the default localizer) | What works out where the robot is: left out for the default localizer, `WpilibLocalizer::new` for WPILib's pose estimator, or a maker of your own. [localizers.md](localizers.md) says what each reports and what the robot does without it. The estimator study (`./gradlew estimatorStudy`) compares them in simulation, and a real robot's logs would show the same: how far the pose is from a taped checkpoint, under each. |

### `RobotDefinition`

`com.michaelgrundvig.frc.world.robot.RobotDefinition`: what the robot is, read by localization, the
simulator, and the views (`RobotConfig.definition`). Its values are records with sections of their
own: the body, the drivetrain as built, the cameras, the IMUs, the PDH, and the mechanisms' drives
([mechanisms](#mechanisms)), and its named places, configurations and interlocks ([named
places](#named-places-configurations-and-interlocks)). It checks them against each other: the wheels
inside the frame, every camera and IMU inside the robot, every joint a drive, coupling, sensor,
camera, configuration, group, or interlock names among its joints, what's left of the body once the links
come off still a body, every interlock obeyed, and each device's feed a channel of the PDH's of its
own. The template runs the guide's robot (`ExampleRobot`), whose mechanisms' numbers illustrate
the guide: replace them with your robot's.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `name` | chosen | none | Which robot this is, as its logs and its checks' messages name it (`RobotDefinition[sim].cameras: ...`). One per robot; keep it. |
| `joints` | as built | none | The mechanisms' joints, each after the one it rides on, as `Robots.java` declares them: [`Joint.Revolute`](#jointrevolute) and [`Joint.Prismatic`](#jointprismatic). The drivetrain's modules aren't among them: they're derived from it (`DrivetrainGeometry.joints`). |
| `couplings` | as built | none | Joints that move with another, with no drive of their own: [`Coupling.Linear`](#couplinglinear) and [`Coupling.Curve`](#couplingcurve). |
| `sensors` | as built | none | What reads the joints and what they carry: [`Sensor.Encoder`](#sensorencoder), [`Sensor.Switch`](#sensorswitch) and [`Sensor.Rangefinder`](#sensorrangefinder). |
| `interlocks` | chosen | none | What one joint may do by where another is, enforced beneath every command: [`Interlock.RunsOnlyWhen`](#interlockrunsonlywhen) and [`Interlock.KeepsOut`](#interlockkeepsout). Its configurations have a section of their own ([`Configuration`](#configuration)). |

### `Body`

`com.michaelgrundvig.frc.world.robot.Body`: the frame, bumpers, and mass. Per robot, in `Robots`.
The template's robot is the largest and heaviest the 2026 rules allowed, with the best case center
of gravity. Its weight is the robot as weighed, every joint where it rests, with a section of its
own ([`Body.Weight`](#bodyweight)), and until it's weighed, the most the season allows; its inertia
is CAD's whole tensor, its yaw alone, or neither, a solid box of its mass, bumpers and height
supplying what isn't given. The definition takes its mechanisms' links from it, and what's left is the chassis.
Its center of mass is optional, since only the simulator reads it: until it's given, it's the
frame's center, a third of the robot's height up. Each default is in the robot's missing-data
report (`RobotDefinition.missing`), the center of mass's said only in the simulator.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `frameLength` | as built | rules (2026: the largest square within R104's 110 in perimeter) | From CAD; check with a tape across the outside of the frame. |
| `frameWidth` | as built | rules (2026, as `frameLength`) | As `frameLength`, side to side. |
| `height` | as built | rules (2026: R104's starting height) | From CAD: the tallest point in the starting configuration. |
| `bumperEnds` | as built | rules (2026: R403's most) | From the outside of the frame to the outside of the bumper, cover on, at the front and the back. |
| `bumperSides` | as built | rules (2026: R403's most) | As `bumperEnds`, at the left and the right. The same as at the ends, often; bumpers square around a frame that isn't are thicker on its shorter sides. |
| `bumperBottom` | as built | rules (2026: the bottom of R405's bumper zone) | From CAD, or a tape with the robot on flat carpet: the floor to the bumpers' bottom edge. AdvantageScope's schematic robot draws its bumpers from it, and the season's bumper zone is checked against it. |
| `bumperTop` | as built | rules (2026: the top of R405's bumper zone) | As `bumperBottom`, to their top edge. No higher than the robot. |
| `inertia` | calibrated | guess (typical) | How hard the robot is to turn, about its center of mass: CAD's whole tensor ([`Body.Turning.Tensor`](#bodyturningtensor)), its yaw alone ([`Body.Turning.Yaw`](#bodyturningyaw)), as PathPlanner's or Choreo's robot MOI, CAD, or [SysId's rotation test](calibration.md#the-robots-mass-center-of-gravity-and-inertia) gives it, or nothing known (`Body.Turning.BOX`). What isn't given, a solid box of the robot's mass, bumpers and height supplies, and the missing-data report says which. A given yaw must stand beside the box's pitch and roll, each of the three at most the sum of the other two, or the definition refuses it: give the whole tensor then. |
| `cg` | calibrated | guess (best case: centered and low) | Optional: nothing on the robot reads it. Until it's given, it's the frame's center, a third of the robot's height up; nothing notes it, since nothing uses it. CAD's mass-properties report, or [the two scales](calibration.md#the-robots-mass-center-of-gravity-and-inertia) for how far forward and the tilt test for its height, every joint where it rests. With the tread's friction, its height says where the wheels slip, which the drive's current limit is chosen under (`DrivetrainControl.Limits.driveStatorCurrent`). |

### `Body.Weight`

`com.michaelgrundvig.frc.world.robot.Body.Weight`: what the whole robot weighs, bumpers and battery
on: on a scale (`Body.Weight.weighed`), or until it's weighed, the most the season allows with a
battery, which its limits give (`Rebuilt.LIMITS.unweighed()`).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `mass` | calibrated | rules (2026: 135 lb with bumpers, R408, and a 14.5 lb battery, R601) | [Weigh the robot](calibration.md#the-robots-mass-center-of-gravity-and-inertia), bumpers and battery on, every joint where it rests; again after big changes. The simulator's drive carries it, a quarter on each wheel, so it sets how fast the simulated robot speeds up and how long its drive motors stay at their current limit. Its links' masses together must leave some of it for the chassis. |
| `weighed` | calibrated | rules (2026, as `mass`) | Whether `mass` is the robot on a scale. Until it is, the missing-data report says the mass is the season's. |

### `Body.Turning.Tensor`

`com.michaelgrundvig.frc.world.robot.Body.Turning.Tensor`: the robot's whole inertia tensor, from
CAD's mass-properties report, about its center of mass along the robot's axes. Its inertia has a
section of its own ([`Inertia`](#inertia)). The template's robot gives its yaw alone.

### `Body.Turning.Yaw`

`com.michaelgrundvig.frc.world.robot.Body.Turning.Yaw`: the robot's yaw inertia alone, what most
teams know: a solid box of the robot's mass, bumpers and height supplies the pitch and roll, and
no products of inertia.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `yaw` | calibrated | guess (typical) | CAD, or [SysId's rotation test](calibration.md#the-robots-mass-center-of-gravity-and-inertia); PathPlanner's and Choreo's robot MOI is often an estimate. The [drive's moves](driving.md) read it: how fast they turn the robot, and the volts each wheel is given to turn it. The simulator doesn't yet: it spins the robot as if its mass sat at its wheels, about 11 kg m² for the template's ([stand-ins](stand-ins.md)); a simulator that models the robot as one body will. |

### `Imus`

`com.michaelgrundvig.frc.world.robot.Imus`: the robot's IMUs, each where it's mounted
(`RobotDefinition.imus`): the gyro (the Pigeon 2) and, if the robot has one, the backup
(Systemcore's own, or a second Pigeon). A roboRIO robot has none: its code then reads no backup,
and with the gyro lost, the heading rests on the wheels. Its values are mounts, with a section of
their own.

### `ImuMount`

`com.michaelgrundvig.frc.world.robot.ImuMount`: where an IMU sits, how it's turned, and whether its
accelerometer has been checked to read along the robot's axes. Per IMU, in `Robots`. Localization
compares the wheels' velocity with what the IMU feels where it sits (an IMU off the center is swung
around it as the robot turns), the drivetrain's IO turns the IMU's own axes into the robot's by its
orientation, and the simulator mounts its IMUs here. A new robot starts from `Imus.UNCHECKED`: both
at the center, level, and unchecked.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `forward` | as built | none (the simulator's IMUs sit at the center) | From CAD, or a tape: how far ahead of the robot's center the IMU sits (negative behind). For Systemcore, where its board sits. |
| `left` | as built | none (as `forward`) | As `forward`, to the left (negative right). |
| `up` | as built | guess (a belly pan) | As `forward`, above the floor. Inside the robot, as a camera is. The slip check reads only the yaw rate, which swings an IMU level whatever its height, so nothing uses it yet: a robot pitching or rolling would swing a high IMU too. |
| `orientation` | as built | none (the simulator's IMUs are mounted so) | How it's turned on the robot: the rotation that takes the axes printed on it into the robot's. It turns the accelerometer's axes into the robot's, and for Systemcore mounted flat, its pitch and roll too. For Systemcore, it also says how the board is mounted, which WPILib takes its yaw by: flat with its face up or down, landscape stood on its long side, portrait on its short side (its long side taken to be its x axis: to confirm on a real one). From how it's mounted; [the accelerometer check](calibration.md#the-accelerometer-reads-along-the-robots-axes) confirms it. For the Pigeon, it must agree with the mount pose saved on it in Phoenix Tuner X, which its own yaw, pitch, and roll follow: the code doesn't write it there yet. |
| `accelerometerChecked` | calibrated | none (the simulator's IMUs read along their axes by construction) | [The accelerometer check](calibration.md#the-accelerometer-reads-along-the-robots-axes): three short drives in a log, for each IMU. Until it passes, the slip check compares the wheels' speed with that IMU's accelerometer by size, not direction, which no turn on its mount changes, once the robot has stood still long enough to learn its bias (`SlipCheck.Settings.steadySeconds`); and a jolt counts by its size alone. Off by default because a mis-mounted accelerometer trusted for direction is far worse than one compared by size: on the simulator's clean driving, one turned a quarter turn on its mount, trusted, read slip in most loops. |

### `Pdh`

`com.michaelgrundvig.frc.world.robot.Pdh`: the robot's power distribution, REV's Power
Distribution Hub (`RobotDefinition.pdh`, `.pdh(Pdh.can(1))`). Its channels, as REV labels
them and WPILib numbers them: 0 to 19 high-current, each taking a breaker up to 40 A; 20 to 22
low-current, and 23 the switchable one, each a fuse or breaker up to 15 A. Each motor controller
says which channel feeds it where it's wired ([`Pdh.Feed`](#pdhfeed)), and the definition checks
them against it. Optional: a robot that declares none checks no feed. CTRE's Power Distribution
Panel isn't supported: its channels' ratings aren't confirmed yet. `ExampleRobot.SIM`, the template's
drivetrain and cameras, has no wiring, so it declares none; the guide's robot it runs declares
`Pdh.can(1)`.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `bus` | as built | none | Which of Systemcore's CAN ports it's on (`Pdh.can(id)` is the first). Never a CANivore: WPILib reads a PDH on Systemcore's own CAN. |
| `id` | as built | none | REV's Hardware Client: its CAN ID, 1 to 63. 1 unless it's been changed, where WPILib looks for a REV PDH by default. |

### `Pdh.Feed`

Which of the PDH's channels feeds one motor controller, and through what breaker: in a joint's
block, one call where the controller is named, `TALON_FX.can(32).pdh(4, Amps.of(40))`; in the
drivetrain's wiring, each module's `driveFeed` and `steerFeed`, `.driveFeed(Pdh.feed(0,
Amps.of(40)))`. The robot refuses a channel that feeds two devices, one the PDH hasn't, a breaker
over its channel's rating, and a feed on a robot that declares no PDH. With a PDH declared, a
wired motor controller that doesn't say its feed is a doubt the robot says at startup.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `channel` | as built | none | Follow the controller's power wires to the PDH, and read the number printed beside its terminals: 0 to 23. |
| `breaker` | as built | none | The rating printed on the breaker or fuse in that channel's holder: up to 40 A on a high-current channel, 15 A on a low-current one. |

## The drivetrain

### `DrivetrainGeometry`

`com.michaelgrundvig.frc.world.robot.DrivetrainGeometry`: the swerve drivetrain as built
(`RobotDefinition.drivetrain`): its four corners, front left, front right, back left, back right,
each with a section of its own. Per robot, in `Robots`. Its block says how it's driven, its limits
and gains ([`DrivetrainControl`](#drivetraincontrol), `.control(...)`), and how it's wired
([`SwerveWiring`](#swervewiring), `.wired(...)`); neither is part of what the drivetrain is. The same module at every corner is the usual drivetrain:
`DrivetrainGeometry.uniform` places them as Phoenix Tuner X's swerve project gives their positions,
and `DrivetrainGeometry.inset` in from the frame's edges, as the template's robot does, 2.625 in
(SDS MK4i's footprint drawing).

### `DrivetrainGeometry.Corner`

One corner's module: its type, with a section of its own, and where its wheel sits.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `position` | as built | datasheet (SDS MK4i's footprint: 2.625 in in from the frame's edges) | Phoenix Tuner X's swerve generator, or CAD: where the wheel touches the floor, from the robot's center. Each in its own quarter: front left ahead and left of the center. Inside the frame. |

### `DrivetrainGeometry.ModuleType`

A type of module, the same wherever one sits: a Kraken X60 driving each wheel and another turning
each module (`driveMotor`, `steerMotor`; an `_FOC` model only with a Phoenix Pro license), each
through its gearing (`driveGearing`, `steerGearing`), with sections of their own:
[`Motor`](#motor) and [`Gearing`](#gearing). The template's robot has one type at every corner; a
drivetrain that mixes them, MK5i in front and MK5n behind, say, gives each corner its own. The
tread's grip and the two inertias are optional, each with a default until it's measured, and the
robot's missing-data report says which are defaults (`RobotDefinition.missing`). The template's
robot leaves all three out.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `wheelRadius` | calibrated | datasheet (a 4 in wheel) | [The wheel radius](calibration.md#the-wheel-radius), before each event: tread wears. The simulator's drive motors feel the robot's mass through it. |
| `wheelCoefficientOfFriction` | characterized | datasheet (PathPlanner's default, which teams already plan with; tread on carpet grips about 1.0 worn, 1.3 new) | Optional, 1.2 until it's given (`DEFAULT_WHEEL_COEFFICIENT_OF_FRICTION`). The robot reads it, so until then it says so at startup, on the Driver Station's console: the drive's moves plan their acceleration on it (`DrivetrainControl.accelerations`), and the slip check judges by it. At 1.53 or past it (the slip check's jolt over g), the robot says it doubts it, since the slip check can't then tell a slide from a collision under it: a path planner's friction is a tuning value, not the tread's. [Tread friction](characterization.md#tread-friction): a pull test, new and worn tread. The simulator's grip; with the center of gravity's height (`Body.cg`), it says where the wheels slip, which the drive's current limit is chosen under. |
| `driveInertia` | characterized | guess (a 4 in wheel, about 0.0004, and a Kraken X60's rotor through 6.75:1, about 0.0006: the rotor's inertia isn't published) | Optional, this estimate until it's given (`DEFAULT_DRIVE_INERTIA`): only the simulator reads it, and says so when it starts (`Robot/SimulatorNotes`). [Feedforward](characterization.md#feedforward): SysId on one module lifted off the floor. The drive's own moving parts (wheel, gears, and the motor's rotor through them), as inertia at the wheel, on top of its share of the robot's mass at `wheelRadius`. About 2% of what the template's robot puts on each wheel, so a rough number costs little. The simulator turns it. |
| `steerInertia` | characterized | datasheet (Phoenix Tuner X's generated simulation value, as AdvantageKit's swerve templates use it) | Optional, this value until it's given (`DEFAULT_STEER_INERTIA`): only the simulator reads it, and says so when it starts. [Feedforward](characterization.md#feedforward): SysId on one module's steering. The simulator's steering turns it, so it sets how fast the steering turns at its current limit, and how high its gains can go. What it assumes is in [stand-ins](stand-ins.md). |
| `couplingRatio` | as built | datasheet (Phoenix Tuner X's swerve generator, for the MK4i, as a team's 2024 code had it) | Tuner X's swerve generator, or the module's datasheet: drive motor turns per turn of the module, the wheel held still. Zero for a module without the coupling. Each drive motor's transmission turns the module through it as well as the wheel (`DrivetrainGeometry.joints`), and Phoenix's drivetrain takes it off the drive's reading. It's the module's gearing, so it's here, not in the wiring. |

### `Motor`

`com.michaelgrundvig.frc.world.robot.Motor`: a DC motor, as its datasheet gives it. The motors FRC
robots commonly use are named presets (`Motor.KRAKEN_X60`), with WPILib's numbers (its `DCMotor`);
any other is described from its datasheet (`Motor.fromDatasheet()`). The template's robot drives and
steers with Kraken X60s.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `name` | as built | none | What the motor is called, for messages. |
| `nominalVoltage` | as built | datasheet (WPILib's `DCMotor`, from CTRE's) | The datasheet: the voltage its numbers were measured at. |
| `stallTorque` | characterized | datasheet (WPILib's `DCMotor`, from CTRE's) | The datasheet, or a dynamometer's curve: torque with the shaft held still. A motor's own often differs from its datasheet by a few percent. |
| `stallCurrent` | characterized | datasheet (WPILib's `DCMotor`, from CTRE's) | As `stallTorque`: the current then. |
| `freeCurrent` | characterized | datasheet (WPILib's `DCMotor`, from CTRE's) | The datasheet, or the motor spinning freely on the bench: the current with nothing on its shaft. |
| `freeSpeed` | characterized | datasheet (WPILib's `DCMotor`, from CTRE's) | As `freeCurrent`: its speed then. |
| `rotorInertia` | characterized | guess (neither CTRE nor REV publishes it: each preset's is estimated from its rotor's size, as the world model audit's M21 does) | The vendor's number if it ever publishes one; otherwise SysId on the bare motor, free on the bench: its acceleration per volt. A gearbox multiplies it by its ratio squared, so it matters most behind a big reduction: at 50:1 a Kraken X60's is about 0.03 kg m² at the arm. |

### `Gearing`

`com.michaelgrundvig.frc.world.robot.Gearing`: what stands between a motor and what it turns. The
template's drivetrain has two: the drive's, whose sources are below, and the steering's.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `ratio` | as built | datasheet (SDS MK4i L2) | The module's datasheet: motor turns per wheel turn, and per turn of the module. |
| `efficiency` | characterized | math (no losses modeled: the drivetrain's numbers didn't change when its gearing became a record) | Why: the share of the motor's torque, through the ratio, that reaches the wheel. A gear stage loses a few percent, so the real one is lower; the simulator takes it off the motors' torque. Measure it with SysId on a module lifted off the floor ([feedforward](characterization.md#feedforward)), or keep 1 until a mechanism's simulation needs it. |

### `Actuator.Motors`

`com.michaelgrundvig.frc.world.robot.Actuator.Motors`: one or more identical motors working
together, only what makes the force: what stands between them and a joint is the drive's
transmission ([mechanisms](#mechanisms)). Its motor has a section of its own. The template's
drivetrain derives one for each module's drive and steering (`DrivetrainGeometry.ModuleType.driveMotors()`,
`steerMotors()`), each through its gearing, which the simulator turns its wheels with.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `count` | as built | none (one motor per wheel and per module) | How many motors drive it together: two on an arm's shaft is 2. |

### `DrivetrainControl`

`com.michaelgrundvig.frc.world.robot.DrivetrainControl`: how the drivetrain is driven, in its
block (`DrivetrainGeometry.inset(...).control(DRIVING)`), all tunable live as the [`drivetrain` tuning](#drivetrain-tuning). Its
values are records with sections of their own.

### `DrivetrainControl.Limits`

Current and speed limits, all tunable live within their [tuning limits](#tuning-limits). Check them
in a log of hard driving: each drive motor's currents (`Drive/Module<name>/DriveStatorCurrentAmps`,
`DriveSupplyCurrentAmps`), the battery's voltage, and whether the wheels slipped
(`Localization/SlipMetersPerSecond`, the IMU against the wheels, with `Localization/ResidualMeters`,
how far the wheels disagree with each other).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `driveStatorCurrent` | chosen | math (weight transfer: the front wheels slip near 47 A with the configured tread, μ 1.2, and the 0.15 m center of gravity, about 50 A at μ 1.3, and near 31 A with worn tread, μ 1.0, and a 0.3 m one) | Why: torque past the tread's grip only spins the wheels and drains the battery. Measure friction and the center of gravity first; they set it. Change if the wheels slip at full stick (`Localization/SpeedSlipMetersPerSecond` rising in a launch: the wheels speeding up faster than the accelerometer feels the robot do), or never come near it and the drivers want more push. Being at the limit isn't slipping: set below the tread's grip, the limit is what keeps the wheels from it, and the estimate doesn't doubt them for it ([`SlipCheck.Settings`](#slipchecksettings)). |
| `driveSupplyCurrent` | chosen | guess | Why: caps what each drive motor draws from the battery, so four together can't drag it into a brownout. It steps down: Phoenix lets a motor draw this for at most `driveSupplyLowerTime` (1 s) at a time, then lowers the limit to `driveSupplyLowerCurrent` (40 A) until the motor draws less than that again, so 60 A is for bursts and 40 A for anything longer (the two rows below). A motor draws no more from the battery than flows through it, so with the template's 30 A `driveStatorCurrent` neither limit is ever reached: they matter once the stator limit is raised past them. Set it from the weakest battery's [internal resistance](characterization.md#battery-internal-resistance): the voltage with four drive motors at the limit, about the rest voltage less 4 × limit × resistance, must stay above Systemcore's brownout voltage (still to measure on a real Systemcore) and the vision coprocessor's [ride-through voltage](characterization.md#coprocessor-brownout-ride-through). Change if the battery sags toward either in hard driving. |
| `driveSupplyLowerCurrent` | chosen | datasheet (Phoenix 6's default `SupplyCurrentLowerLimit`, written out; the 40 A breaker's rating) | Why: once a drive motor has been held at `driveSupplyCurrent` for `driveSupplyLowerTime`, Phoenix lowers its supply limit to this, until it draws less than this again. Phoenix does it whether the code says so or not (its defaults are 40 A after 1 s), so the template sets it where it shows. At the breaker's rating, a motor can push for as long as the driver holds the stick: an FRC 40 A breaker carries 40 A indefinitely, but the datasheets let one trip at 60 A (150%) and 80 A (200%) as early as 3.9 s and 1.5 s ([Snap Action's AT2](https://www.snapaction.net/assets/img/at2/AT2-Spec-Sheet-Revision-2023.pdf), and its MAXI-size MX5), 9 s and 5 s ([CTRE's](https://ctre.download/files/datasheet/CTRE%20Breaker%20Datasheet.pdf)), or 5 s at 80 A ([REV's](https://www.revrobotics.com/content/docs/REV-11-1860-1863-DS.pdf), which lists no 150%); a breaker already warm trips sooner. Setting it to `driveSupplyCurrent` turns the step-down off, and a long push can then trip the breaker. Check: in a log of a pushing match, a drive motor's `Drive/Module<name>/DriveSupplyCurrentAmps` flat at `driveSupplyCurrent` for a second, then flat at this, is the step-down at work; a motor that drops out for seconds (`Drive/Module<name>/DriveConnected` false, then true again) tripped its breaker anyway. Change: never above the breaker's rating; lower it to a smaller breaker's. |
| `driveSupplyLowerTime` | chosen | datasheet (Phoenix 6's default `SupplyCurrentLowerTime`, written out) | Why: longer than any sprint, and well short of a breaker's trip. In the simulator, which carries the robot's configured mass, a full-stick sprint, or one reversed at full speed, holds a drive motor at 60 A or more for at most 0.2 s, with `driveStatorCurrent` raised to 120 A (0.12 s at 80 A, and never at 60 A or the template's 30 A). It leaves out wheels slipping past their grip and the battery sagging ([stand-ins](stand-ins.md)), and either changes a real launch at a high limit. Only a long push, the wheels spinning, holds a motor there for a second. The fastest 40 A breaker trips at 60 A after 3.9 s at the earliest, and at 80 A, the top of `driveSupplyCurrent`'s tuning, after 1.5 s: raise `driveSupplyCurrent` toward 80 A, and shorten this. Check: in a log, how long `Drive/Module<name>/DriveSupplyCurrentAmps` stays at `driveSupplyCurrent` before it drops to `driveSupplyLowerCurrent`: this long. Change if logs show the step-down cutting into sprints, or a breaker tripping though the step-down came. |
| `steerStatorCurrent` | chosen | datasheet (Phoenix Tuner X's swerve generator, as the 2026 code kept it) | Why: steering needs little torque, and less current is less heat. Change if modules lag their angle setpoints under load (`Drive/Module<name>/SteerAngle` against `Drive/SetpointsOptimized`). |
| `maxLinearSpeed` | chosen | guess (headroom under the 4.73 m/s free speed) | Why: velocity control needs headroom under the free speed to correct with. Lower it if modules can't reach their setpoints at full stick (`Drive/ModuleVelocities` against `Drive/SetpointsOptimized`). |
| `maxAngularSpeed` | chosen | guess (driver-friendly) | Why: fast enough to turn around, slow enough to control. Ask the drivers. At full spin (6.3 rad/s) the cameras' images are thrown out: they're over `VisionConfig.maxYawRateRadiansPerSecond`. |

### `DrivetrainControl.Gains`

Drive feedforward and feedback, and steering feedback, all tunable live. Check them in AdvantageScope:
each module's speed against its setpoint (`Drive/ModuleVelocities` against
`Drive/SetpointsOptimized`), and its angle (`Drive/Module<name>/SteerAngle`).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `driveKs` | characterized | simulation (the simulator's wheels have no friction) | [Feedforward](characterization.md#feedforward), per drivetrain design: robots built alike share it. |
| `driveKv` | characterized | math (ideal: nominal voltage over free speed, `ModuleType.idealDriveKv()`) | [Feedforward](characterization.md#feedforward), per drivetrain design. A real drivetrain's is a little higher. |
| `driveKp` | chosen | simulation | Why: corrects what feedforward misses. Tune it live after kS and kV: raise it until speeds track their setpoints, and back off at the first buzz. |
| `steerKp` | chosen | 2026 code (kP 100 V/rot, converted) | Why: turns the module to its angle quickly. Tune it live: raise it until a new angle is reached quickly, and back off at overshoot or buzzing. The steering's loop sees a new angle only when the CANcoder sends one, every 10 ms, so it overshoots from about 40 and buzzes past about 77 (at `steerKd` 0.08, in the simulator). |
| `steerKd` | chosen | 2026 code (kD 0.5, converted) | Why: damps the steering's overshoot. Raise it if the module overshoots its angle. Too much buzzes too, since it damps on a speed up to 10 ms old: past about 0.2 at `steerKp` 50, in the simulator. |
| `skewLookahead` | chosen | guess | Why: while the robot spins, its wheels reach their angles late, so a field-relative drive aims at the heading this far ahead. Drive straight while spinning, and tune until the path stays straight. Change it if the path curves the way the robot spins. |
| `headingKp` | chosen | guess | Why: how hard a [drive move](driving.md) (`turnTo`, `goTo`, `follow`) turns the robot back onto its path's heading, and how hard `facing` turns it onto its target, in radians a second for each radian off. Its path turns at half the turning acceleration the robot's model allows, from its motors at the stator limit, its tread and its yaw inertia, and each wheel is given the volts its share of that acceleration takes, so most of a turn is the path's own. Turn a quarter turn and watch `Drive/Outcome`: raise it if turns settle slowly short of their heading, lower it if they overshoot and swing back. |
| `translationKp` | chosen | guess | Why: how hard a [drive move](driving.md) (`goTo`, `follow`) drives the robot back onto its path's position, in meters a second for each meter off, its path planned at half the acceleration the robot's model allows. Go 2 m and watch `Drive/Outcome` and how far the robot strays from the straight line: raise it if it settles slowly short, lower it if it overshoots or weaves. |

### `SwerveWiring`

`com.michaelgrundvig.frc.world.robot.SwerveWiring`: where the drivetrain's devices are on the CAN
bus, and how they read, in its block (`.wired(...)`). Phoenix Tuner X's swerve generator reports every value here; copy them
across. Its coupling ratio is the module's gearing, so it goes in the definition
(`DrivetrainGeometry.ModuleType.couplingRatio`). The template's robot runs only in simulation and has
no wiring, so nothing is set today.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `canBus` | as built | none | Tuner X: Systemcore's port, or the CANivore's name. SPARKs are on Systemcore's own port. |
| `driveController` | as built | none | What the drive motors are on: `TALON_FX` (a Kraken or Falcon), or `SPARK_MAX` or `SPARK_FLEX` (REV's MAXSwerve, say). The robot refuses one that can't drive the definition's motor. |
| `steerController` | as built | none | What the steering motors are on, as the drive's: the template's modules are TalonFXs or SPARKs for both, so far. On a SPARK, each module's absolute encoder is on its data port. |
| `gyroId` | as built | none | Tuner X: the Pigeon 2's CAN ID. |
| `gyroYawScale` | calibrated | none | [The gyro's scale](calibration.md#the-gyros-scale), once per gyro, and after replacing or remounting it. |

### `SwerveWiring.ModuleWiring`

One module's devices. Its drive's and its steering's feeds, the PDH channel each motor controller is
powered from and its breaker (`driveFeed`, `steerFeed`), are [`Pdh.Feed`](#pdhfeed)s: empty where
they aren't said.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `driveId` | as built | none | Tuner X, or REV's Hardware Client for a SPARK. |
| `steerId` | as built | none | Tuner X, or REV's Hardware Client. |
| `encoderId` | as built | none | Tuner X: the CANcoder's ID. Empty for a SPARK's steering, whose encoder is on its data port. |
| `encoderOffset` | calibrated | none | Tuner X's swerve generator, with the wheels straight ahead; again whenever a module is rebuilt. On REV's modules: with the wheels straight ahead, read each steering SPARK's absolute encoder in REV's Hardware Client, and negate it. |
| `driveInverted` | as built | none | Tuner X's swerve generator. |
| `steerInverted` | as built | none | Tuner X's swerve generator. |
| `encoderInverted` | as built | none | Tuner X's swerve generator. |

## The cameras

### `CameraConfig`

`com.michaelgrundvig.frc.world.robot.CameraConfig`: one camera on the robot. Per camera, in
`Robots`. The template's four are a 2026 competition robot's, as that team's 2026 code gives them.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `name` | chosen | none | The name PhotonVision knows the camera by. |
| `forward` | calibrated | 2026 code (a 2026 robot's cameras) | [A camera's mount](calibration.md#a-cameras-mount), starting from CAD; again after mounting, bumping, or remounting it. |
| `left` | calibrated | 2026 code (a 2026 robot's cameras) | As `forward`. |
| `up` | calibrated | 2026 code (a 2026 robot's cameras) | As `forward`. |
| `roll` | calibrated | 2026 code (a 2026 robot's cameras) | As `forward`. |
| `tiltUp` | calibrated | 2026 code (a 2026 robot's cameras) | As `forward`. |
| `yaw` | calibrated | 2026 code (a 2026 robot's cameras) | As `forward`. |
| `noiseScale` | calibrated | guess (every camera typical) | How much noisier this camera is than the typical one, whose noise [camera noise](characterization.md#camera-noise) measured: its logged tag residual (`Vision/<camera>/MeanTagResidualMeters`) over the other cameras', once every mount is calibrated. |
| `joint` | as built | none | What the camera rides on, if not the chassis: a joint, whose link it's mounted to. Its mount (`forward` to `yaw`) is then in that joint's frame, from CAD. It must sit on the link's shapes (within 10 cm), and inside the robot with the joint where it rests. Localization reads only the chassis's cameras yet. |

### `CameraConfig.MountTolerance`

How far a camera may really sit from its configured mount, as a bound along and about each axis.
Per camera, in `Robots`, as `.mountTolerance(...)`; left out, `MountTolerance.MEASURED`. The default
localizer starts the camera's turn on its mount as unsure as an error drawn evenly within it (1° is
0.58° as a standard deviation) and learns the turn from the camera's images as the robot drives;
the offset adds to `cameraShiftStdDevMeters`. Too small, and a camera off by more is trusted as if
it weren't.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `along` | calibrated | math (5 mm, the end of what a mount measured from CAD and checked lands within, as the [camera placement study's robustness](camera-placement.md#robustness) has it) | After [a camera's mount](calibration.md#a-cameras-mount), five runs from the same spots: three times the spread of where they put the lens. |
| `about` | calibrated | math (1°, as `along`) | As `along`, for the camera's angles. |

### `CameraConfig.Sensor`

The camera itself: per model of camera, at the settings it runs. In `Robots`, shared by the cameras
that use it. The simulator builds its cameras from it.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `widthPixels` | as built | guess (a typical AprilTag camera) | The resolution the pipeline runs at, in PhotonVision. |
| `heightPixels` | as built | guess (a typical AprilTag camera) | As `widthPixels`. |
| `fieldOfView` | characterized | guess (typical) | The angle the camera sees across its image, as `fieldOfViewAcross` says: PhotonVision's lens calibration, at the resolution the robot runs, reports the diagonal one; a camera's datasheet, and most teams' code, the horizontal one. The simulator's cameras and the views take the diagonal, worked out from a horizontal one through a lens with square pixels. |
| `fieldOfViewAcross` | as built | none (as `fieldOfView`, a diagonal one) | Which way `fieldOfView` is across the image: `HORIZONTAL` for a datasheet's or a Limelight's, `DIAGONAL` for PhotonVision's calibration. |
| `frameRate` | characterized | guess | [Coprocessor capacity](characterization.md#vision-coprocessor-capacity): results per second, with as many cameras as one coprocessor carries on the robot. |
| `latency` | characterized | guess | [Camera latency](characterization.md#camera-latency-and-time-sync): from the image's capture to its result reaching the robot, with as many cameras as one coprocessor carries on the robot ([coprocessor capacity](characterization.md#vision-coprocessor-capacity)). |
| `sightRange` | characterized | guess (the simulator's since the template began) | [Camera noise](characterization.md#camera-noise): the farthest a tag is found in most images. The simulator's cameras find tags no farther, and AdvantageScope's schematic robot draws each camera's field of view out to it. |

## Mechanisms

A mechanism is joints, each carrying a link, on the chassis or on another joint; drives that move
them; couplings that move one joint with another; and sensors that read them. Each refers to the
parts it names by the part itself, never by a label, so a reference can't name something that isn't
there; the definition checks only that every joint referred to is one of its own. Labels are for
logs, writers, and messages, unique within the robot. What robot code needs from a mechanism
(Phoenix's ratios, an absolute encoder's discontinuity point, the gravity offset, ideal gains to
start tuning from) is derived from it (`RobotMath`), not entered. The guide's robot the template
runs has mechanisms only to illustrate the guide, so nothing here has a source of the template's: a mechanism's numbers come from CAD and its parts'
datasheets as it joins `Robots.java`.

### `Joint.Revolute`

`com.michaelgrundvig.frc.world.robot.Joint.Revolute`: a joint that turns, in radians. Its range
and its link have sections of their own ([`Range`](#range), [`Link`](#link)), and so does how
it's driven, its block's `.motors(...)`, `.absoluteEncoder(...)`, `.limitSwitch(...)`,
`.control(...)`, and its interlocks, `.keepsOut(...)` and `.runsOnlyWhen(...)` ([`Driven.Motors`](#drivenmotors) and the rest). How it's driven isn't part of what
the joint is: two joints the same but for it are one joint, re-tuned or re-wired.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `label` | chosen | none | What logs, writers, and messages call it: `arm`, `front-left steer`. Unique among the robot's joints. |
| `parent` | as built | none | What it turns on: the chassis (`Part.CHASSIS`), or another joint, standing for that joint's link. A joint is built after its parent, so joints never loop. |
| `origin` | as built | none | From CAD: where its axis sits in its parent's frame, and how its own frame is turned there, with the joint at 0. Position 0 is wherever CAD draws it; the gravity offset Phoenix wants is derived from it (`RobotMath.gravityOffset`), not entered. |
| `axis` | as built | none | From CAD: the axis it turns about, in its own frame. A position grows turning right-handed about it, so pick its sign for the direction the motors' positive voltage turns it. Any length: only its direction counts. |
| `rest` | as built | none | Where it rests as the robot is weighed and starts a match, its stowed position, inside its range. Its block says it, `rest(...)`, beside its stops, `range(...)`; one whose block doesn't starts at its zero, which must then be inside its stops (refused otherwise, saying to give it). A practice robot's joint that rests elsewhere moves it with the same verb, `rest(...)`, keeping its stops. The body's mass properties are the robot with every joint here, and the season's starting limits are checked here. |

### `Joint.Prismatic`

`com.michaelgrundvig.frc.world.robot.Joint.Prismatic`: a joint that slides, in meters: an
elevator's stage, a telescope. Its range, its link, and how it's driven have sections of their own.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `label` | chosen | none | As a revolute joint's: `elevator`, `carriage`. |
| `parent` | as built | none | As a revolute joint's. |
| `origin` | as built | none | From CAD: where its link sits on its parent with the joint at 0, its lowest or most retracted usually. |
| `axis` | as built | none | From CAD: the direction it slides as its position grows, in its own frame. Any length. |
| `rest` | as built | none | As a revolute joint's: where it rests stowed, in meters. |

### `Driven`

`com.michaelgrundvig.frc.world.robot.Driven`: how a joint is driven, everything its block says beyond
its shape and places. Its motors, servos, absolute encoder, limit switches and control have
sections of their own; its interlocks are written as the robot's are ([`Interlock.RunsOnlyWhen`](#interlockrunsonlywhen),
[`Interlock.KeepsOut`](#interlockkeepsout)).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `interlocks` | chosen | none | What restricts the joint by where another is, how fast one runs, or what a sensor reads, in its block: `.keepsOut(...).when(...)`, `.runsOnlyWhen(...)`. One per physical clash. A joint restricted by one written after it, such as an arm riding on it, has its rule in the robot's `interlocks` instead. |

### `Driven.Motors`

`com.michaelgrundvig.frc.world.robot.Driven.Motors`: the motors turning a joint, in its block:
`.motors(Motor.KRAKEN_X60.through(Gearing.of(25)))`, four together with `.count(4)`, or each on
its controller with `.on(...)` (`TALON_FX.can(41)`, `.clockwise()` for one turning the other way). The joint's
drive is worked out from it, under the joint's label; a drive between joints, a differential's, is
the robot's own (`RobotDefinition.drives`).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `count` | as built | none | How many turn it together, the same motor each; with controllers named, one each. |
| `pitchRadius` | as built | none | A sliding joint's: where its belt, chain or rack meets its drum, sprocket or pinion, from the part's drawing. None for a joint that turns. |
| `controllers` | as built | none | Each motor's controller, the leader first: its kind, its bus and CAN ID, from Tuner X or REV's Hardware Client, and which way its motor turns for positive. None runs the joint in the simulator alone. One kind of controller, on one bus, that drives the motor. |

### `Driven.Servo`

`com.michaelgrundvig.frc.world.robot.Driven.Servo`: a servo pushing between a joint's parent and its
link, in its block (`.servo("hood left", WCP_0408, onParent, onLink)`): a hood's linear servo. Its
drive is worked out from it, under its own label. The template doesn't drive servos yet: drive
them from a subsystem of your own ([mechanisms](mechanisms.md#8-when-the-template-cant-drive-it)).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `label` | chosen | none | What logs call it: `hood left`. Unique among the robot's drives. |
| `onParent` | as built | none | From CAD: where its body is fixed, measured from the joint's axle, in the parent's frame. |
| `onLink` | as built | none | From CAD: where its rod meets the link, in the joint's frame. |

### `Driven.AbsoluteEncoder`

`com.michaelgrundvig.frc.world.robot.Driven.AbsoluteEncoder`: an absolute encoder on a joint,
turning with it, in its block: `.absoluteEncoder(AbsoluteEncoder.of())`, with its CANcoder, a
SPARK's data port or a Systemcore port (`.absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))`), or written in
full, geared, as `.absoluteEncoder(AbsoluteEncoder.of().ratio(0.536).on(CANCODER.can(34)...))`. On a
joint that slides, it turns with the drum its motors' pitch radius names, on its shaft or geared
down from it. Its sensor is worked out from it: "arm absolute encoder", which says where the joint
is from the moment the robot starts. It's read one way at most, its wiring in its own section: a CANcoder
(`MechanismWiring.Cancoder`), a SPARK's data port (`MechanismWiring.DataPortEncoder`) or a
Systemcore port (`MechanismWiring.PortEncoder`); unwired, only the simulator reads it.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `ratio` | as built | none | Its turns for each of the joint's, from the gears or belt between them: count the teeth, the joint's over the encoder's. On a joint that slides, its turns for each of its drum's, the drum, sprocket or pulley its motors' pitch radius names: 1 on the drum's shaft, 1/12 geared twelve to one down from it. Positive: which way it turns is its wiring's. One of its turns, or a potentiometer's range, must cover the joint's travel with a fortieth of it to spare at each end, or where the joint is can't be told from it alone: the definition refuses it, saying the most that fits. An elevator's drum turns several times over its travel, so an encoder on an elevator is usually geared down from the drum. |

### `Driven.LimitSwitch`

`com.michaelgrundvig.frc.world.robot.Driven.LimitSwitch`: a switch a joint presses at one end of
its travel, in its block (`.limitSwitch(LimitSwitch.lower().at(Degrees.of(0)))`, with its wiring
on a port, a CANdi, or the joint's own controller's pin in `.on(...)`). Its sensor is worked out from it, named for its joint
and its end: "pivot lower limit", what `PIVOT.switchAt(End.LOWER)` gives a condition.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `end` | as built | none | Which end of the joint's travel it marks: `LOWER` or `UPPER`. One at each end at most. |
| `position` | as built | none | Where along the joint it trips, from CAD or by moving the joint onto it and reading the joint's position. Unknown, it can stop the joint but not home it. |
| `wiring` | as built | none | Where it's connected and which raw signal means tripped ([a sensor's wiring](#a-sensors-wiring)). None runs it in the simulator alone. |

### `Range`

`com.michaelgrundvig.frc.world.robot.Range`: how far a joint moves, from its 0, unwrapped: a
turret's may span more than a turn. A joint that turns freely, a wheel or a turret on a slip ring,
has none. A revolute joint whose range spans a whole turn or more takes a computed angle as a
heading, at the turn of it inside its limits nearest where it is
([mechanisms](mechanisms.md#moves)).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `lower` | as built | none | From CAD, then by moving the mechanism to its hard stop: where it stops, in radians or meters. The soft limit inside it is the mechanism's own. |
| `upper` | as built | none | As `lower`, the other way. |
| `kind` | as built | none | `ANGLE` for a joint that turns, `LENGTH` for one that slides: set by how it's written (`Range.degrees`, `Range.meters`, or `Range.of` with angles or distances). The joint refuses a range of the other kind, as its stops or as a region an interlock keeps it out of. |

### `Link`

`com.michaelgrundvig.frc.world.robot.Link`: what a joint carries, in the joint's frame: its mass
properties ([`MassProperties`](#massproperties)) and its shapes.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `label` | chosen | none | What logs and writers call it: `arm`, `carriage`. Unique among the robot's links. |
| `shapes` | as built | none | From CAD, simplified by hand to boxes ([`Shape.Box`](#shapebox)) in the joint's frame. They're what reaches out and up, for the season's limits, and what a camera on the link sits on. None for a link nothing needs the shape of. |

### `Coupling.Linear`

`com.michaelgrundvig.frc.world.robot.Coupling.Linear`: a joint that moves in proportion to
another, with no drive of its own: a cascade elevator's upper stage. A joint follows one other at
most.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `leader` | as built | none | The joint that moves it. |
| `follower` | as built | none | The joint it moves. Where it rests must be where the coupling puts it, with the leader where it rests. |
| `ratio` | as built | none | From CAD: how far the follower moves for each of the leader's units: 2 for a cascade's carriage, which rides its stage's rigging. |
| `offset` | as built | none | Where the follower is with the leader at 0. Usually 0. |

### `Coupling.Curve`

`com.michaelgrundvig.frc.world.robot.Coupling.Curve`: a joint that moves with another along a
curve: a linkage, or a hopper's wall that folds with its floor.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `leader` | as built | none | As a linear coupling's. |
| `follower` | as built | none | As a linear coupling's. |
| `table` | as built | none | From CAD, swept through the leader's range: the leader's position against the follower's, the leader's rising, straight between points and held past the ends. More points where it bends. |

### `Drive`

`com.michaelgrundvig.frc.world.robot.Drive`: what moves a mechanism: an actuator, through a
transmission, each with sections of their own. Motors turn a shaft, so they go through gearing, a
drum, or combined terms; a linear servo pushes along a line, so it pushes between two
points.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `label` | chosen | none | What logs and messages call it: `arm`, `front-left drive`. Unique among the robot's drives. |
| `actuator` | as built | none | What makes the force: [`Actuator.Motors`](#actuatormotors) or a [servo](#actuatorservo). |
| `transmission` | as built | none | How the force reaches the joints: [`Transmission.Geared`](#transmissiongeared), [`Transmission.Drum`](#transmissiondrum), [`Transmission.Combined`](#transmissioncombined), or [`Transmission.Between`](#transmissionbetween). |

### `Actuator.Servo`

`com.michaelgrundvig.frc.world.robot.Actuator.Servo`: a servo on PWM, a linear servo between two
points or a hobby servo's shaft, with its own loop and nothing reading back where it is. Robot code
tells it a share of its travel (`Servo.command`), from the drive's coordinate at the position it
wants (`RobotMath.coordinate`): its pulse that share of the way from its shortest to its longest,
which WPILib 2027 sends as a `PWM`'s pulse in microseconds.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `from` | as built | none | The datasheet: its length eye to eye fully retracted, at its shortest pulse, in meters; or its shaft's angle there, in radians. |
| `to` | as built | none | As `from`, fully extended, at its longest pulse: closed length plus stroke. |
| `force` | as built | none | The datasheet: the most it pushes, in newtons, or turns, in newton meters. |
| `speed` | as built | none | The datasheet: how fast it moves with nothing on it, in meters or radians a second. |

### `Transmission.Geared`

`com.michaelgrundvig.frc.world.robot.Transmission.Geared`: motors turning a revolute joint
through gearing. Its joint and its gearing have sections of their own ([`Gearing`](#gearing)).
A revolute joint's `.motors(motor.through(gearing))` writes a drive with one.

### `Transmission.Drum`

`com.michaelgrundvig.frc.world.robot.Transmission.Drum`: motors driving a prismatic joint through
gearing and a drum, sprocket, or pulley: an elevator. Its joint and its gearing have sections of
their own. A prismatic joint's `.motors(motor.through(gearing).drum(pitchRadius))` writes a drive
with one.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `pitchRadius` | as built | none | The sprocket's or pulley's datasheet: half its pitch diameter, where the chain, belt, or rope runs. |

### `Transmission.Combined`

`com.michaelgrundvig.frc.world.robot.Transmission.Combined`: motors that turn more than one joint at
once, each by its own ratio ([`Transmission.Term`](#transmissionterm)): a differential wrist, or a
swerve module's drive motor, which turns its wheel and, through the coupling, its steering too.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `efficiency` | characterized | none | As [`Gearing.efficiency`](#gearing): the share of the motors' torque that reaches the joints. |

### `Transmission.Term`

`com.michaelgrundvig.frc.world.robot.Transmission.Term`: one joint a combined transmission turns, or
an encoder reads.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `joint` | as built | none | A joint the motors turn, or the encoder reads. Each joint once. |
| `ratio` | as built | none | From CAD: the motors' turns, or the encoder's, for each turn of the joint, the others held still: an encoder's is Phoenix's `SensorToMechanismRatio`, 1 on the joint's shaft, 92/18 on an 18 tooth gear against a turret's 92 tooth ring. A differential wrist's two motors: the same ratio on its pitch, opposite ones on its roll. For a prismatic joint, per turn of the drum, sprocket or pinion its `pitchRadius` names, as a drum's gearing is; for one an encoder reads straight off the slide, with none, per meter. |
| `pitchRadius` | as built | none | A prismatic joint's drum, sprocket or pinion, as [`Transmission.Drum.pitchRadius`](#transmissiondrum): half its pitch diameter, from its datasheet. Motors always slide a joint through one, so a combined transmission's prismatic term names it, and the Phoenix helper reads the joint in its turns. Empty for a revolute joint. |

### `Transmission.Between`

`com.michaelgrundvig.frc.world.robot.Transmission.Between`: a force along the line between two
points, each on the chassis or a joint's link: a linear servo, eye to eye. How hard it
turns a joint changes as the joint moves, and is worked out from where the points are.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `a` | as built | none | What one end is mounted on: the chassis, or a joint, standing for its link. |
| `onA` | as built | none | From CAD: where that end's pivot is, in that part's frame. |
| `b` | as built | none | As `a`, the other end, on a different part. Positive force pushes the ends apart. |
| `onB` | as built | none | As `onA`, the other end. |

### `Sensor.Encoder`

`com.michaelgrundvig.frc.world.robot.Sensor.Encoder`: an encoder reading a joint, or a sum of
joints on a shaft several turn, as a differential's motor's rotor reads its two sides. Its terms
are [`Transmission.Term`](#transmissionterm)s, as a combined transmission's are. A motor's own
rotor isn't written: its drive implies it (`Drive.rotor()`), never absolute.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `label` | chosen | none | What logs and messages call it: `turret-18t`. Unique among the robot's sensors. |
| `absolute` | as built | none | Whether it knows where it is at power on: a CANcoder, or a through-bore's absolute output. An absolute one reads one joint. A joint's absolute encoders must tell its whole range apart, alone or, geared nearly alike, as a pair decoded by their difference, which the definition checks (`RobotMath.absoluteSpan`); where the decoder's window sits is derived (`RobotMath.absoluteWindow`), and Phoenix's discontinuity point for one (`RobotMath.discontinuityPoint`). |
| `countsPerTurn` | as built | none | Optional, and noted nowhere when it's left out: only a pair of absolute encoders geared nearly alike, decoded by their difference, reads it, since whether their difference picks the right turn depends on it, and the definition refuses that pair without each one's. The encoder's datasheet: how many readings its turn has, 4096 for a CANcoder. None for a motor's rotor. |
### `Sensor.Switch`

`com.michaelgrundvig.frc.world.robot.Sensor.Switch`: a two-state sensor, tripped or clear: a limit
switch, a beam break, or a switch nothing the model describes closes. Tripped is actuated (pressed,
its beam blocked), whatever its wiring's polarity. What it means to the robot is a condition over it
(`tripped()`, `clear()`), never part of the switch.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `label` | chosen | none | As an encoder's: `claw beam`. |
| `part` | as built | none | What it's mounted on: the chassis, or a joint's link. A limit switch's is its joint's parent, which the joint presses it against. |
| `pose` | as built | none | From CAD, where it sits in its part's frame, x along its beam or its actuator; optional, since a team rarely knows it, and only the simulator and the 3D view use it. Unknown, it's at the part's origin, and the missing-data report says so. |
| `kind` | as built | none | What kind of switch: [`Sensor.Switch.Limit`](#sensorswitchlimit), or `Beam` (a beam break, through-beam or reflective) or `Contact` (a button, a cage sensor), which have no values. |
| `wiring` | as built | none | Where it's connected and which raw signal means tripped (`.on(CANDI.can(30).s1().normallyOpen())`): a Systemcore port, a CANdi's input, or a motor controller's limit pin ([a sensor's wiring](#a-sensors-wiring)). None runs it in the simulator alone. Not part of what the switch is: two the same but for it are one switch. |

### `Sensor.Switch.Limit`

A limit switch: one a joint's link presses, or a magnet on it passes, at one end of its travel,
written in its joint's block, `.limitSwitch(End.LOWER, Degrees.of(0), ...)`, which names it for its
joint and its end ([`Driven.LimitSwitch`](#drivenlimitswitch)).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `joint` | as built | none | The joint it reads. |
| `position` | calibrated | none | Where the joint is as it trips, inside its range: move the mechanism slowly onto it and read the joint's encoder; again after the switch is moved or bent. Optional: unknown, the switch can stop the joint but not home it, and the missing-data report says so. |
| `end` | as built | none | Which end of the joint's travel it's at: `LOWER` trips at or below its position, `UPPER` at or above. |

### `Sensor.Rangefinder`

`com.michaelgrundvig.frc.world.robot.Sensor.Rangefinder`: a distance along its axis, from its mount
on a part: a CANrange, a LaserCAN, an analog ultrasonic. Used as "is something there", it's still a
rangefinder: the threshold is a condition (`nearerThan`, `fartherThan`), at a distance short of
its `range`. One at the range or beyond is refused: the condition could never change, so either
the distance or the range is wrong.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `label` | chosen | none | As an encoder's: `hopper range`. |
| `part` | as built | none | As a switch's. |
| `pose` | as built | none | As a switch's, x along its axis. |
| `range` | as built | none | The datasheet: the farthest it reads, for a CANrange 1 m in its short-range mode and 3 m in its long-range one (CTRE's CANrange User's Guide). A condition's distance must be inside it. |
| `fieldOfView` | as built | none | The datasheet: its cone's full angle; a CANrange's is set from 6.75 to 27 degrees, across and up and down apart: give the wider. |

## Named places, configurations and interlocks

A joint names the places it goes and what it runs at ([`Position`](#position),
[`Output`](#output), its `positions` and `outputs`), and the robot names its configurations (several
joints, each at a named place) and its interlocks (what one joint may do by where another is), in its
definition. They're facts about the robot, read by everything that reads it: robot code moves to
them by name, Pitwall offers them, logs show them. What to do with them is the robot program's own
code. Each reference is the joint itself, and each place the joint's own constant
(`PIVOT.at(PIVOT_OUT)`, `PIVOT.atLeast(Degrees.of(110))`), checked as they're written. A place is a
name on its joint, and its value the robot's: a practice robot's joint may put it elsewhere
(`PIVOT.replaced(PIVOT_OUT, Degrees.of(128))`), and code naming it finds it there. The joint keeps
where it was (`formerPositions`, `formerOutputs`), so the competition robot's constant is still its
own; one of the name at a value it has never had is another joint's or another robot's, and refused. The definition checks them all when it's built:
each position inside its joint's range, each output within what the joint's motors can do (a speed
within `RobotMath.freeSpeed`, refused above it and doubted above nine tenths of it,
`RobotDefinition.findings`; a voltage within their nominal voltage),
every configuration obeying every interlock with the joints it doesn't name where they rest, the
robot obeying them as it starts, no joints holding each other there or at a configuration so that
none can move to free another, and no named place the interlocks leave unreachable; the season's extension limit is checked at each configuration (`SeasonLimits`).
The guide's robot the template runs has mechanisms only to illustrate the guide, so nothing here
has a starting value of the template's.

### `Position`

`com.michaelgrundvig.frc.world.robot.Position`: a place a joint goes, by name, in the joint's units:
`Position.degrees("out", 130)`, `Position.of("clear", Inches.of(20))`.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `name` | chosen | none | What code, logs and Pitwall call it: `out`, `clear`. Unique among its joint's positions and outputs. |
| `kind` | as built | none | `ANGLE` for a joint that turns, `LENGTH` for one that slides: set by how it's written (`Position.degrees`, or `Position.of` with an angle or a distance). The joint refuses a position of the other kind, so a slide's inches never land on a pivot as radians. |
| `value` | chosen | none | Why: where the mechanism does a job, inside the joint's range: from CAD first, then moved to where it works on the robot. Check: the joint's logged position as it does the job; change it when that's off, never to make an interlock or a check pass. |

### `Output`

`com.michaelgrundvig.frc.world.robot.Output`: what a joint is run at, by name, of one of three kinds,
as teams run their rollers: a velocity in the joint's units per second (`Output.rpm("intaking",
2000)`, the roller's turns a minute, not its motor's), a voltage (`Output.volts("pushing", 6)`), or a
duty cycle (`Output.dutyCycle("eject", -0.8)`). Every joint also has `stopped`, built in: its motors
at neutral, braking or coasting as its control says.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `name` | chosen | none | What code, logs and Pitwall call it: `intaking`, `feeding`. Unique among its joint's positions and outputs, and never `stopped`, which every joint has. |
| `kind` | chosen | none | Why: how the job runs the joint. `VELOCITY` holds a speed under load with a velocity loop, and needs its kV and kP tuned; `VOLTAGE` gives the same push whatever the battery does, and its speed falls under load; `DUTY_CYCLE` is a share of whatever the battery has, so it sags with it. |
| `value` | chosen | none | Why: how hard the job wants it; negative runs it backward. A velocity within what the joint's motors turn it at (the definition refuses one above its free speed, naming both, and doubts one above 90% of it, which a velocity loop can't hold under load); a voltage within the motors' nominal 12 V; a duty cycle from -1 to 1. Check: the joint's logged velocity, voltage and current while it runs; a velocity that never gets there is asking too much. |
| `motion` | as built | none | A velocity's only: `ANGLE` for a speed of turning, `LENGTH` for one of sliding, set by how it's written (`Output.rpm`, or `Output.of` with an angular or a linear speed). The joint refuses a speed of the other kind, so an rpm never runs an elevator in meters a second. |

### `Configuration`

`com.michaelgrundvig.frc.world.robot.Configuration`: several joints at named places, by a name of
their own, such as "stowed", "amp" or "feeding" (`RobotDefinition.configurations`). It names only
the joints it sets, each once, at one of its positions or outputs ([`Condition.At`](#conditionat));
those joints are what a command moving to it requires.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `name` | chosen | none | What code, logs and Pitwall call it: `stowed`, `amp`. Unique among the robot's configurations. |

### `Condition.At`

`com.michaelgrundvig.frc.world.robot.Condition.At`: a joint at one of its named positions or outputs,
written `ARM.at(ARM_SCORING)`: what a configuration is made of. An interlock refuses one: it holds
only with the joint exactly there, which a measured position never is, so an interlock's edge is
`atLeast`, `atMost` or `between`. Its place, a [`Position`](#position) or an [`Output`](#output), is
kept as the code names it, and the definition refuses a configuration on another joint's, saying
whose it is.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `joint` | chosen | none | The joint. |

### `Condition.AtLeast`

`com.michaelgrundvig.frc.world.robot.Condition.AtLeast`: a joint at a position or beyond it, toward
its range's upper end, written `PIVOT.atLeast(Degrees.of(110))`, or from one of its named positions
on, `PIVOT.atLeast(PIVOT_OUT)`: what an interlock's edge usually is, a clearance, the same wherever
the joint rests. A named position (its `place`, a [`Position`](#position)) is kept as the code names
it: the definition refuses an interlock on another joint's, and the robot a gate on one, saying
whose it is.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `joint` | chosen | none | The joint. |
| `position` | as built | none | Where, in the joint's units: inside its range, not its lower end, where it would always hold. For an interlock, the clearance, from CAD or measured on the robot where the parts just clear. |

### `Condition.AtMost`

`com.michaelgrundvig.frc.world.robot.Condition.AtMost`: a joint at a position or short of it, toward
its range's lower end, written `ELEVATOR.atMost(Inches.of(32))`, or up to one of its named
positions, `ELEVATOR.atMost(ELEVATOR_CLEAR)`: as `Condition.AtLeast`, the other way.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `joint` | chosen | none | The joint. |
| `position` | as built | none | As `Condition.AtLeast`'s, not its upper end. |

### `Condition.Between`

`com.michaelgrundvig.frc.world.robot.Condition.Between`: a joint between two positions, both
included, written as numbers, `PIVOT.between(Degrees.of(20), Degrees.of(60))`, or as two of its named
positions (its `fromPlace` and `toPlace`), kept as `Condition.AtLeast` keeps one.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `joint` | chosen | none | The joint. |
| `from` | as built | none | One end, in the joint's units, inside its range. |
| `to` | as built | none | The other; either way round. |

### `Condition.UpToSpeed`

`com.michaelgrundvig.frc.world.robot.Condition.UpToSpeed`: a joint running at least a speed, forward,
written `SHOOTER.upToSpeed(RPM.of(3000))`: an interlock's condition for what mustn't run until
another joint is up to speed. Only the robot judges it, from the joint's motors; a joint that isn't
answering doesn't hold it.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `joint` | chosen | none | The joint. |
| `speed` | chosen | none | Why: the slowest it may run for the restricted joint to go on, in its units a second. Check: logs of the restricted joint held (`HeldBy`) while the other ran fast enough say it's too high. |

### `Interlock.RunsOnlyWhen`

`com.michaelgrundvig.frc.world.robot.Interlock.RunsOnlyWhen`: a joint that moves only while another is
somewhere, written in its block, `.runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))`, or, for a joint
written before the one it reads, in the robot's interlocks, `Interlock.that(ELEVATOR).runsOnlyWhen(...)`: while the condition
doesn't hold, a joint held to a position stays where it is and one run at a speed stops, each going
on when it holds again.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `joint` | chosen | none | Why: the joint that breaks something run at the wrong time: a roller that would chew the bumper stowed. |
| `condition` | chosen | none | Why: where another joint must be first, by a clearance or one of its named positions, [`Condition.AtLeast`](#conditionatleast), [`Condition.AtMost`](#conditionatmost) or [`Condition.Between`](#conditionbetween); how fast it runs, [`Condition.UpToSpeed`](#conditionuptospeed); or what a sensor must read (`tripped()`, `clear()`, `nearerThan`, `fartherThan`), which holds it while the sensor is lost. Check: a log of the interlock holding the joint (its `HeldBy`) while nothing was at risk says it's too tight. |

### `Interlock.KeepsOut`

`com.michaelgrundvig.frc.world.robot.Interlock.KeepsOut`: a joint kept out of a region of its travel while
another is somewhere, written in its block, `.keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(Inches.of(32)))`, or, for a joint written before the one it reads, in the robot's interlocks, `Interlock.that(ELEVATOR).keepsOut(...).when(...)`:
its target is held at the region's edge until the condition clears. The region is a
[`Range`](#range) in the joint's units. It may be written larger than the joint's travel: the part
inside the travel is what counts. It covers some of the travel and not all of it.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `joint` | chosen | none | Why: the joint that would hit something in that region: an arm swinging through the bumper. |
| `condition` | chosen | none | Why: where the other joint is while the region is closed: an elevator too low to clear. From CAD, with a margin for the joint's stopping distance. Or what a sensor reads while it's closed (a reef nearer than a distance, a coral not in the claw); a lost sensor keeps it closed. |

## A mechanism's wiring and control

A mechanism is described once, in the definition above, and how the program drives it comes out
of it (`ControllerPlan`): its ratios, its encoder's discontinuity point, its gravity's kind and
angle, the feedforward gains it seeds, and the one feedforward law every loop driving it runs. Its
motor controllers' settings are that plan in their own terms, a TalonFX's and a CANcoder's
(`com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism`) or a SPARK's (`com.michaelgrundvig.frc.runtime.lib.rev.RevMechanism`). Its
joint's block holds the rest, how the mechanism is wired and how it's driven, and the program reads
them there. A motor controller's units for each joint, its rotations (a revolute joint's own, a
prismatic joint's drum's), are fixed in `MechanismRotations`; these records are in the joint's own
units, radians or meters. The guide's robot the template runs has mechanisms only to illustrate
the guide, so nothing here has a starting value of the template's.

### A motor's controller

Each motor's controller is named in its joint's block, the leader first, one for each motor
(`.motors(Motor.KRAKEN_X60.through(gearing).on(TALON_FX.can(41), TALON_FX.can(42).clockwise()))`),
with the PDH channel that feeds it (`.pdh(4, Amps.of(40))`); and its absolute encoder's CANcoder
([`MechanismWiring.Cancoder`](#mechanismwiringcancoder)), or that encoder on a SPARK's data port
([`MechanismWiring.DataPortEncoder`](#mechanismwiringdataportencoder)) or a Systemcore port
([`MechanismWiring.PortEncoder`](#mechanismwiringportencoder)):
`.absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))`,
`.absoluteEncoder(DATA_PORT.offset(Rotations.of(0.82)))`.

- **Its kind**: `TALON_FX` (in a Kraken or a Falcon), `SPARK_MAX` or `SPARK_FLEX` (REV's), or
  `TALON_FXS` (a Minion, or REV's motors by their connector). The definition refuses a motor its
  controller can't drive, and a joint's motors on different kinds or buses.
- **Its bus**: Tuner X, Systemcore's port or the CANivore's name (`TALON_FX.can(CANIVORE, 23)`).
- **Its ID**: Tuner X, or REV's Hardware Client for a SPARK.
- **Which way it turns** (`.clockwise()`): drive the joint forward (its positive direction in the
  definition) by hand or at a low voltage in Tuner X, and watch the motor's shaft end: clockwise or
  not. The helper sets each motor's `Inverted` from it, and each follower's alignment from whether
  it turns the leader's way.

### `MechanismWiring.DataPortEncoder`

An absolute encoder on the leading SPARK's data port, such as REV's Through Bore Encoder, named in
its joint's block by its offset (`DATA_PORT.offset(Rotations.of(0.82))`, then `.inverted()` for one
whose reading falls): read once at startup to tell the motor where the joint is, or, closing the
SPARK's loop on it (`.closesLoop()`), its loop's sensor ([absolute encoders](mechanisms.md#absolute-encoders)). The
SPARK applies its direction and offset itself: it's told to invert as `reversed` says, and a
`zeroOffset` worked out from `offset`, the raw reading at the low edge of the window one turn of the
encoder tells apart, so its reading wraps outside the joint's range.

Closed on it, the SPARK runs its loop on the encoder (its feedback sensor), in the encoder's turns:
REVLib 2027 has no conversion factors. The robot works out the rest, as for any joint on a SPARK: a
position goes to the SPARK as the encoder's turns, the kP as a share of 12 V per turn of the
encoder rather than of the rotor (so the same control's kP, on an encoder on the axle of a pivot
geared 25:1, is 25 times what it is per rotor turn), the soft limits in the encoder's turns, and
the feedforward in volts, with the joint's speed from the motor's own encoder. The encoder reads
zero at the low edge of its window, so its reading never wraps inside the joint's range, and REV's
position wrapping stays off: on, it would take the short way round, through a hard stop. Where the
joint is, logged and for every move, is the encoder's word. None of this is confirmed on a SPARK
yet: that it applies its offset and direction, and its soft limits, to the sensor its loop runs on,
as REV documents, and what it does with an encoder that's unplugged ([stand-ins](stand-ins.md)).
REV's simulation runs its loop on the motor's own encoder whatever it's told; in the template's
simulator a joint on a SPARK runs as a SPARK, its kP on the device and its feedforward the robot's,
and its loop on the encoder, which a test can slip or freeze. Its motor and joint move as one
there, so a belt's slip the encoder would correct isn't simulated.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `offset` | calibrated | none | Put the joint at its zero (the pose its origin gives in CAD), read the encoder's absolute position in REV's Hardware Client with no offset, and negate it; again whenever the encoder moves. |
| `reversed` | as built | none | Move the joint forward and watch the encoder's reading in REV's Hardware Client: true (`.inverted()` after its offset) if it falls. |
| `closedLoop` | chosen | none | Why: false, the encoder tells the motor's own encoder where the joint is at startup, and the SPARK's loop runs on that; true (`.closesLoop()`), the SPARK's loop runs on this encoder itself, at the controller's rate, so backlash or slip between the motor and the joint doesn't move where the joint goes. Its kP is then per turn of the encoder, worked out from the same control. Only a joint held at positions, without a homing recipe, and with a range closes its loop on it. Unconfirmed on a SPARK yet ([stand-ins](stand-ins.md)). Check: a log, seeded once, of `Joints/<label>`'s position (the motor's) against `Joints/<label>/AbsoluteAt` (the encoder's, as the SPARK reads it): a gap that follows the direction of travel is backlash, and one that grows or jumps is slip; either is a reason to close the loop on the encoder. |

### `MechanismWiring.PortEncoder`

An absolute encoder on one of Systemcore's ports, read by the robot, not by a motor controller: a
REV Through Bore Encoder's absolute output (`PortEncoder.dutyCycle(1)`), or an analog encoder or
potentiometer (`PortEncoder.analog(2, 10)`). The robot tells the motor where the joint is from its
first good reading, or, closing the joint's loop on it (`.closesLoop()`), runs the loop on it each
loop ([absolute encoders](mechanisms.md#absolute-encoders)).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `port` | as built | none | The port it's plugged into, 0 to 5, as the robot's wiring has it. Not a port a switch is on. |
| `signal` | as built | none | How it reports: `DUTY_CYCLE` for a through-bore's absolute output, `ANALOG` for an analog encoder or a potentiometer. |
| `turnsPerRange` | as built | none | How many of its turns its signal's full range covers: 1 for an encoder that wraps each turn, from its datasheet for a multi-turn potentiometer (10 for a ten-turn pot). |
| `offset` | calibrated | none | Put the joint at its zero (the pose its origin gives in CAD), read the encoder's share of its range with no offset (logged as `Joints/<joint>/Absolute/Share`), times `turnsPerRange`, and negate it; again whenever the encoder moves. Within its range either way. |
| `reversed` | as built | none | Move the joint forward and watch its reading: true (`.inverted()` after its offset) if it falls, as WPILib's `setInverted(true)`. |
| `closedLoop` | chosen | none | Why: false, the encoder tells the motor's own encoder where the joint is at startup, and the motor controller's loop runs on that, at 1 kHz; true (`.closesLoop()`), the robot runs the joint's loop on this encoder itself, each loop, at 50 Hz, so backlash or slip between the motor and the joint doesn't move where the joint goes. The robot's loop is coarser: tune kP again, lower if the joint buzzes. Only a joint held at positions, without a homing recipe, and with a range closes its loop on it. Check: a log of `Joints/<label>`'s position (the motor's) against `Joints/<label>/AbsoluteAt` (the encoder's): a gap that follows the direction of travel is backlash, and one that grows or jumps is slip; either is a reason to close the loop on the encoder. Closed on a duty-cycle encoder that's lost, the joint is faulted and holds on its motor; an analog one can't say it's lost. |

### `MechanismWiring.Cancoder`

The mechanism's CANcoder, named in its joint's block as a motor's controller is, with its offset:
`CANCODER.can(CANIVORE, 34).offset(Rotations.of(0.82))`, read once at startup (`ROTOR`), with
`.clockwise()`, `.remote()`, `.fused()` or `.sync()` after its offset. Its joint's TalonFX counts
the joint's turns, or a sliding joint's drum's: with the CANcoder read once at startup, its
`SensorToMechanismRatio` is the rotor's turns for each of them (5 on the guide's elevator, geared
5:1 to its drum); with the TalonFX reading it (`REMOTE`, `FUSED` or `SYNC`), it's the CANcoder's
turns for each of them (a twelfth, for the guide's CANcoder geared twelve to one down from the
drum), and its `RotorToSensorRatio` the rotor's turns for each of the CANcoder's, 60.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `bus` | as built | none | Tuner X: Systemcore's port, or the CANivore's name. Its joint's motors' bus, where their TalonFX reads it: the robot refuses another. |
| `id` | as built | none | Tuner X. |
| `offset` | calibrated | none | Put the joint at its zero (the pose its origin gives in CAD), read the CANcoder's absolute position in Tuner X with no offset, and negate it; again whenever the encoder or its magnet moves. The discontinuity point is derived, never set. |
| `turnsClockwise` | as built | none | Move the joint forward and watch the CANcoder from its LED side: clockwise (`.clockwise()` after its offset) or not, so its reading grows with the joint. |
| `feedback` | chosen | none | What the TalonFX reads its position from: `ROTOR` (the CANcoder read only by code, at boot), `REMOTE` (no license), or `FUSED` or `SYNC` (Phoenix Pro). `FUSED` keeps the rotor's rate with the CANcoder's position, and is the best where it's licensed. |

### `MechanismControl`

`com.michaelgrundvig.frc.world.robot.MechanismControl`: how one mechanism is driven, in its joint's block
(`.control(...)`). Its
gains, soft limits, profile and current limits have sections of their own. `position(kP)` and
`velocity(kP)` start one from the defaults: the feedforward seeded, no profile, the default current
limits, braking, the default dead channel; `withKp`, `withKi`, `withKd`, `withKs`, `withKv`,
`withKa`, `withKg`, `withProfile`, `withSoftLimits`, `withStatorLimit`, `withSupplyLimit`,
`withBands`, `withHoming`, `withDeadChannel`, `withEncoderGap` and `coasting` change one value each. Each takes values
with their units, converted: `withProfile(DegreesPerSecond.of(450),
DegreesPerSecondPerSecond.of(1700))`, `withSoftLimits(Range.degrees(5, 130))`,
`withBands(Degrees.of(1), Degrees.of(2))`, a roller's `withBands(Percent.of(3), Percent.of(6))`.
The gains also take plain numbers, in the joint's units, radians or meters, as SysId and Pitwall
give them; one per turn is written with its unit, `position(Volts.per(Rotation).of(188))`. A value
with its unit remembers whether it's an angle or a length, and the joint refuses one of the other
kind.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `loop` | as built | none | `POSITION` or `VELOCITY`, which its factory says: `position(kP)` for a joint held at positions (it has a range or named positions), `velocity(kP)` for one that only runs at outputs, a roller or a flywheel. Its kP is volts per its loop's error, per radian or per radian per second, so the definition refuses a control written for the other loop than its joint's. |
| `neutralMode` | chosen | none | Why: `BRAKE` holds an arm or an elevator where it stops; `COAST` lets a flywheel or a roller spin down. |
| `homing` | chosen | none | Why: how a joint read by its motor alone finds where it is once the robot is on. Empty for a joint with an absolute encoder, which the definition must have on it; any other joint held at positions names its recipe: `Homing.toHardStop(End.LOWER)` pushes it into a stop on the first enable, `Homing.toSwitch(End.LOWER)` drives it to its limit switch, and `Homing.assumeAtRest()` trusts that it was stowed by hand at power on. A roller has none. |

### `MechanismControl.Gains`

The slot's gains, in volts per the joint's units (radians or meters), or given with their units
and converted (`Volts.per(Rotation)`, as Phoenix's slot holds them). kV, kA and kG are seeded from
the definition until tuned (`RobotMath.idealGains`); a tuned one always wins, and the helper notes
a kV or kG more than twice, or less than half, the ideal, naming the two frames when it looks like a
unit slip. kG needs no mass: a joint held against gravity with none written is a doubt at startup,
and its kG is what the simulator's gravity on it is ([mechanisms](mechanisms.md#gravity)).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `kP` | chosen | none | Why: corrects what feedforward misses. Tune it live after the feedforward: raise it until the joint reaches its setpoints quickly, and back off at overshoot or buzzing. |
| `kI` | chosen | none | Why: removes an error that stays. Usually 0: a steady error is a missing kG or kS. |
| `kD` | chosen | none | Why: damps overshoot. Raise it if the joint overshoots its setpoints. |
| `kS` | characterized | none | [Feedforward](characterization.md#feedforward): SysId's kS. The definition has no friction, so there's no seed. |
| `kV` | characterized | math (seeded: the motors' nominal voltage over the joint's free speed) | [Feedforward](characterization.md#feedforward): SysId's kV, in the joint's units: SysId run on Phoenix's signals gives it per rotation, so write it with its unit, `withKv(Volts.per(RotationsPerSecond).of(0.12))`, or divide a revolute joint's by 2π. |
| `kA` | characterized | math (seeded: what the joint moves and its motors' rotors, through the gearing) | As `kV`. The seed is low when a link has no mass or inertia (`RobotDefinition.missing`). |
| `kG` | characterized | math (seeded: the links' weight at its worst, 0 without mass) | As `kV`, or by hand: the volts that just hold it still, an arm where its weight pulls hardest. Positive to hold the joint up. It needs no mass, and it's what the simulator's gravity on the joint is (`withKg(0.4)`); 0 for a joint a counterbalance holds. Phoenix's gravity type comes from the joint: `Arm_Cosine` for an arm, `Elevator_Static` for a slide. |
| `gravityAngle` | characterized | math (where the links' mass pulls hardest; the joint's zero, horizontal as WPILib takes an arm's, without mass) | An arm's only, given with its kG (`withKg(0.4, Degrees.of(-90))`): where its weight pulls hardest, the angle at which holding it takes all of kG forward. Let the arm hang still, its motors coasting, and read its angle: add 90° (hanging at -90°, it's 0°). Phoenix takes it within a quarter turn of the joint's zero. |
| `kind` | as built | none | `ANGLE` or `LENGTH`: what the gains given with their units are per (`position(Volts.per(Rotation).of(188))`, `withKv(Volts.per(InchesPerSecond).of(0.3))`). Empty when each is a plain number, in the joint's units. The joint refuses gains per the other kind, and a control one gain per an angle beside another per a length. |

### `MechanismControl.Bands`

How near a target a joint must come to be there (`enter`), and how far it may then stray and still
be (`stay`): a move arrives inside `enter`, and a joint's gates (`isAt`, `isAtGoal`) hold until it's
past `stay`. For a joint held at positions, angles or lengths
(`withBands(Degrees.of(1), Degrees.of(2))`); for a roller, shares of the speed it's asked for
(`withBands(Percent.of(3), Percent.of(6))`). Empty for the template's: 2° and 4°, 1 cm and 2 cm, or
3% and 6% of a speed.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `enter` | chosen | none | Why: as tight as the mechanism needs to be to do its job there, and no tighter than it can settle: watch `Joints/<label>/At` and the error in a log of a few moves. |
| `stay` | chosen | none | Why: wide enough that the reading's noise and a flywheel's dip as a shot leaves don't flicker the gate. At least `enter`. |
| `kind` | as built | none | `ANGLE` or `LENGTH`, set by giving the bands with their units (`withBands(Degrees.of(1), Degrees.of(2))`); empty for a roller's shares (`Percent.of(3)`). The joint refuses bands of the other kind, a roller's given as angles or lengths, and shares on a joint held at positions. |

### `MechanismControl.DeadChannel`

When one of the joint's motor controllers reads as drawing nothing on its PDH channel while the
joint works: its breaker tripped, its power unplugged, or its block naming the wrong channel
([mechanisms](mechanisms.md#the-pdh)). For longer than `time`, its leading controller draws more
than `drawing` from the battery, by its own reading, and the controller's channel reads under
`dead`: it's flagged, in `Joints/<label>/DeadChannels` and an alert, until its channel draws again.
Only a controller whose block says its channel is checked, and only while the PDH answers.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `drawing` | chosen | guess | Why: each of a joint's motors drives the same load, so while its leader draws this much, each draws about as much on its own channel. Working is what the leader draws, not what it's asked: a joint asked for volts at a steady speed with no load draws next to nothing, alive or not (the simulator, with no friction, nothing at all). Check: in a log of the joint working, `Joints/<label>/SupplyCurrentAmps` against each of its channels (`PDH/Devices/<name>`): raise it if a healthy motor's channel lags its leader's reading enough to be flagged. |
| `dead` | chosen | guess | Why: under it, a channel draws nothing: four times the PDH's resolution on a high-current channel (0.125 A, REV's specifications), and a quarter of `drawing`, so a motor drawing a little less than its leader isn't flagged. Under `drawing`. |
| `time` | chosen | guess | Why: long enough that the PDH's reading, a CAN frame behind the controller's, and a moment's spike don't flag a motor; short enough to say so during a match's first push. |

### `MechanismControl.EncoderGap`

How far a joint's absolute encoder and its motor's count may disagree, still, before one of them is
taken to have slipped: a magnet loose on its shaft, a belt or chain that skipped. Past it for a
quarter second, with a loop's travel at the joint's speed allowed on top, since the two aren't read
at the same instant, the joint is faulted, held where its motor says, until they agree again
([mechanisms](mechanisms.md#absolute-encoders)). An analog encoder can't say it's unplugged, and a
magnet turned on its shaft still reads, so this is how either shows. Checked where the joint has
both: an encoder on a Systemcore port, on a SPARK's data port, or a CANcoder a TalonFX reads once.
Empty for the template's: 5°, or 2 cm on a joint that slides, or eight of the encoder's counts at
the joint where that's more.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `margin` | chosen | guess | Why: the backlash and give between the motor and the encoder (a gearbox's lash, a chain's or a belt's stretch), which no resolution shows, and the readings' own steps: an FRC gearbox, belt or chain gives a degree or two at the joint, an elevator's rigging a centimeter, and a count of a CANcoder (4096 a turn) or a through-bore (1024) on the axle is under half a degree; geared down, an encoder's count is coarser at the joint, so eight of them is the least. A magnet that slipped, or a belt that skipped a tooth, is usually well past it. Check: in a log of the joint working hard, the gap between `Joints/<label>/AbsoluteAt` and its position; widen it, `withEncoderGap(Degrees.of(8))`, if a healthy joint's gap comes near. |
| `kind` | as built | none | `ANGLE` or `LENGTH`, as it was given with its unit (`withEncoderGap(Degrees.of(8))`, `withEncoderGap(Inches.of(1))`). The joint refuses one of the other kind. |

### `MechanismControl.Homing.ToHardStop`

A joint pushed into one of its hard stops on the first enable: found once it has moved slower than
2% of its free speed for a quarter of a second, and faulted if it hasn't by the timeout. Its
position there is the stop's, the end of its range in the definition. `toHardStop(end)` pushes at
2 V and leaves the timeout to the joint; `toHardStop(end, timeout)` gives one.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `end` | chosen | none | Why: which end's hard stop, `LOWER` or `UPPER` (as a limit switch's `End`): the stop it can reach safely from anywhere, usually the one gravity helps toward: an elevator's bottom. |
| `volts` | chosen | top team (hoods homed at 1 to 2 V, intake deploys at 2 to 4 V, elevators brought down at 1 V or less) | Why: enough to move it from anywhere, against gravity if the stop is above, and gentle on the stop. Too few and it may not move at all, which reads as finding the stop: check on the robot that it moves. An elevator brought down has its weight pushing too, so it can take less. |
| `timeout` | chosen | math (empty: twice the time the joint's motors, through its gearing, take to cross its range at `volts`, and 0.5 s, as a move's default deadline) | Why: longer than its slowest trip to the stop at `volts`, from its far end. Empty works that out from the model, which leaves friction and gravity half the push. Give one for a joint slower than that: one pushed up against more than half `volts` of its weight, or one whose gearing in the definition is a guess. Its `State` says the time while it homes, and a fault says it. |

### `MechanismControl.Homing.ToSwitch`

A joint driven toward its limit switch at one end on the first enable: home the loop the switch
trips, at the switch's position in the definition, taken in the middle of that loop's travel; and
faulted if it hasn't tripped by the timeout. One that starts on its switch backs off until it
clears first, so it's home where the switch trips. `toSwitch(end)` drives it at 2 V and leaves the
timeout to the joint; `toSwitch(end, timeout)` gives one.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `end` | chosen | none | Why: which end's limit switch: the definition's limit switch on the joint at that end, whose position must be known. |
| `volts` | chosen | top team (elevators brought down to their switch at 1 V or less) | Why: enough to move it from anywhere, and, backing off, away from the switch, against gravity if that's up. It's home within half the joint's travel in a loop at this speed: slower is closer. Check on the robot that it moves both ways. |
| `timeout` | chosen | math (empty: twice the time the joint's motors, through its gearing, take to cross its range at `volts`, a turn for one that turns freely, and 0.5 s) | Why: longer than its slowest trip to the switch at `volts`, from its far end. Empty works that out from the model, which leaves friction and gravity half the push; give one for a joint slower than that. |

### `MechanismControl.SoftLimits`

Where the motor controller stops driving the joint, inside the joint's hard stops and around its
named positions, which the definition checks: as a range, `withSoftLimits(Range.degrees(5, 130))`,
`withSoftLimits(Range.of(Inches.of(1), Inches.of(47)))`.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `lower` | chosen | none | Why: keeps the motors from driving the joint into its stop. A little inside the hard stop, by what the mechanism coasts past at speed. |
| `upper` | chosen | none | As `lower`, the other end. |
| `kind` | as built | none | `ANGLE` or `LENGTH`, set by giving the limits as a range (`Range.degrees`, `Range.of(Inches.of(1), Inches.of(47))`); empty only when built by name, in the joint's units. The joint refuses limits of the other kind. |

### `MechanismControl.Profile`

How fast a joint's moves go, with their units
(`withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700))`, or
`InchesPerSecond` for one that slides): each move is profiled on the robot (WPILib's
`TrapezoidProfile`), from where the joint is to where it's going. The definition refuses a cruise
above the joint's free speed (`RobotMath.freeSpeed`).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `cruiseVelocity` | chosen | none | Why: the fastest the profile moves the joint. Under its free speed, with room for a sagging battery: a joint that lags its profile at cruise in the logs is asked too much. |
| `acceleration` | chosen | none | Why: how hard the profile pushes. Raise it until the joint lags its profile or the current limit holds it. |
| `kind` | as built | none | `ANGLE` or `LENGTH`, set by giving the profile with its units (`DegreesPerSecond`, `InchesPerSecond`); empty only when built by name, in the joint's units. The joint refuses a profile of the other kind. |

### `MechanismControl.CurrentLimits`

Each motor's current limits, all set where they show, as the drivetrain's are
([`DrivetrainControl.Limits`](#drivetraincontrollimits)).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `stator` | chosen | guess | Why: caps the motor's torque, and its heat. Enough to move the mechanism at its fastest, and no more: raise it for a mechanism that stalls short of its target. |
| `supply` | chosen | guess | Why: caps what the motor draws from the battery, in bursts. |
| `supplyLower` | chosen | guess | Why: the supply limit after `supplyLowerTime`: no more than the motor's breaker's rating. |
| `supplyLowerTime` | chosen | guess | Why: how long the motor may draw `supply`: well short of its breaker's trip. |

## A sensor's wiring

Where each switch and rangefinder is connected, carried by the sensor: a joint's limit switch in its
block
(`.limitSwitch(LimitSwitch.lower().at(Degrees.of(0)).on(SensorWiring.port(1).normallyClosed()))`),
any other switch with `.on(...)` (`.on(CANDI.can(CANIVORE, 30).s1().normallyOpen())`), a
rangefinder with `.on(CANRANGE.can(CANIVORE, 31))`. A sensor with no wiring only runs in the
simulator. The definition refuses two sensors on one input, a controller's limit input on a joint
that isn't wired or whose controller has none, a joint's limit switch on another joint's
controller, and a CANdi's or CANrange's CAN ID another of its kind has. The template's robot has
no sensors, so nothing here has a starting value.

- **A switch's input**: a Systemcore port ([`SensorWiring.Port`](#sensorwiringport)), a CANdi's
  input ([`SensorWiring.Candi`](#sensorwiringcandi)), or a motor controller's limit input
  ([`SensorWiring.ControllerLimit`](#sensorwiringcontrollerlimit)). With six Systemcore ports, the
  CAN and SPARK inputs are most of the room.
- **A switch's polarity**: `.normallyOpen()` or `.normallyClosed()` after its input, from the
  switch's datasheet and how it's wired (for a banner sensor, light-on or dark-on): actuate it by
  hand and watch the raw input. Normally closed reads tripped with its wire cut, so its motor stops:
  the one to choose for a limit switch.
- **A rangefinder's input**: a CANrange ([`SensorWiring.CanRange`](#sensorwiringcanrange)). An
  analog rangefinder on a Systemcore port needs its volts for each meter, which the wiring doesn't
  hold yet.

### `SensorWiring.Port`

One of Systemcore's own I/O ports, set as what its sensor needs (alpha spec: six):
`SensorWiring.port(1)`.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `port` | as built | none | The port's number on the case, 0 to 5. |

### `SensorWiring.Candi`

One of a CANdi's two inputs: CTRE's CAN input board, which a TalonFX can read as its hardware limit.
`CANDI.can(CANIVORE, 30).s1()`.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `bus` | as built | none | Tuner X: Systemcore's port, or the CANivore's name. |
| `id` | as built | none | Tuner X. Both its inputs share it: one device. |
| `input` | as built | none | 1 or 2: the connector it's on, S1 or S2. |

### `SensorWiring.CanRange`

A CANrange: CTRE's CAN rangefinder. `CANRANGE.can(CANIVORE, 31)`.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `bus` | as built | none | As a CANdi's. |
| `id` | as built | none | Tuner X. |

### `SensorWiring.ControllerLimit`

A limit input on a joint's leading motor controller, which can stop its motor by itself: a SPARK's,
a TalonFXS's, or a Falcon 500's TalonFX's. A Kraken's TalonFX has none (CTRE's Talon FX docs): its
switch goes on a CANdi.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `joint` | as built | none | The joint's label: it must be wired, on a controller that has limit inputs. A joint's limit switch goes on its own joint's controller, which stops its motor on it by itself; anything else on an input is read, never stopped on. |
| `direction` | as built | none | `FORWARD` or `REVERSE`: the pin it's on, as the controller labels them. Whether a SPARK's forward pin still stops its forward with the motor inverted is unconfirmed (open question M22), so the robot doesn't yet check a limit switch's pin against its end. |

## The season

### `SeasonLimits`

`com.michaelgrundvig.frc.world.game.SeasonLimits` (in the world model): what the game manual allows a robot to be. One per season, in its
game class in the model (`Rebuilt.LIMITS`); every robot's definition is checked against it in the tests
(`SeasonLimitsTest`, by `SeasonLimits.problems`), with its joints where they rest and in each of its
named configurations. Where the robot reaches between its configurations, mid-move, isn't checked: no
guard reads the limits while the robot runs. The next season's manual replaces every number at
kickoff.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `framePerimeter` | as built | rules (2026: R104) | The game manual. |
| `startingHeight` | as built | rules (2026: R104) | The game manual: the tallest a robot may start. |
| `extension` | as built | guess (2024's limit; 2026's isn't checked yet) | The game manual: how far beyond its frame perimeter a robot may reach during a match. |
| `bumperZoneBottom` | as built | rules (2026: R405) | The game manual: the lowest the bumpers may sit. |
| `bumperZoneTop` | as built | rules (2026: R405) | The game manual: the highest they may reach. |
| `weight` | as built | rules (2026: R408, with bumpers) | The game manual: the most a robot may weigh without its battery. |
| `battery` | as built | rules (2026: R601's battery, as `Robots` weighs it) | A battery on a scale, with its leads: what the robot's weighed mass includes and the weight leaves out. |

## Localization

### `VisionConfig`

`com.michaelgrundvig.frc.localization.VisionConfig`: which camera results are thrown out, and how
far the rest are trusted. Per robot (`RobotConfig.vision`); every robot starts from
`VisionConfig.DEFAULTS`. Five are tunable live. Each image's outcome, with the number that decided
it, is in `Vision/<camera>/Outcomes`; the estimator's verdict in `Vision/<camera>/Decisions`.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `maxAmbiguity` | chosen | 2026 code (a top team used 0.19) | Why: a single tag above it fits two poses about equally well, and the wrong one is far off. Change if "ambiguous single tag" throws out results that agreed with the estimate, or single tags under it are then rejected by the gate. |
| `maxHeightErrorMeters` | chosen | 2026 code | Why: the robot is on the floor, and tags are where the field says; a solve that misses either by 20 cm is a misread. Check the "off the floor" and "height is off" outcomes against the other cameras. |
| `maxDistanceMeters` | chosen | guess | Why: noise grows with distance squared, and beyond some distance a result adds more doubt than it removes. Set it from [camera noise](characterization.md#camera-noise): where a tag's noise passes what odometry drifts in a second or two. Check the "too far" outcomes. |
| `maxYawRateRadiansPerSecond` | chosen | top team (5 rad/s over the last 0.3 s) | Why: motion blur, rolling shutter, and a capture time a little off all grow with turning. Check: images just under it should pass the gate as often as the robot's still ones. |
| `linearStdDevBaseline` | characterized | 2026 code | [Camera noise](characterization.md#camera-noise): position error for one tag at 1 m. Check: in clean driving, the gate should reject about 1 in 1000 measurements; far more and this is too small. |
| `angularStdDevBaseline` | characterized | 2026 code | [Camera noise](characterization.md#camera-noise): heading error for one tag at 1 m. |
| `distanceExponent` | characterized | 2026 code (teams used 2 to 4) | [Camera noise](characterization.md#camera-noise): how fast the error grows with distance. |
| `headingConstrainedFactor` | characterized | top team (its factor for the same idea) | [Camera noise](characterization.md#camera-noise): how much less a position solved with the gyro's heading scatters. |
| `speedStdDevFactor` | characterized | unknown (the template credits top teams, none by name) | [Camera noise, while moving](characterization.md#while-moving). |
| `yawRateStdDevFactor` | characterized | unknown (the template credits top teams, none by name) | [Camera noise, while moving](characterization.md#while-moving). |
| `maxHeadingStdDevToConstrain` | chosen | unknown (the template credits top teams, none by name) | Why: a heading-constrained solve is only as good as the heading it's given; until the estimate knows its heading this well (a robot not yet placed), full solves find it; a driver's heading reset no camera has backed is never solved with, however sure. Check how soon after placing the robot its outcomes say "heading-constrained". |
| `minTagsForHeading` | chosen | guess (the fewest a multi-tag solve has) | Why: one tag's heading is too noisy and too ambiguous to correct the gyro. Raise it if two-tag solves pull the heading around (`Localization/Pose`'s heading stepping against `Drive/Gyro/Yaw`). |
| `maxYawRateForHeading` | chosen | unknown (the template credits top teams, none by name) | Why: correct the gyro's heading only when the robot is turning slowly, where a capture time a little off barely moves the heading. Check the heading offset (`Localization/Pose`'s heading less `Drive/Gyro/Yaw`): lower it if the offset steps while the robot turns; raise it if the offset is rarely corrected in a match. |
| `maxReprojectionErrorForHeadingPixels` | chosen | unknown (the template credits top teams, none by name) | Why: only a solve that fits its tags well corrects the gyro. Check against [camera noise](characterization.md#camera-noise): a still camera's multi-tag solves should fit well under it. |
| `mountToleranceMeters` | chosen | simulation (calibrated cameras on a season-worn robot showed 0.7 to 1 cm while driving) | Why: a calibrated camera's tags land a little off from noise and the estimate's own error; past this, the camera has moved on its mount, the alert says to recalibrate it, and the excess makes it count for less ([how the robot notices](calibration.md#how-the-robot-notices-a-camera-that-moved)). Check `Vision/<camera>/MountOffsetMeters` on the real robot: freshly calibrated cameras should sit well under it. Raise it if they don't; lower it if a camera known to be knocked, and placing the robot visibly off, stays under it. |
| `latencyMarginSeconds` | chosen | guess | Why: an image arriving more than this after its camera's usual `latency`, or before it was taken, has a wrong capture time or a coprocessor falling behind, so it's left out. It can't catch a clock off by less than it: 0.1 s is 30 cm at 3 m/s. Set it a little above the spread of `Vision/<camera>/LatencySeconds` once [the capture times are checked](characterization.md#camera-latency-and-time-sync). Raise it if images in time are left out (`Vision/<camera>/LeftOutOfStep`); lower it toward that spread if the check finds capture times off by more than a few milliseconds. |

### `EstimatorConfig`

`com.michaelgrundvig.frc.localization.EstimatorConfig`: how the pose estimator weighs odometry, the
gyro, and the cameras. Per robot (`RobotConfig.estimator`); every robot starts from
`EstimatorConfig.DEFAULTS`, which the code calls starting points, not measurements.

The estimator's uncertainty should match its error. In the simulator, the
[consistency study](testing.md#is-the-estimator-honest-the-consistency-study) checks it over many
runs (`./gradlew consistencyStudy`), and one run's log compares `Localization/PositionStdDevMeters`
with `Sim/PoseErrorMeters`; on a robot, compare it with how far the cameras move the estimate when
they see tags again after a stretch without. Honest is a position NEES (the error squared, over the
covariance the estimate claimed) averaging 2 and a heading NEES averaging 1, with the error outside
the claimed covariance's 99.9% ellipse in about a thousandth of loops; the defaults read 0.99 to
2.1 and 0.37 to 1.05 in the study's calibrated runs, the collision's 1.40 among them. Well
above, the estimate is too sure of itself, and its gate rejects good measurements; well below, it's
too unsure, and the cameras pull it around. On a robot, without the truth, the same test is a
camera's correction squared over the covariance claimed for it (the estimate's and the camera's
together, the χ² gate's number): it should average 2 for a position, 3 with a heading.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `translationStdDevPerMeter` | characterized | guess | [Odometry error](characterization.md#odometry-error), per drivetrain design and tread: robots built alike share it. |
| `translationStdDevPerRadian` | characterized | guess | [Odometry error](characterization.md#odometry-error): spinning in place. |
| `residualScale` | chosen | math (the residual is already how far the wheels' disagreement may have put their travel off: one wheel of four slipping puts it off by a quarter of its slip, and the residual says that much, within a tenth; two slipping together put it off by about twice what it says, 2.2 times on a square, so a pair is doubted about half as far as it put the estimate off, a [known limit](testing.md#what-it-proves-and-what-it-doesnt)) | Why: all of what the disagreement may have put the travel off counts as suspect. Lower it if the uncertainty balloons in clean driving (`Localization/ResidualMeters` high, the cameras finding little to correct). In the simulator, whose wheels each carry their own share of the robot and can't drag each other into line, a hard change of direction sets them apart for half a second or more, and the estimate doubts them by tens of centimeters there: on a robot, wheels set apart that far are scrubbing over the carpet. |
| `tiltStartDegrees` | chosen | guess | Why: a small tilt is carpet and bumpers, not wheels off the floor. Check: `Localization/TiltDegrees` stays under it in flat driving. |
| `tiltZeroDegrees` | chosen | top team | Why: by this tilt, some wheels are off the floor. Check with [tilt on a bump](calibration.md#tilt-on-a-bump). |
| `slipCorrelationSeconds` | chosen | guess | Why: a slip (a push, a launch) keeps its direction for a while, so its errors add up rather than average out. Check: after a push or a launch, the cameras' correction should fall inside the uncertainty the estimate claimed (`PositionStdDevMeters`). Lengthen it if corrections often land outside; shorten it if they're always far inside. |
| `collisionStdDevMeters` | chosen | guess | Why: a hit moves the robot further than its wheels see. A jolt past the grip and short of a whole collision adds its share of this (`DriveConditions.collisionShare`). Check: after logged hits (`Localization/CollisionShare`), how far the cameras moved the estimate; it should cover the largest. |
| `collisionWindowSeconds` | chosen | guess | Why: one hit jolts for several loops; count it once. Check: one hit's jolts span less than it. |
| `handlingStdDevPerSqrtSecond` | chosen | guess (the idea: a top team's loose trust while disabled) | Why: a disabled robot gets carried, and its wheels can't tell, so the cameras should lead. Check: placed and disabled, the estimate settles on the cameras before the match. |
| `headingDriftDegreesPerMinute` | characterized | guess | [Gyro drift and noise](characterization.md#gyro-drift-and-noise): the gyro's worst drift, standing still. |
| `headingStdDevDegreesPerTurn` | characterized | guess (a MEMS gyro's scale is within about 1% of true before anyone calibrates it, 3.6° a turn; the simulator's season-worn one reads 0.3% high) | [The gyro's scale, repeated](characterization.md#repeatability-of-the-gyros-scale): how far its scale may still be off after calibrating it; until it's calibrated, how far an uncalibrated one may be. Where the estimator estimates the gyro's errors (`gyroErrors`), this is only how unsure it starts: the cameras find the scale within a turn or two, and a value too small costs more than one too large (in a five-seed trial at 1° a turn, a gyro reading 3% high took three turns to find, and the estimate was 1.4 cm off over the estimator study's run against 0.6 cm at 3.6°). Where it only considers them, it takes this as the same error every turn, so five turns are five times as far off: lower it once calibrated. |
| `gyroErrors` | chosen | simulation (the [estimator study](../../docs/estimator-comparison.md): with the gyro reading 3% high the estimate was 9.1 cm off considering its errors and 0.6 cm estimating them; drifting 10° a minute, 0.46° and 0.04° of heading) | Why: whether the cameras correct the gyro's scale and its steady drift as the robot drives (`ESTIMATED`), or the estimator only carries their doubt (`CONSIDERED`). Estimated, whatever corrects the heading also says how the gyro misreads its turns and how fast it drifts, and the heading then stays right through a turn and through a stretch without tags. It rests on the cameras: choose `CONSIDERED` for a robot whose cameras you would not trust with the gyro. `Localization` logs neither value yet; `PoseEstimator.gyroScalePercent()` and `gyroDriftDegreesPerMinute()` have them. |
| `gyroScaleWanderPercentPerMinute` | chosen | guess (a MEMS gyro's scale moves a few hundredths of a percent for each degree Celsius, and a robot's electronics warm about a degree a minute in a match) | Why: an estimate of the scale that nothing may move grows too sure to follow the gyro as it warms. Used only where `gyroErrors` is `ESTIMATED`. Raise it if the estimated scale lags a gyro that is known to change; it is too small to see in a match otherwise. |
| `gyroDriftStdDevDegreesPerMinute` | characterized | guess (what a gyro's calibration at rest may leave, or a robot bumped while it calibrated: a few degrees a minute) | [Gyro drift and noise](characterization.md#gyro-drift-and-noise): the gyro's steady drift standing still, over several power-ups; their spread. It is how unsure the estimator starts of a drift that is the same every second, where it estimates the gyro's errors; `headingDriftDegreesPerMinute` is the drift that does not repeat. Too small costs more than too large: in a five-seed trial at 1° a minute, a gyro drifting 10° a minute was 0.21° off in heading over the estimator study's run, against 0.06° at 5°. A robot standing still reads the drift off its gyro directly, each second of standing, so the value matters only until it has stood a few seconds or its cameras have seen it turn. A stretch of standing whose drift is past three times this is taken for a robot being carried, and teaches nothing. |
| `gyroDriftWanderDegreesPerMinute` | chosen | guess (a compensated gyro's drift changes by about a tenth of a degree a minute for each minute it warms) | Why: as `gyroScaleWanderPercentPerMinute`, for the steady drift. |
| `backupHeadingStdDevDegreesPerTurn` | characterized | guess (an uncalibrated IMU: the simulator's season-worn backup reads 0.5% high, 1.8° a turn) | [The backup IMU's scale](characterization.md#repeatability-of-the-gyros-scale), from the gyro-scale procedure's spins. The procedure calibrates the gyro alone: lowering `headingStdDevDegreesPerTurn` after it leaves this as it is. |
| `headingNoiseDegreesPerTurn` | characterized | guess (small beside the scale's error; the simulator's IMUs have none, so the study can't check it) | [The gyro's scale, repeated](characterization.md#repeatability-of-the-gyros-scale): the spread of each turn's error about the five turns' straight line, once the scale is taken out. |
| `wheelHeadingStdDevDegreesPerTurn` | characterized | guess | [Odometry error](characterization.md#odometry-error): heading from the wheels against the gyro's, spinning at driving speed. Used only while the gyro is lost. |
| `cameraShiftStdDevMeters` | characterized | simulation (a **stop-gap**, 4 mm. The mount's own shift is the camera's `mountTolerance` now, 2.9 mm as a standard deviation by default, and the two add in squares to the 5 mm this was before; this covers a real error the estimator doesn't model. A single tag's solve from noisy corners comes out short, the same in every image: PhotonVision's own `solvePNP_SQUARE`, on the template's sensor with the simulator's corner noise, puts a tag seen head-on 4.5 mm short at 2 m, 14.6 at 3.6 m and 40 at 6 m; 30° off its face, 0.2, 0.8 and 2.9 mm; from 0.5 m above its axis, 0.2, 4.0 and 30.8 mm. It grows with the corner noise and about with the distance squared, likely a single tag's near face-on ambiguity, and with noiseless corners it's gone. It's one-sided, it hangs on range and angle, and every camera seeing a tag face-on shares it, so a constant for each camera models it poorly. At 3 mm, the mount's alone, the consistency study's turning run sat at exactly its 2.92% allowance outside the ellipse and the 2° camera rose to 3.80% (2.83% before tags were considered on their own); at 5 mm, 1.83% and 2.87%. Part of what the extra 2 mm absorbs is seed 2's tag 31, drawn 3.4 times `tagPlacementStdDevMeters` off: [testing](testing.md#what-the-simulated-cameras-repeat)) | How far a camera's positions sit off the same way image after image, beyond its mount, its turn and its tags' placement: on surveyed spots of a practice field whose tags are surveyed too (WPIcal), each camera's average position over 10 s against the spot, facing two ways (the shift turns with the robot); their root mean square, after taking out what the camera's turn explains (its tags' distance times the yaw it's off). Measure the short solve on its own (a tag head-on at 2, 4 and 6 m) before trusting a constant for it. |
| `tagPlacementStdDevMeters` | characterized | guess (a field's tags within about half an inch of the layout; the consistency study draws its tags from this, so it can't say what it should be) | How far each tag sits from the field's layout, along the field and across it: a survey of the field (WPIcal) against the layout. The estimator keeps it apart for each tag, the same for every camera that sees it, so two cameras seeing the same tags can't average it down: standing still with every camera seeing the same two tags, the consistency study read a NEES of 3.42 while each camera's shift held it, 2.17 since. At an event, a field built off its layout raises it; a field surveyed, with its survey as the layout, lowers it. |
| `cameraWatchMeasurements` | chosen | math (a camera's own images scatter by a centimeter or so; an average of 25 is good to about 2 mm, which is what a calibrated mount is allowed, and at 25 images a second that is one second) | Why: how many of a camera's latest accepted measurements its mount is watched by. A camera whose measurements keep landing to one side of the others', by more than three times what the estimator still allows its mount (how sure it is of the camera's turn, as learned, and `cameraShiftStdDevMeters`), has that much taken off them and counts for less, within a second or two of being knocked, and is trusted again a few seconds after it stops. The doubt is added to each of its measurements as noise, after the gate has tested the measurement without it. Fewer, and it is quicker and noisier. `PoseEstimator.cameraDoubtMeters` says how far each is doubted. The slower [mount check](calibration.md#a-cameras-mount) still raises the alert to recalibrate. |
| `cameraClockStdDevSeconds` | characterized | guess (a few milliseconds: a working time sync, and where in its exposure an image counts as taken) | [Camera latency and time sync](characterization.md#camera-latency-and-time-sync): how far a camera's capture times are off, over several checks; their spread. The estimator's watch on its cameras finds how late each one's images are stamped, from how its measurements miss the others' as the robot moves and turns, and takes off what a lateness past three times this puts them off by. It finds a camera late against the other cameras only: all of them late alike looks like nothing. `PoseEstimator.cameraLateSeconds` says what it found. |
| `cameraShiftStdDevDegrees` | characterized | simulation (the consistency study, tags off by `tagPlacementStdDevMeters`: two tags a third of a meter apart, each 1 cm off, turn a multi-tag heading by about 2°; at 1°, standing after the gyro-scale procedure the heading grew sure of a wrong value, a heading NEES of 2.05 and 4.9% of loops outside the ellipse, against 0.95 and 0.1% at 2°) | How far a camera's multi-tag headings sit off the same way while it sees the same tags, beyond its turn: on surveyed spots of a practice field, facing known headings, each camera's average multi-tag heading over 10 s against the truth; their root mean square, less the camera's turn. A field surveyed with WPIcal, or tags that move together on one panel, need less. |
| `headingCorrection` | chosen | simulation (the [estimator study](../../docs/estimator-comparison.md) and the [consistency study](testing.md#three-ways-to-correct-the-heading)) | Why: what may correct the heading beside the gyro: measurements with a heading of their own only; those, with a camera's mount doubt added to what it repeats; or every fix, a full update, which is the default. Every fix is what lets the cameras find the gyro's scale and drift (`gyroErrors`): two cameras whose tags lie in different directions disagree by the heading's error times the distance between them, which shows a tenth of a degree where a multi-tag heading shows a degree or two. It leans on the cameras' capture times while the robot turns: check them to within about 2 ms ([camera latency and time sync](characterization.md#camera-latency-and-time-sync)). With cameras off their mounts by twice what the configuration allows, the heading grows surer than it is (past its 95% bound in a quarter of the estimator study's loops, against a twentieth with headings only); the estimator's watch on its cameras (`cameraWatchMeasurements`) takes a camera knocked further than that out of it. The [consistency study](testing.md#three-ways-to-correct-the-heading) compares the three (`--heading=`). |
| `cameraShiftCorrelationMeters` | chosen | guess | Why: about a meter on, a camera sees other tags, or the same ones from another angle, and what its multi-tag headings share (`cameraShiftStdDevDegrees`) is mostly new. Its shift on its mount and each tag's placement don't change, and aren't let go. Check in the consistency study: lengthen it if the driving runs' heading NEES sits above its band, shorten it if well below. |
| `cameraShiftCorrelationDegrees` | chosen | guess | Why: as `cameraShiftCorrelationMeters`, for turning: an eighth of a turn brings other tags into view. |
| `historySeconds` | chosen | guess (a top team kept 1 s) | Why: a camera measurement must arrive within it to be applied; several times any camera's [latency](characterization.md#camera-latency-and-time-sync). Check for "too late" in `Decisions`. |
| `futureToleranceSeconds` | chosen | guess | Why: clocks differ a little, but a measurement stamped this far past the newest odometry has a wrong capture time. Check for "from the future" in `Decisions` ([time sync](characterization.md#camera-latency-and-time-sync)). |
| `visionTimeoutSeconds` | chosen | guess | Why: after a second without an accepted measurement, the estimate is dead reckoning, and says so. Check: `Localization/Mode` reads DEAD_RECKONING only where the cameras are really blind. |
| `disagreementWindowSeconds` | chosen | guess | Why: how far back "lately" reaches for `RecentDisagreementMeters`, which the scoring gate reads. Lengthen it if `NotReadyBecause` flickers on one stray measurement; shorten it if the robot stays not ready long after the cameras agree again. |
| `gateChiSquaredPosition` | chosen | math (chi-squared, 2 degrees of freedom: 99.9% of good measurements pass) | Why: rejects measurements its noise model says are almost certainly wrong. A good noise model rejects about 1 in 1000 good measurements; far more "gate:" rejections in clean driving mean the uncertainties are too small, not that the gate is too tight. |
| `gateChiSquaredPose` | chosen | math (3 degrees of freedom, 99.9%) | As `gateChiSquaredPosition`, for a measurement with heading. |
| `recoveryMeasurements` | chosen | guess | Why: three rejected measurements that agree with each other outvote odometry; with fewer, one bad camera could. Check: `Localization/Recoveries` counts only real knocks (a hit, a push, a bad placement). |
| `recoveryWindowSeconds` | chosen | guess | Why: those measurements must be recent, so they describe the same moment. As `recoveryMeasurements`. |
| `recoveryAgreementMeters` | chosen | guess | Why: measurements this close agree, given camera noise at range. Keep it above the noise at the distances the cameras see tags. |
| `recoveryStdDevMeters` | chosen | guess | Why: after a jump, the estimate is that unsure, so the next measurements settle it. Change if the measurements right after a recovery are rejected (too small), or the estimate wanders for seconds after one (too large). |
| `recoveryAgreementDegrees` | chosen | guess | Why: rejected measurements whose headings agree within this, and not with the gyro, mean the heading is wrong (a bad reset, a robot placed backwards); a camera's heading within this of a driver's heading reset backs it, and until one does, positions solved with that heading can't outvote the estimate's. Check: heading recoveries happen only after a real wrong heading; lower it if one follows a single camera's noise. |
| `recoveryHeadingStdDevDegrees` | chosen | guess | Why: as `recoveryStdDevMeters`, for heading. Change as `recoveryStdDevMeters`. |
| `lostAfterRejections` | chosen | guess | Why: two rejections in a row with none accepted, and the estimate says it's lost. Check: LOST in `Localization/Mode` away from real knocks means it's too low, or the noise model too sure. |
| `lostPositionStdDevMeters` | chosen | guess | Why: after a heading recovery, the position was worked out with the wrong heading, so it may be far off. Change if, after a heading recovery, the cameras find the position further off than this. |

### `LocalizationGate`

`com.michaelgrundvig.frc.localization.LocalizationGate`: whether an estimate is good enough to score
from; the robot uses `LocalizationGate.SCORING`. Check `Localization/ReadyToScore` and
`NotReadyBecause` in a match log: ready whenever the robot is lined up with tags in view, and never
while the estimate is off.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `maxVisionAgeSeconds` | chosen | top team | Why: odometry drifts after the cameras last checked it. Change if `NotReadyBecause` blames an old measurement while the robot is lined up with tags in view (the cameras are slower than it), or shots taken just under it miss. |
| `maxDisagreementMeters` | chosen | top team | Why: cameras that just disagreed by more haven't settled the estimate. Lower it toward the task's tolerance once [camera noise](characterization.md#camera-noise) says the cameras do better. |
| `maxPositionStdDevMeters` | chosen | guess | Why: the scoring task's tolerance. Set it from the game: how far off a shot or a placement may land. |
| `maxTiltDegrees` | chosen | guess (the idea: top teams wouldn't shoot tilted) | Why: a tilted robot aims off, and its cameras solve worse. Change if shots taken just under it miss, or the robot reads over it on level floor. |

### `SlipCheck.Settings`

`com.michaelgrundvig.frc.localization.SlipCheck.Settings`: how closely the wheels must agree with
the IMU before the estimate doubts them for sliding over the floor together, which their
disagreement with each other can't show; the robot uses `Settings.DEFAULTS`. How fast they may be
sliding counts as suspect travel (`DriveConditions.slipMetersPerSecond`). The drive motors' current
limit isn't asked: at the template's 30 A the drive pushes the robot at 4.4 m/s², well under the
tread's 11.8 m/s² (`driveStatorCurrent`), and in the simulator's estimator golden the drivetrain
read as at its grip's edge (a drive motor at its limit, or a wheel short of its speed) in 322 of 500
loops with the wheels gripping. Doubting a quarter of the travel there made the estimate claim up to
65 cm through the hard driving (31 cm on average over those loops), and let a tag misread by 1.5 m
through its gate. Check in a log of hard driving: `Localization/TurnApartDegrees` and
`SlideMetersPerSecond` stay under their tolerances through clean launches, turns, and stops, and
`TurnSlipMetersPerSecond` and `SpeedSlipMetersPerSecond` rise only when a wheel spins, the wheels
spin in a launch, or the robot is pushed, with the cameras then finding the estimate off by about as
much as it doubted.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `windowSeconds` | chosen | guess | Why: how far back the turn's comparison looks: long enough for the gyro's noise to average out, short enough that a slide shows while it lasts. The turn worn tread puts between the wheels and the gyro grows with it: change `turnToleranceDegrees` with it. |
| `turnToleranceDegrees` | chosen | simulation (the simulator's season-worn robot, its tread worn 1 to 2% unevenly, turns its wheels' reckoning up to 0.44° from the gyro's in a quarter second, straight ahead at full speed; one wheel spinning 8 cm/s faster than the floor shows past it) | Why: above what worn tread and the gyro's noise put between them. Raise it if `TurnApartDegrees` passes it in clean straight driving; lower it once the wheel radii are calibrated. |
| `turnScaleTolerance` | chosen | guess (as `ImuCrossCheck.Settings`: worn tread reckons a few percent more turn than the robot makes, a gyro's scale a few tenths of a percent less) | Why: on top, this share of the turn. Raise it if fast spins in clean driving raise `TurnSlipMetersPerSecond`. |
| `speedBiasMetersPerSecSq` | chosen | guess (a Pigeon 2's bias, guessed at under 0.1 m/s² with a mount half a degree off level, and the simulator's at 0.094) | Why: what the accelerometer didn't feel is let go toward none at this rate and no faster, so a bias up to it never builds up, while a slide that holds at a steady speed stays seen until it stops and the accelerometer feels it stop. What it let go comes back the other way when the slide stops: a push held for two seconds leaves the tolerance's worth, 0.3 m/s. A wheel reading fast (worn tread) only builds up while the robot speeds up, and comes back as it slows, so it needs no room here. Raise it if `SlideMetersPerSecond` creeps up in clean driving; lower it once the accelerometer's bias is measured. |
| `speedToleranceMetersPerSecond` | chosen | simulation (on the simulator's season-worn robot, its accelerometer a Pigeon 2's with its noise, bias, sampling and filter, clean driving puts the wheels and the accelerometer at most 0.21 m/s apart: launches, stops, spinning a turn a second while driving) | Why: above how far the two drift apart without a slide: noise, and the lag of the part's filter as the robot's acceleration changes. Past it, all of the difference counts as the slide's speed, not what's past it: taking the tolerance off every slide, the doubt trailed a 1 m/s push by 0.3 m/s from its start, and the scoring gate let through 8 loops with the robot more than 15 cm off (3 counting all of it). It was tuned against the simulator's accelerometer, whose 16 ms filter is a guess: CTRE publishes no noise, offset, or filter figures for the Pigeon 2's accelerometer. Measure it on the robot: [the accelerometer check](calibration.md#the-accelerometer-reads-along-the-robots-axes)'s measured pass, clean launches, stops, and spins, should keep `SlideMetersPerSecond` under 0.2 m/s; raise it if they don't. |
| `stillMetersPerSecond` | chosen | guess (well above the wheels' and the gyro's noise standing still, well below any driving) | Why: the robot stands still when its wheels roll slower than this, and turn it slower than this at their reach, and the IMU turns it slower too. Standing still, a reading the accelerometer holds steady (`steadySeconds`) is its bias, which the speed comparison learns and takes off. |
| `steadySeconds` | chosen | math (a push sets the robot going in well under a second, the simulator's at 3 m/s², and then holds it at a speed, which reads as no acceleration; one speeding it up steadily for 2 s, at even 0.5 m/s², would have it sliding 1 m/s past wheels that don't roll. And it learns a checked accelerometer's bias of up to 0.3 m/s², twice what the speed comparison lets go of, before the comparison holds its tolerance's worth) | Why: standing still, a reading the accelerometer holds this long is its bias, whatever its size (gravity leaking into a case tilted on its mount, which a size limit refused, and the slide it then added up claimed meters), and the older half's average is taken for it, which a push only just begun hasn't reached. Until it's learned, an accelerometer not yet checked isn't compared with the wheels: an unchecked robot stands this long after it starts, or after its IMU comes back, before the speed comparison sees anything. Lengthen it if `Localization/AccelerometerBiasMetersPerSecSq` jumps while the robot is pushed standing; shorten it if the robot must drive sooner. |
| `steadyMetersPerSecond` | chosen | simulation (the simulator's 0.3 m/s² of vibration on each reading adds up over 2 s to a speed about 0.06 m/s off a steady one, past 0.1 in one window in 40, which only waits a loop to learn; a push building at 1 m/s³ strays past it half a second in) | Why: the reading holds when the speed it adds up to over `steadySeconds` keeps within this of what its average adds up to; further, it rose or fell (a push building, or come and gone), and nothing is learned. A push building slower than 0.2 m/s³ passes for steady, and the bias follows it 1.5 s behind ([known limits](testing.md#what-it-proves-and-what-it-doesnt)). Raise it if a real robot's vibration standing keeps `Localization/AccelerometerBiasMetersPerSecSq` from settling; a Pigeon 2 still reads within a few hundredths of a m/s². |
| `largestBiasMetersPerSecSq` | chosen | math (a Pigeon 2's bias is a tenth of this; gravity leaks 1 m/s² into a case tilted 6° on its mount) | Why: a bias learned past this is gravity's share, the IMU turned on the robot otherwise than its definition's `orientation` says, and the alert "check its orientation in the robot's definition" comes on. Its pitch and roll can be right all the while (a Pigeon's follow the mount pose saved on it in Tuner X), so only the accelerometer shows it. |
| `maxSlideMetersPerSecond` | chosen | guess (faster than FRC robots drive) | Why: no slide counts faster than this. Past it, what the speed comparison holds is a bias it hasn't learned adding up, not a slide: standing half a minute with a 0.3 m/s² bias it couldn't learn, it claimed 15 m. |
| `joltMetersPerSecSq` | chosen | math (over the tread's grip, 11.8 m/s² at its 1.2, which driving can't pass, and under the Pigeon 2's ±2 g range, 19.6 m/s², where a hit along one axis must still show: a top team used 20, which a Pigeon 2 can't read along an axis) | Why: what the IMU feels and the wheels' own change of velocity doesn't explain (a robot stopping hard is felt by both, a hit only by the IMU) is a whole collision past it. Past the tread's grip (`wheelCoefficientOfFriction` times g) and short of it, a share of one, rising with it: up to the grip, the floor can explain it, bringing the robot to wheels that got ahead of it. So a hit counts the same standing or braking. A grip at or past it leaves no share between: up to the grip none, past it a whole one, by size too (`LocalizationService.doubts` says so at startup). Before the accelerometer is checked, the jolt's size alone decides. Check: `Localization/UnexplainedJoltMetersPerSecSq` stays under the grip in hard driving, and goes over this in hits. |
| `accelerometerRangeMetersPerSecSq` | as built | datasheet (Phoenix 6's `AccelerationX` signal reads from -2 to 2 g) | The most the accelerometer reports on each axis. A reading at it is past what the part can say, so it's a jolt whatever the wheels did. Another IMU, its own range. |

### `ImuCrossCheck.Settings`

`com.michaelgrundvig.frc.localization.ImuCrossCheck.Settings`: how closely the gyro and the backup
IMU must agree before an alert says one is failing; the robot uses `Settings.DEFAULTS`. Check
`Localization/ImuDisagreementDegrees`: well inside the tolerance in normal driving, and over it only
for a real fault.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `windowSeconds` | chosen | guess | Why: long enough to see two IMUs turn apart, short enough to catch a frozen one in a second. Change if a frozen gyro (the `gyro-frozen` scenario) takes too long to flag, or hard turns set off the alert. |
| `toleranceDegrees` | chosen | guess | Why: above what the two drift and scatter apart in the window ([gyro drift and noise](characterization.md#gyro-drift-and-noise), for both). Change if the alert comes on in normal driving. |
| `scaleTolerance` | chosen | guess | Why: above the two IMUs' difference in scale; the gyro's is calibrated, the backup's isn't ([the backup's scale](characterization.md#repeatability-of-the-gyros-scale)). Change if fast spins set off the alert with both IMUs healthy. |

## The simulator

### `SensorErrors`

`com.michaelgrundvig.frc.simulation.SensorErrors`: how the simulated robot's sensors differ from the truth.
`SensorErrors.TYPICAL` is a robot partway through a season (the simulator and its scenarios use it);
`NONE` is exact, for tests of exact numbers. The code calls TYPICAL's values plausible estimates:
replace them with a real robot's once it's calibrated.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `wheelRadiusScale` | characterized | guess | The spread of each wheel's radius in [the wheel radius](calibration.md#the-wheel-radius)'s report, partway through a season, over the radius configured. |
| `steerOffsetDegrees` | characterized | guess | How far modules point off straight after their offsets are set: square the wheels against a straightedge, and read each module's angle (`Drive/Module<name>/SteerAngle`). |
| `cameraMountErrors` | characterized | guess (calibrated mounts stay put) | How far a mount moves between calibrations: [a camera's mount](calibration.md#a-cameras-mount)'s "before" residual, run again weeks later. Scenarios, and the Pitwall page, knock a mount while the simulator runs (`com.michaelgrundvig.frc.runtime.sim.Faults`). |
| `tagErrors` | characterized | none (the simulator's field is exactly its layout) | How far a field's tags sit from its layout: a survey of the field (WPIcal). The consistency study draws them within the field's half-inch tolerance; the Pitwall page moves a tag while the simulator runs (`com.michaelgrundvig.frc.runtime.sim.Faults`). |
| `seed` | chosen | none | Any number: fixed, so a run repeats exactly. |

### `SensorErrors.GyroErrors`

One gyro's errors. TYPICAL gives the gyro and the backup IMU each their own; EXACT has none.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `scale` | characterized | guess | [The gyro's scale](calibration.md#the-gyros-scale) before it's calibrated (one over its first `gyroYawScale`), and [the backup's](characterization.md#repeatability-of-the-gyros-scale). |
| `driftDegreesPerMinute` | characterized | guess | [Gyro drift and noise](characterization.md#gyro-drift-and-noise), a typical run. The backup's is unknown until measured. |
| `noiseDegrees` | characterized | guess | [Gyro drift and noise](characterization.md#gyro-drift-and-noise), a typical run. |

### `SensorErrors.CornerNoise`

How far the simulated cameras' tag corners land from where a perfect camera would put them, the same
for every camera. TYPICAL (and NONE) give PhotonVision's simulation example; a study widens it to
see whether a result hangs on the simulator's kind cameras.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `meanPixels` | characterized | datasheet (PhotonVision's simulation example, 0.25 px) | [Camera noise](characterization.md#camera-noise): the average distance of a corner from the mean of its readings, standing still. |
| `stdDevPixels` | characterized | datasheet (PhotonVision's simulation example, 0.08 px) | As `meanPixels`: the spread of that distance. |

### `SensorErrors.AccelErrors`

How one simulated accelerometer reads: its errors, and the part's own limits. TYPICAL gives the gyro
and the backup IMU each a Pigeon 2's (`AccelErrors.PIGEON_2`); NONE gives each an ideal one
(`AccelErrors.IDEAL`), the robot's average acceleration over each loop, exactly. The simulated IMU
feels the robot's true motion where it sits, gravity's share as it tilts, and a hit's jolt.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `biasForward` | characterized | guess (CTRE publishes no bias for the Pigeon 2's accelerometer, factory-calibrated, and the boot plot in its User's Guide shows a few mg; a mount half a degree off level puts 0.09 m/s² of gravity in it) | Standing level and still, the average of `Drive/Gyro/AccelerationXMetersPerSecSq` over ten seconds. |
| `biasLeft` | characterized | guess (as `biasForward`) | As `biasForward`, along y. |
| `noise` | characterized | guess (no noise is published; this is a driving robot's vibration, which the part's own noise of a few mg is far under) | The spread of `AccelerationXMetersPerSecSq` while driving steadily straight. |
| `turnedDegrees` | characterized | guess (an orientation set within half a degree) | How far the part really lies turned about the robot's up axis from the orientation its `ImuMount` gives it. What the [accelerometer check](calibration.md#the-accelerometer-reads-along-the-robots-axes) leaves: a robot speeding up straight ahead reads this share of it sideways. |
| `flipped` | characterized | none (a checked mount) | Whether the part is really mounted the other way up from its `ImuMount`'s orientation. The accelerometer check: strafing left reads negative. |
| `forwardMeters` | characterized | guess (a place measured to a centimeter) | How far from where its `ImuMount` places it the IMU really sits. |
| `leftMeters` | characterized | guess (as `forwardMeters`) | As `forwardMeters`, to the left. |
| `rangeMetersPerSecSq` | as built | datasheet (Phoenix 6's `AccelerationX` signal reads from -2 to 2 g) | A fact of the part: a hit past it reads at it. |
| `sampleHz` | chosen | datasheet (`GyroIOPigeon2` asks for the accelerometer at 100 Hz, the CAN FD default in Phoenix 6's documentation) | How often the code asks for it. |
| `filterSeconds` | characterized | guess (CTRE doesn't publish the filter; sampled at 100 Hz, an accelerometer needs a low-pass under 50 Hz, and this is 10 Hz. At 5 ms the simulated robot's 100 Hz steering jolts, read once a loop, alias into a slide of up to 0.8 m/s spinning while driving; at 16 ms, 0.16) | A Pigeon 2 spinning in place, logged at its fastest: how much of the steering's 100 Hz shudder its readings still show. |

## Mass, shapes, and materials

What the field and the robot are both made of (`com.michaelgrundvig.frc.world.physical`): how much a
part weighs and how hard it is to turn, its shape, and its material. The template's robot's body
holds the one starting value, its mass properties. Nothing else in the robot holds one yet: a
mechanism's links' mass properties and shapes come from `Robots.java` as its joints join the
definition.

### `MassProperties`

`com.michaelgrundvig.frc.world.physical.MassProperties`: a part's mass, where it's centered, and how
hard it is to turn, in the part's own frame: a link's, in its joint's (`Link.mass`). Its inertia has
a section of its own. A link's mass is optional, and a rough one does: the simulator reads it, and
beyond that it only seeds its joint's kG and kA until they're tuned. What nobody has given is none,
`MassProperties.NONE`: zero mass at the joint, turning freely, never a guess. Its joint's gravity
is then its kG's alone, and its kA seeds from its motors' rotors alone; the simulator's notes list
it (`Robot/SimulatorNotes`), never the robot's console. The guide's robot the template runs gives
its intake arm a shape, not a measured mass. Its shooter's flywheel is its one link with a mass, an
estimate, so that the simulator's shooter takes the half second a real one does to reach 3000 rpm.
Without it the shooter would be there within one loop.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `mass` | calibrated | guess (the guide's shooter's flywheel, 1.8 kg: four 4 in wheels of 0.25 kg and two 4 in steel disks a quarter of an inch thick, 0.4 kg each; none weighed) | CAD's mass-properties report, checked on a scale where the part comes off. Zero until then. The links' masses together are part of the robot's, and must leave some for the chassis. |
| `cg` | calibrated | none | CAD's mass-properties report, in the joint's frame. With the mass, it seeds what the joint's motors hold against gravity (`RobotMath.gravityTorque`), and where an arm's weight pulls hardest, until its kG is tuned. At the joint's origin until then. |

### `Inertia`

`com.michaelgrundvig.frc.world.physical.Inertia`: a part's inertia tensor about its center of mass,
along its frame's axes, in kg m², as the tensor's own entries (a product of inertia is minus the
integral). A link's is zero until CAD gives it (`Inertia.ZERO`), and the robot's body's, a
whole tensor from CAD (`Body.Turning.tensor`). `Inertia.box` is the model's stand-in for what
isn't given: a solid box of the part's size, its mass spread evenly, which a robot isn't, so it's
a guess, flagged in the missing-data report. The template's robot gives none: its yaw alone. The
guide's robot gives one, its shooter's flywheel's (`ExampleRobot`).

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `ixx` | calibrated | math (the guide's shooter's flywheel, across its shaft, as a solid cylinder 0.5 m long: a twelfth of its mass times three times its radius squared plus its length squared, which nothing reads) | CAD's mass-properties report, about the center of mass in the part's frame. A whole robot's roll: what a tip resists. |
| `iyy` | calibrated | math (the guide's shooter's flywheel, about its shaft, as a solid cylinder of its mass 4 in across: half its mass times its radius squared, 0.00232 kg m²) | As `ixx`, about y: a whole robot's pitch. A flywheel's about its shaft, on a roller's own axis, is what its motors spin up. |
| `izz` | calibrated | math (as `ixx`) | As `ixx`, about z: a whole robot's yaw, which it may give alone (`Body.Turning.Yaw`). |
| `ixy` | as built | none | As `ixx`: the tensor's entry for x and y. Check the CAD program's sign: some report the integral, which this is minus. Zero for a part symmetric across either axis. |
| `ixz` | as built | none | As `ixy`, for x and z. |
| `iyz` | as built | none | As `ixy`, for y and z. |

### `Material`

`com.michaelgrundvig.frc.world.physical.Material`: what something is made of, by name. Raised ground
is the field's carpet (`Material.CARPET`) unless it says otherwise. How two materials grip and
bounce comes later, with the simulator's contacts, per pair.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `name` | as built | none | What it's called, the same everywhere: two parts named alike are made of the same stuff. |

### `Shape.Box`

`com.michaelgrundvig.frc.world.physical.Shape.Box`: a box, centered on its pose, in its owner's
frame (a link's). Its material has a section of its own.

| Value | Kind | Source | How to measure it, or why |
|---|---|---|---|
| `pose` | as built | none | From CAD, or the game manual's drawings: its center, and how it's turned. |
| `size` | as built | none | As `pose`: its length, width, and height along its own axes. Simplified by hand: what a robot runs into and a camera can't see through. |

## Tuning limits

How far the pit may take each value a mechanism tunes live: where each limit came from, and why.
The limits themselves are declared with the value in the mechanism's `tuning`, and listed together
in `RobotContainer.tunings`. All are chosen: wide enough to tune in, narrow enough that a slip of a
finger can't hurt the robot. The record's own checks still apply inside them. Change a limit when
tuning on a robot keeps wanting to go past it, after asking why.

Each driven joint has a tuning of its own, named for it (`JointMechanism.tuning`), with limits that
follow its committed values, since its units are its own, each widened to take in what's
committed. Its motor controller takes a change on the next loop. Its feedforward, profile and named positions aren't tunable yet.

### `drivetrain` tuning

`Drive.tuning`.

| Knob | Source | Why |
|---|---|---|
| `gains.driveKs` | guess | Static friction takes a few tenths of a volt; past a volt, the robot creeps. |
| `gains.driveKv` | guess | The top, twice the ideal 2.54: room for a real drivetrain's losses, not for a unit mistake. |
| `gains.driveKp` | guess | The top, ten times the template's: room to find where it buzzes. |
| `gains.steerKp` | guess | The top, about six times the template's. |
| `gains.steerKd` | guess | The top, far above the template's; damping this high only adds noise. |
| `gains.skewLookahead` | guess | The top, five loops; past it, the robot steers ahead of itself. |
| `limits.driveStatorCurrent` | datasheet (120 A: Phoenix 6's default stator limit) | Up to Phoenix 6's own default; under 10 A the robot barely moves. |
| `limits.driveSupplyCurrent` | guess | Room above the template's 60 A; four motors drawing more brown out a battery. |
| `limits.driveSupplyLowerCurrent` | datasheet (40 A: the drive motors' breakers' rating) | Never above what the breaker carries indefinitely. |
| `limits.driveSupplyLowerTime` | datasheet (the fastest 40 A breaker, Snap Action's, can trip after 1.5 s at 80 A, the top of `limits.driveSupplyCurrent`) | At any supply limit in its range, the step-down comes at least half a second before the fastest breaker can trip (a warm breaker trips sooner than its datasheet's cold figures). Under 0.1 s it would cut into every hard launch; even 0.1 s cuts into one with `driveStatorCurrent` raised to 80 A or more, which holds a drive motor at 60 A or more for 0.12 to 0.2 s in the simulator. |
| `limits.steerStatorCurrent` | guess | Steering needs little; the top is for finding out how little. |
| `limits.maxLinearSpeed` | math (the top: the free speed its motors and gearing give, from its definition) | Slow enough for a first drive at the bottom. The top is the drivetrain's own free speed, which velocity control needs room under to correct with: a robot geared faster is tuned faster. |
| `limits.maxAngularSpeed` | guess | Slow enough for a first drive at the bottom, and three times the template's at the top. |

### `vision` tuning

`Vision.tuning`.

| Knob | Source | Why |
|---|---|---|
| `maxAmbiguity` | math (an ambiguity is a ratio from 0 to 1) | Its whole range. |
| `maxHeightErrorMeters` | guess | A meter off the floor is no robot. |
| `linearStdDevBaseline` | guess | Never zero, which would trust a camera completely. |
| `angularStdDevBaseline` | guess | As `linearStdDevBaseline`. |
| `distanceExponent` | top team (teams used 2 to 4) | The range teams used, and flat. |
| `mountToleranceMeters` | guess | From tighter than a calibrated camera can hold to a camera far off its mount. |
| `latencyMarginSeconds` | guess | From tighter than a camera's own jitter to a second, still inside the estimator's 1.5 s of history. |

### `pivot` tuning

`JointMechanism.tuning`, from the guide's pivot (`ExampleRobot`).

| Knob | Source | Why |
|---|---|---|
| `gains.kP` | guess | Up to ten times what's committed, in the joint's own units, which tells its scale: room to find where it buzzes. |
| `gains.kD` | guess | As `gains.kP`, from at least 1. |
| `gains.kS` | guess | Up to 2 V, or what's committed if more: static friction takes a few tenths of a volt. |
| `currentLimits.stator` | datasheet (120 A: Phoenix 6's default stator limit) | From 5 A, where it barely moves, up to Phoenix 6's own default, or what's committed if more. |
| `currentLimits.supply` | guess | From 5 A up to 80 A, or what's committed if more: past it, a mechanism drags the battery down with the drivetrain. |

### `roller` tuning

`JointMechanism.tuning`, from the guide's roller (`ExampleRobot`).

| Knob | Source | Why |
|---|---|---|
| `gains.kP` | guess | Up to ten times what's committed, in the joint's own units, which tells its scale: room to find where it buzzes. |
| `gains.kD` | guess | As `gains.kP`, from at least 1. |
| `gains.kS` | guess | Up to 2 V, or what's committed if more: static friction takes a few tenths of a volt. |
| `currentLimits.stator` | datasheet (120 A: Phoenix 6's default stator limit) | From 5 A, where it barely moves, up to Phoenix 6's own default, or what's committed if more. |
| `currentLimits.supply` | guess | From 5 A up to 80 A, or what's committed if more: past it, a mechanism drags the battery down with the drivetrain. |

### `hood` tuning

`JointMechanism.tuning`, from the guide's hood (`ExampleRobot`).

| Knob | Source | Why |
|---|---|---|
| `gains.kP` | guess | Up to ten times what's committed, in the joint's own units, which tells its scale: room to find where it buzzes. |
| `gains.kD` | guess | As `gains.kP`, from at least 1. |
| `gains.kS` | guess | Up to 2 V, or what's committed if more: static friction takes a few tenths of a volt. |
| `currentLimits.stator` | datasheet (120 A: Phoenix 6's default stator limit) | From 5 A, where it barely moves, up to Phoenix 6's own default, or what's committed if more. |
| `currentLimits.supply` | guess | From 5 A up to 80 A, or what's committed if more: past it, a mechanism drags the battery down with the drivetrain. |

### `elevator` tuning

`JointMechanism.tuning`, from the guide's elevator (`ExampleRobot`).

| Knob | Source | Why |
|---|---|---|
| `gains.kP` | guess | Up to ten times what's committed, in the joint's own units, which tells its scale: room to find where it buzzes. |
| `gains.kD` | guess | As `gains.kP`, from at least 1. |
| `gains.kS` | guess | Up to 2 V, or what's committed if more: static friction takes a few tenths of a volt. |
| `currentLimits.stator` | datasheet (120 A: Phoenix 6's default stator limit) | From 5 A, where it barely moves, up to Phoenix 6's own default, or what's committed if more. |
| `currentLimits.supply` | guess | From 5 A up to 80 A, or what's committed if more: past it, a mechanism drags the battery down with the drivetrain. |

### `arm` tuning

`JointMechanism.tuning`, from the guide's arm (`ExampleRobot`).

| Knob | Source | Why |
|---|---|---|
| `gains.kP` | guess | Up to ten times what's committed, in the joint's own units, which tells its scale: room to find where it buzzes. |
| `gains.kD` | guess | As `gains.kP`, from at least 1. |
| `gains.kS` | guess | Up to 2 V, or what's committed if more: static friction takes a few tenths of a volt. |
| `currentLimits.stator` | datasheet (120 A: Phoenix 6's default stator limit) | From 5 A, where it barely moves, up to Phoenix 6's own default, or what's committed if more. |
| `currentLimits.supply` | guess | From 5 A up to 80 A, or what's committed if more: past it, a mechanism drags the battery down with the drivetrain. |

### `feeder` tuning

`JointMechanism.tuning`, from the guide's feeder (`ExampleRobot`).

| Knob | Source | Why |
|---|---|---|
| `gains.kP` | guess | Up to ten times what's committed, in the joint's own units, which tells its scale: room to find where it buzzes. |
| `gains.kD` | guess | As `gains.kP`, from at least 1. |
| `gains.kS` | guess | Up to 2 V, or what's committed if more: static friction takes a few tenths of a volt. |
| `currentLimits.stator` | datasheet (120 A: Phoenix 6's default stator limit) | From 5 A, where it barely moves, up to Phoenix 6's own default, or what's committed if more. |
| `currentLimits.supply` | guess | From 5 A up to 80 A, or what's committed if more: past it, a mechanism drags the battery down with the drivetrain. |

### `indexer` tuning

`JointMechanism.tuning`, from the guide's indexer (`ExampleRobot`).

| Knob | Source | Why |
|---|---|---|
| `gains.kP` | guess | Up to ten times what's committed, in the joint's own units, which tells its scale: room to find where it buzzes. |
| `gains.kD` | guess | As `gains.kP`, from at least 1. |
| `gains.kS` | guess | Up to 2 V, or what's committed if more: static friction takes a few tenths of a volt. |
| `currentLimits.stator` | datasheet (120 A: Phoenix 6's default stator limit) | From 5 A, where it barely moves, up to Phoenix 6's own default, or what's committed if more. |
| `currentLimits.supply` | guess | From 5 A up to 80 A, or what's committed if more: past it, a mechanism drags the battery down with the drivetrain. |

### `shooter` tuning

`JointMechanism.tuning`, from the guide's shooter (`ExampleRobot`).

| Knob | Source | Why |
|---|---|---|
| `gains.kP` | guess | Up to ten times what's committed, in the joint's own units, which tells its scale: room to find where it buzzes. |
| `gains.kD` | guess | As `gains.kP`, from at least 1. |
| `gains.kS` | guess | Up to 2 V, or what's committed if more: static friction takes a few tenths of a volt. |
| `currentLimits.stator` | datasheet (120 A: Phoenix 6's default stator limit) | From 5 A, where it barely moves, up to Phoenix 6's own default, or what's committed if more. |
| `currentLimits.supply` | guess | From 5 A up to 80 A, or what's committed if more: past it, a mechanism drags the battery down with the drivetrain. |

### Experiment rigs

A rig is a mechanism as Pitwall's experiments run it. Its gains are values its mechanism already
tunes (the steering's are the drivetrain's `gains.steerKp` and `gains.steerKd`, above), so their
limits are checked there. A rig's own limits are another kind of number: not how far a value may
be tuned, but where an experiment may send the mechanism, so the setpoints of every test motion stay
inside them. Rigs are built from a running mechanism and registered in `RobotContainer`, so the
test doesn't read them: add a row here when you add a rig or an experiment.

| Rig or experiment | Numbers | Kind | Source | Why |
|---|---|---|---|---|
| The steering rig (`Drive.steeringRig`): where an experiment may point the wheels | -180° to 180° | chosen | guess | A module turns all the way around, so any angle is safe; the limits keep a test motion within one turn, as the wheel's angle is scored. |
| The steering's experiment (`RobotContainer`): "Steer to 90° and back" | 0° to 90° and back, 0.8 s at each, after a 0.5 s lead-in | chosen | guess | A quarter turn shows rise, overshoot, and settling; 0.8 s is several times the steering's settling (122 ms in the simulator), so each step settles before the next. |
| Each joint's rig (`JointExperiments`): where an experiment may send it | its range, else its places' span; a speed's, still to its fastest named speed | chosen | math (the joint's own range, places and speeds, from its definition) | The joint's own bounds, so a test motion never asks it for anything its commands couldn't. |
| Each joint's experiments (`JointExperiments`): a step between each two of its places, or from still to each of its speeds, and back | held twice the time its profile takes to make the move, or its velocity loop to reach the speed, rounded up to 0.1 s; at least 0.5 s, at most 3.3 s, the lead-in as long | chosen | math (the joint's own profile and model) | Long enough to see it arrive and settle; never past the 10 s a run may take, which Pitwall refuses as the robot starts. |

## Switches

The drive team's switches: WPILib tunables a mechanism publishes for itself, not configuration.
Each keeps its setting into a match, within its limits, with an alert naming each one changed since
the robot started; its limits and how it starts are the mechanism's, where it publishes it.
`ValuesRegisterSimTest` builds the robot and checks every switch it publishes has a row here; a
switch published for each camera is written with `<camera>` in its path.

| Switch | Source | Why |
|---|---|---|
| `Vision/autoStartHoldSeconds` | guess (a top team held theirs 2 s) | Why: holds the cameras' measurements back as autonomous starts, trusting the placed start while the robot launches. Off, since the estimator gates every measurement against the placed start anyway; 30 s is longer than any autonomous period has been. Turn it on if logs show the cameras pulling the estimate off its start in autonomous's first seconds. |
| `Vision/<camera>/enabled` | none | Why: leaves one camera out, fogged over, knocked loose, or blinded by lights, while the others carry on. |

## The camera study's assumptions

The camera placement study ([docs/camera-placement.md](camera-placement.md)) scores where cameras
could go by simulating what they'd see, so it assumes how the cameras and the robot err. Its
options (`CameraStudyConfig`, in the robot template's studies) are a run's choices, not
configuration; these are the assumptions inside them, and the ranges a robustness run draws from. The test doesn't check this list. Measuring the cameras ([camera noise](characterization.md#camera-noise))
replaces the nominal values, and narrows the ranges.

| Assumption | Template's starting value | Kind | Source | How to measure it, or why |
|---|---|---|---|---|
| Tag corner noise, nominal | 0.25 px average, 0.08 px spread | characterized | datasheet (`VisionSim`'s, PhotonVision's simulation example) | [Camera noise](characterization.md#camera-noise). |
| How far tags are seen, nominal | 7 m | characterized | guess (`VisionSim`'s) | [Camera noise](characterization.md#camera-noise): the farthest a tag is found in most images. |
| The gyro's heading error, nominal | 0.25°, one standard deviation | characterized | guess | The estimator's heading uncertainty while cameras correct it (`Localization/HeadingStdDevDegrees`), checked in the simulator against `Sim/HeadingErrorDegrees`. |
| Each camera off its mount, nominal | none | characterized | guess (calibrated mounts) | [A camera's mount](calibration.md#a-cameras-mount): its result's "how sure" after calibrating. |
| Corner noise, robustness range | 0.1 to 0.6 px | chosen | guess | A sharp, still global-shutter image locates corners to about a tenth of a pixel; defocus, a softer lens, or blur, half a pixel or more ([the study's reasons](camera-placement.md#robustness)). |
| Sight, robustness range | 5 to 7 m | chosen | guess | Where detection gives out for a field-size tag at the camera's resolution, with exposure and blur ([the study's reasons](camera-placement.md#robustness)). |
| Gyro heading error, robustness range | 0.1° to 0.5° | chosen | guess | A few tenths of a degree while cameras correct it; half a degree after a hit or a stretch without two tags ([the study's reasons](camera-placement.md#robustness)). |
| Mount error, robustness range | 0 to 5 mm and 0° to 1° per axis | chosen | guess | A mount checked by calibration lands within a few millimeters and a fraction of a degree; a bumped or uncalibrated one, a degree or more ([the study's reasons](camera-placement.md#robustness)). |

## The consistency study's assumptions

The estimator consistency study ([docs/testing.md](testing.md#is-the-estimator-honest-the-consistency-study))
checks the estimator's uncertainty against the simulator's truth, so it assumes how a calibrated
robot errs: by default, as the estimator's configuration claims, so that an honest estimator reads
a NEES near its number of dimensions. Its options are a run's choices, not configuration; these are
the assumptions inside them. The test doesn't check this list.

| Assumption | Template's starting value | Kind | Source | How to measure it, or why |
|---|---|---|---|---|
| The robot's wear | `SensorErrors.TYPICAL`, but each IMU's scale drawn per seed | characterized | guess (the simulator's season-worn robot) | Its rows [above](#sensorerrors). |
| Each camera off its mount | Gaussian, 3 mm along and the mount tolerance's spread (0.58°) about each axis, drawn for each seed | calibrated | guess (3 mm: what the mount procedure leaves, within the mount's tolerance and `cameraShiftStdDevMeters`, which holds what the camera's own solve repeats too); math (the turn: as the configuration claims) | [A camera's mount](calibration.md#a-cameras-mount): the spread of what five runs from the same spots find. Wider with `--mount-error-mm` and `--mount-error-degrees`. |
| Each tag off the field's layout | Gaussian, `tagPlacementStdDevMeters` along each axis, drawn for each seed | characterized | math (as the configuration claims) | A survey of the field (WPIcal). Wider with `--tag-error-mm`. |
| The gyro's scale | Gaussian, off by `headingStdDevDegreesPerTurn` / 360, drawn for each seed | characterized | math (as the configuration claims) | [The gyro's scale, repeated](characterization.md#repeatability-of-the-gyros-scale). |
| The backup IMU's scale | Gaussian, off by 0.5%, drawn for each seed | characterized | guess (`SensorErrors.TYPICAL`'s backup) | [The backup IMU's scale](characterization.md#repeatability-of-the-gyros-scale). |
| The vision noise model | half the configured one (`--noise-scale=0.5`) | characterized | simulation (at 1, the study reads a position NEES near 0.7; at half, near 2) | [Camera noise](characterization.md#camera-noise): how noisy the robot's own cameras are against the 2026 model. |

## Numbers still written in code

These aren't configuration yet, and the test doesn't check this list. A number worth measuring
belongs in a configuration record, where the test makes the register cover it; until it moves, it's
listed here. Other numbers in code (deadbands, the calibration procedures' own settings) are the
code's choices, explained beside them.

| Where | Number | Kind | Source | How to measure it, or why |
|---|---|---|---|---|
| `VisionSim`: tag corner noise | 0.25 px average, 0.08 px spread | characterized | datasheet (PhotonVision's simulation example) | [Camera noise](characterization.md#camera-noise): how far the corners scatter, standing still. |
| `VisionSim`: latency's spread | 5 ms | characterized | datasheet (PhotonVision's simulation example) | [Camera latency](characterization.md#camera-latency-and-time-sync). |
| `PhoenixDrivetrain`: how often each module's CANcoder sends its position and velocity (the simulator's steering hears it as often) | 100 Hz | chosen | datasheet (Phoenix 6's default for a CANcoder's position) | Why: without Phoenix Pro, the steering's loop sees a new angle only this often, which sets how high its gains can go. Faster allows higher gains, for more traffic on the CAN bus: check the bus's load before raising it. Check that a robot's CANcoders send this often: Phoenix Tuner X's plot of a CANcoder's position shows a new sample every 10 ms, and in a log of a module turning, its odometry angles (`Drive/Module<name>/OdometrySteerAnglesRad`) change only every 10 ms, the same angle two or three readings running. |
| `ModuleIOSim`: how often the steering's motor controller runs its loop | 1 ms | as built | datasheet (CTRE's Phoenix 6 documentation: a TalonFX's closed loop runs at 1 kHz) | A fact of the motor controller. |
| `ModuleIOSim`: the battery's voltage, which no simulated motor gets more of | 12 V | characterized | datasheet (WPILib's simulated battery, which its `DCMotorSim` held each motor to) | [Battery internal resistance](characterization.md#battery-internal-resistance): a real battery sags under the drive's load, which the simulator leaves out ([stand-ins](stand-ins.md#the-simulators-physics)). |
| `Localization`: a tilt past a bump's slope still counted as the ground | 3° | calibrated | guess | [Tilt on a bump](calibration.md#tilt-on-a-bump), on a real robot: bumpers, suspension, and the IMU's mount. |
| `Vision`: how far back "lately" reaches for turning | 0.3 s | chosen | top team | Why: an image taken just after a fast turn is still blurred, and its capture time matters most. |
| `Vision`: how many of a camera's latest tags its mount is judged by | 500 | chosen | simulation (a knock putting tags 6 cm off alerts after 417 tags, 10 cm after 251) | Why: enough for noise and the estimate's own error to average out, few enough that a knock shows within seconds of seeing tags. Check: after a real knock, how long until the alert. |
| `Vision`: how recently another camera must have agreed for a camera's tags to count toward its mount | 2 s | chosen | simulation (half a second left out cameras facing other ways, which see tags at other moments) | Why: the other cameras hold the pose a camera is judged against. |

## Records that aren't configuration

| Record | Why it isn't configuration |
|---|---|
| `ImuReadings` | What one IMU read in one loop. |
| `DrivetrainReadings` | What the drivetrain read in one loop, for localization. |
| `RobotStatus` | What the drive team's dashboard shows, built each loop. |
| `DriveHardware` | The drivetrain's devices, handed to it: hardware, not numbers. |
| `Scenario` | A named situation for the simulator. Its numbers are what a test needs, written in `Scenarios`. |
