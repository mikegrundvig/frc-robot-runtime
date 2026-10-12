# Coming from WPILib command-based code

This page is for someone who has shipped a few seasons of command-based robot code. You know
subsystems with IO layers, AdvantageKit, a swerve template, a path tool, PhotonVision and SysId.
Nothing here explains FRC. It says what maps to what, what is gone, what is new, where your
habits will mislead you, and what you give up.

If you want the gentle version first, read [the guide](guide.md). Every block of code below is
copied from compiled source, and a test fails when it drifts.

## The short version

You stop writing subsystems. You write a description of the robot as typed constants, and the
runtime builds each mechanism from it, on real hardware, in the simulator and in replay.

Commands are WPILib's Commands v3. A command body is a coroutine, so `initialize`, `execute`,
`isFinished` and the composition factories are replaced by ordinary control flow.

Safety rules between mechanisms move out of commands and into the description, where they hold
under every command.

The description is validated when the program starts. A value that cannot work stops the program
with a message. It is not logged as a warning.

Localization, the simulator, autos, live tuning and the pit page are the runtime's, not yours.
You give up writing them, and you also give up the freedom to write them your own way. The one
exception is the pose estimator, which you can swap for WPILib's or for your own.

It targets WPILib 2027 alpha on Systemcore. It has not driven a real robot yet.

## What maps to what

| You know | Here |
|---|---|
| Subsystem class, IO interface, IOReal, IOSim | A joint block in the description |
| `Constants.java` | The robot's description, typed and checked |
| `initialize` / `execute` / `isFinished` / `end` | A coroutine body |
| `sequence`, `parallel`, `race`, `until` | Statements, `fork`, `awaitAny`, `runUntil` |
| Requirements, default commands | Joint ownership, a joint's idle |
| `Trigger` | `Gate`, a trigger with a name and a logged reason |
| "if arm < x, don't run" inside commands | Interlocks and keep-outs in the description |
| `SwerveDrivePoseEstimator` plus vision | The runtime's localization, or WPILib's estimator behind it |
| PathPlanner or Choreo, `SendableChooser` | `Place`, `Keyframe`, `drive.follow`, `autos.add` |
| Dashboard tuning | Pitwall's Tune and Changes views |
| SysId | Model feedforward and step experiments |
| AdvantageKit | Still AdvantageKit |
| `simulationPeriodic`, `DCMotorSim`, maple-sim | A simulator built from the description |
| Unit tests with mocked IO | Whole-robot simulation tests |

The sections below take these in turn.

### Subsystems and IO layers

A mechanism is one block of constants. This is a pivot with a motor, an absolute encoder and a
profiled position loop, from `ExampleRobot`:

```java
public static final Joint.Revolute PIVOT =
    Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
        .range(Range.degrees(0, 135))
        .rest(PIVOT_IN) // its hard stops, and where it rests
        .carries(
            Link.box(
                "intake arm", new Translation3d(0, 0, 0.2), new Translation3d(0.05, 0.55, 0.36)))
        .positions(PIVOT_IN, PIVOT_OUT)
        .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(32).pdh(4, Amps.of(40))))
        .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))
        .control(
            MechanismControl.position(30)
                .withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700)));
```

That block is the subsystem, its constants, its real IO and its sim IO. The runtime turns it into
a `JointMechanism`, which is a v3 `Mechanism`. The IO layer still exists underneath as `JointIO`,
with `JointIOTalon` and `JointIOSpark` behind it and a simulated controller in the simulator. You
do not write or choose them. The wiring in the block decides.

`joints.of(PIVOT)` gives you the mechanism. It already has `goTo`, `run`, `runUntil`, `follow`,
`hold`, `home`, and the gates `isAt`, `isAtGoal`, `isReady` and `isStalled`.

What the motor controller is told is worked out from the block. That covers the gains, the
current limits, the soft limits, the neutral mode, the gravity feedforward type and the encoder
fusion. You never touch a `TalonFXConfiguration` or a `SparkBaseConfig`.

### Constants

There is no constants file. A value lives in the block of the thing it describes. Records are
built through staged builders that ask for each required value by name, so two values cannot swap
places and a missing one does not compile.

Most values carry a unit type. A few do not, and you should know which. A `Translation3d` is
meters. A gain is a bare number in volts per the joint's own unit, so `position(30)` is 30 V per
radian on a pivot and 30 V per meter on an elevator. You can write a gain with its unit when you
have one from Phoenix or SysId, and [mechanisms.md](mechanisms.md#control-and-gains) shows how.

### Command lifecycle and compositions

A command body is a coroutine. It runs until it pauses, and resumes at the same line on the next
loop. This shot is from the examples. It spins a flywheel, holds a heading, and feeds only while
a combined gate is true:

```java fragment
DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB));
Gate ready = Gate.all("ready to feed", aim.aimed(), shooter.isAtGoal());
co.fork(shooter.follow(() -> RPM.of(3000)));
co.fork(aim); // the sticks drive, and the robot holds its aim...
while (true) {
  co.waitUntil(ready); // ...and the feed runs only while it is aimed
  co.await(joints.run(FEEDING).onlyWhile(ready));
}
```

Here is how the forms you know translate.

| v2 | Here |
|---|---|
| `sequence(a, b)` | `co.await(a); co.await(b);` |
| `parallel(a, b)` | `co.fork(a); co.fork(b);` then await each |
| `deadline(b, a)` | `co.fork(a); co.await(b);` |
| `race(a, b)` | `co.awaitAny(a, b)` |
| `a.until(cond)` | `joint.runUntil(output, gate)`, or `move.onlyWhile(gate)` |
| `a.withTimeout(t)` | `command.withTimeout(t)`, or `move.within(t)` on a move |
| `a.onlyIf(cond)`, `a.unless(cond)` | `move.onlyIf(gate)` |
| `isFinished` | The coroutine returns, or the move ends |
| `end(interrupted)` | Nothing to write for a joint. It goes idle |

A forked command is cancelled when its parent ends. So `co.fork(a); co.await(b);` is a deadline
group with `b` as the deadline, not `parallel`. To wait for both, await both.

`within` and `withTimeout` differ. `within` sets a move's own deadline, and the move then ends
with the outcome `TIMED_OUT` and a logged reason. `withTimeout` is v3's, and it cancels.

What is new is the outcome. Every move, on a joint or the drivetrain, ends as `ARRIVED`,
`TIMED_OUT`, `CANCELED` or `REFUSED`, and your code can branch on it:

```java fragment
var deploy = pivot.goTo(PIVOT_OUT);
co.await(deploy);
if (!deploy.arrived()) {
  co.await(pivot.goTo(PIVOT_IN)); // back off
}
```

Every move to a position has a default deadline, taken from its profile's time. An auto cannot
hang on a jammed mechanism.

### Requirements and default commands

Each driven joint is its own `Mechanism`. A command requires exactly the joints it names, and a
new command that requires one cancels whatever held it. That part is unchanged.

What changes is what a joint does with no command. It has an idle of its own. A joint that goes
to positions holds its goal, which is where its last move arrived or where it came to rest after
a cancel. A joint that only runs at outputs stops. You do not need a default command to hold an
arm up.

A default command still works. It replaces the idle.

The drivetrain is the one mechanism with a default command set for you. The runtime sets
`drive.driven()`, which is field-relative stick driving, and binds the driver's back button to a
heading reset.

### Triggers and gates

A `Gate` is a v3 trigger. You bind it, await it, or guard a move with it. It adds two things. It
has a name, and when it is false it logs why under `Gates/<name>/Blocking`. A gate made with
`Gate.all` lists the terms that are false.

```java
Gate aimed =
    Gate.within(
        scheduler, "aimed", () -> headingErrorDegrees(), 2.0, 4.0); // in at 2°, out past 4°
Gate readyToShoot =
    Gate.all("ready to shoot", shooter.isAtGoal(), aimed)
        .debounce(
            Milliseconds.of(80), Debouncer.DebounceType.FALLING); // rides out a flywheel's dip
```

`Gate.within` is a band with hysteresis. Any condition the model can state becomes a gate through
`joints.gate(...)`, which covers a joint's position, a roller's speed and a sensor's reading:

```java fragment
Gate coralIn = joints.gate("coral in claw", CLAW_BEAM.tripped());
Gate hopperFull = joints.gate("hopper full", HOPPER.nearerThan(Inches.of(6)));
```

Bindings are recorded too. The robot logs each one with its button and its command's name, and
says at startup when two commands share a button the same way.

### Safety checks between mechanisms

You have written `if (pivot.getAngle() > 110) roller.run()` inside a command, and then again in
the next command. Here the rule is written once, on the joint it restricts:

```java fragment
// The roller runs only once the pivot is out of the frame.
.runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
```

```java fragment
// The arm keeps out of the bumper's band while the elevator is too low to clear it.
.keepsOut(Range.degrees(-10, 30))
.when(ELEVATOR.atMost(Inches.of(32)))
```

An interlock is applied after the command's request and before the output, every loop. A command
does not see it. A roller told to run early waits and starts when the condition holds. An arm
told to pass through a closed band stops at its edge and carries on when the band opens. The
command keeps running the whole time.

If the joint an interlock reads is unhomed, faulted or silent, the interlock assumes the worst
and holds. A lost sensor does the same.

The description checks interlocks against each other and against every configuration at startup.
Two rules that would leave two joints holding each other for good are refused as a deadlock.

Soft limits are separate and still exist, in the joint's control. They go to the motor
controller.

### Pose estimation and vision

By default the runtime does not use `SwerveDrivePoseEstimator`. It has its own estimator, in the
localization library.

What is the same: odometry predicts, vision corrects, and measurements are applied at their
capture time with latency compensation.

What differs in the default estimator:

- The state is a position and a heading offset. The gyro owns short-term heading, and cameras
  only correct its slow drift.
- Every camera measurement is gated against the estimate before it is applied. A misread tag is
  rejected and does not move the pose.
- When several rejected measurements agree with each other, the estimator decides odometry was
  wrong and recovers to them. That is its answer to a collision.
- Wheel slip, judged from the modules' disagreement and the accelerometer, lowers the wheels'
  trust for that step.
- A late measurement is applied in order. The estimator rewinds, applies it and replays, so
  arrival order does not change the result.
- The robot reads each tag's corners and both candidate poses from PhotonVision, with the
  capture time, and works from those.
- It says what it rests on, such as tracking or dead reckoning, and how sure it is.

You do not call `addVisionMeasurement`, and there are no standard deviations to hand-tune per
tag count. The tuning is in `EstimatorConfig` and `VisionConfig`, and
[values.md](values.md) says what each number means. Cameras are PhotonVision only.

**You can have `SwerveDrivePoseEstimator` back.** The estimator sits behind an interface,
`Localizer`, and WPILib's is the second implementation. One line in the robot's block chooses it:

```java fragment
.localizer(WpilibLocalizer::new)
```

It gets the same wheel readings the default one gets, and camera measurements from the same
pipeline, with their standard deviations. The solving and filtering of images stays the runtime's
either way. Its images are solved from their tags alone, never with the gyro's heading, because
WPILib's estimator does not say how sure its heading is.
WPILib's estimator keeps no uncertainty and does not say what it did with a measurement, so the
pit page says "not reported" where it would show those, the scoring gate does not judge the
pose, and camera mounts are not checked.

A study in the template compares the two in simulation. In clean driving they are roughly equal.
The default one is better against bad camera data and wheel slip. WPILib's, behind a strong
team's vision filter, is better when the gyro reads wrong or a camera has been knocked.
[localizers.md](localizers.md) has the contract, both estimators, and how to write and test your
own.

### Paths and autos

There is no path file. A path is written in Java as keyframes, and the drivetrain plans a spline
through them when the move starts.

```java
static final Place PICKUP = Place.at("Pickup", new Pose2d(5, 2, Rotation2d.CCW_90DEG));

static void moves(Drive drive) {
  DriveMove toTower = drive.goTo(Rebuilt.TOWER);
  DriveMove leg = drive.follow(Keyframe.at(PICKUP), Keyframe.at(Rebuilt.TOWER));
}
```

A `Place` is written once for blue and resolved for the robot's alliance when the move starts.
That replaces path flipping. `drive.goTo` is a straight line with a turn. `drive.follow` is a
cubic spline through keyframes, limited by the model's own acceleration figures. A named keyframe
gives a gate, `leg.passed("name")`, which replaces event markers.

An auto is a name, a start place and a command factory:

```java fragment
autos.add("Grab one", Rebuilt.starts(robot.body()).center(), this::grabOne);
co.awaitAny(drive.goTo(PICKUP), intake());
co.await(pivot.goTo(PIVOT_IN));
```

There are no named commands. The auto calls the same Java methods your buttons call, so the
compiler checks every name.

The runtime owns the chooser. "Do nothing" is always first and is the default. The pit picks on
Pitwall while disabled, the choice is logged as an input, and the factory is called fresh when
autonomous starts. Before the match, a placement check compares the estimated pose with the
chosen auto's start and says how far off the robot is.

What you lose is real. There is no GUI, no obstacle avoidance, and no reading of PathPlanner or
Choreo files. [stand-ins.md](stand-ins.md) lists file reading as waited on.

### Tuning

Live tuning goes through Pitwall, the web page the robot serves on port 5820. Its Tune view edits
a joint's kP, kD, kS and current limits, and the drivetrain's gains, while the robot runs. Each
value has limits, and a change is checked against them and against the description's own checks
before it applies.

Nothing is saved on the robot. The Changes view lists each difference as the builder call that
commits it, such as `.withKs(0.25)`. You paste that into the description. When the field
connects, every value returns to what is committed.

The tunables are WPILib's own underneath, so a NetworkTables dashboard still sees them. A change
from one applies only while tuning is on.

A joint's feedforward, its profile and its named positions are not tunable live yet.

### SysId

There is no SysId routine in the runtime. [characterization.md](characterization.md) says so and
describes what is used instead.

A joint's kV, kA and kG start from the description. They are worked out from the motor, the
gearing and, if you gave one, the link's mass. You overwrite them with `withKv`, `withKa` and
`withKg` when you have measured values. kS has no seed, because the model has no friction.

For the loop gains, Pitwall's Experiment view runs step tests. Every driven joint gets one made
from its own named places or speeds, with no code. A run is recorded and scored for rise,
overshoot, settling and steady-state error, and runs are laid side by side.

Nothing stops you running SysId by hand and writing its numbers in. The gains accept SysId's
units.

Why there is no routine: the method is sound, and the cost is in its seams. You need a routine in
robot code, a log on the robot, a desktop tool, a vendor's high-rate logger for the short
mechanisms, and gains copied back by hand. The seeded feedforward and the step experiments cover
what most teams use SysId for on an arm or an elevator, and nothing is copied anywhere.

That judgement has not met a real robot. If step runs on one show the seeded feedforward is badly
off, a characterization routine built into the runtime is the next thing to write.

### Logging and replay

This is AdvantageKit. `Robot` extends `LoggedRobot`. Every device read goes through an IO
interface with logged inputs, and the estimator and the joints are written as pure logic over
those inputs. Replay works the usual way, with `-Preplay` and `AKIT_LOG_PATH`.

Two things are added. Requests from the Pitwall page are logged as inputs, so a replay reproduces
live tuning and the pit's auto choice. And architecture tests fail the build if a component or
library reads the clock, an unseeded random draw or shared state, since a replay would then
decide differently.

Each joint logs under `Joints/<label>/`. Read `State`, `Command`, `HeldBy`, `Goal`, `At` and
`Outcome`, in that order, when one will not move.

### Simulation

You do not write `simulationPeriodic` or a `DCMotorSim`. Each joint's plant is built from its
block. That uses the motor, the gearing, the hard stops and what it carries. The drivetrain's
plant uses the body's mass. Simulated cameras render the tags they would see from their mounts.

The physics is the runtime's own and it is simple. The drivetrain is kinematic and each joint is
a closed-form plant. It is not maple-sim, there is no rigid-body engine, and game pieces are not
bodies. A sensor reads a piece only when a test says it does. The battery holds 12 V.
[stand-ins.md](stand-ins.md#the-simulators-physics) lists each gap with a number.

What it adds is a second pose. The simulator knows where the robot really is, apart from where
the code believes it is, and its sensors err the way a worn robot's do. Faults can be made while
it runs. From the page you can unplug or knock a camera, freeze or reset the gyro, stale a
module's encoder, and shove the robot. In a test you can also silence, restart or jam a joint and
trip a breaker.

### Tests

The usual unit test builds a subsystem over a fake IO. Here the common test runs the robot. This
is one mechanism alone:

```java
JointsSimHarness sim = new JointsSimHarness(definition);
Move out = sim.joints.of(PIVOT).goTo(PIVOT_OUT);
sim.run(out, 1.5);
assertThat(out.arrived()).isTrue();
assertThat(sim.simulation.position(PIVOT)).isCloseTo(Math.toRadians(130), within(0.03));
```

`RobotSim` runs the whole program, with gamepads, the Driver Station, the drivetrain and
localization. The clock moves only when the test steps it. `AutoCheck` runs every declared auto
on both alliances. A monkey test plays a match of random input and faults against your robot and
checks invariants after every loop. [testing.md](testing.md) covers all of it.

## Where your habits will mislead you

- **A held `whileTrue` does not restart.** If the command ends by itself while the button is
  still down, nothing starts again. A binding starts on the edge.
- **Releasing a button does not retract anything.** The command is cancelled and each joint goes
  idle. A pivot holds where it is. Bind the retract if you want one.
- **A default command replaces the idle hold.** Give an arm a default command that does nothing
  useful and you have removed the thing holding it up.
- **Do not put safety checks in commands.** The interlock already holds beneath your command. A
  second copy in the command can only disagree with it.
- **A run-until has no deadline unless you give one.** `runUntil(output, gate)` runs until the
  gate is true. If the sensor is dead, that is forever. Add `.within(...)`.
- **A refusal stops the program.** A place outside its joint's range, or a speed past the motor's
  free speed, throws at startup, in the simulator too. Look for the message, not a warning.
- **A move that cannot be is different at startup and at run time.** Built as the robot starts,
  it throws. Built while the robot runs, it ends `REFUSED` and the robot carries on.
- **A literal angle in behaviour code is a smell.** Positions are named constants on their joint.
  `pivot.goTo(Degrees.of(130))` works, but it will not follow a practice robot's different "out".
- **A practice robot is not a branch.** There is no `if (isPractice)`. It is the competition
  robot's description with its differences:

  ```java fragment
  COMPETITION
      .named("practice")
      .with(
          PIVOT
              .motors(motors -> motors.on(TALON_FX.can(33).pdh(4, Amps.of(40))))
              .replaced(PIVOT_OUT, Degrees.of(128)),
  ```

- **Interlocks do not finish your command for you.** A joint held by one keeps its command, and
  the move times out if the hold outlasts its deadline. Read the outcome.
- **A forked command dies with its parent.** `co.fork` is not `schedule`.
- **Build commands in factories.** A v3 command that has run cannot be composed or scheduled
  again. Autos take a factory for this reason, and your command methods should return a fresh
  command each call.
- **Do not bind inside a running command with `whileTrue` on a gate that is already true.** It
  never starts. Loop on `waitUntil`, as the shot above does.

## Trade-offs, what is not built yet, and adding what it lacks

**Status.** It has not driven a real robot yet. The Phoenix drivetrain and joint code is tested
against CTRE's simulated devices. Several REV behaviours are marked in [stand-ins.md](stand-ins.md)
as measured only in REV's simulation and not confirmed on hardware. The Systemcore IMU's axes and
signs are also unconfirmed.

This is pre-release. The three parts below are different things. A trade-off is what a finished
choice costs. "Not built yet" is intended work. The last part is what you add yourself, because the
code is open.

### Trade-offs

**You work inside the runtime's loop.** The order of a loop, the drive's control law and the log
keys are the runtime's. You can read all of it, and it is your code to change. A change you keep to
yourself is a fork to maintain.

**You describe a mechanism, and the runtime drives it.** You do not write its subsystem, its
controller settings or its simulated plant. The price is that you learn the description's vocabulary
and debug from the joint's log.

**It is Commands v3 on WPILib 2027 alpha, on Systemcore, in Java 25.** It cannot run on a roboRIO
robot. The vendor libraries are alphas tied to one WPILib alpha, and PhotonLib is pinned to a
development build. v2 libraries cannot be added the usual way.

**Values and paths are code.** A number changes with a rebuild and a deploy, unless the pit page
tunes it live. An auto is a Java method.

**There is no SysId routine.** That is covered above, with what is used and when it would be built.

### Not built yet

- A joint moved by servos, a differential pair, and an arm moved through a linkage.
- Reading a path tool's files ([stand-ins.md](stand-ins.md#waiting-on)), and steering a path around
  obstacles.
- Choosing an auto from the driver station, and keeping the choice across a restart.
- Moving between configurations by a route.
- A bench bypass for interlocks.
- A string potentiometer on an elevator.
- Live tuning of a joint's feedforward, profile and named places.
- Failing a joint from the Pitwall page. A test can do it today.
- A practice robot that differs in motor count or drivetrain gearing.
- AdvantageKit's swerve templates for 2027. The drivetrain here stands in until they ship
  ([stand-ins.md](stand-ins.md)).

### Adding what it lacks

**A new kind of joint.** Today the runtime drives a motor, or several together, turning one shaft.
That covers a pivot or arm, an elevator or slide through a drum, a turret, and a roller or flywheel,
with position, velocity, voltage and duty-cycle control. A new kind is made the way those are. The
model's `Joint` and `MechanismControl` say what it is and how it is driven. A `JointIO` driver in
the runtime turns requests into what the hardware takes, and `RobotHardware` picks the driver from
the block's wiring. `JointMechanism` then gives it moves, gates and interlocks as it does for any
joint.

**A new driver.** A driver for another motor controller is a new `JointIO`. `JointIO`'s Javadoc
numbers what a driver must keep, J1 to J40, and the conformance suite checks each clause. You give
the suite a bench for your driver (`JointBench`), and it passes or writes down where it differs. A
swerve module's driver has the same in `ModuleIO`.

**One odd mechanism.** There is a lighter route for a single joint. Keep it in the description,
leave out its control, and drive it with a v3 `Mechanism` of your own, using WPILib's API or the
vendor's. [mechanisms.md](mechanisms.md#8-when-the-template-cant-drive-it) has a worked hood on
linear servos.

**A device not on the list below.** A sensor, an encoder or a gyro comes in the same way: its IO
interface is in the runtime, beside the drivers that exist.

### Supported today

- Motor controllers for joints: TalonFX, TalonFXS, SPARK MAX, SPARK Flex.
- Motors with presets: Kraken X60 and X44, Falcon 500, Minion, NEO, NEO Vortex, NEO 550.
- Absolute encoders: a CANcoder, an encoder on a SPARK's data port, or a duty-cycle or analog
  encoder on a Systemcore port. A SPARK does not read a CANcoder.
- Sensors: a switch on a Systemcore port, a CANdi input or a controller's limit input, and a
  CANrange.
- Power: REV's Power Distribution Hub.
- Gyro: a Pigeon 2, with Systemcore's own IMU as a backup.
- Drivetrain: swerve, on TalonFX modules with CANcoders or on SPARK modules.
- Vision: PhotonVision.

A Phoenix Pro license is not assumed. The guide's drivetrain runs without FOC, and its steering
reads the CANcoder as a remote sensor.

## Reading the code to check a claim

Open these first.

- `Robot.loop` and `RobotContainer.periodic` show the order of one loop as named steps.
- `JointMechanism` is a joint's commands, its idle, and where interlocks are applied. Its class
  comment says what a loop does.
- `Move` and `Outcome` are how a move ends and what it reports.
- `Gate` is the trigger with a name and a reason.
- `Drive`, `DriveMove` and `PathFollower` are the drivetrain's moves and the control law.
- `Localizer`, in the localization library (`localization/`), is the estimator's contract. `DefaultLocalizer` and
  `PoseEstimator` are the default estimator, `WpilibLocalizer` is WPILib's, and
  `LocalizationService` runs the loop around whichever the robot chose.
- `ExampleRobot` is a whole description. `Joint` and `MechanismControl`, in the model, are what a
  block can say.

The runtime's tests are grouped by what they test, and most claims on this page have one. Search
the tests for the behaviour before you trust a sentence here.
