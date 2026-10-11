# Start here: how a robot is written

This is the readable way in. It takes about twenty minutes, top to bottom, and it explains what
your team writes, what the program builds from it, and the calls you will use most. It is not a
reference, and it links to the guides that have the detail.

Every block of code below is copied from code that compiles. Most of it is the guide's robot,
`runtime/src/main/java/com/michaelgrundvig/frc/runtime/ExampleRobot.java`, and the examples beside
its tests in `runtime/src/test/java/com/michaelgrundvig/frc/runtime/examples/`. A test fails when
this page and that code drift apart.

## The idea

You describe your robot once, in Java. The description says what the robot is made of. It has a
body of a certain size and weight. It has a drivetrain. It has joints, which are the parts that
move, and each joint has motors, sensors, and a few named places it goes.

Everything else is built from that description. You do not write a class for your intake or your
elevator. The program reads the description and makes each joint's commands for you. The same
description drives the real motors on the robot and a simulated copy on your laptop. It also fills
in the web page your pit crew uses, and the checks that run before a match.

The reason to work this way is that a description can be checked before anything moves. Say your
code sends an arm to 140 degrees and the arm's hard stop is at 135. In most robot code you find
that out when the arm hits the stop. Here the program refuses to start, and tells you which number
is wrong and what its limit is.

Two things do the checking. The Java compiler catches a misspelled name, because every joint and
every place is a constant. The description catches the rest when it is built. That happens when
the program starts, in the simulator as well as on the robot. It also happens in a test that
builds every robot your team has, so a bad number fails the build on your laptop.

It will not catch everything. It knows nothing about a loose wire or a bad strategy. What it
catches are mistakes of arithmetic and bookkeeping, which are the ones that are easy to make at
midnight before an event.

## The four pieces

The code comes in four parts, and you write only the last one. The **model** is the vocabulary
for describing a robot and a field. It runs nothing, and it is where the checks live. The
**runtime** is the robot program every robot shares. It builds mechanisms from your description,
runs commands, drives, works out where the robot is on the field, and logs. **Pitwall** is a web
page the robot serves for your pit crew, and the simulator serves it too. The **template** is
your team's own code, in `src/main/java/frc/robot`. The rest of this page is about what goes
there.

## What you write

There are three kinds of file.

**`Main`** is where the program starts. It is one call that hands the runtime three things. This
is the guide's own `Main`, from the examples folder:

```java
RobotProgram.start(
    () -> ExampleRobot.COMPETITION, // which robot this is
    Rebuilt::game, // this season's field and venues
    Controls::new); // what it does
```

The first argument answers "which robot is this?". The second is the season's field, which the
model already describes. The third is your `Controls` class. The template's own `Main` passes
`Robots::current` first, a method in `Robots.java` that picks the robot the program is running
on. It also passes a fourth argument, a stamp that tells the log which build of the code ran.

**The robot's description** is a class of constants, one class for each design. The guide's is
`ExampleRobot`. The template also carries two real ones, `BetaRobot` and `DeltaRobot`, which are
one team's two 2026 robots described this way.

**`Controls`** is where behaviour goes. It holds your commands, your button bindings and your
autos. The template never edits it, so pulling a newer template does not collide with your work.

## Describing a robot

### The body and the drivetrain

The body is the frame, the bumpers, the height and the weight. This is the guide's robot:

```java fragment
Body.builder()
    .frameLength(Inches.of(27.5))
    .frameWidth(Inches.of(27.5))
    .height(Inches.of(30))
    // ... the bumpers ...
    .weight(WEIGHT)
```

Every value has a name, and its unit is written beside the number. You cannot swap the length and
the width by accident. The builder also asks for each required value in turn, so the code does
not compile if you leave one out.

The drivetrain is described the same way. You give the swerve modules, where they sit on the
frame, and how they are driven. A swerve module is one wheel that can both spin and steer.

```java
private static final DrivetrainGeometry DRIVETRAIN =
    DrivetrainGeometry.inset(MODULES, BODY, Inches.of(2.625)).control(DRIVING);
```

`MODULES` holds the motors, the gear ratios and the wheel size. `DRIVING` holds the limits and
the gains. One of those limits is the stator current, which is the current flowing through the
motor's windings. It sets how hard the motor can push, so limiting it keeps the wheels from
spinning on the carpet.

The description also lists the cameras and where each is mounted. The cameras look for AprilTags,
which are the square black and white markers placed around the field at known spots. Seeing one
tells the robot where it is. [values.md](values.md) explains every number in these blocks and how
to measure yours.

### A joint, piece by piece

A joint is one moving part. Everything about it goes in one block. Here is the pivot that swings
the guide's intake out of the frame, with its two named places above it:

```java
public static final Position PIVOT_IN = Position.degrees("in", 1.5);
public static final Position PIVOT_OUT = Position.degrees("out", 130);

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

Read it one call at a time.

**Where it is.** `Joint.pivot(...)` gives the joint its name, which is what the logs will call
it. It says the joint hangs from the chassis. It says where the axle is, in meters forward, left
and up from the robot's origin on the floor. The last argument is the axis it turns about.

**Its travel.** `.range(...)` is the hard stops, the angles where metal meets metal. `.rest(...)`
is where the joint sits when the robot is switched on. `.carries(...)` is the thing it moves,
drawn as a simple box.

**Its named places.** `.positions(...)` lists the places this joint goes. Your behaviour code
says `PIVOT_OUT`, never 130. If the out angle changes, you change one line here.

**Its motors.** `.motors(...)` reads left to right. There is one Kraken X60 motor. It turns the
pivot through a 25 to 1 gear reduction. It is run by a TalonFX motor controller at CAN ID 32, and
the CAN ID is the controller's address on the robot's data bus. That controller gets its power
from channel 4 of the PDH, the power distribution hub, through a 40 amp breaker.

**Its own sensors.** `.absoluteEncoder(...)` is a CANcoder on the pivot's axle. An absolute
encoder reports the true angle the moment it has power, so the pivot knows where it is at
startup. The offset is the correction for how its magnet happened to be mounted.

A joint can also have a limit switch at an end of its travel. The guide's pivot has none. This
line, from the examples' `HoodSimTest`, gives a hood one:

```java fragment
HOOD.limitSwitch(
    LimitSwitch.lower().at(Degrees.of(0)).on(SensorWiring.port(3).normallyClosed()));
```

It says the switch is at the lower end, trips at 0 degrees, and is wired to port 3. A joint with
no absolute encoder has to find where it is some other way, and running gently into a switch is
one of them. That is called homing.

**How it is driven.** `.control(...)` says how the program moves the joint. `position(30)` means
the joint is held at positions, and 30 is its kP. The kP is how hard the motor pushes for each
unit of error between where the joint is and where it should be. `withProfile(...)` caps the
speed and the acceleration of every move, so the pivot ramps up and ramps down. It does not lurch.

A second part of the control is the feedforward. That is the voltage you already know the motor
needs, sent before any error shows up. The program works out a first guess from the motors and
the gearing, and you write your own measured value when you have one.

### Interlocks and keep-outs

An interlock is a safety rule between parts. It holds no matter which command is running. The
guide's intake has a roller on the end of the pivot, and the roller must not spin while it is
still inside the frame. The roller's block says so:

```java fragment
// The roller runs only once the pivot is out of the frame.
.runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
```

No button and no auto can get around this. If a command runs the roller early, the roller waits,
and it starts by itself when the pivot passes 110 degrees.

A keep-out is the other kind of interlock. It closes off part of a joint's travel while something
is in the way. The guide's arm rides on an elevator, and low down it would swing into the bumper:

```java fragment
// The arm keeps out of the bumper's band while the elevator is too low to clear it.
.keepsOut(Range.degrees(-10, 30))
.when(ELEVATOR.atMost(Inches.of(32)))
```

While the elevator is at 32 inches or lower, the arm stops at the edge of that band. Once the
elevator rises past 32 inches, the arm carries on. You can tell both joints to move at once and
the rule sorts out the order.

### Configurations

A configuration is a name for several joints at once. This one is the intake deployed and
running:

```java
public static final Configuration INTAKE =
    Configuration.of("intake", PIVOT.at(PIVOT_OUT), ROLLER.at(INTAKING));
```

The description checks it against the interlocks. A configuration that ran the roller with the
pivot stowed would be refused.

The robot's description ends with three lists, from `ExampleRobot`. A joint, a sensor or a
configuration that is not listed is not part of the robot.

```java fragment
.joints(List.of(PIVOT, ROLLER, HOOD, ELEVATOR, ARM, FEEDER, INDEXER, SHOOTER))
.sensors(List.of(CLAW_BEAM, HOPPER))
.configurations(List.of(FEEDING, STOWED, INTAKE))
```

### A practice robot

Many teams build a second robot to practice with. It is never quite identical. A motor ends up on
another CAN ID, or an encoder's magnet sits at another angle. Here a practice robot is the
competition robot with its differences, and nothing else is written twice:

```java
public static final RobotConfig PRACTICE =
    COMPETITION
        .named("practice")
        .with(
            // The pivot's motor is at another CAN ID, and it swings out to 128°.
            PIVOT
                .motors(motors -> motors.on(TALON_FX.can(33).pdh(4, Amps.of(40))))
                .replaced(PIVOT_OUT, Degrees.of(128)),
            // The arm's magnet has its own offset, and the arm is lighter.
            ARM.absoluteEncoder(encoder -> encoder.offset(Rotations.of(0.47)))
                .retuned(control -> control.withKg(0.3))
                .rest(Degrees.of(-30))) // it stows at -30°
        .with(CLAW_BEAM.on(CANDI.can(36).s1().normallyOpen())) // a beam on another CANdi
        .with(CameraConfigBuilder.builder(BACK_LEFT).up(Inches.of(20.5)).build()); // lower
```

The same behaviour code runs on both. `PIVOT_OUT` means 130 degrees on one robot and 128 on the
other, and your code does not need to know which it is on. If the competition pivot gets new
gearing, the practice robot gets it too. [mechanisms.md](mechanisms.md) covers every kind of
joint, sensor and interlock.

## What you get back

### A joint's commands

In your behaviour code you look a joint up by its constant:

```java
JointMechanism pivot = joints.of(PIVOT);
JointMechanism roller = joints.of(ROLLER);
```

Each one comes with its commands already written. These are the three you will use most.

- `pivot.goTo(PIVOT_OUT)` moves the joint to one of its places and ends when it gets there.
- `roller.run(INTAKING)` runs the joint at one of its named outputs, a speed or a voltage, until
  something cancels it. That fits a button you hold down.
- `pivot.hold()` takes the joint and holds it at its goal until something cancels it.

A command that moves a joint is called a move, and a move can be adjusted. This line is from the
runtime's tests:

```java fragment
Move out = pivot.goTo(PIVOT_OUT).within(Seconds.of(5));
```

`within` sets a deadline. Every move already has one, worked out from how long the trip should
take, so a jammed joint cannot hang your auto forever. `within` replaces that deadline with yours.

A move always ends with an outcome. `ARRIVED` means it got there. `TIMED_OUT` means the deadline
passed. `CANCELED` means another command took over. `REFUSED` means it would not start. Your code
can ask what happened and react. This is from `ExampleCommands`. The `co.await` call runs a
command and waits for it, and the next section explains it:

```java fragment
var deploy = pivot.goTo(PIVOT_OUT);
co.await(deploy);
if (!deploy.arrived()) {
  co.await(pivot.goTo(PIVOT_IN)); // back off
}
```

### Gates

A gate is a yes or no question with a name. Each joint has a few. This one is from the runtime's
tests:

```java fragment
Gate atOut = pivot.isAt(PIVOT_OUT);
```

`shooter.isAtGoal()` is another. It asks whether the shooter is at the speed it was told to
reach. You can wait on a gate, bind a button to one, or combine several. When a gate is false it
logs why, so "why isn't it shooting?" has an answer you can read.

### Refusals and doubts

A refusal is the description saying no. The program does not start, in the simulator or on the
robot, and the message names the value and the limit. These three are copied from the tests that
check them.

One team's 2026 autos ran a feeder roller at 5000 rpm. Through that roller's gearing, its motor
tops out at 1500:

```text
RobotDefinition[beta].joints: full send's feeding, 5000 rpm, is 3.333 times its free speed,
1500 rpm: its motors can't turn it that fast; give it a speed under that
```

A configuration that runs the intake roller while the pivot is stowed breaks the interlock:

```text
RobotDefinition[beta].configurations: configuration intaking runs intake, but intake runs only
when pivot at least 110°, and pivot is at 1.5°, where it rests
```

A practice robot whose "out" was moved past the hard stop:

```text
pivot's out, 140°, is outside its hard stops, 0° to 135°: it can't get there
```

A doubt is softer. The value is allowed, but it looks wrong, so the robot starts and says so. The
same team's intake asks for exactly the fastest its motor can spin:

```text
intake's intaking, 2000 rpm, is 100% of its free speed, 2000 rpm: a velocity loop has little or
nothing left to hold it under load
```

Doubts go in the log under `Robot/Doubts`. On a real robot they also show on the Driver Station's
console. Read them after every change to the description.

## Behaviour in Controls

The runtime builds your `Controls` once, after the mechanisms exist. It hands the constructor
everything behaviour can reach. That is the command scheduler, the robot's description, the
joints, the drivetrain, the two gamepads and the list of autos. This is the guide's, from the
examples folder:

```java
Controls(
    Scheduler scheduler,
    RobotDefinition robot,
    Joints joints,
    Drive drive,
    RecordingGamepad driver,
    RecordingGamepad operator,
    Autos autos) {
  this.drive = drive;
  pivot = joints.of(PIVOT);
  roller = joints.of(ROLLER);

  operator.rightTrigger().whileTrue(intake());
  operator.faceRight().onTrue(pivot.goTo(PIVOT_IN));

  autos.add("Grab one", Rebuilt.starts(robot.body()).center(), this::grabOne);
}
```

The two `operator` lines are bindings. `whileTrue` runs a command for as long as the button is
held. `onTrue` starts it on the press and lets it finish. The gamepads are WPILib's, so anything
you have seen in a WPILib binding works here.

A command that uses more than one joint is a method that returns a `Command`. This is `intake()`:

```java
Command intake() {
  return Command.requiring(pivot, roller)
      .executing(
          co -> {
            co.fork(pivot.goTo(PIVOT_OUT));
            co.await(roller.run(INTAKING));
          })
      .named("Intake");
}
```

This is WPILib's Commands v3, and the body is a coroutine. A coroutine is a function that can
pause and pick up again on a later pass of the robot's loop. So you write the steps in order, top
to bottom, with plain `if` and `while`. `co.await(x)` runs `x` and waits for it. `co.fork(x)`
starts `x` and moves on without waiting. `co.waitUntil(gate)` pauses until a gate is true. Above,
the pivot starts swinging out and the roller command starts at once. The interlock keeps the
roller still until the pivot is clear.

`Command.requiring(pivot, roller)` says which joints this command owns while it runs. Each joint
is owned separately, so intaking does not cancel a command that is spinning up the shooter.

A sensor becomes a gate in one line. `CLAW_BEAM` is a beam-break sensor in the description:

```java fragment
Gate coralIn = joints.gate("coral in claw", CLAW_BEAM.tripped());
```

For a whole robot's behaviour in one file, read `src/main/java/frc/robot/BetaControls.java`. It
is one team's 2026 intake, shot, bindings and two autos.

## Driving

You do not bind the driving sticks. The runtime does it, with this line in its `RobotContainer`:

```java fragment
drive.setDefaultCommand(drive.driven());
```

`drive.driven()` is stick driving. The left stick moves the robot and the right stick turns it.
Forward on the stick is always away from your own driver station, whichever alliance you are on.
A default command is the one that runs when nothing else is using the drivetrain.

`drive.facing(...)` is stick driving with help. The left stick still moves the robot, and the
robot turns itself to hold a heading. A `Heading` says what to face. These bindings are from the
examples' `Aiming` class:

```java
driver.faceDown().whileTrue(drive.facing(Heading.downfield(Degrees.of(-20))));
driver.faceRight().whileTrue(drive.facing(Heading.downfield(Degrees.of(20))));
driver.faceLeft().whileTrue(drive.lockWheels());
```

The first two hold the robot 20 degrees either side of straight downfield while a button is held.
`drive.lockWheels()` turns the wheels into an X, which makes the robot hard to push.

A heading can also be a spot on the field, and `aimed()` is the gate that says the robot is
facing it. These two lines open a shot in `Aiming`:

```java
DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB));
Gate ready = Gate.all("ready to feed", aim.aimed(), shooter.isAtGoal());
```

The shot then feeds only while `ready` is true. The driver keeps driving with the left stick the
whole time.

The drivetrain can also drive itself. `drive.goTo(place)` drives to a spot and stops.
`drive.follow(...)` drives a smooth path through a list of keyframes. A keyframe is a spot the
path passes through, with the way the robot should face there. These are from `PlaceMoves`:

```java fragment
static final Place PICKUP = Place.at("Pickup", new Pose2d(5, 2, Rotation2d.CCW_90DEG));
DriveMove toTower = drive.goTo(Rebuilt.TOWER);
DriveMove leg = drive.follow(Keyframe.at(PICKUP), Keyframe.at(Rebuilt.TOWER));
```

A `Place` is written once, for the blue alliance. When the move starts, the robot finds the
matching spot for the alliance it is on. You write one path and it is right on both sides. These
moves have deadlines and outcomes, the same as a joint's. [driving.md](driving.md) has the rest.

## Autos

An auto is the routine the robot runs alone in the first seconds of a match. You declare one with
a name, the place the robot starts, and a method that makes its command. The guide's `Controls`
constructor did that with `autos.add("Grab one", ...)`. This is the method it names:

```java
Command grabOne() {
  return Command.noRequirements(
          co -> {
            co.awaitAny(drive.goTo(PICKUP), intake());
            co.await(pivot.goTo(PIVOT_IN));
          })
      .named("Grab one");
}
```

`co.awaitAny` runs both commands and moves on when the first one ends. Here the drive arrives at
the pickup, and that ends the intake too.

The runtime adds an auto called "Do nothing" by itself. It is first in the list and it is the
default, so there is always a safe choice. The pit chooses an auto on Pitwall before the match.

The start place gets checked. Before the match, the robot compares where it thinks it is with
the chosen auto's start, and the status says something like "Ready on Left: 2 cm, 1° off". If the
robot was set down on the wrong mark, you find out while you can still move it.
[autos.md](autos.md) covers choosing, logging and testing them.

## Running it with no robot

The simulator runs the same program on your laptop:

```bash
./gradlew run                # the guide's robot
./gradlew run -Probot=beta   # another robot from Robots.all(), by name
```

Nothing in your code changes for the simulator. The joints are simulated from the description,
with the motors, the gearing and the hard stops you wrote. The drivetrain moves with the weight
you gave it. The cameras see the tags they would see from where you mounted them.

Then open http://localhost:5820 in a browser. That is Pitwall. Its Pit view has the robot's
status, which includes whether it stands on its start and whether it knows where it is well
enough to score. The Tune view changes gains while the robot runs. The Experiment view runs a
test motion and scores it. The World view draws the field in 3D with the robot where the code
believes it is. In the simulator it also draws where the robot really is, so you can watch the
two drift apart.

The Sim view exists only in the simulator. It stands in for the Driver Station. You enable the
robot there and drive it from the keyboard or a gamepad.

The Sim view can also break things on purpose. You can unplug a camera, freeze the gyro, or shove
the robot sideways, all while it runs. The robot's code is never told. It has to notice from its
sensors, the way it would on the field. Failing a joint's motor or encoder is possible too, but
today only a test can do that. The page cannot yet.

Tests run the whole robot the same way, with no window and no waiting. A test builds the robot,
presses buttons or pushes sticks, steps the clock, and checks what happened. The clock moves only
when the test steps it, so the answer is the same on any laptop. `DrivingSimTest` in the examples
folder is a short one to read first.

One line runs every auto you declared, on both alliances. It fails if an auto throws, leaves the
field, or has a move refused. This is from `ExampleAutosSimTest`:

```java fragment
var results = AutoCheck.check(sim, ExampleRobot.COMPETITION, Controls::new);
```

## Where to go next

- [mechanisms.md](mechanisms.md) has every kind of joint, control, sensor and interlock.
- [driving.md](driving.md) has the drivetrain's moves, assisted driving and keyframes.
- [autos.md](autos.md) covers declaring, choosing and testing autos.
- [simulator.md](simulator.md) lists the faults and scenarios, and how far to trust the simulator.
- [testing.md](testing.md) shows how to write a test against the whole robot.
- [values.md](values.md) explains every number in a description and how to find yours.
- [calibration.md](calibration.md) covers measuring the real robot from the Pitwall page.
- [venues.md](venues.md) covers teaching the robot where a field's tags really are.
