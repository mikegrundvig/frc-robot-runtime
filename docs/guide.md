# Start here: how a robot is written

This is the readable way in. It takes about half an hour, top to bottom. It explains what your
team writes, what the program builds from it, how the program runs, and the calls you will use
most. It is not a reference, and it links to the guides that have the detail.

The whole page follows one robot. It is the guide's robot, `ExampleRobot`, and the class that
holds its behaviour, `ExampleControls`. Every block of code below is copied from code that
compiles, and nearly all of it is from those two files, which sit side by side:

- `runtime/src/main/java/com/michaelgrundvig/frc/runtime/ExampleRobot.java`
- `runtime/src/main/java/com/michaelgrundvig/frc/runtime/ExampleControls.java`

A test fails when this page and that code drift apart. Another test, `ControlsSimTest`, runs each
behaviour described here in the simulator. You can run it yourself too. The template runs this
robot with these controls until you write your own, so every button below works in the simulator.

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
catches are mistakes of arithmetic and bookkeeping, which are the ones people make at midnight
before an event.

## If you are new to FRC

Skip this if you have written robot code before.

The robot carries a small computer. On a 2027 robot it is called Systemcore, and older robots
have a roboRIO. It runs one Java program, and that program is what this page is about.

The drive team's laptop runs the Driver Station. It sends the robot the gamepads' sticks and
buttons, and it says whether the robot is enabled. A disabled robot runs its program but its
motors get no power. A match has two enabled periods. In autonomous the robot drives itself for
the first seconds. In teleop the drivers drive.

The program does not run once from top to bottom. It runs the same short pass again and again,
fifty times a second, so each pass has 20 milliseconds. A pass is called a loop.

Each motor is run by a motor controller, a box that feeds the motor power. The controllers and
many sensors hang on one shared cable called the CAN bus. Each device on it has a number, its
CAN ID.

WPILib is the library every FRC Java program is built on. Two of its words matter here. A command
is a job the robot does over many loops, such as "swing the intake out". A mechanism is a part of
the robot that only one command may use at a time. Older WPILib code calls that a subsystem.

## The pieces

The **model** is the vocabulary for describing a robot and a field. It runs nothing, and it is
where the checks live. The **runtime** is the robot program every robot shares. It builds
mechanisms from your description, runs commands, drives, works out where the robot is on the
field, and logs. **Pitwall** is a web page the robot serves for your pit crew, and the simulator
serves it too. You write none of those three.

## What you write

There are three kinds of file.

**`Main`** is where the program starts. It is one call that hands the runtime three things:

```java
RobotProgram.start(
    () -> ExampleRobot.COMPETITION, // which robot this is
    Rebuilt.GAME, // this season's field and venues
    ExampleControls::new); // what it does
```

The first argument answers "which robot is this?". The second is the season's field, which the
model already describes. The third is the class that holds the robot's behaviour. The guide's
robot has `ExampleControls`. Yours is called `Controls`.

**The robot's description** is a class of constants. The guide's is `ExampleRobot`. It has an
intake, which is a pivot with a roller on its end. It has an elevator with an arm on it. It has a
shooter, with a feeder and an indexer that push pieces into it, and a hood over it.

**`Controls`** is where behaviour goes. It holds your commands, your button bindings and your
autos. The guide's is `ExampleControls`, and this page calls it the guide's controls.

If you know WPILib: there is no `RobotContainer`, no `Constants` and no subsystem classes for you
to write. `Main`, a description and `Controls` are the whole of a team's code.

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

Every value is set by name, so you cannot swap the length and the width by accident. Most carry
their unit, like `Inches.of(27.5)`. A few are bare numbers whose unit you have to know. You will
meet two below. A `Translation3d` is a point in meters, and a gain is a plain number. The builder
also asks for each required value in turn, so the code does not compile if you leave one out.

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
unit of error between where the joint is and where it should be. Here it means 30 volts for each
radian of error. `withProfile(...)` caps the
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

The robot's description ends by listing its joints, its sensors and its configurations. One that
is not listed is not part of the robot.

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

In your controls you look a joint up by its constant, as in `joints.of(PIVOT)`. Each one comes with
its commands already written. These are the three you will use most.

- `pivot.goTo(PIVOT_OUT)` moves the joint to one of its places and ends when it gets there.
- `roller.run(INTAKING)` runs the joint at one of its named outputs, a speed or a voltage, until
  something cancels it. That fits a button you hold down.
- `pivot.hold()` takes the joint and holds it at its goal until something cancels it.

A command that moves a joint is called a move. A move to a place has a deadline, worked out from
how long the trip should take, so a jammed joint cannot hang your auto forever.

A move always ends with an outcome. `ARRIVED` means it got there. `TIMED_OUT` means the deadline
passed. `CANCELED` means another command took over. `REFUSED` means it would not start. Your code
can ask what happened and react. A worked example further down does that.

### Gates

A gate is a yes or no question with a name. Each joint has a few. `pivot.isAt(PIVOT_OUT)` asks
whether the pivot is at its "out" place. `shooter.isAtGoal()` asks whether the shooter is at the
speed its command told it to reach. You can wait on a gate, bind a button to one, or combine several.
When a gate is false it logs why, so "why isn't it shooting?" has an answer you can read.

### Refusals and doubts

A refusal is the description saying no. The program does not start, in the simulator or on the
robot, and the message names the value and the limit. These three are copied from the model's own
tests, which use a pivot and a roller with the same numbers as the guide's.

A roller's speed set to 5000 rpm, when its motor through its gearing tops out at 2000:

```text
RobotDefinition[test].joints: roller's intaking, 5000 rpm, is 2.5 times its free speed, 2000 rpm:
its motors can't turn it that fast; give it a speed under that
```

A configuration that runs the roller while the pivot is stowed, which breaks the interlock:

```text
configuration rolling runs roller, but roller runs only when pivot at least run, and pivot is at
1.5°, where it rests
```

A practice robot whose "out" was moved past the hard stop:

```text
pivot's out, 140°, is outside its hard stops, 0° to 135°: it can't get there
```

A doubt is softer. The value is allowed, but it looks wrong, so the robot starts and says so.
This one is about a roller asked for exactly the fastest its motor can spin:

```text
roller's intaking, 2000 rpm, is 100% of its free speed, 2000 rpm: a velocity loop has little or
nothing left to hold it under load
```

Doubts go in the log under `Robot/Doubts`. On a real robot they also show on the Driver Station's
console. Read them after every change to the description.

## How the program runs

You never write the loop, but everything you write runs inside it, so it helps to know its shape.

### One pass

Fifty times a second the runtime makes one pass. These are the four steps, from the runtime's
`Robot.loop`, with the timing code between them left out:

```java fragment
pitwall.periodic();
container.periodic();
runTheCommands(container, scheduler, loopHealth, afterAThrow);
container.afterCommands();
```

First, anything the pit asked for on the Pitwall page is taken in.

Second, the robot looks at the world. This is that step, from `RobotContainer.periodic`:

```java fragment
takeThePitsChoices(enabled);
Optional<Pose2d> placed = matchField.read(enabled);
stepTheSimulatedWorld(placed);
startOrEndTheAuto();
readTheSensors(enabled);
localize(placed, enabled);
judgeTheField();
```

The names say what happens. The pit's choice of auto is taken, and the match state is read from
the Driver Station. In the simulator, the pretend world moves forward one tick. If autonomous
began on this pass, the chosen auto's command is started. Then every sensor is read, the
drivetrain's first and then the joints'. Then the robot works out where it is on the field from
its wheels, its gyro and its cameras. Last, it judges that position. It checks whether the robot
is on its starting mark, for one.

Third, the command scheduler runs. The scheduler is the part of WPILib that keeps the list of
running commands. It checks every button binding and every gate. It starts the commands those
call for. Then it gives each running command its turn. One thing comes first. If the robot was
disabled since the last pass, every running command is cancelled before the scheduler runs.

Fourth, each joint is told what to do. A joint does what a command asked of it on this pass. If
no command asked anything, it does what it does when idle, which comes up below.

The order matters. Sensors are read before commands run, so a command always decides on fresh
readings. Joints are told after commands run, so what a command asks takes effect on the same
pass.

### Enabled, disabled, autonomous, teleop

The loop runs in every mode. What changes is what the robot may do.

**Disabled** means disabled. The loop still reads sensors and still works out where the robot
is. Gates are still worked out, so the pit can read them. But nothing moves. No joint is driven
and no wheel is driven, whatever asks. That holds in the simulator too. The pit's choice of auto
is taken only while the robot is disabled.

Disabling cancels every running command. It happens on the pass the robot is disabled, and you
write nothing for it. Each move says so in its outcome:

```text
pivot → out: CANCELED after 0.20 s, pivot at 38.49°: the robot was disabled
```

So nothing is left running to carry on later, and no deadline counts while the robot sits
disabled.

Buttons start nothing while the robot is disabled. A press made then is spent. It does not
start its command then, and it does not start it when the robot is enabled with the button still
down. A button held from before the disable does not start its command again either. Let go and
press again, and the command starts as usual. One button is different. The driver's back button
resets the heading whether or not the robot is enabled, because you want it before a match and
it moves nothing.

When the robot is enabled again, each joint holds wherever it is at that moment. It does not
jump back to an old target, and no old move carries on. A command starts only when something
asks for it afresh. The one thing that starts by itself is a default command, because the
scheduler starts a default command whenever its mechanism is free. So stick driving is back the
moment you enable, and if the driver is holding a stick over, the robot drives.
`DisablingSimTest` runs each of these cases.

**Autonomous** is enabled with nobody driving. On the pass it begins, the runtime starts the
chosen auto's command. On the pass it ends, the runtime cancels that command if it is still
running.

**Teleop** is enabled with the drivers driving. The gamepads' bindings start commands.

## Behaviour in Controls

The runtime builds your `Controls` once, after the mechanisms exist. It hands the constructor
everything behaviour can reach. That is the command scheduler, the robot's description, the
joints, the drivetrain, the two gamepads and the list of autos. This is the guide's constructor:

```java
public ExampleControls(
    Scheduler scheduler,
    RobotDefinition robot,
    Joints joints,
    Drive drive,
    RecordingGamepad driver,
    RecordingGamepad operator,
    Autos autos) {
  this.joints = joints;
  this.drive = drive;
  pivot = joints.of(PIVOT);
  roller = joints.of(ROLLER);
  shooter = joints.of(SHOOTER);
  feeder = joints.of(FEEDER);
  indexer = joints.of(INDEXER);
  pieceIn = joints.gate("piece in", CLAW_BEAM.tripped());

  operator.rightTrigger().whileTrue(intake());
  operator.faceRight().onTrue(pivot.goTo(PIVOT_IN));
  operator.faceDown().onTrue(deploy());
  driver.rightTrigger().whileTrue(shoot());

  autos.add("Grab one", Rebuilt.STARTS.center(), this::grabOne);
  autos.add("Grab and shoot", Rebuilt.STARTS.center(), this::grabAndShoot);
}
```

It does three things. It looks up the joints it will use. It makes one gate, `pieceIn`, from a
beam-break sensor in the description. A beam-break is a light beam across the intake that a game
piece blocks. Then it binds four buttons and declares two autos.

The four button lines are bindings. `whileTrue` starts a command when the button goes down and
cancels it when the button comes up. `onTrue` starts a command on the press and lets it finish.
The gamepads are WPILib's, so any binding you have seen in WPILib works here.

Each thing the robot does is a method that returns a `Command`. This is `intake()`:

```java
Command intake() {
  return Command.requiring(pivot, roller)
      .executing(
          co -> {
            co.fork(pivot.goTo(PIVOT_OUT));
            co.await(roller.runUntil(INTAKING, pieceIn).within(Seconds.of(8)));
          })
      .named("Intake");
}
```

This is WPILib's Commands v3, and the body is a coroutine. A coroutine is a function that can
pause part way through and pick up from the same line on a later pass of the loop. So you write
the steps in order, top to bottom, with plain `if` and `while`. `co.fork(x)` starts `x` and moves
on without waiting. `co.await(x)` starts `x` and pauses until `x` ends. `co.waitUntil(gate)`
pauses until a gate is true.

`Command.requiring(pivot, roller)` says which joints this command owns while it runs. Only one
command can own a joint at a time. Each joint is owned separately, so intaking does not cancel a
command that is spinning up the shooter.

If you know WPILib: a command here has no `initialize`, `execute` and `isFinished`. The coroutine
replaces all three. You also do not write a subsystem class for each mechanism. Every joint in the
description is already a v3 `Mechanism`, with its commands written.

## One button press, start to finish

Follow the operator's right trigger, which is bound to `intake()`. Each step below is what the
code does, and `ControlsSimTest` checks the result.

**The press is seen.** On some pass, the Driver Station's data arrives with the trigger down. In
the scheduler step, the binding sees that the trigger went from up to down. That change is
what starts a command, so the binding starts `Intake`. The log records which bindings fired on
each pass under `Controls/Fired`. Each is a fact, not a sentence: which binding it is, which
gamepad it is on, and how it starts. The robot's list of its bindings, `Controls/Bindings`, says
each one in words. This one reads "operator right trigger: while held: Intake".

**The command takes its joints.** `Intake` requires the pivot and the roller. If another command
owned either one, that command is cancelled now. Here nothing did, so both joints were idle.

**The coroutine runs until it pauses.** On the same pass, the scheduler gives `Intake` its first
turn. It runs `co.fork(pivot.goTo(PIVOT_OUT))`, which starts the pivot's move. It runs
`co.await(...)`, which starts the roller's move and then pauses. That is all `Intake` does on
this pass.

**The joints are told.** Still on the same pass, the last step of the loop tells each joint what
its command asked. The pivot's move plans a profile from where the pivot is now to 130 degrees,
and the pivot's motor controller is sent the first point on it.

**The interlock holds the roller.** The roller was asked for 2000 rpm. Before that reaches the
motor, the roller's interlock is checked. The pivot is still inside the frame, so the roller is
held still. The log says which interlock holds it under `Joints/roller/HeldBy`. As a sentence,
that interlock reads "roller runs only when pivot at least 110°". The command does not know and
does not need to.

**Every pass after that.** The scheduler gives `Intake` a turn. It is still waiting on the
roller's move, so it pauses again at once. Underneath, the pivot's move sends the next point of
its profile, and the motor controller's own fast loop keeps the motor on it.

**The pivot passes 110 degrees.** The interlock lets go and the roller spins up. Nothing in
`intake()` ordered that. You told two joints to move at once, and the rule in the description
produced the right order.

**The pivot arrives.** Its move ends, and the log gets one record under `Joints/pivot/Outcome`.
The record holds facts: the move arrived, it took 0.60 seconds, and where the pivot was. Read as a
sentence, it is "pivot → out: ARRIVED in 0.60 s". The pivot now holds at 130 degrees.

**It ends.** There are three ways, and the next section shows each. The beam can see a piece.
The operator can let go of the trigger. Another command can take one of the joints. In every case
the roller goes idle, and an idle roller stops. Unless another command took it, the pivot goes
idle too, and an idle pivot holds where it is. It stays out until something sends it in.

If you know WPILib: you do not need a default command to hold an arm up, because idle already
does that. A default command still works and replaces the idle. Interlocks are also not written
in commands. They sit below every command, in the description.

## Worked examples

All four are in the guide's `ExampleControls`, and each has a test in `ControlsSimTest`.

### A sensor ends the intake

Look again at the line in `intake()` that runs the roller:

```java fragment
co.await(roller.runUntil(INTAKING, pieceIn).within(Seconds.of(8)));
```

`runUntil` runs the roller until the gate `pieceIn` is true. Every pass, the sensor step reads the
beam, and the scheduler step works out the gate. On the pass the beam is blocked, the gate turns
true and the roller's move ends with `ARRIVED`. The `co.await` returns, the coroutine reaches its
last line, and `Intake` is over. The roller goes idle and stops with the piece in it. The log
reads "roller: intaking until piece in: ARRIVED in" and the time.

The trigger is still held down. Nothing starts again, because a binding starts its command on the
change from up to down, and that already happened.

`within(Seconds.of(8))` is the deadline, and it is there because you wrote it. A `runUntil` has
no deadline of its own, since nothing says how long a piece takes to show up. Without one it ends
when its gate turns true or when something cancels it, and never before. With this one, if no
piece shows up, or the sensor's wire has come off, the move ends with `TIMED_OUT` after eight
seconds, and the roller does not run for as long as the trigger is held.

### A second button interrupts

The operator's face right button is bound to `pivot.goTo(PIVOT_IN)`. Suppose it is pressed while
the trigger is still held and the intake is running.

On that pass the binding starts the move to "in". That move requires the pivot. `Intake` owns the
pivot, and only one command can, so the scheduler cancels `Intake`. The whole of it is cancelled,
and its roller move with it. The log reads "roller: intaking until piece in: CANCELED after" and
the time.

Now look at each joint. The roller has no command, so it is idle and it stops. The pivot has a
new command, so it swings in, and its line reads "pivot → in: ARRIVED in" and the time. One
button undid the other, and you wrote no code that said so. The ownership rule did it.

### A move that fails, and what to do then

`deploy()` swings the intake out without rolling. It checks how the move ended:

```java
Command deploy() {
  return Command.requiring(pivot)
      .executing(
          co -> {
            var out = pivot.goTo(PIVOT_OUT);
            co.await(out);
            if (!out.arrived()) {
              co.await(pivot.goTo(PIVOT_IN)); // back off
            }
          })
      .named("Deploy");
}
```

`co.await(out)` pauses until the move ends, however it ends. Normally that is `ARRIVED`, the `if`
is skipped, and the command is done.

Now say something is wedged in the intake. The pivot's motor pushes and the pivot does not move.
A move to a place has a deadline, so this one ends anyway. In the test, where the pivot is jammed at
rest, the log reads "pivot → out: TIMED_OUT after 1.62 s, pivot at 1.5°". The coroutine picks up
on the next line, sees that the move did not arrive, and sends the pivot back in.

Without the `if`, the command would end and the pivot would hold wherever it had got to. That is
safe, and it may not be what you want. The outcome lets you choose.

### An aimed shot

The driver's right trigger is bound to `shoot()`. It is the busiest command on the robot:

```java
Command shoot() {
  DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB));
  Gate ready = Gate.all("ready to feed", aim.aimed(), shooter.isAtGoal());
  return Command.requiring(drive, shooter, feeder, indexer)
      .executing(
          co -> {
            co.fork(shooter.follow(() -> RPM.of(3000)));
            co.fork(aim); // the sticks drive, and the robot holds its aim...
            while (true) {
              co.waitUntil(ready); // ...and the feed runs only while it is aimed
              co.await(joints.run(FEEDING).onlyWhile(ready));
            }
          })
      .named("Shoot");
}
```

The first two lines set things up. `aim` is a drive command that turns the robot to face the hub,
the season's goal, while the driver's left stick still moves the robot. `ready` is one gate made
of two. It is true when the robot is aimed and the shooter is at its speed.

On the pass the trigger goes down, `Shoot` takes the drivetrain, the shooter, the feeder and the
indexer. Stick driving was the drivetrain's default command, and it gives way. The coroutine
forks two things that never end by themselves. One spins the shooter toward 3000 rpm. The other
starts turning the robot. Then it reaches `co.waitUntil(ready)` and pauses.

It pauses even if the robot was already facing the hub. A moment ago the shooter was stopped with
nothing asking it for anything, and that counted as at its goal. Now `Shoot` has the shooter, so
`shooter.isAtGoal()` is about what `Shoot` asked for, which is 3000 rpm. A gate never hands you
the answer to the last command's question.

For the next passes nothing is fed. The shooter is still spinning up and the robot is still
turning. The shooter's part takes about half a second, which is 25 passes: its description gives
its flywheel a mass, and two motors need that long to bring it to 3000 rpm. The log says which
half is missing under `Gates/ready to feed/Blocking`, and names it in
`Gates/ready to feed/BlockingText`.

On the pass both are true, the wait ends. `joints.run(FEEDING)` runs the feeder and the indexer
together at their feeding speeds. `FEEDING` is a configuration from the description.
`onlyWhile(ready)` checks the gate on every pass.

Say the robot gets bumped and its aim swings off. `ready` turns false, the feed is cancelled on
that pass, and both rollers stop. The `while` loop goes round, and the coroutine waits for
`ready` again. The shooter keeps spinning the whole time. When the aim comes back, feeding
resumes.

When the driver lets go, `Shoot` is cancelled, and everything it forked is cancelled with it. The
feeder and the indexer stop. The shooter coasts down, because its description says it coasts. The
drivetrain goes back to stick driving.

## Driving

You do not bind the driving sticks. The runtime does it, with this line in its `RobotContainer`:

```java fragment
drive.setDefaultCommand(drive.driven());
```

`drive.driven()` is stick driving. The left stick moves the robot and the right stick turns it.
Forward on the stick is always away from your own driver station, whichever alliance you are on.
A default command is the one that runs when nothing else owns the drivetrain.

`drive.facing(...)` is stick driving with help. The left stick still moves the robot, and the
robot turns itself to hold a heading. You saw it face the hub in `shoot()`. `aim.aimed()` is its
gate. It is true when the robot faces the heading to within 2 degrees and has nearly stopped
turning.

A heading can also be an angle. These bindings are from `Aiming`, a small class in the same
examples folder:

```java
driver.faceDown().whileTrue(drive.facing(Heading.downfield(Degrees.of(-20))));
driver.faceRight().whileTrue(drive.facing(Heading.downfield(Degrees.of(20))));
driver.faceLeft().whileTrue(drive.lockWheels());
```

The first two hold the robot 20 degrees either side of straight downfield while a button is held.
`drive.lockWheels()` turns the wheels into an X, which makes the robot hard to push.

The drivetrain can also drive itself. `drive.goTo(place)` drives to a spot and stops. These are
the two spots the guide's controls use:

```java
private static final Place PICKUP = Place.at("Pickup", new Pose2d(2, 6, Rotation2d.ZERO));
private static final Place SHOT = Place.at("Shot", new Pose2d(3, 3, Rotation2d.ZERO));
```

A `Place` is written once, for the blue alliance, in meters from the field's corner. When a move
starts, the robot finds the matching spot for the alliance it is on. You write one path and it is
right on both sides.

`drive.follow(...)` drives one smooth path through several keyframes. A keyframe is a spot the
path passes through, with the way the robot should face there. A keyframe can have a name, and a
command can wait for the path to pass it. [driving.md](driving.md) shows how.

Drive moves have outcomes, the same as a joint's. A drive to a place or along a path has a
deadline too. `drive.runUntil(...)` has none unless you give it one, like a joint's `runUntil`.

## Autos

An auto is the routine the robot runs alone in the first seconds of a match. You declare one with
a name, the place the robot starts, and a method that makes its command. The guide's constructor
declared two. This is the longer one:

```java
Command grabAndShoot() {
  return Command.noRequirements(
          co -> {
            co.awaitAny(drive.goTo(PICKUP), intake()); // the first leg ends the intake
            co.fork(pivot.goTo(PIVOT_IN)); // the intake comes in on the way back
            co.await(drive.goTo(SHOT));
            co.await(shoot().withTimeout(Seconds.of(3)));
          })
      .named("Grab and shoot");
}
```

It reuses `intake()` and `shoot()`, the same methods the buttons use. An auto is ordinary
commands in a row.

Take it a line at a time. `co.awaitAny` starts both commands and moves on when the first one
ends, cancelling the other. So the robot drives to the pickup with the intake running, and
arriving ends the intake. Then the pivot starts in, and the coroutine does not wait for it. The
robot drives to the shot spot. Then `shoot()` runs for three seconds, which `withTimeout` cuts
off.

A test runs this auto in the simulator on each alliance. These are lines from its report for
blue, in the order the moves ended:

```text
pivot → out: ARRIVED in 0.60 s
drive → Pickup: ARRIVED in 2.14 s
roller: intaking until piece in: CANCELED after 2.14 s
pivot → in: ARRIVED in 0.60 s
drive → Shot: ARRIVED in 2.40 s
shooter follows a target: CANCELED after 3.00 s, shooter at 3008 rpm, its target 3000 rpm
feeder and indexer: feeding: CANCELED after 2.38 s
```

You can read the match from them. Each line starts with what moved. The roller ran for the whole
first leg and was cancelled when the drive arrived. The shooter chased its target for all 3
seconds, and the line says how fast it was turning at the end. The last line says the feeder and
the indexer ran for 2.38 of the 3 seconds. The first 0.62 seconds were the wait for `ready`, while
the shooter spun up and the robot turned to the hub. The turn was the slower of the two: the
shooter was at its speed after half a second.

The runtime adds an auto called "Do nothing" by itself. It is first in the list and it is the
default, so there is always a safe choice. The pit chooses an auto on Pitwall before the match.

The start place gets checked. Before the match, the robot compares where it thinks it is with
the chosen auto's start, and the status says something like "Ready on Left: 2 cm, 1° off". If the
robot was set down on the wrong mark, you find out while you can still move it.
[autos.md](autos.md) covers choosing, logging and testing them.

## Running it with no robot

The simulator runs the same program on your laptop:

```bash
./gradlew run
```

Nothing in your code changes for the simulator. The joints are simulated from the description,
with the motors, the gearing and the hard stops you wrote. The drivetrain moves with the weight
you gave it. The cameras see the tags they would see from where you mounted them, and not the
ones the hub or a tower stands in front of.

The field is solid. Drive into the hub, a trench's column, the tower or a wall, and the robot
stops there. Come at one at an angle and it slides along, if its motors are strong enough to
slide the treads. This robot's are not, at 30 amps: its wheels stall against the wall, and the
drive motors sit at their current limit. Raise the limit far enough and the wheels break loose
and spin instead. Then the encoders count travel that never happened, and the robot's idea of
where it is drives on without it. The World view shows both. The solid shape is where the code
thinks the robot is. The dashed one is where it really is.

Then open http://localhost:5820 in a browser. That is Pitwall. Its Pit view has the robot's
status, which includes whether it stands on its start and whether it knows where it is well
enough to score. The Tune view changes gains while the robot runs. The World view draws the field
in 3D with the robot where the code believes it is, and where it really is beside that. The Robot
view draws each joint as the description says it, with what it was told and what holds it. The Sim
view stands in for the Driver Station.

You don't have to live in the Sim view, though. The World and Robot views are where you watch, so
the controls come to you.

**Enable and Disable are in the bar at the top**, on every view. Press Enable. Press Esc to
disable, from anywhere on the page, even while you are typing in a field. Nothing else enables the
robot, and no key does. On a real robot those two buttons are not there at all. The Driver Station
enables a real robot.

**Press P for the Sim panel.** It opens beside whatever view you are on and stays open as you
change views. Press P again to close it. Drag its left edge to make it wider. It holds what you
reach for while watching, in this order:

- the mode and your alliance station
- driving, from the keyboard or a gamepad
- the auto to run
- the simulator's clock: pause it, or step one new loop
- what fails

**To drive**, switch on "Drive with the keyboard" in the panel. W and S go forward and back, A and
D go left and right, J and L (or the arrow keys) turn, and R resets the heading. The keys drive
from any view. The bar says "Keys drive the robot" while they do, with a Switch off beside it, so
you always know where your keys are going. Typing in a field still types.

**To run an auto**, pick it in the panel and press Choose. Then press "Set autonomous and enable".
That button does exactly what it says. Choosing an auto never enables the robot.

**To break something**, find it in the panel under "What fails". Type in the filter, like `camera`
or `slip`. The faults you used last are at the top. You can unplug a camera, freeze the gyro, or
shove the robot sideways, all while it runs. The robot's code is never told. It has to notice from
its sensors, the way it would on the field.

Or just click the thing. In the World view, click a camera, the gyro, a wheel, or the robot's
bumpers. The panel that opens says what you clicked and ends in its faults. Apply one there and
undo it there. Click the bumpers to shove, hit, tilt or place the robot. "Apply where I click"
places it at the next spot you click on the field.

Failing a joint's motor or encoder is possible too, but today only a test can do that. The page
cannot yet, so the Robot view's cards have no faults to offer.

The whole page's keys:

| Key | What it does |
|---|---|
| Esc | Disables the simulated robot if it is enabled, and stops anything running. Otherwise it drops the edit in a field, or takes the time strip back to live |
| P | Opens and closes the Sim panel |
| W A S D, J L, ← →, R | Drive, while keyboard driving is on |
| ← →, Shift+← →, Space, L | Move through time on the strip, while keyboard driving is off |
| / | Jumps to the filter in the Tune view |

Esc is the robot's first. If the robot is enabled, Esc disables it and does nothing else. Press it
again for whatever else you wanted.

### Reading what happened, in order

Live is too fast. An intake is out in half a second, and you can't see from the cards whether the
roller ran before the pivot was clear. Pitwall's Robot view remembers the last five minutes, loop
by loop, so you can go back and look.

Hold the operator's right trigger for two seconds, then let go. Along the bottom of the Robot view
is a strip of time, with a mark for everything that happened. Drag along it, or press Shift+Left
to go back one event at a time. The cards now show the robot as it was in that loop, and the rows
that changed in that loop are marked. Left and Right go one loop, which is 20 ms. Esc goes back to
live.

Beside the strip is the list of events, in order, with their times:

```text
1.78 s  operator right trigger: while held: Intake
1.78 s  pivot owned by Intake
1.78 s  pivot told out, 130°
1.78 s  roller owned by Intake
1.78 s  roller told intaking, 2000 rpm
1.78 s  roller held: roller runs only when pivot at least 110°
2.18 s  roller no longer held
2.40 s  pivot → out: ARRIVED in 0.62 s
```

Read it top to bottom. The roller was told to run in the same loop as the pivot, the interlock
held it, and it was let go 0.4 s later, as the pivot passed 110°. That is the order you wanted,
and the description enforced it.

A wrong order reads the same way. If your shot shows "feeder told feeding" above "shooter at goal
holds", it fed before the shooter was up. You found that without a print statement. Click a
joint's name on its card to see only that joint's events. A move that timed out or was refused
shows as a problem, in the list and on the strip.

Scrubbing only moves what the page shows. It never rewinds the robot or the simulator. "Step one
new loop", under Simulator clock in the Sim panel, is a different thing. It runs one new loop of a
paused simulator.

Tests run the whole robot the same way, with no window and no waiting. A test builds the robot,
presses buttons, steps the clock, and checks what happened. The clock moves only when the test
steps it, so the answer is the same on any laptop. `ControlsSimTest` is the one to read first. It
has a test for each worked example above, and each is about a dozen lines.

`AutoCheck.check(sim, ExampleRobot.COMPETITION, ExampleControls::new)` runs every auto you declared, on
both alliances. It fails if an auto throws, leaves the field, or has a move refused. The report
quoted above came from it, in `ExampleAutosSimTest`.

## In the template

Everything above is the guide's robot. The template is the project your team starts from, in
`src/main/java/frc/robot`, and it differs in a few ways. Each exists because a real team has
more than one robot.

**It runs the guide's robot until you write yours.** `./gradlew run` starts `ExampleRobot` with
`ExampleControls`, so the buttons on this page work. Enable teleop in Pitwall's Sim view and hold
the operator's right trigger, and the intake swings out and rolls. Hold the driver's right
trigger, and the robot turns to the hub and shoots.

**Your behaviour goes in `Controls`.** The template's own `Controls` starts empty. It binds
nothing. It is the class every robot you add gets, and the template never changes it. Write your
commands and bindings there, and copy from `ExampleControls` as you go.

**`Robots.java` lists every robot.** `Robots.all()` returns each robot your team has. A test
builds every one, which is how a bad number in any of them fails the build.

**`Main` picks a robot when the program starts.** The template passes `Robots::current` where the
guide's `Main` named one robot. `Robots.current()` decides which robot the program is running on.
It passes `Robots::controls` third, which makes the controls written for that robot. The guide's
two robots get `ExampleControls`, and a robot you add gets your `Controls`. It also passes a
fourth argument, a stamp that tells the log which build of the code ran.

**Two real robots are included.** `BetaRobot` and `DeltaRobot` are one team's two 2026 robots
described this way, with their behaviour in `BetaControls` and `DeltaControls`. Read them when
you want a whole robot that was built and played. The simulator runs either by name:

```bash
./gradlew run -Probot=beta
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
- [coming-from-wpilib.md](coming-from-wpilib.md) maps this stack onto command-based WPILib code.
