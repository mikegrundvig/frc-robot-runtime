# Mechanisms

Describe each mechanism once, in one place. The template turns every joint your motors drive into
a Commands v3 `Mechanism` with its commands already written: go to a place, run at a speed, hold,
home. You write what the robot *does* with them, in plain Commands v3.

You never write a mechanism class. There's no state machine to learn and no framework between you
and Commands v3, but there are new words: about 25 for a robot like this guide's, an intake, an
elevator with an arm and an interlock, and a practice robot. The ones the rest hang on are a
joint's **block**, its **positions** and **outputs**, a **configuration**, an **interlock** and its
**conditions**, and the joint's commands, its **moves** and its **gates**. Each is shown where it's
first used.

1. [An intake](#1-an-intake-a-pivot-and-a-roller): a pivot and a roller, whole.
2. [An elevator with an arm on it](#2-an-elevator-with-an-arm-on-it), and an interlock between them.
3. [Use it](#3-use-it): moves, outcomes, gates, and the buttons that start them.
4. [Why won't it move?](#4-why-wont-it-move)
5. [A practice robot](#5-a-practice-robot): the same code on a robot built a little differently.
6. [Reference](#6-reference): control, gravity, homing, wiring, absolute encoders, sensors,
   interlocks in depth, the PDH.
7. [What the definition refuses](#7-what-the-definition-refuses)
8. [When the template can't drive it](#8-when-the-template-cant-drive-it)

The guide's robot is compiled and tested, and it's the robot the template runs until you write
yours: `runtime/src/main/java/com/michaelgrundvig/frc/runtime/ExampleRobot.java` has it, `ExampleControls.java` beside it has
its buttons and autos, `runtime/src/test/java/com/michaelgrundvig/frc/runtime/examples/` has more
commands (`ExampleCommands`), and a test fails when the guide and the code drift apart.

## 1. An intake: a pivot and a roller

Everything about a mechanism goes in one block in `Robots.java`: where its joint sits, its range,
what it carries, the places it goes, its motors and their wiring, its own sensors, its interlocks,
and how the program drives it. This intake is a pivot that swings out, with a roller on its end.
The places come first, as constants, so your code names them, and a misspelled one won't compile.

```java
public static final Position PIVOT_IN = Position.degrees("in", 1.5);
public static final Position PIVOT_OUT = Position.degrees("out", 130);

public static final Joint.Revolute PIVOT =
    Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
        .range(Range.degrees(0, 135)).rest(PIVOT_IN)          // its hard stops, and where it rests
        .carries(Link.box("intake arm", new Translation3d(0, 0, 0.2),
            new Translation3d(0.05, 0.55, 0.36)))
        .positions(PIVOT_IN, PIVOT_OUT)
        .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(32).pdh(4, Amps.of(40))))
        .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))
        .control(MechanismControl.position(30)
            .withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700)));

public static final Output INTAKING = Output.rpm("intaking", 2000);
public static final Output OUTTAKING = Output.volts("outtaking", -4);

public static final Joint.Revolute ROLLER =
    Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35))   // it rides on the pivot's end
        .outputs(INTAKING, OUTTAKING)
        .motors(Motor.KRAKEN_X60.through(Gearing.of(3)).on(TALON_FX.can(31).pdh(5, Amps.of(40))))
        .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))   // only once the pivot is out of the frame
        .control(MechanismControl.velocity(0.02));

public static final Configuration INTAKE =
    Configuration.of("intake", PIVOT.at(PIVOT_OUT), ROLLER.at(INTAKING));
```

Then list them in the robot's definition: `.joints(List.of(PIVOT, ROLLER))` and
`.configurations(List.of(INTAKE))` ([the robot's definition](#the-robots-definition)).

**The pivot**, a call at a time:
- `Joint.pivot(...)` names it, as the logs will, and says what it hangs from (the chassis), where
  its axle is on that, in meters forward, left and up, and the axis it turns about.
  `Joint.PITCH_DOWN` swings a link that points forward at its zero down as its angle grows; this
  one stands up at its zero, the pose CAD draws it in, so it swings forward and out.
- `.range(...)` is its hard stops, and `.rest(PIVOT_IN)` where it rests as the robot starts. A
  joint with stops and no rest rests at its zero; if that's outside its stops, the robot refuses to
  start, saying to give `.rest(...)`.
- `.carries(...)` is what it moves, its **link**. `Link.box` draws it as a box for the extension
  checks, its mass unknown; `Link.weighed` adds a mass from CAD or a scale, which the simulator
  reads and which seeds kG and kA until you tune them ([Gravity](#gravity)); `Link.unseen` is
  neither. A mass is optional, a rough one does, and one left out is never guessed.
- `.positions(...)` are its named places. They're relative to the joint's parent, so an arm's
  places on an elevator mean the same at any height.
- `.motors(...)` is one Kraken X60 turning it through 25:1, on a TalonFX at CAN ID 32, fed by the
  PDH's channel 4 through a 40 A breaker. Several motors together name a controller each in
  `.on(...)`, the leader first. With no `.on(...)`, they run in the simulator alone ([Wiring and
  devices](#wiring-and-devices), [The PDH](#the-pdh)).
- `.absoluteEncoder(...)` is a CANcoder turning with it, so the pivot knows where it is the moment
  the robot starts ([Absolute encoders](#absolute-encoders)). A joint without one names how it
  finds where it is ([Homing](#homing)).
- `.control(...)` says how the program drives it: held at its positions with a kP of 30 V per
  radian of error, its moves profiled to at most 450°/s, speeding up at 1700°/s² ([Control and
  gains](#control-and-gains)).

**The roller**:
- `Joint.roller(...)` hangs it on `PIVOT`, so it rides on the pivot's end. A roller turns freely:
  it has no range and no positions.
- `.outputs(...)` are its named ways to run: a speed (`Output.rpm`, the roller's own turns a minute,
  not its motor's), a voltage (`Output.volts`) or a share of the battery (`Output.dutyCycle`). Every
  joint also has `Output.STOPPED`. Two joints that both have a "feeding" each get their own
  constant, `FEEDER_FEEDING` and `INDEXER_FEEDING`: a name belongs to its joint.
- `.runsOnlyWhen(...)` is its **interlock**: it runs only with the pivot at 110° or beyond, out of
  the frame. An interlock holds beneath every command, so no button or auto can spin the roller
  inside the robot ([more below](#2-an-elevator-with-an-arm-on-it)).
- `.control(MechanismControl.velocity(0.02))` holds its outputs' speeds with a kP of 0.02 V per
  radian per second of error.

**A configuration**, `INTAKE`, is several joints at their places or outputs at once: the pivot out
and the roller intaking. It names only the joints it moves, and a command moving to it requires
exactly those ([Use it](#3-use-it)).

**A block says each thing once.** What a joint has one of (its link, its positions, its outputs,
its motors, its absolute encoder, a switch at each end, its control) is refused said twice, saying
how to change it: a second `.absoluteEncoder(...)` meant as a pair is "pivot already has an
absolute encoder: to re-wire it, absoluteEncoder(encoder -> encoder.on(...)); to offset it,
absoluteEncoder(encoder -> encoder.offset(...)); a decoded pair isn't supported yet". What it has several of, servos and interlocks, adds up. Its hard stops and
where it rests are one verb each, `.range(...)` and `.rest(...)`, which change them when said
again, as a practice robot does ([A practice robot](#5-a-practice-robot)). A joint can't tell its
block from a finished joint, so in a block too, a second `.range(...)` or `.rest(...)` replaces the
first without a word: say each once.

**Other joints.** `Joint.elevator` starts one that slides straight up ([next](#2-an-elevator-with-an-arm-on-it)),
`Joint.slide` one that slides along any axis, and `Joint.roller("spindexer", Part.CHASSIS, at,
Vector3.UP)` a roller about another axis. For anything else, the builders
(`Joint.Revolute.builder()`, `Joint.Prismatic.builder()`) take every value by name: first what
every joint has, then what it may not (its range, where it rests, its places, what drives it), each
left out if it hasn't.

## 2. An elevator with an arm on it

An arm on an elevator is two joints: the elevator, sliding up the chassis, and the arm, riding on
its carriage. Low down, the arm would swing into the bumper, so while the elevator is too low to
clear it, the arm keeps out of that band: an interlock.

```java
public static final Position ELEVATOR_DOWN = Position.of("down", Inches.of(0));
public static final Position ELEVATOR_CLEAR = Position.of("clear", Inches.of(32));

public static final Joint.Prismatic ELEVATOR =
    Joint.elevator("elevator", Part.CHASSIS, new Translation3d(0, 0, 0.1))
        .range(Range.meters(0, 1.2)).rest(ELEVATOR_DOWN)
        .positions(ELEVATOR_DOWN, ELEVATOR_CLEAR)
        .motors(
            Motor.KRAKEN_X60
                .through(Gearing.of(5))
                .drum(Inches.of(0.75))
                .on(
                    TALON_FX.can(20).pdh(0, Amps.of(40)),
                    TALON_FX.can(21).clockwise().pdh(1, Amps.of(40))))
        // Its CANcoder turns once for twelve turns of the drum: less than once over its travel.
        .absoluteEncoder(
            AbsoluteEncoder.of().ratio(1.0 / 12).on(CANCODER.can(24).offset(Rotations.of(0.31))))
        .control(MechanismControl.position(30).withKg(0.35));

public static final Position ARM_STOWED = Position.degrees("stowed", -45);
public static final Position ARM_SCORING = Position.degrees("scoring", 120);

public static final Joint.Revolute ARM =
    Joint.pivot("arm", ELEVATOR, new Translation3d(0.1, 0, 0.3), Joint.PITCH_UP)
        .range(Range.degrees(-60, 150)).rest(ARM_STOWED)
        .positions(ARM_STOWED, ARM_SCORING)
        .motors(Motor.KRAKEN_X60.through(Gearing.of(60)).on(TALON_FX.can(22).pdh(2, Amps.of(40))))
        .absoluteEncoder(CANCODER.can(23).offset(Rotations.of(0.12)))
        // The arm keeps out of the bumper's band while the elevator is too low to clear it.
        .keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(Inches.of(32)))
        .control(MechanismControl.position(30).withKg(0.35, Degrees.of(-90)));
```

**The elevator**:
- `Joint.elevator(...)` slides straight up, from 0.1 m above the chassis's origin. Its range, its
  places and its speeds are lengths: `Range.meters(0, 1.2)`, `Position.of("down", Inches.of(0))`.
- `.motors(...)` is two Krakens through 5:1 onto a drum, a sprocket or a pulley (`.drum(...)`),
  whose pitch radius, where the chain or belt meets it, is 0.75 in. The leader is first. The follower is
  mounted facing the other way, so it turns clockwise, seen facing its shaft, as the elevator
  rises: `.clockwise()`.
- `.absoluteEncoder(...)`: the drum turns ten times over the elevator's 1.2 m, so a CANcoder on its
  shaft couldn't tell one turn from the next. Geared twelve to one down from the drum, it turns
  less than once over the whole travel, and the elevator knows where it is at startup. On the
  drum's shaft, the robot refuses it, saying the gearing that works ([Absolute
  encoders](#absolute-encoders)).
- `withKg(0.35)` holds it up: the volts its motors push with to hold it still, the same all along
  its travel ([Gravity](#gravity)). Its kP of 30 is volts per meter of error.

**The arm**:
- `Joint.pivot("arm", ELEVATOR, ...)` hangs it on the elevator, so it rides up and down with the
  carriage, and its places are the carriage's: "scoring" is 120° at any height.
- It stands straight up at its zero. `Joint.PITCH_UP` swings a link that points forward at its zero
  up as its angle grows, so this one, pointing up, swings back.
- `withKg(0.35, Degrees.of(-90))`: its weight pulls hardest a quarter turn from straight up, where
  it's level, and its kG says where that is ([Gravity](#gravity)).
- `.keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(Inches.of(32)))` is its interlock: while
  the elevator is at 32 in or below, the arm keeps out of -10° to 30°, where it would hit the
  bumper. Its target stops at the band's edge on the side it's on, and it goes on once the
  elevator rises past 32 in.

**Write an interlock in the block of the joint it restricts**, as the roller's and the arm's are,
one for each physical clash. A block can name only joints written above it, so a joint restricted
by one written after it, such as the elevator by the arm that rides on it, has its rule in the
robot's definition instead:

```java fragment
.interlocks(
    List.of(
        // The elevator keeps out of its lowest 0.3 m while the arm reaches down past 130°.
        Interlock.that(ELEVATOR).keepsOut(Range.meters(0, 0.3)).when(ARM.atLeast(Degrees.of(130)))))
```

That's the only time a rule goes there: one written in both places is refused.

**An edge is usually a clearance**, a number from CAD or measured where the parts just clear, not
a place the joint goes, so it's written as one: `PIVOT.atLeast(Degrees.of(110))` holds with the
pivot at 110° or beyond, `ELEVATOR.atMost(Inches.of(32))` with the elevator at 32 in or below, and
`between` inside a band. [Interlocks in depth](#interlocks-in-depth) has the rest: edges at named
places, speeds and sensors, and joints that restrict each other.

**Configurations** come after the blocks, since they name joints and their places:

```java
static final Configuration STOWED =
    Configuration.of(
        "stowed", PIVOT.at(PIVOT_IN), ELEVATOR.at(ELEVATOR_DOWN), ARM.at(ARM_STOWED));
```

### The robot's definition

The robot's definition in `Robots.java` lists its joints, each after the one it rides on, its
sensors, its configurations and any interlocks of its own, and the PDH that feeds its motors. The
guide's robot also has a hood, a shooter, the feeder and indexer that feed it, and two sensors
([Limit switches and sensors](#limit-switches-and-sensors)); `ExampleRobot` has them all:

```java fragment
.pdh(Pdh.can(1))
.joints(List.of(PIVOT, ROLLER, HOOD, ELEVATOR, ARM, FEEDER, INDEXER, SHOOTER))
.sensors(List.of(CLAW_BEAM, HOPPER))
.configurations(List.of(FEEDING, STOWED, INTAKE))
```

Get something wrong, a place outside its joint's range, a speed its motors can't reach, a
configuration that breaks an interlock, and the robot won't start, saying what to change. The test
that builds every robot fails too, so it's caught before a match: [What the definition
refuses](#7-what-the-definition-refuses) lists every check.

## 3. Use it

### Moves

Look a joint up by its constant, and use its commands. Every one is ready to bind or await:

```java
JointMechanism pivot = joints.of(PIVOT);
JointMechanism roller = joints.of(ROLLER);
```

| Command | What it does | Ends |
|---|---|---|
| `pivot.goTo(PIVOT_OUT)` | Moves to one of its positions, profiled from where it is | When it's there (inside its band for 40 ms), or at its deadline |
| `pivot.goTo(PIVOT_OUT).atSpeed(Percent.of(50))` | The same at half its cruise speed | As above, with a longer deadline |
| `pivot.goTo(Degrees.of(118))` | Moves to a computed position; refused outside its limits. A turret takes the nearest turn of it inside them (below) | As above |
| `pivot.goTo(PIVOT_OUT).within(Seconds.of(1))` | The same with a tighter deadline | As above |
| `hood.follow(() -> Degrees.of(angleFor(distance())))` | Chases a target that changes every loop, a position or a speed | Never: cancel it |
| `pivot.hold()` | Owns the joint and holds its goal | Never |
| `roller.run(INTAKING)` | Runs at one of its outputs | Never: for "while held" |
| `roller.runUntil(INTAKING, hasPiece)` | Runs until a condition holds | When it holds. It has no deadline of its own |
| `roller.runUntil(INTAKING, hasPiece).within(Seconds.of(8))` | The same with a deadline | When it holds, or `TIMED_OUT` after 8 s |
| `pivot.home()` | Homes it again by its recipe | When it's homed |
| `joints.goTo(STOWED)` | Moves a configuration's joints together; requires exactly those joints | When every one is there |
| `joints.run(FEEDING)` | Holds a configuration, outputs included | Never |

A move to a place has a **default deadline** from the model: twice its profile's time from where
it starts, plus half a second. A move that can't get there doesn't hang your auto: it times out,
and says where it was. A `runUntil` has no default, because nothing says how long its condition
takes to hold: it runs until the condition holds or it is canceled, so a held intake runs for as
long as it's held. Give it a deadline with `.within(...)` wherever running on would be wrong, as
in an auto, where a lost sensor would otherwise hold the roller until autonomous ends.

**When nothing drives it, a joint idles:**
- **A joint held at positions holds its goal:** where its last move arrived, or, if that move was
  canceled or timed out, where it came to a stop. It never sags, and never snaps back to an old
  target.
- **A roller stops.**
- **A default command replaces the idle**, as in any v3 code:
  `hood.setDefaultCommand(hood.goTo(HOOD_DOWN))`.

**Interlocks hold beneath every command.** No command, button or auto can skip one. While
`runsOnlyWhen` doesn't hold, the joint stays where it is, or a roller stops. While `keepsOut` is
closed, the joint's target stops at the region's edge on the side it's on; run at an output toward
it, the joint is held at the edge, its profile braking it there from the loop before it couldn't
stop short of it, for as long as the output pushes it that way. The command keeps running and
carries on once the interlock lets go, so moving joints together is safe without ordering them
yourself. If the joint an interlock reads hasn't homed, isn't answering, or its reading didn't come
this loop (a SPARK's frame missed), the interlock assumes the worst and holds.

**A law of your own**, such as a push held to a current, is a command that sets the joint's
voltage every loop. The joint goes back to its idle the loop you stop:

```java
roller.run(co -> { while (true) { roller.setVoltage(pushVolts()); co.yield(); } }).named("Push");
```

**On a turret**, a joint whose range spans a whole turn or more (-200° to 200°, say), an angle
your code works out is a heading: the turret can face it at more than one position. `goTo` and
`follow` take whichever turn of it, the angle plus or minus whole turns, lies inside its limits
(its soft limits, else its range) nearest where the turret is now. At 170°, asked for -170°, it
goes on to 190°, not back round; asked for a heading it can reach only the long way, it goes the
long way. So `turret.follow(() -> bearing.minus(heading))` needs no wrapping of your own. A named
position goes exactly where it's named: a team that named -180° meant -180°. Soft limits narrower
than a turn leave some headings with no turn inside: a move there is refused, and a chase holds at
the limit nearer the heading. A joint narrower than a turn, an arm or a hood, takes an angle as it
is: one off by whole turns is a slip (degrees given as radians), and is refused. A joint with no
range at all turns freely and goes the short way round.

### Outcomes

**Every move ends with an outcome**, and a reason that's already logged:

| Outcome | Means |
|---|---|
| `ARRIVED` | It got there, or its condition held |
| `TIMED_OUT` | Its deadline passed. The reason says where the joint was, and what interlock held it |
| `CANCELED` | Another command took the joint, its button was let go, or its `onlyWhile` gate fell |
| `REFUSED` | It wouldn't start: a target outside the limits, a place not the joint's own or an option that can't be, a joint that isn't answering or failed to home, or its `onlyIf` gate false |

Each reason starts with what moved: one joint (`"pivot → out: ..."`, `"roller: intaking: ..."`), or
a configuration's joints (`"feeder and indexer: feeding: CANCELED after 2.38 s"`). A move that timed
out or was canceled says where each joint it positioned was. A `follow` says it in the quantity it
was chasing, with its target:
`"shooter follows a target: CANCELED after 3.00 s, shooter at 3008 rpm, its target 3000 rpm"`.

A configuration's move also says when each joint got there, so the slow one is plain:
`"pivot, elevator and arm → stowed: ARRIVED in 0.62 s (pivot 0.58 s, elevator 0.62 s, arm 0.41 s)"`, and
`move.arrivals()` gives the same as numbers.

Recovery is an `if`:

```java
Command.requiring(pivot).executing(co -> {
  var deploy = pivot.goTo(PIVOT_OUT);
  co.await(deploy);
  if (!deploy.arrived()) {
    co.await(pivot.goTo(PIVOT_IN));    // back off
  }
}).named("Deploy or back off");
```

**A mistake stops the robot starting, not a match.** A move that can't be, built as the robot
starts (in a binding, or an auto built with them), throws, so the robot won't start and says why,
and the test that builds every robot fails too:
- a place the joint doesn't have, another joint's or one left out of its `.positions(...)`:
  `pivot.goTo(ELEVATOR_CLEAR)`;
- one of its names at a value it has never had: with the feeder's "feeding" at 1000 rpm and the
  indexer's at 1500, `feeder.run(INDEXER_FEEDING)` is "feeder's feeding runs at 1000 rpm, and this
  feeding, at 1500 rpm, isn't one it has had: another joint's or another robot's; give feeder's
  own";
- an option that can't be: `.atSpeed(Percent.of(150))`, past all of its cruise; `.atSpeed(...)` on
  a move that isn't to a place; `.within(...)` on one that never ends on its own (`run`, `hold`,
  `follow`: use `withTimeout`); `.within(...)` with a deadline that isn't a time above zero (none,
  a negative one, one without end, or one that isn't a number); an angle for a joint that slides,
  or a distance for one that turns.

Built once the robot runs, in a command or anywhere else, the same move ends `REFUSED` as it
starts, with the same reason: a robot on the field is never stopped by one. A gate that can't be
holds false, its `Blocking` saying why (`pivot.isAt(ELEVATOR_CLEAR)`,
`joints.gate(PIVOT.atLeast(ELEVATOR_CLEAR))`, a sensor the robot hasn't), and `joints.of(...)`
given a joint with no mechanism gives a stand-in, `FAULTED`, whose every move is refused and every
gate false. An interlock or a configuration on another joint's place is refused as the definition
is built.

### Gates: wait for things, and know why not

A `Gate` is a v3 trigger with a name, and it logs why it's false. Bind it, await it, or use it to
guard a move. Every joint has gates already:

| Gate | True when |
|---|---|
| `pivot.isAt(PIVOT_OUT)` | It's within its band of one of its positions, or runs at one of its outputs |
| `shooter.isAtGoal()` | It's where it's asked to be: at its position, or within its band of its speed |
| `pivot.isReady()` | It knows where it is, and its motor controllers answer |
| `indexer.isStalled()` | It's run at an output but has moved slower than a tenth of it for a quarter second |
| `joints.isAt(STOWED)` | Every joint of a configuration is at its place |

Make your own from anything:

```java
Gate aimed = Gate.within(scheduler, "aimed", () -> headingErrorDegrees(), 2.0, 4.0);  // in at 2°, out past 4°
Gate readyToShoot = Gate.all("ready to shoot", shooter.isAtGoal(), aimed)
    .debounce(Milliseconds.of(80), Debouncer.DebounceType.FALLING);     // rides out a flywheel's dip
```

Make gates where you build your commands, not inside one that runs over and over.

**Anything the model says is a gate.** A condition you'd write in an interlock, on where a joint
is, how fast one runs or what a sensor reads, becomes a gate with `joints.gate(...)`, named as it
reads ("pivot at least out") or for what it means to the robot:
`joints.gate(PIVOT.atLeast(PIVOT_OUT))`, `joints.gate("intake out of the frame",
PIVOT.atLeast(Degrees.of(110)))`, `joints.gate(ELEVATOR.between(...))`,
`joints.gate(SHOOTER.upToSpeed(RPM.of(3000)))`. A place it names is where the robot running it
puts it, as a move's is. One on a joint that doesn't know where it is (homing, faulted, silent) is
false and says so ("pivot doesn't know where it is"). The robot reads its switches and
rangefinders each loop, with the joints (logged under `Sensors/<label>`), and any condition on one
becomes a gate the same way:

```java fragment
Gate coralIn = joints.gate("coral in claw", CLAW_BEAM.tripped());
Gate hopperFull = joints.gate("hopper full", HOPPER.nearerThan(Inches.of(6)));

roller.runUntil(INTAKING, coralIn)      // ends ARRIVED the loop it trips, and has no deadline
co.waitUntil(coralIn);                  // holds a step
coralIn.onTrue(countPiece());           // once each time it trips
coralIn.debounce(Milliseconds.of(40), Debouncer.DebounceType.RISING)  // a beam that flickers
```

A gate on a sensor that's lost never opens, and a move waiting on it runs until its deadline, if
you gave it one (`.within(Seconds.of(3))`), and then says why: "roller: intaking
until coral in claw: TIMED_OUT after 3.00 s: coral in claw is false: claw beam lost" ([Limit
switches and sensors](#limit-switches-and-sensors)).

Guard a move with one:

```java fragment
pivot.goTo(PIVOT_OUT).onlyIf(intakeClear)          // checked once: REFUSED if false
joints.run(FEEDING).onlyWhile(readyToShoot)       // checked every loop: CANCELED when it falls
```

"Feed whenever ready, pause when not" is a loop:

```java
Command shoot() {
  return Command.requiring(shooter, feeder, indexer).executing(co -> {
    co.fork(shooter.follow(() -> RPM.of(speedFor(distance()))));
    while (true) {
      co.waitUntil(readyToShoot);
      co.await(joints.run(FEEDING).onlyWhile(readyToShoot));
    }
  }).named("Shoot");
}
```

One v3 trap: a binding made *inside* a command with `whileTrue` on a gate that's already true never
starts, because `whileTrue` only starts on a change. Use the loop above, or `retryWhileTrue`.

### Where behavior goes

Write what the robot does in one class, `Controls` (`src/main/java/frc/robot/Controls.java`): its
intake, its shot, its autos, and the drive team's buttons for them. It's yours: the template never
changes it, so the template's updates never collide with your code. The robot builds it once,
after its mechanisms, and hands it everything behavior reaches: the robot's definition, which says
which robot it runs on, the joints, with the sensors read beside them, the drivetrain, and two
gamepads, the driver's on port 0 and the operator's on port 1, and the list its autos are declared in.
Write each command as a method, and
bind it in the constructor. This is the guide's robot's, `ExampleControls`, which the template runs
on the guide's robot until you write yours:

```java fragment
public final class ExampleControls {
  private final JointMechanism pivot, roller, shooter, feeder, indexer;

  public ExampleControls(Scheduler scheduler, RobotDefinition robot, Joints joints, Drive drive,
      RecordingGamepad driver, RecordingGamepad operator, Autos autos) {
    pivot = joints.of(PIVOT);
    roller = joints.of(ROLLER);
    pieceIn = joints.gate("piece in", CLAW_BEAM.tripped());

    operator.rightTrigger().whileTrue(intake());
    operator.faceRight().onTrue(pivot.goTo(PIVOT_IN));

    autos.add("Grab one", Rebuilt.starts(robot.body()).center(), this::grabOne);
  }

  /** Deploy, and roll until a piece is in. The interlock holds the roller until 110°. */
  Command intake() {
    return Command.requiring(pivot, roller).executing(co -> {
      co.fork(pivot.goTo(PIVOT_OUT));
      co.await(roller.runUntil(INTAKING, pieceIn).within(Seconds.of(8)));
    }).named("Intake");
  }
}
```

The last line of the constructor declares an auto: its name, where the robot starts, and a method that
makes its command. The robot adds "Do nothing" itself, runs the one the pit chooses when autonomous
starts, and makes a fresh command each time ([autos](autos.md)).

Each driven joint is its own v3 `Mechanism`, so a command requires exactly the joints it moves.
Deploying while the shooter spins is fine, and one button can't cancel another's unrelated work.

The driver's sticks drive the robot, field-relative from the driver's end of the field, and the
driver's back button, with the robot facing straight downfield, resets its heading: the template
binds those itself, in `RobotContainer`, so they're there whatever `Controls` does.

**The robot knows its bindings.** The gamepads are WPILib's `CommandGamepad`, and bind as it does;
each binding is recorded as it's made, with its button and its command's name: "operator right
trigger: while held: Intake", and one made with a gate, "operator right bumper and ready to shoot:
while held: Shoot" (any other condition is "an unnamed condition": make it a gate to name it). The
robot logs the list (`Controls/Bindings`), and each loop the bindings a press or a release started
(`Controls/Fired`). Two commands bound to one button the same way start together, which is
sometimes meant: the robot says so as it starts, with its other doubts (`Robot/Doubts`). A binding
made inside a running command lives only while that command runs, and isn't in the list.

### Test it in the simulator

Every driven joint is simulated from its definition: the motors through the gearing, what the
joint carries, gravity on it (what its kG holds, [Gravity](#gravity)), and its hard stops.
Behavior can be tested without a robot.

- **One mechanism alone:** `JointsSimHarness` runs the joints, the scheduler and nothing else:

  ```java
  JointsSimHarness sim = new JointsSimHarness(definition);
  Move out = sim.joints.of(PIVOT).goTo(PIVOT_OUT);
  sim.run(out, 1.5);
  assertThat(out.arrived()).isTrue();
  assertThat(sim.simulation.position(PIVOT)).isCloseTo(Math.toRadians(130), within(0.03));
  ```

- **The whole robot:** `RobotSim` runs it, drivetrain and gamepads included ([testing](testing.md)).

`src/main/java/frc/robot/BetaRobot.java` has a real 2026 robot described this way, and
`BetaControls.java` its behavior, beside the template's robot and its `Controls`. It runs end to end
in the simulator, gamepads, interlock, gates and two autos included (`BetaRobotSimTest` and
`BetaAutosSimTest`, in `src/test/java/frc/robot/robots2026/`). `DeltaRobot.java` and `DeltaControls.java` are the same
team's second robot, of Beta's design on other hardware: a pivot driven open loop at 5 V and -3 V, a
two-motor roller and another shooter leader, tested the same way. The drivetrain has moves too: [driving](driving.md).

## 4. Why won't it move?

Every joint logs under `Joints/<label>/`, every sensor under `Sensors/<label>`, and every gate
under `Gates/<name>`. Open them in AdvantageScope or Pitwall, and read them in this order:

| Key | Tells you |
|---|---|
| `State` | `READY`, `UNHOMED`, `HOMING` toward what and for how long at most, or `FAULTED` and why: a homing that found no stop, a motor controller that didn't take its settings at startup, which nothing then drives, one that restarted on a joint assumed at rest ([Homing](#homing)), or an absolute encoder that isn't trusted, apart from its motor's count or its magnet bad ([Absolute encoders](#absolute-encoders)) |
| `Command` | The command driving it this loop, or `idle` |
| `HeldBy` | The interlock holding it, if one is, or the limit switch: "pivot lower limit tripped" |
| `Goal`, `GoalName` and `At` | Where it's headed, and where it is in its names: "between in and out" |
| `Outcome` | How its last move ended: "pivot → out: TIMED_OUT after 1.60 s, pivot at 40.3°, held: …" |
| `Rejected` | What a command asked of it this loop that isn't a number, a target divided by zero, say, and which command: no request, so it idled that loop, beneath its interlocks; the first raises an alert |
| `FollowersConnected`, `Faults`, `Restarts` | Its devices' health, ten times a second: each follower answering, each device's faults as its vendor numbers them (a TalonFX's or CANcoder's fault field, a SPARK's faults with its warnings above), and how many times each has restarted since the robot started; the leader first, then each follower, then its CANcoder |

"Why isn't it shooting?" is `Gates/ready to shoot/Blocking`: the terms that are false.

An alert says the rest, each naming what to check: a motor drawing nothing on its PDH channel
([The PDH](#the-pdh)), a limit switch that reads tripped nowhere near where it trips, or a sensor
that's lost ([Limit switches and sensors](#limit-switches-and-sensors)).

Code that throws doesn't stop the robot. A gate whose condition throws holds false, its `Blocking`
saying what it threw ("aimed threw: …"), with an alert while it does. On a robot, a command that
throws is ended, with the command that started it, and the robot carries on without them: an alert
says where, and the Driver Station's log has the stack (the robot's log too, `Errors/Commands`).
Commands that throw in 25 loops in a row, as a binding's condition that always throws does, run
nothing, so the program ends and restarts rather than drive on that way. A replay of the robot's
log does the same, so a match replays past a throw. In the simulator and in tests, the first throw
ends the program, so a bug fails its test.

## 5. A practice robot

**How it's driven isn't part of what the joint is, nor where its places are.** A joint re-tuned or
re-wired is the same joint, and so is one whose places are placed otherwise: a position or an
output is a name on its joint, and where it is, or how fast, the robot's. Everything that names
the joint or its places still finds them, the robot's own. So a practice robot of the same design
is the competition robot, `COMPETITION` in `Robots.java`, under its own name, with its
differences, and the same code runs on either, `pivot.goTo(PIVOT_OUT)` to each robot's own out:

```java
static final RobotConfig PRACTICE =
    COMPETITION.named("practice")
        .with(
            // The pivot's motor is at another CAN ID, and it swings out to 128°.
            PIVOT.motors(motors -> motors.on(TALON_FX.can(33).pdh(4, Amps.of(40))))
                .replaced(PIVOT_OUT, Degrees.of(128)),
            // The arm's magnet has its own offset, and the arm is lighter.
            ARM.absoluteEncoder(encoder -> encoder.offset(Rotations.of(0.47)))
                .retuned(control -> control.withKg(0.3))
                .rest(Degrees.of(-30))) // it stows at -30°
        .with(CLAW_BEAM.on(CANDI.can(36).s1().normallyOpen())) // a beam on another CANdi
        .with(CameraConfigBuilder.builder(BACK_LEFT).up(Inches.of(20.5)).build()); // lower
```

List it in `Robots.all()` beside the competition robot, `List.of(COMPETITION, PRACTICE)`, and the
test that builds every robot checks it.

A part is changed by naming it and saying what differs, in the words its block uses: `motors ->
motors.on(...)` is the same `.on(...)` the block wrote. Each change keeps the rest of what it
changes, so a CAD revision that regears the competition robot's pivot regears the practice robot's
too:

| Change | What it changes | What it keeps |
|---|---|---|
| `motors(motors -> motors.on(...))` | Its motors' controllers, the leader first | The motor, how many, and their gearing |
| `motors(motors -> motors.gearing(...))` | Its motors' gearing | Their controllers |
| `absoluteEncoder(encoder -> encoder.offset(...))` | Its absolute encoder's offset, its magnet's own | Its CANcoder, data port or Systemcore port and the rest of their wiring, so a CANcoder moved to another ID on the competition robot moves on the practice robot too |
| `absoluteEncoder(encoder -> encoder.on(...))` | The device its absolute encoder is read on | Its ratio, and, on another CANcoder or port of the same kind, how its loop reads it: one whose CANcoder only moved ID stays fused, unless its CANcoder says its own, `.remote()` |
| `limitSwitch(End.LOWER, limit -> limit.on(...))` | The input its switch at that end is on | Where it trips |
| `retuned(control -> ...)` | Its control, a wither at a time | The rest of its control |
| `replaced(PIVOT_OUT, Degrees.of(128))` | Where one of its positions is, or how one of its outputs runs: `replaced(OUTTAKING, Volts.of(-6))`, at a speed, a voltage or a share of the battery, `Percent.of(60)` | Its name, so every move, configuration and interlock that names it finds it there |
| `range(Range.degrees(0, 131))` | Its hard stops | Where it rests, which must be inside them |
| `rest(Degrees.of(-30))` | Where it rests as the robot starts | Its stops, which it must be inside |
| `CLAW_BEAM.on(...)` | A sensor of the robot's own: its wiring | The sensor |
| `DRIVETRAIN.encoderOffsets(...)` | The drivetrain's module offsets, front left, front right, back left, back right, as Tuner X gives them | The rest of its wiring |
| `DRIVETRAIN.wired(...)` | The drivetrain's wiring, whole | Its modules, which may differ only in what the simulator alone reads, Tuner X's inertias |
| `CameraConfigBuilder.builder(BACK_LEFT)...build()` | The robot's camera of its name, whole, as the back-left one above, half an inch lower | Nothing: give it all, through its builder |

A few things to know:
- **`replaced(...)` takes only a name the joint has**: its places' names are what its code says,
  the same on every robot. The joint keeps where each was, so the competition robot's constant is
  still its own; one of the name at a value it has never had is another joint's or another
  robot's, and refused.
- **A rest given as a position**, `rest(ARM_STOWED)`, is the joint's own, so after
  `replaced(ARM_STOWED, ...)` it rests at the practice robot's. A rest given in its block as a
  position stays where it was when that position is re-placed: move it too.
- **Every check runs on the practice robot's values**: a place outside its range, an interlock's
  edge or a soft limit past its stops, a place where a configuration breaks an interlock, the
  interlocks as the robot starts. They're checked together, on the finished joint, so its changes
  go in any order: a practice pivot whose stops and out both move, each past the other's, is
  `PIVOT.range(...).replaced(PIVOT_OUT, ...)` either way round.
- **Chain a joint's changes.** A joint given twice is refused, since the second would drop the
  first's change. `PIVOT.motors(Motor.KRAKEN_X60.through(...))` in a practice robot, which would
  restate its gearing and leave it behind when the competition robot's changes, is refused as a
  second say of its motors: "pivot already has its motors: to re-wire them, motors(motors ->
  motors.on(...)); to re-gear them, motors(motors -> motors.gearing(...))".

Some differences have no change yet: a drivetrain re-geared or its modules placed otherwise, a
joint on another count of motors, and a joint only one of the robots has. Write those through the
builders for now, `RobotDefinitionBuilder.builder(COMPETITION.definition()).joints(...)`, knowing a
later change to that part of the competition robot won't reach the practice robot.

## 6. Reference

### Control and gains

`.control(...)` says how the program drives the joint: gains, soft limits, motion profile, current
limits, neutral mode, bands and homing. `MechanismControl.position(kP)`, for a joint held at
positions, and `velocity(kP)`, for one that only runs at outputs, start from the defaults (the
feedforward from the definition, 40 A, braking), and each `with...` changes one value; a flywheel
adds `.coasting()`. A joint with motors needs one: the robot won't start without it, and says what
to give. Its kP is volts per its loop's error, so the robot refuses the other loop's factory:
"roller only runs at outputs (...), and its control is MechanismControl.position(...), for a joint
held at positions: write MechanismControl.velocity(kP), its kP in volts per radian per second of
error, or per turn per second, Volts.per(RotationsPerSecond).of(...)".

Each value is given with its unit: `withProfile(DegreesPerSecond.of(450),
DegreesPerSecondPerSecond.of(1700))`, `withSoftLimits(Range.degrees(5, 130))`,
`withBands(Degrees.of(1), Degrees.of(2))`. The gains may also be plain numbers in the joint's own
units, as SysId and Pitwall give them: a kP of 30 is 30 V per radian of error on a pivot, and per
meter on an elevator; one per turn, as Phoenix's slot holds it, is written with its unit,
`position(Volts.per(Rotation).of(188))`, as is SysId's kV per turn per second,
`withKv(Volts.per(RotationsPerSecond).of(0.12))`. A value with its unit knows whether it's an angle
or a length, so an elevator's soft limits in degrees are refused, saying how to write them.
[values.md](values.md) says what each value means and how to find yours.

- **Gains.** kV, kA and kG start from the definition, worked out from the motors, gearing and mass,
  until you measure them ([characterization](characterization.md)): `withKs`, `withKv`, `withKa`
  and `withKg` take what you measure. kP, kI and kD are yours. kG is the one that matters most for
  a joint held against gravity: [Gravity](#gravity).
- **Profile.** Moves are profiled on the robot, by WPILib's `TrapezoidProfile`: the cruise speed and
  acceleration of every move. With no profile, a joint moves at a quarter of its free speed, safe
  for a first enable.
- **Soft limits** keep the motors from driving the joint into its stops. The definition refuses a
  named position outside them, where it would be sent and never get.
- **Bands** (`MechanismControl.Bands`) say how near a target counts as there:
  - `enter` is how close it must come to arrive;
  - `stay` is how far it may then drift and still count.

  The defaults are 2° and 4°, 1 cm and 2 cm, or 3% and 6% of a speed:
  `withBands(Degrees.of(1), Degrees.of(2))` changes them, and a roller's, which are shares of the
  speed it's asked for, `withBands(Percent.of(3), Percent.of(6))`.
- **Live tuning.** Each joint's kP, kD and kS, and its current limits, can be tuned on Pitwall
  while the robot runs (`Joints/<label>/config`); its motor controller takes a change on the next
  loop. Nothing is saved: Pitwall lists what differs from `Robots.java`, each with the call that
  commits it in the joint's block: `.withKs(0.25)`, `.withSupplyLimit(Amps.of(50.0))`, and a kP as
  its factory, `MechanismControl.position(31.5)`. Its feedforward, profile and named positions
  aren't tunable live yet.

**What the motor controller is told** is worked out once from the block: the gains, tuned or
seeded, gravity, the limits, and one feedforward law (kS, kV, kA and kG) that every loop driving the
joint runs, the motor controller's, the robot's own and the simulator's. It's worked out when the
robot starts, in the simulator, and in the test that builds every robot, for the controller the
joint is wired to, or with no wiring, the one its motor runs on ([Wiring and
devices](#wiring-and-devices)). What the controller can't take is refused there: a gravity angle
past Phoenix's quarter turn, or a ratio or a gain past what Phoenix holds. What it takes but looks
wrong (a tuned kG far from what the joint's weight needs, a gravity offset at Phoenix's limit) is a
doubt said as the robot starts, in the log (`Robot/Doubts`) and on the Driver Station's console.

### Gravity

An arm or an elevator is held up by its **kG**: the volts its motors push with to hold it still,
which you tune on the robot and write in its control. It needs no mass. The guide's elevator is held
with 0.35 V, `withKg(0.35)`, and its arm with 0.35 V where its weight pulls hardest, at -90°,
`withKg(0.35, Degrees.of(-90))`.

- **An elevator's kG** holds it the same all along its travel.
- **An arm's kG** holds it where its weight pulls hardest, and less as it turns, along one cosine
  (Phoenix's `Arm_Cosine`, WPILib's `ArmFeedforward`). Where that is: the angle you give with its
  kG; else where its link's mass puts it, if it's weighed; else its zero, which WPILib takes as
  level. An arm zeroed level, raised as its position grows, needs no angle. One zeroed straight up,
  as the guide's is, is `Degrees.of(-90)`. To find it, let the arm hang still, its motors coasting,
  and read its angle: add 90° (hanging at -90°, it's 0°). Phoenix takes an angle within a quarter
  turn of the joint's zero; past that, the robot's configuration is refused, in the simulator too,
  saying how to turn the joint's axis around.
- **A counterbalanced joint**, a gas spring or surgical tubing holding it up, writes `withKg(0)`.
- **Without a kG written**, a joint's kG is seeded from its links' mass: none without one. A joint
  held at positions that gravity acts on (an arm whose axle isn't vertical, an elevator; not a
  roller or a turret) with no kG written is a note the robot says as it starts, something it
  doesn't know, on the robot and in the simulator: "not known: pivot has no kG: it holds against
  gravity by kP alone, so it sits below its targets until its kG is tuned and written,
  withKg(...), or written as 0 for a counterbalanced joint". A kG seeded from a mass is noted too,
  since only a tuned one is known to hold it.
- **In the simulator**, a joint's gravity is what its kG holds, through its motors, so a joint
  with a tuned kG holds there as it does on the robot, mass or none; without one, its links'
  weight. Its inertia comes from its links' mass, and from its motors' rotors alone without one.
  What the simulator does without a mass it says when it starts (`Robot/SimulatorNotes`), and
  never on the robot.

### Homing

Homing is how a joint finds where it is. A joint with an absolute encoder needs none. Any other
joint held at positions names a recipe in its control, `.withHoming(...)`: the definition refuses
it until you choose.

| Recipe | When to use it |
|---|---|
| none (empty) | The joint has an absolute encoder in its block. Nothing to do: it's told where it is at startup, from a CANcoder or a SPARK's data port by its motor controller, or from a Systemcore port by the robot ([Absolute encoders](#absolute-encoders)). |
| `Homing.toHardStop(End.LOWER)` | No absolute encoder. On the first enable, the joint is pushed gently (2 V) into that hard stop, and its position there becomes the stop's. Moves wait for it; if it finds no stop in time (below), the joint is faulted and its moves are refused. Check on the robot that 2 V really moves it. |
| `Homing.toSwitch(End.LOWER)` | No absolute encoder, and a limit switch at that end whose position its block knows. On the first enable, the joint is driven gently (2 V) toward it, and its position as the switch trips becomes the switch's; one starting on it backs off first. It's as close as half the joint's travel in a loop at that speed: slower is closer. Moves wait for it; if the switch doesn't trip in time (below), the joint is faulted, and says if the switch is lost. |
| `Homing.assumeAtRest()` | No absolute encoder, and you stow the joint by hand before every power-on. It trusts that. |

**A canceled homing homes again.** A `home()` canceled, by a preset on the same joint, say, leaves
the joint unhomed. The next move on it homes it again first, if the robot is enabled, and waits no
longer than its homing may take (below): a homing that faults ends the move `REFUSED`, saying why.
Disabled, even mid-homing, a joint homes on the next enable.

**A motor controller that restarts**, its breaker reset or its power connector loose, counts its
motor from 0 again, and its joint finds where it is again as it first did. One with an absolute
encoder is told by it again at once. One homed by a recipe is unhomed, and homes again on its next
move or enable, as after a canceled homing; one homing as it restarts starts over. One assumed at
rest is faulted, saying so, since only power-on puts it at rest: stow it, then restart the robot
code. A SPARK is told the pit's tuning again, which a restart loses; a TalonFX keeps its own. Each
restart is counted in the log (`Restarts`, [Why won't it move?](#4-why-wont-it-move)).

**How long homing may take** comes from the joint: twice the time its motors, through its gearing,
take to cross its whole range at 2 V, and half a second more, as a move's deadline is. A tall
elevator geared slow gets the time it needs, and a quick pivot isn't left pushing for long on a
stop it never finds. A joint that turns freely, homed to its switch, gets a turn's worth. While it
homes, its `State` says the time ("HOMING: toward its lower stop, for at most 9.5 s"), and a fault
says it and where it came from: "elevator's homing found no stop in 9.5 s (its travel takes 4.5 s
at 2.0 V), pushing toward its lower one". A joint slower than its model, one pushed up against its
weight, say, takes a time of its own: `Homing.toHardStop(End.UPPER, Seconds.of(4))`.

### Wiring and devices

Each motor's controller is named where the motor is: its kind (`TALON_FX`, `SPARK_MAX`,
`SPARK_FLEX`, or `TALON_FXS`), its CAN ID, and, on another bus, the bus
(`TALON_FX.can(CANIVORE, 23)`). Add `.clockwise()` for a motor that turns clockwise, seen facing
its shaft, when the joint moves forward; a follower turned the other way from its leader is one of
these. The robot refuses a motor its controller can't drive, saying which can, and a joint's motors
on different kinds of controller or different buses. Two devices of one kind on one bus can't
share an ID, the drivetrain's included (a TalonFX and a SPARK may): the robot won't start, and
names both.

A joint with no wiring runs in the simulator on the controller its motor runs on, its control
checked as that controller would take it: a NEO's a SPARK MAX, a NEO Vortex's a SPARK Flex, a
Minion's a TalonFXS, any other's a TalonFX. On a robot it stays still, and an alert says so. A
drive that turns two joints at once, a differential's, is the robot's own (`.drives(...)` in the
definition), and the template doesn't drive it yet ([When the template can't drive
it](#8-when-the-template-cant-drive-it)).

**On REV's SPARK MAX and SPARK Flex**, a joint is driven the same way, from the same control: its
SPARK's settings are what a TalonFX's are, in a SPARK's terms, and only what a SPARK can't take is
refused. A few things differ:
- The SPARK's loop runs kP, and kI holding a position; the feedforward, gravity and kD are worked
  out on the robot each loop and sent with it as volts. Its kI is in REV's units, per millisecond of
  its loop, which isn't yet confirmed on a SPARK ([stand-ins](stand-ins.md)).
- A SPARK limits its motor's current (from the control's stator limit), not the battery's.
- An absolute encoder goes on the SPARK's data port
  (`.absoluteEncoder(DATA_PORT.offset(Rotations.of(0.82)))`), read once at startup, or the SPARK's
  loop closed on it ([Absolute encoders](#absolute-encoders)); a SPARK doesn't read a CANcoder: the
  robot's configuration refuses a joint on a SPARK with one, in the simulator too.
- Its settings are saved on the SPARK at startup, so a brownout doesn't lose them.

### Absolute encoders

An absolute encoder knows where the joint is the moment the robot starts, so a joint with one needs
no homing. It goes in the joint's block, read one of three ways, and the joint uses it one of two
ways:

- **Seeded once**, as each is unless its wiring says otherwise: the encoder tells the motor's own
  encoder where the joint is at startup, and the motor's encoder runs the joint's loop from then
  on. Backlash, or a belt that slips, between the motor and the joint then moves where the joint
  goes.
- **Closed on it**: the loop runs on the encoder itself, so the joint goes where the encoder says,
  whatever the motor's count does.

| Where it's plugged in | In the block | Seeded once | Closed on it |
|---|---|---|---|
| A CANcoder | `.absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))`, on its motors' bus | The robot reads it at startup and tells the joint's TalonFX | `.remote()`, `.fused()` or `.sync()` after its offset: the TalonFX's own loop, at 1 kHz |
| A SPARK's data port, such as REV's Through Bore | `.absoluteEncoder(DATA_PORT.offset(Rotations.of(0.82)))` | The SPARK, read at startup | `DATA_PORT.offset(...).closesLoop()`: the SPARK's own loop, at the controller's rate |
| A Systemcore port: a duty-cycle encoder, or an analog encoder or potentiometer | `.absoluteEncoder(PortEncoder.dutyCycle(1).offset(Rotations.of(0.82)))`, or `PortEncoder.analog(2, 10)` for a ten-turn pot | The robot, from its first good reading | `PortEncoder.dutyCycle(1).offset(...).closesLoop()`: the robot's own loop, each loop, at 50 Hz |

With no wiring (`.absoluteEncoder(AbsoluteEncoder.of())`), only the simulator reads it.

**What closing on it costs.** A motor controller's loop runs at 1 kHz; the robot's runs once a
loop, at 50 Hz, twenty times coarser: a kP a motor controller holds steady may make it buzz, so tune
it again, lower if it does. The motor's own encoder still gives the joint's speed, which is
smoother than an absolute encoder's. Only a joint held at positions, without a homing recipe and
with a range, closes its loop on its encoder: the robot refuses one that turns freely, since the
encoder's reading wraps within its travel.

**Closed on a SPARK's data-port encoder**, the SPARK runs its loop in the encoder's turns, and the
robot works out the rest from the same control, as for any joint on a SPARK; where the joint is,
logged and for every move, is the encoder's word. The same control's kP is per turn of the encoder,
so on an encoder on the axle of a pivot geared 25:1 the SPARK is sent 25 times what it is per rotor
turn. How the SPARK is told the rest, and what isn't confirmed on a SPARK yet, is in
[values.md](values.md#mechanismwiringdataportencoder).

**Closed on a Systemcore port encoder**, the robot works out the motor's voltage each loop from
where the encoder says the joint is: the control's kP and kD, with the feedforward a SPARK gets
(kS, kV, kA and kG), sent as a voltage. The profile, the interlocks, the holds and the outcomes are
a joint's as always, and where the joint is, for every one of them, is the encoder's word. Its
motor's count is still told where the joint is at startup and by `mechanism.home()`, and its motor
controller's soft limits stay on that count. A duty-cycle encoder that's lost faults the joint, as
a motor controller gone silent does: its moves are refused, those running end saying why ("pivot's
absolute encoder is lost, on port 1: ..."), and the interlocks that read it hold. It doesn't go
limp: it holds where it comes to a stop, by its motor's count, on its motor controller's own loop,
until the encoder's first reading makes it ready again. An analog encoder can't say it's lost: a
loose wire reads as somewhere, and a loop closed on it drives the joint there until it's apart from
its motor's count for a quarter second (below).

**Geared**: an encoder that isn't on the joint's axle is written in full, with its turns for each
of the joint's: `.absoluteEncoder(AbsoluteEncoder.of().ratio(0.536).on(CANCODER.can(34).offset(...)))`.
Where the joint is, is known within one of the encoder's turns; the robot says if that window
doesn't cover the joint's travel.

**On an elevator**, or any joint that slides, the encoder turns with the drum its motors' pitch
radius names, on the drum's shaft or geared to it, and its ratio is its turns for each of the
drum's, as the guide's elevator's is ([section 2](#2-an-elevator-with-an-arm-on-it)). One of its
turns must cover the whole travel, so it's usually geared down from the drum. It's read any of the
three ways, seeded once or closed on, as a pivot's is, and so needs no homing; its motor controller
counts the drum's turns ([values.md](values.md#mechanismwiringcancoder)). A potentiometer on a
Systemcore port turns with the drum too: a ten-turn one on the drum's shaft covers ten of its
turns, `PortEncoder.analog(2, 10)`. The robot refuses an encoder whose turn doesn't cover the
travel, saying the gearing that does; the guide's CANcoder on the drum's shaft: "elevator's
absolute encoder turns 10.03 times over its travel, 0 m to 1.2 m: one of its turns must cover the
travel, with a fortieth of it to spare at each end, to tell where elevator is. Gear it to at most
0.0947 of its turns for each turn of elevator's drum, or leave it out and give elevator's control a
homing recipe". One that turns more than once isn't read from a known start, nor after homing once
to pick its turn: homed, the motor's own count knows as much. The encoder reads 0 at the
elevator's zero, and must read within a turn of 0 all along its travel, so one on an elevator
measured from the floor, its travel far above its zero, is refused, saying so: put the zero at its
bottom stop, or gear the encoder slower.

**Offsets.** Each encoder's offset makes it read zero with the joint at zero, the pose its origin
gives in CAD, and its direction, said after the offset, makes its reading grow as the joint moves
forward. A team's code often holds both already: a CANcoder's `MagnetOffset` is its offset as it
is, and a `SensorDirection` of `Clockwise_Positive` is
`CANCODER.can(...).offset(...).clockwise()`. A SPARK's `zeroOffset` is the raw reading at zero,
before inverting, so it's the negative here, `DATA_PORT.offset(-zeroOffset)`, or
`DATA_PORT.offset(+zeroOffset).inverted()`; from it, the template tells the SPARK a `zeroOffset`
of its own, at the edge of the encoder's window. The same goes for WPILib's `DutyCycleEncoder`
expected zero.

**A Systemcore port encoder, on the robot.** Until its first reading, the joint waits, unhomed, and
moves on it wait. With none in 2 s it's faulted and says so; it's still seeded when a reading
comes. `mechanism.home()` seeds it again from the encoder, if the motor's count slipped. Its
reading is logged raw (`Joints/<joint>/Absolute`). A duty-cycle encoder says when it's unplugged;
an analog one can't.

**Backlash and slip show in the log.** Where an encoder on a Systemcore port or a SPARK's data port,
or a CANcoder a TalonFX is seeded from once, says the joint is, is logged each loop
(`Joints/<joint>/AbsoluteAt`), beside where the motor controller says (`Joints/<joint>`'s
position). A gap between them that follows the joint's direction of travel is backlash, and one
that grows or jumps is slip: a small one is a reason to close the loop on the encoder. Closed on a
SPARK's encoder, the joint's position is the encoder's.

**A slip the robot doesn't trust.** Past what backlash and the encoder's counts explain, 5° or 2 cm
unless its control says otherwise (`withEncoderGap(Degrees.of(8))`,
[values.md](values.md#mechanismcontrolencodergap)), with a loop's travel at its speed beside, for a
quarter second, the two disagree: one of them slipped, a magnet loose on its shaft, a belt that
skipped, an encoder frozen, and the robot can't tell which. An analog encoder can't say it's
unplugged, and a magnet turned on its shaft still reads, so this is how either shows. The joint is
faulted, saying so ("pivot's absolute encoder and its motor's count are 20° apart, more than 5°:
..."), its moves are refused, and the interlocks that read it hold. It holds where its motor says,
where its motor controller's loop runs on its motor's count, and is stopped otherwise, until the
two agree again. One whose encoder the robot reads is told where it is from the encoder again by
`mechanism.home()`; one whose controller reads it, by restarting the robot code. A loop closed on
an encoder runs a quarter second on a slipped or frozen one first: a fast joint may get where the
encoder wrongly sends it, or, frozen, to its stop. A CANcoder that says its magnet is too close or
too far (red) isn't trusted either: the joint isn't seeded from it, and is faulted until it says
the magnet is good.

### Limit switches and sensors

**A joint's limit switches** are in its block:
`.limitSwitch(LimitSwitch.lower().at(Degrees.of(0)).on(Direction.REVERSE.normallyClosed()))` on its
own controller's limit pin, or with a Systemcore port (`SensorWiring.port(1).normallyClosed()`) or
a CANdi's input (`CANDI.can(30).s1().normallyClosed()`) in place of the pin. Each is named for its joint and end, "pivot lower limit", and
`PIVOT.switchAt(End.LOWER)` is the switch, for a condition. One whose position isn't known
(`.limitSwitch(LimitSwitch.lower())` without an angle) can stop its joint but not home it.

**Every other switch and rangefinder** is a constant of its own, on its part, the chassis or a
joint's link (the arm a roller rides on, not the roller, which spins), carrying its wiring:

```java
public static final Sensor.Switch CLAW_BEAM =
    Sensor.Switch.beam("claw beam", PIVOT).on(CANDI.can(30).s1().normallyOpen());
public static final Sensor.Rangefinder HOPPER =
    Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))
        .on(CANRANGE.can(31));
```

A device on a CANivore names its bus first, as a motor's controller does:
`CANDI.can(CANIVORE, 30).s1()`.

Add them to the definition (`.sensors(...)`). A switch is tripped or clear, whatever its wiring;
its kind says what trips it: a `beam` break, or a `contact` switch nothing in the model closes (a
button, a cage sensor). A rangefinder reads a distance along its axis. Where on its part it sits is
optional: a team rarely knows a beam's exact points, and the robot notes at startup what isn't
placed. What one means is a condition over it, either way: `CLAW_BEAM.tripped()`,
`CLAW_BEAM.clear()`, a rangefinder's `nearerThan(Inches.of(6))`.

A switch's wiring is where it's connected (a Systemcore port, a CANdi's input, or a limit input on
a joint's motor controller: a SPARK's, a TalonFXS's or a Falcon's; a Kraken has none) and its
polarity, normally open or normally closed (`.normallyOpen()` or `.normallyClosed()` after the
input), which nothing above the wiring sees. Wire a limit switch normally closed: a cut wire then
reads tripped, and its joint stops. A sensor with no wiring runs in the simulator alone. The
definition refuses two sensors on one input, a controller's limit input where there's none, and a
joint's limit switch on another joint's controller, which would stop the wrong motor.

**The controller's own stop.** A joint's limit switch on its controller's limit input stops the
motor in the controller itself, with no code in the loop; so does one on a CANdi on a TalonFX's
bus, which the TalonFX reads over CAN as its limit (a Kraken has no limit inputs of its own). The
controller stops on nothing else: a beam on a SPARK's input is read, never stopped on, and an input
with nothing wired stops nothing. Neither resets the joint's position as it trips: homing is the
robot's. A switch on the controller's own input is read through the controller, with its joint, and
lost with it. The simulator's controller stops the same way, each millisecond, from the same
wiring, a loop before the robot's code sees the switch. And beneath every command and interlock, the robot's code stops a joint at any of its
tripped limit switches, wherever it's wired: a move past one holds where it tripped
(`HeldBy`: "pivot lower limit tripped"), an output toward it goes neutral, and leaving it is free.

**A doubted switch.** A limit switch is checked against where its homed joint is. Tripped nowhere
near where it trips, or clear well past it (a tenth of the joint's range either way, for a quarter
second), it's doubted, in the log (`Joints/<label>/SwitchDoubts`) and an alert, saying what to
check: "pivot lower limit reads tripped with pivot at 130°, nowhere near where it trips, 30°: a cut
wire, if it's normally closed, or a switch out of place". A doubted switch still stops its joint.

**A lost sensor** (its CAN device silent, or no wiring on a robot) holds no condition either way:
neither tripped nor clear. An alert names it, a gate on it never opens ([Gates](#gates-wait-for-things-and-know-why-not)),
and an interlock that reads it restricts: a joint kept out stays out, one that runs only when it
holds is held.

**In the simulator** a limit switch trips where its joint really is, and
`world.sensors().cut(switch, polarity)` cuts its wire. A test says what a beam or a rangefinder
sees (`world.sensors().trip(...)`, `see(...)`, and `silence(...)` for a lost one).

### Interlocks in depth

[Section 2](#2-an-elevator-with-an-arm-on-it) says where an interlock is written, and
[Moves](#moves) how it holds while the robot runs. This is the rest.

**Its condition.** An edge that really is one of the joint's places names it,
`PIVOT.atLeast(PIVOT_OUT)`, and follows it where a practice robot puts it; one that's a clearance
is a number, `PIVOT.atLeast(Degrees.of(110))`. Either means the same wherever the joint rests, so
moving its rest in a CAD revision leaves them alone. `SHOOTER.upToSpeed(RPM.of(3000))` holds a
joint until another is up to speed. `ARM.at(ARM_SCORING)` is a configuration's, and an interlock
refuses it: it holds only with the arm exactly there, which a measured position never is. A
sensor's conditions work as well (`.runsOnlyWhen(CLAW_BEAM.clear())`), and a lost sensor
restricts.

**What's checked before the robot runs.** Interlocks written in blocks and in the definition are
enforced and checked together. The definition checks interlocks on joints against each other and
the configurations; one on a sensor, or on a speed, it can only check is the robot's, since neither
is known until the robot runs.

**Two joints may restrict each other**, as the arm and the elevator do once the elevator's rule is
in the definition, so long as one can always move to free the other. With the elevator below 32 in
and the arm reaching down past 130°, each holds the other, but the arm, kept above the bumper's
band, can still swing up under 130° and let the elevator down, and the elevator can rise past 32 in
and free the arm. Rules that hold each joint where only the other could free it are a deadlock, and
the definition refuses them where the robot starts or a configuration puts it, saying where each is
held. On the guide's robot, the elevator kept out of 0.1 m to 0.6 m while the arm is at most 0° is
one, since the arm rests below the bumper's band, at -45°: "as the robot starts, it leaves arm and
elevator holding each other by their interlocks, so neither can free the other: a deadlock (arm
keeps out of -10° to 30° while elevator at most 0.8128 m, so arm stays between -60° and -10°;
elevator keeps out of 0.1 m to 0.6 m while arm at most 0°, so elevator stays between 0 m and 0.1
m)". The stowed configuration, which puts them there too, is refused the same way.

**A joint may rest inside a region it's kept out of**, as an elevator at its bottom stop kept out
of its lowest stretch while the intake is in does. It's a doubt said at startup, not a refusal:
until the region opens, the joint may only leave it, by the edge with room past it (the nearer, if
both have), and the definition's deadlock check counts that way out. A configuration may not put a
joint there.

**A region may be written larger than the joint's travel.** It's the part inside the travel that
counts: on an arm that travels from -30° to 120°, `Range.degrees(-40, 0)` keeps it out of -30° to
0°, the same as a region whose edge sits at the arm's low stop, and the arm leaves it by 0°. A
region with none of the travel in it is refused, and so is one that covers all of it.

### The PDH

Each motor controller says which of the PDH's channels feeds it, and the breaker in that channel,
where it's wired: `TALON_FX.can(32).clockwise().pdh(4, Amps.of(40))`, channel 4 as labeled on the
PDH, through a 40 A breaker. The PDH itself is the robot's, declared once in its definition,
`.pdh(Pdh.can(1))`: REV's Power Distribution Hub on Systemcore's CAN, at its ID, 1 unless it's been
changed. The drivetrain's motor controllers say theirs in its wiring, each module's `driveFeed` and
`steerFeed`. The robot refuses a channel that feeds two devices, one the PDH hasn't (it has 0 to
23), a breaker over its channel's rating (40 A on 0 to 19, 15 A on 20 to 23), and a feed on a robot
with no PDH. With a PDH declared, a wired motor controller that doesn't say its channel is noted at
startup: which channel feeds it isn't checked. Both are optional: a robot that declares no PDH says
nothing of feeds. CTRE's Power Distribution Panel isn't supported yet.

The robot reads the PDH each loop, with the joints: each channel's current, their total and the
battery's voltage (`PDH/ChannelCurrentAmps`, `PDH/TotalCurrentAmps`, `PDH/Volts`); each device it
feeds by name (`PDH/Devices/pivot motor 32`, `PDH/Devices/front-left drive`); and each joint's
motors together (`Joints/<label>/PdhCurrentAmps`), beside its leading controller's own reading
(`Joints/<label>/SupplyCurrentAmps`), which is its motor's alone. A PDH that doesn't answer is an
alert. In the simulator, each channel reads what the simulated motor it feeds draws, and a test can
zero one (`world.pdh().zero(4, true)`), as a tripped breaker leaves it.

**A motor drawing nothing** is flagged: a follower whose breaker tripped, so its leader drives the
joint alone and nothing else says so, or a controller its block puts on the wrong channel. While a
joint works, its leading controller drawing more than 2 A from the battery by its own reading, each
of its motors draws about as much on its own channel; one whose channel reads under 0.5 A for half
a second is flagged, in `Joints/<label>/DeadChannels` and an alert, saying what to check: "arm motor
12 draws nothing on channel 5 while arm's motors work: its breaker tripped, its power is unplugged,
or its block names the wrong channel". The joint's control says otherwise,
`.withDeadChannel(Amps.of(3), Amps.of(0.5), Seconds.of(1))`, and
[values.md](values.md#mechanismcontroldeadchannel) says why each is what it is. What it's asked
doesn't count: a roller at a steady speed with no load draws next to nothing, alive or not. A leader
that loses power stops answering on CAN, and its joint's moves are refused saying so; a lone motor
unplugged from a SPARK, which then draws nothing whatever it's asked, isn't flagged here.

## 7. What the definition refuses

The robot checks its definition as it starts, in the simulator too, and in the test that builds
every robot. It refuses:
- a position, or where its joint rests, outside the joint's range; a name used twice;
- a position, an output, or a configuration's or a condition's place that isn't that joint's own:
  a name it hasn't, or one of its names at a value it has never had;
- a speed faster than the joint's motors can turn it, through its gearing, whether one of its
  outputs or its profile's cruise (an output within a tenth of it is a doubt, since the motor can't
  hold that under load);
- a value of the other kind: a range, soft limits, a profile, bands, gains or an output's speed
  given in angles for a joint that slides, or in lengths for one that turns: "creeping, 10 rpm, is
  a speed of turning, and elevator slides: give it a speed along it, Output.of("creeping",
  InchesPerSecond.of(...))";
- a joint with motors and no control; a control that doesn't fit its joint: written for the other
  loop, `position(kP)` on a roller or `velocity(kP)` on an arm; soft limits past its hard stops, or a named position outside them; a
  homing recipe it can't carry out (a roller's, a hard stop on one that turns freely, a switch it
  hasn't, or one whose position isn't known); a joint held at positions with neither an absolute
  encoder nor a recipe; a loop closed on an absolute encoder where it can't be ([Absolute
  encoders](#absolute-encoders));
- an absolute encoder that can't tell where its joint is: one of its turns not covering the joint's
  travel, saying the gearing that does; or, on a joint that slides, one without the motors that
  say its drum, or without stops;
- wiring that can't work: an encoder on a TalonFX's data port, which it hasn't; two sensors on one
  input; a switch on a motor controller's limit input that has none, or another joint's; two
  devices of one kind on one bus at one ID; a PDH channel feeding two devices, or a breaker over
  its rating;
- an interlock on a joint exactly at one of its positions, which it never measures exactly; a
  configuration that breaks an interlock, joints whose interlocks hold each other for good where
  the robot starts or a configuration puts them, or an interlock written twice. A joint resting
  inside a region it's kept out of is a doubt, not a refusal.

What's doubtful, though allowed, or not known, the robot says as it starts, in the log
(`Robot/Doubts`) and on the Driver Station's console; what only the simulator reads, a link's mass,
it says only when it simulates (`Robot/SimulatorNotes`).

## 8. When the template can't drive it

The block says some mechanisms the template can't drive yet: a hood on linear servos, a
differential's pair. Some it can't say at all, such as an arm a motor swings through a linkage.
Either way, the joint stays in the definition, and a subsystem of your own drives it with WPILib's
API or its vendor's.

**Keep the joint in its block, and leave it undriven.** Write its shape, its range, its places,
what it carries and its sensors as for any joint, and its servos if it has them (`.servo(...)`), or
a differential's drives in the robot's definition. Leave out `.control(...)`: a control asks the
template to drive the joint, and one on a joint it can't drive stops the robot starting, saying
why. Motors in a joint's block are the template's to drive, so motors your own code drives stay out
of it.

```java
static final Position HOOD_DOWN = Position.degrees("down", 0);
static final Position HOOD_LOB = Position.degrees("lob", 30);

static final Actuator.Servo WCP_0408 =
    Actuator.Servo.builder().from(0.168).to(0.268).force(50).speed(0.032).build();

static final Joint.Revolute HOOD =
    Joint.pivot("hood", Part.CHASSIS, new Translation3d(-0.2, 0, 0.55), Joint.PITCH_UP)
        .range(Range.degrees(0, 40)).rest(HOOD_DOWN)
        .positions(HOOD_DOWN, HOOD_LOB)
        // Each servo's body on the chassis, from the hood's axle, and its rod's end on the hood.
        .servo(
            "hood left",
            WCP_0408,
            new Translation3d(0.054, 0.1, -0.233),
            new Translation3d(0.054, 0.1, 0))
        .servo(
            "hood right",
            WCP_0408,
            new Translation3d(0.054, -0.1, -0.233),
            new Translation3d(0.054, -0.1, 0)); // no .control(...): Hood drives it
```

Kept there, it's still the robot's own:
- **Checked** as every joint is: its places inside its range, each name once, its sensors' wiring
  against every other device's.
- **Its sensors are read** with the robot's: a beam on its link, or a switch at its end, is a gate,
  `joints.gate("hood down", HOOD.switchAt(End.LOWER).tripped())`, though only your code stops the
  joint there.
- **Its places are its constants**, so `HOOD_LOB` means one thing in every file, and a practice
  robot changes the joint as it changes any (`replaced(...)`, `range(...)`).

What you give up until the template drives it:
- **Its moves and gates.** `joints.of(HOOD)` says why there's none ("hood has no mechanism: it's
  moved by hood left and hood right, and the template drives only motors turning a shaft so far"):
  no outcomes, no `isAt`, no `Joints/hood` in the log. Your subsystem's commands are plain v3
  commands, and it logs what you need.
- **Interlocks.** None may name it, as the joint held or the one read: nothing could enforce it,
  and the robot refuses it. A clash with it is your code's to prevent. A configuration that names
  it can't run, and the robot says so as it starts, with its other doubts, so the rest of the robot
  still runs.
- **The simulator's plant.** It isn't simulated, and the template takes it as resting where its
  block rests it: a camera or a joint riding on it is placed from there. A switch on it is a test's
  to trip.

**Read the model; don't restate it.** Your subsystem takes the robot's definition and finds its
own joint there, so a practice robot's range and places are the ones it uses:
- the joint: `robot.own(HOOD)`, this robot's own, a practice robot's stops and places its own;
- a place, by its constant: `hood.own(HOOD_LOB).value()`, in radians, or meters for one that
  slides;
- its hard stops: `hood.range()`;
- each servo's share of its travel at an angle, from where its block puts the servo: each is one of
  `hood.ownDrives()`, and `servo.command(math.coordinate(drive, Map.of(hood, angle)))` its share,
  `math` being `robot.math()`;
- an absolute encoder on a Systemcore port, in its block: its wiring,
  `hood.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port)`, names the port to read
  with WPILib's `DutyCycleEncoder`, and its `read(...)` turns what the port reads into the
  encoder's turns; `math.fromAbsolute(math.absoluteEncoder(hood), turns)` turns those into where
  the joint is. A CANcoder is read through its joint's motor controller, so a joint without motors
  in its block can't have one.

Only what the block can't say yet is written in the subsystem: for servos, their PWM ports and the
pulses their datasheet gives. WPILib 2027's alpha has no `Servo` class: a servo on one of
Systemcore's ports is a `PWM`, told a pulse in microseconds.

```java
final class Hood implements Mechanism {
  private final Scheduler scheduler;
  private final Joint hood;
  private final RobotMath math;
  private final List<Drive> servos;
  private final List<PWM> pulses;

  /** The angle its servos were last told, in radians: NaN until they're first told. */
  private double told = Double.NaN;

  Hood(Scheduler scheduler, RobotDefinition robot, PWM left, PWM right) {
    this.scheduler = scheduler;
    hood = robot.own(HOOD); // this robot's own hood
    math = robot.math();
    servos = hood.ownDrives(); // its servos as its block names them: left, then right
    pulses = List.of(left, right);
  }

  @Override
  public Scheduler getRegisteredScheduler() {
    return scheduler;
  }

  /** Goes to one of its places, and ends once its servos have had time to get there. */
  Command goTo(Position place) {
    return run(co -> co.wait(Seconds.of(tell(hood.own(place).value()))))
        .named("Hood to " + place.name());
  }

  // Tells each servo the share of its travel that puts the hood at an angle, held to its range,
  // and says how long the slower takes to get there. Nothing reads back where it is: it's assumed
  // where it was told before, or the first time, at the far end of its range.
  private double tell(double angle) {
    Range range = hood.range().orElseThrow();
    double to = Math.max(range.lower(), Math.min(range.upper(), angle));
    double far = to - range.lower() > range.upper() - to ? range.lower() : range.upper();
    double from = Double.isNaN(told) ? far : told;
    double seconds = 0;
    for (int i = 0; i < servos.size(); i++) {
      Drive drive = servos.get(i);
      Actuator.Servo servo = (Actuator.Servo) drive.actuator();
      double length = math.coordinate(drive, Map.of(hood, to)); // eye to eye, there
      // A WCP-0408 takes a pulse of 1000 µs fully retracted, to 2000 µs fully extended.
      pulses.get(i).setPulseTimeMicroseconds((int) Math.round(1000 + 1000 * servo.command(length)));
      double travel = Math.abs(length - math.coordinate(drive, Map.of(hood, from)));
      seconds = Math.max(seconds, travel / servo.speed());
    }
    told = to;
    Logger.recordOutput("Hood/Told", Math.toDegrees(to));
    return seconds;
  }
}
```

Build it in `Controls`, with the robot's definition `Controls` is given and its servos' ports,
`new Hood(scheduler, robot, new PWM(0), new PWM(1))`, and bind its commands as any v3
mechanism's: `operator.faceUp().onTrue(hood.goTo(HOOD_LOB))`. `runtime/src/test/java/com/michaelgrundvig/frc/runtime/examples/`
has it compiled (`ServoHoodRobot`, `Hood`), and `HoodSimTest` checks the pulses it sends.

## Not yet

- **A string potentiometer on an elevator.** It reads the carriage, not the drum, and its block
  has nowhere for its length per range or an offset in meters. A potentiometer on the drum's
  shaft, or geared to it, is an absolute encoder ([On an elevator](#absolute-encoders)).
- **A bench bypass for interlocks.** It waits for a pit mode.
- **Moving between configurations by a route,** such as first raising the arm so the elevator can
  drop. Until then, write the two steps.
