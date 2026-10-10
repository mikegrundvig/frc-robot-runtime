# Autos

An auto is a name, the place the robot starts, and a function that makes its command. You declare
each in `Controls`; the template chooses one from the pit, places the robot on its start, and runs it
when autonomous begins.

## Declare one

`Controls` gets the robot's `Autos` list. Add an auto with `autos.add(name, start, factory)`. The start
is one of the field's starts, written for blue like any place; the factory is a method that returns
the auto's command. This is the guide's robot ([mechanisms](mechanisms.md#where-behavior-goes)), with
one auto, from `runtime/src/test/java/com/michaelgrundvig/frc/runtime/examples/Controls.java`:

```java fragment
private static final Place PICKUP = Place.at("Pickup", new Pose2d(6, 3, Rotation2d.ZERO));
autos.add("Grab one", Rebuilt.starts(robot.body()).center(), this::grabOne);

/** Drives to the pickup with the intake running, then brings the intake in. */
Command grabOne() {
  return Command.noRequirements(
          co -> {
            co.awaitAny(drive.goTo(PICKUP), intake());
            co.await(pivot.goTo(PIVOT_IN));
          })
      .named("Grab one");
}
```

It is written once and is right on both alliances: `drive.goTo(PICKUP)` finds the place for the
robot's alliance when the move starts ([driving](driving.md#places-for-either-alliance)), and the
start is found the same way. `Rebuilt.starts(robot.body())` gives the field's three starts for this
robot, `.left()`, `.center()` and `.right()`.

A mistake in the declaration stops the robot starting, as a move to a place a joint doesn't have does
([outcomes](mechanisms.md#outcomes)): a blank name, or two autos of one name.

## What the robot does with it

- **"Do nothing" is always there,** first in the list, and it is the default. A robot with no autos
  declared has only that, and the choice is never empty. It starts the robot where the Driver Station's
  location says, as before.
- **A fresh command every run.** The factory is called when autonomous starts, not when the robot
  boots, so the alliance is the one on the field and running autonomous twice is the same as once.
  Build commands in the factory, not in the constructor: a command that has run can't be composed
  into another or scheduled again. A gate can be made in either place: one made in the constructor
  is worked out and logged every loop, and one made in the factory or inside a command is worked
  out whenever it's read, and leaves nothing behind when its run is over.
- **Nothing you write starts or stops it.** When autonomous becomes enabled, the template schedules
  the chosen auto's command. When autonomous ends, disabled or into teleop, it cancels the command if
  it is still running. An auto that shoots until the buzzer is cut off, and the log says so.
- **Every wait has a deadline.** A move times out and says why, so an auto never hangs on one. Give
  your own waits one too (`co.waitUntil(gate, Seconds.of(2))`, `command.withTimeout(...)`).
- **It is logged.** `Auto/Chosen` is the choice, and `Auto/Outcome` how each run went, with its time:
  "Left Double: CUT OFF after 15.00 s, as autonomous ended".
- **One that throws doesn't stop the robot.** An auto that throws as it is built runs nothing, says
  why in `Auto/Outcome` and a HIGH alert, and the robot carries on. One that throws as it runs is
  handled as any command's is: on the robot the loop carries on and reports it, and in the simulator
  the loop ends, so the bug fails its test. Either way the run is `FAILED` and the alert says why.

## Choosing and starting

The pit chooses on Pitwall's Pit view, in the Autonomous card, as it chooses a venue. The robot takes
the choice at the start of a loop while it is disabled, never while it drives; a choice made while
enabled waits, and the card says so. The choice is logged as an input, so a replay runs the auto the
robot did.

The chosen auto's start is where the robot is placed (in the simulator) and what the placement check
reads before the match: the dashboard's `OnStart` and `Placement` say "Ready on Left: 2 cm, 1° off", or
that the robot is on another start, or on none. An auto's start is usually one of the field's
starts, and may be a place of its own: the check reads that one too.

## Test them

`AutoCheck` runs every auto a robot declares, "Do nothing" included, on each alliance in the
simulator, from its own start, for the 15 seconds of autonomous:

```java
var results = AutoCheck.check(sim, ExampleRobot.COMPETITION, Controls::new);
```

It fails, with each run reported, if an auto throws, if any drive or joint move in it ends `REFUSED`,
if the robot leaves the field, or if the auto can't start: the alliance is unknown, or the placement
check doesn't read ready on its start. Each run reports where the robot ended and each move's outcome
line, so a failure reads without a debugger:

```
Grab one on RED: ended at (10.53, 5.07) 180.0°, Grab one: COMPLETED in 2.82 s
    pivot → out: ARRIVED in 0.60 s
    drive → Pickup: ARRIVED in 2.22 s
    roller: intaking: CANCELED after 2.22 s
    pivot → in: ARRIVED in 0.60 s
```

Add a test of what the auto should do beyond that, in your robot's own test (`BetaAutosSimTest` has
them: where each ended against its last waypoint, that the intake was out before the first path, and
that it shot only once aimed and at speed).

## Not here

- **Paths from a file,** PathPlanner's or Choreo's. A path read from a file could be handed to an
  auto as a command ([stand-ins](stand-ins.md#waiting-on)); the template reads none.
- **Named commands and event markers.** Write the sequence in the factory: it is Java, so the
  compiler checks every name.
