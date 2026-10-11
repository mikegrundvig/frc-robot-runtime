# Driving

The driver's sticks drive the robot ([with the sticks](#driving-with-the-sticks)), and the robot can
hold a heading while they do. The drivetrain also has moves, as a joint does
([mechanisms](mechanisms.md#3-use-it)). Each move is a
Commands v3 command that ends on its own, with an outcome (`ARRIVED`, `TIMED_OUT`,
`CANCELED` or `REFUSED`) and a reason, logged as `Drive/Outcome`. For example: "drive → (2.00,
1.00) 90.0°: ARRIVED in 2.04 s".

| Move | Does | Ends |
|---|---|---|
| `drive.runUntil(new ChassisVelocities(1.5, 0, 0), bumped)` | Drives at a robot-relative velocity until a condition holds | When the condition holds. It has no deadline of its own: give it one with `within`, and it times out then, saying why a gate never opened |
| `drive.turnTo(Rotation2d.fromDegrees(90))` | Turns where it stands to a field heading, the nearer way round | Within 2°, nearly still, for 40 ms |
| `drive.goTo(new Pose2d(2, 1, Rotation2d.fromDegrees(90)))` | Drives to a field pose in a straight line, turning on the way | Within 2 cm and 2°, nearly still, for 40 ms |
| `drive.goTo(Rebuilt.TOWER)` | The same, to a `Place` as it is for the robot's alliance ([places](#places-for-either-alliance)) | As `goTo` to a pose; refused if the alliance is unknown |
| `drive.follow(keyframe, keyframe, ...)` | Drives a smooth path through keyframes, stopping at the last | As `goTo`, at the last keyframe |
| `drive.goTo(pose).within(Seconds.of(4))` | The same move with another deadline | As above |

A move that drives a path has a default deadline: twice the path's time, plus half a second. A
move that times out says where the robot got to: "TIMED_OUT after 0.50 s, at (0.30, 0.00) facing
0.0°". A `runUntil` has no default, because nothing says how long its condition takes to hold: it
drives until the condition holds or it is canceled. Wherever driving on would be wrong, give it a
deadline, as the example below does.

`runtime/src/test/java/com/michaelgrundvig/frc/runtime/examples/Explorer.java` shows the moves in use. Its robot drives forward
until its bumper switch trips, then turns a quarter turn either way, and repeats:

```java
Command explore() {
  return Command.requiring(drive).executing(co -> {
    while (true) {
      DriveMove forward = drive.runUntil(FORWARD, bumped).within(Seconds.of(15));
      co.await(forward);
      if (!forward.arrived()) {
        return; // the bumper never tripped: the move says why, and the robot stops
      }
      co.await(drive.turnTo(nextQuarter()));
    }
  }).named("Explore");
}
```

## Driving with the sticks

The template binds the driver's sticks for you: `drive.driven()` is the drive's default command, and
it runs whenever nothing else drives.

- **Left stick:** translates, on the field's axes. Forward is away from the driver's own wall, so a
  red driver's forward is the other way, and nobody thinks about it.
- **Right stick:** turns, counterclockwise when pushed left.
- **Shaping:** past a small deadband, each stick is squared, for fine control near center. Full
  stick is the top speed and the top turning rate ([values](values.md#drivetraincontrollimits)).

### Facing something while driving

`drive.facing(heading)` drives from the same left stick, and the robot holds a heading by itself.
It runs until something else takes the drive. Say where to face with a `Heading`:

| Heading | Faces |
|---|---|
| `Heading.field(Rotation2d.fromDegrees(90))` | A heading on the field, whichever alliance |
| `Heading.downfield(Degrees.of(-20))` | An angle from the driver's forward, counterclockwise: 0 faces away from the drivers, -20 is 20° to their right. A snap button written once, right for both alliances |
| `Heading.toward(Rebuilt.HUB)` | A place on the field, as it is for the robot's alliance, from wherever the robot is |

The heading is worked out again every loop, so it follows the alliance and the robot's place on the
field. A robot that shoots from its back says so, and its back faces the heading instead:

```java
DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB).withBack());
```

Each loop, the robot turns toward the heading as fast as its turning limits allow, the nearer way
round. Facing a place, it also turns with the place as the robot moves past it, so strafing around
the hub doesn't leave it behind. It uses `headingKp`, the same gain a move's turn uses
([values](values.md#drivetraincontrolgains)). It spins up no faster than a move would, at half the
model's turning acceleration, and slows by the same law that steers it. In the simulator, a quarter
turn from rest settles inside 2° in 0.76 s, goes 1.4° past, and ends within 0.5°.

- **The driver takes the heading back.** While the right stick is past its deadband, the robot turns
  as the stick says. Let go, and it turns back to the heading.
- **Nothing to face.** If the alliance isn't known yet, or the robot sits on the place it should
  face, the robot doesn't turn. The left stick still drives.
- **The log.** `Drive/HeadingTargetDegrees` and `Drive/HeadingErrorDegrees`, a gap while there's no
  heading.

Bind it as you bind anything. These hold 20° either side of downfield while a button is held, and
lock the wheels while another is:

```java
driver.faceDown().whileTrue(drive.facing(Heading.downfield(Degrees.of(-20))));
driver.faceRight().whileTrue(drive.facing(Heading.downfield(Degrees.of(20))));
driver.faceLeft().whileTrue(drive.lockWheels());
```

### Is it aimed?

`aim.aimed()` is a [gate](mechanisms.md#gates-wait-for-things-and-know-why-not): open while the robot
faces the heading to within 2° and has nearly stopped turning. It says what the robot does, whether or
not `facing` is running, and when it's closed it says why: "aimed at the Hub is false: 3.4° off the
Hub", or "alliance unknown". `aim.within(Degrees.of(4))` is the same assist with another tolerance.

Link the movement to an action with the gate. This feeds only while the robot is aimed and the
shooter is at speed, and the driver's sticks drive all the while:

```java
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
    .named("Shoot at the hub");
```

### Locking the wheels

`drive.lockWheels()` turns the wheels into an X, each along its line to the robot's center, with the
drive motors holding still, until something else takes the drive. The robot is hard to push or spin. Each
wheel turns whichever way is shorter.

## How a move drives

A move plans its path when it starts, from where the robot is and how fast it's moving then. The
limits come from the robot's own model ([values](values.md)):

- **Linear acceleration.** Each wheel pushes with the lesser of two forces: what its motor gives at
  the stator current limit, through the gearing, and what its tread grips on its share of the
  robot's weight.
- **Turning acceleration.** The same wheel forces, about the robot's yaw inertia.

A move plans at half of these accelerations. The yaw inertia is an estimate until the robot is
characterized, and the simulator turns the robot more sluggishly than the model says.

Each loop, the move works out what to ask of the drivetrain:

- **Speed and turning rate:** what the path asks for at the end of the loop, plus:
  - a push back toward the path's place (`translationKp`);
  - a push back toward the path's heading (`headingKp`);
  - a push toward the path's speed.
- **Each wheel's volts:** each wheel also gets the volts its share of the path's acceleration takes.
  That way the robot keeps up with the path, rather than trailing it.

Both gains are in [values](values.md#drivetraincontrolgains). In the simulator, a quarter turn
planned at 0.83 s arrives in 0.88 s, and a path through three keyframes passes each within 2 cm
and 2°.

## Keyframes

A keyframe is a field pose the path passes through. The robot turns to face the keyframe's
heading on the way:

```java
DriveMove leg = drive.follow(
    Keyframe.at(new Pose2d(3, 1, Rotation2d.ZERO)),
    Keyframe.at(new Pose2d(5, 2, Rotation2d.CCW_90DEG)).named("pickup"),
    Keyframe.at(new Pose2d(7, 2, Rotation2d.PI)));
```

- **The path's shape** is a smooth curve through the keyframes' places: WPILib's cubic spline. It
  leaves toward the first keyframe, and arrives from the one before the last.
- **The path's speed** is kept under the top speed, and under the acceleration both along the path
  and across it in curves.
- **The robot's heading:** once the robot has passed a keyframe and faced its heading, it turns
  toward the next keyframe's heading, as fast as its turning limits allow, the nearer way round. If
  it hasn't finished turning to the last heading when it reaches the last place, it turns there.
- **Refused:** a keyframe within 5 cm of where the robot starts, or of the keyframe before it. No
  smooth path goes through those. A path with no keyframes, or two of one name, is refused as it's
  made, as a joint's move to a place it hasn't is ([mechanisms](mechanisms.md#outcomes)): it
  throws as the robot starts, and made once it runs, it ends `REFUSED` as it starts. So is
  `leg.passed(...)` given a name none of its keyframes has: made once the robot runs, it's a gate
  that holds false, saying why.

### Do something at a keyframe

A named keyframe is something code can wait for. `leg.passed("pickup")` is a
[gate](mechanisms.md#gates-wait-for-things-and-know-why-not): it opens when the path passes the
keyframe, and stays open until the move starts again. `runtime/src/test/java/com/michaelgrundvig/frc/runtime/examples/PickUp.java`
has it whole:

```java fragment
Command pickUp() {
  return Command.requiring(drive, pivot).executing(co -> {
    DriveMove leg = drive.follow(...);
    co.fork(leg);                       // drive the path...
    co.waitUntil(leg.passed("pickup"));     // ...and as it passes pickup,
    co.await(pivot.goTo(PIVOT_OUT));    // swing the intake out
    co.await(leg);                      // then wait for the path's end
  }).named("Pick up");
}
```

The gate opens on the path's timing, not on the robot's position. The robot follows the path to
within a few centimeters, so in the simulator the gate opens within 7 cm of the keyframe.

## Places for either alliance

A pose is in field coordinates, so a path of poses is right on one alliance only. A `Place` is written
once, for blue, and found for the robot's alliance when the move starts (red's is blue's turned half a
turn, or found from red's own tags). `drive.goTo(place)` and `Keyframe.at(place)` take one, so the
same move is right on both:

```java
static final Place PICKUP = Place.at("Pickup", new Pose2d(5, 2, Rotation2d.CCW_90DEG));

static void moves(Drive drive) {
  DriveMove toTower = drive.goTo(Rebuilt.TOWER);
  DriveMove leg = drive.follow(Keyframe.at(PICKUP), Keyframe.at(Rebuilt.TOWER));
}
```

The alliance is read when the move starts, never when it's built, so a move made as the robot boots,
before the Driver Station connects, still goes to the right side. If the alliance is still unknown when
it starts, it ends `REFUSED` and the robot doesn't move: "drive → Pickup: REFUSED: alliance unknown".
`follow` plans its path when it starts, so a path of places is right on both with no other change; a
pose in it stays in field coordinates.

## Not yet

- **Paths from a file,** such as PathPlanner's or Choreo's. The path tool is each team's choice
  ([stand-ins](stand-ins.md)). A planned path is a WPILib `HolonomicTrajectory`, so a path read
  from a file could be followed the same way. [Autos](autos.md) are written with `follow` now.
- **Obstacles.** A path goes where its keyframes put it. Nothing steers it around anything.
- **The robot's real limits.** Moves plan at half the model's accelerations until the robot's
  yaw inertia is measured ([calibration](calibration.md#the-robots-mass-center-of-gravity-and-inertia)).
