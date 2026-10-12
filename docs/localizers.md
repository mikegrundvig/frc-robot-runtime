# Localizers: choosing what works out where the robot is

A localizer takes the wheels' readings, the IMUs' readings and the cameras' measurements, and
says where the robot is. The runtime has two. You can also write your own.

Most teams never open this page. A robot that says nothing about its localizer gets the default
one, and everything in the other guides assumes it.

## What can be replaced, and what cannot

The runtime talks to its localizer through one interface, `Localizer`, in the localization
library. Anything that implements it can drive the robot.

One part stays in front of every localizer. The runtime reads each camera's tag detections and
solves them into measurements: a pose, the time the image was taken, and how uncertain the pose
is. That solving and filtering is the library's `VisionPipeline`, and it is not replaceable. Your
localizer is handed the same measurements, with the same uncertainties, that the default one is.
It never sees an image.

So the seam is: odometry and measurements in, a pose out.

## The contract

The whole contract is in `Localizer`'s Javadoc, as numbered clauses, L1 to L23. Read it there. It
is written to be enough to implement a localizer without reading either of ours. In short:

- **Frames, units and time (L1, L2).** Poses are on the field, in meters and radians. Times are
  nanoseconds on the robot's clock. A localizer reads no clock of its own, so a replay gives the
  same poses.
- **The pose (L3 to L5).** `pose()` always answers. `poseAt(time)` answers for the last half
  second at least, and is empty outside what the localizer keeps.
- **Odometry (L6 to L8).** With odometry alone the pose moves as the wheels and the gyro say. A
  lost gyro, a wheel that did not report and a loop with no readings are all ordinary.
- **Placing the robot (L9, L10).** After `place`, the pose is the placed pose.
- **Measurements (L11 to L16).** A measurement that agrees is used, and never overshot. It is
  applied at its capture time. One with no heading does not turn the estimate. Bad data costs one
  measurement, never the loop.
- **Cost (L17).** A loop costs the same after two minutes as after two seconds.
- **The options (L18 to L23).** See below.

The contract says what a localizer must do to be driven by. It says nothing about accuracy.

Each loop the runtime calls a localizer in a fixed order:

```java fragment
void odometry(DrivetrainReadings readings, LoopContext loop);
void place(PoseReset placed);
void measurements(List<VisionMeasurement> taken);
```

`odometry` comes first, every loop. `place` comes only in a loop where the drive team placed the
robot. `measurements` comes last, every loop, with an empty list when no camera saw a tag.

## The options

A localizer must give a pose. It may also report more. Each option is one method that returns an
`Optional`, and returns empty unless you override it.

| Option | What it is |
|---|---|
| `covariance()` | How sure the pose is |
| `mode()` | What the pose rests on: tags seen lately, odometry alone, nothing yet |
| `health()` | The numbers behind the mode |
| `decisions()` | What became of each measurement |
| `fieldVelocities()` | How the robot moves over the field |
| `wheels()` | How far the wheels are trusted: tilt, ground slope, slip, a collision |
| `trust()` | How far each camera is trusted, and tags it has no place for |
| `yawSource()` | What turns odometry: the gyro, the backup IMU, or the wheels |

Do not fill an option with a guess. The runtime copes without each one, and it tells the truth
about what is missing. This is what it does:

| Without | The runtime |
|---|---|
| `covariance()` | Pitwall's Localization line says how sure the pose is "is not reported by this localizer". The World view draws no uncertainty ellipse. No image is solved using the pose's heading, so every image is solved from its tags alone. No camera's mount is checked or calibrated against the pose. |
| `mode()` | The Localization line does not say what the pose rests on. No camera's mount is checked or calibrated. |
| `health()` | Its numbers are left out of the log. |
| `covariance()`, `mode()` or `health()` | The scoring gate cannot judge the pose. It holds nothing back, and the Scoring line reads "Not judged". `readyToScore` is true. |
| `decisions()` | Each measurement is logged and drawn as "given to the localizer", not as accepted or rejected. The Cameras line counts measurements given, with no share accepted. No camera's mount is checked. |
| `fieldVelocities()` | The robot's speed comes from the wheels and its turning from the IMU. |
| `wheels()` | Tilt, slope and slip are left out of the log. Nothing warns of an IMU mounted another way than the robot's description says. |
| `trust()` | Nothing warns of cameras or tags the localizer cannot keep apart. |
| `yawSource()` | It is left out of the log. |

When any option is missing, the log's `Localization/NotReported` names the ones that are.

## The two localizers

**The default localizer** (`DefaultLocalizer`) is the library's own estimator. It gates every
measurement against the estimate, recovers when the cameras agree against odometry, lowers the
wheels' trust when they slip, and keeps an honest uncertainty. It reports every option.
[coming-from-wpilib.md](coming-from-wpilib.md#pose-estimation-and-vision) says how it differs
from what you know.

**WPILib's pose estimator** (`WpilibLocalizer`) is `SwerveDrivePoseEstimator`, fed what the
default one is fed: every wheel reading, and the pipeline's measurements with their standard
deviations. It keeps the required contract and reports none of the options, because WPILib's
estimator keeps no uncertainty, no mode and no record of what it did with a measurement.

WPILib's estimator leaves some things to the code that calls it. `WpilibLocalizer` does them, and
its Javadoc says how. The ones worth knowing:

- It turns the estimator by the gyro, by the backup IMU while the gyro is lost, and by the wheels
  with neither.
- A placement's uncertainty is dropped. WPILib's reset takes a pose and nothing else.
- It gives each loop's measurements to WPILib oldest first. WPILib discards its newer corrections
  when it is handed an older measurement.
- A measurement with no heading is given a heading standard deviation of a million radians.
- Its state standard deviations are WPILib's defaults, 0.1 m and 0.1 rad, unless you give your
  own.

## Choosing one

A robot chooses its localizer in its own block, with one line. This is the guide's robot on
WPILib's estimator:

```java
public static final RobotConfig ON_WPILIB =
    RobotConfig.builder()
        .definition(ExampleRobot.SIM.definition())
        .vision(ExampleRobot.SIM.vision())
        .estimator(EstimatorConfig.DEFAULTS)
        .localizer(WpilibLocalizer::new)
        .build();
```

Leave the line out and the robot gets the default localizer. `Main` does not change either way.

`estimator` is the default localizer's tuning. A robot on another localizer still has it, unused.
To tune WPILib's estimator, give it state standard deviations:

```java fragment
.localizer(WpilibLocalizer.with(0.03, Math.toRadians(1)))
```

With WPILib's estimator chosen, Pitwall's status lines read:

```
Localization   What the pose rests on and how sure it is: not reported by this localizer
Scoring        Not judged: what the pose rests on is not reported by this localizer
Cameras        4 of 4 cameras connected, 212 measurements given, how many were accepted is not reported
```

## Writing your own

Implement `Localizer`. Six methods have no default:

```java fragment
void odometry(DrivetrainReadings readings, LoopContext loop);
void place(PoseReset placed);
void measurements(List<VisionMeasurement> taken);
Pose2d pose();
Optional<Pose2d> poseAt(long timestampNanos);
long timestampNanos();
```

`DrivetrainReadings` holds every wheel reading since the last loop, as `OdometrySample`s, and
what both IMUs read. A `VisionMeasurement` holds a pose, its capture time, and a
`PoseCovariance`. A measurement solved with the robot's heading has an infinite heading variance,
which means "this says nothing about heading".

Report an option only where your localizer really knows the answer. This is the default
localizer reporting how sure it is:

```java fragment
@Override
public Optional<PoseCovariance> covariance() {
  return Optional.of(estimate.covariance());
}
```

Then choose it in your robot's block, with a `LocalizerMaker`. A constructor that takes the
robot's definition and the field is one:

```java fragment
.localizer(WpilibLocalizer::new)
```

Two things to know before you start.

- Your localizer picks its own gyro. `DrivetrainReadings` has the gyro and the backup IMU, and
  either may be lost in any loop. Each IMU has its own zero, so switch between them by how far
  each turned, not by what each reads.
- Without `covariance()`, no image is solved with your heading. If your localizer trusts its
  gyro, report a covariance so the pipeline can use it.

## Testing your own

The contract is also a test suite, `LocalizerConformance`, in the localization library's test
fixtures. It drives a scripted robot whose wheels and gyro are exact, and checks each clause. A
test class of a few lines runs it on a localizer. This is the one for WPILib's:

```java
class WpilibLocalizerConformanceTest extends LocalizerConformance {
  WpilibLocalizerConformanceTest() {
    super(LocalizerBench.of("WPILib's pose estimator", WpilibLocalizer::new));
  }
}
```

Name the options your localizer reports after the maker, such as `Option.COVARIANCE`. The suite
runs the clauses for those, and checks that the rest really are empty.

Both localizers here pass it. The default one passes all 21 scenarios. WPILib's passes the 16
that every localizer must, and the 5 for options it does not report are skipped.

The suite does not check accuracy. For that, run the estimator study on your localizer.

## What the comparison found

The template has a study that runs both localizers on the same simulated robot and scores each
against the simulator's true pose. Every contender is run through the `Localizer` interface. It
is in the template's [docs/estimator-comparison.md](../../docs/estimator-comparison.md), and
`./gradlew estimatorStudy` runs it.

What it found, in simulation only:

- In clean driving the two are roughly equal. Both hold the pose to about a centimeter.
- The default localizer is better against bad camera data, such as a camera that keeps misreading
  a tag, and against wheels that slip.
- WPILib's estimator, behind the kind of vision filter strong teams write for it, is better when
  the gyro reads wrong or a camera has been knocked on its mount. The default localizer leans on
  the gyro for heading, and that costs it there.
- Most of each difference comes from how images are solved and filtered, not from the estimator.
  Fed the same measurements, as `WpilibLocalizer` is, the two are close.

No real robot was involved. Do not quote a number from it as a fact about your robot.
