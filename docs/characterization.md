# Characterization

Some numbers belong to a kind of part rather than to one robot: how noisy a camera is at a
distance, how late its results arrive, how far a gyro drifts, how well a tread grips the carpet.
Measure each once, on a bench or a practice floor, for each model of part the team uses, and again
when the part, its software, or its settings change (a new camera, a new PhotonVision version, a
different exposure). Every robot with that part then shares the number. The
[values register](values.md) lists which values each procedure sets; the procedures for numbers that
differ robot to robot, run from the Pitwall page, are in [calibration](calibration.md).

Record each result the way calibrations do: the builder call that sets the value, with the date
and what was measured (`.latency(Milliseconds.of(24)) // characterized YYYY-MM-DD: camera
latency, under full load`). [The values register](values.md#the-register-describes-values-robotsjava-holds-a-robots-numbers)
says where it goes. Keep the logs, named for the procedure and the date, so the number can be
worked out again.

Most of these read the robot's own log, so they need a Systemcore running the robot program; each
procedure's equipment says whether it does. Run the program (disabled is fine: it logs its
sensors), open the log in AdvantageScope, and read the values from its plots, or export them
(**File > Export Data**) to a spreadsheet for the arithmetic. AdvantageScope exports a rotation as
a quaternion (W, X, Y, Z); its yaw, in radians, is `atan2(2(W·Z + X·Y), 1 - 2(Y² + Z²))`.

A robot's own numbers measured by hand (its mass, its center of gravity, how its IMU reads crossing
a bump) are calibrations, with the rest in [calibration](calibration.md#the-robots-mass-center-of-gravity-and-inertia).

## Safety first

These procedures drive a robot that weighs as much as a person, pull on it, and power electronics
from a bench supply. Do them with a mentor there. Each procedure says what to watch for; these hold
for all of them:

- **Driving:** clear the area, and keep everyone out of the robot's path, including its path if it
  slides or lurches. One named person at the Driver Station does nothing but watch the robot, ready
  to disable it, and says "enabling" before it moves.
- **Never hold anything against an enabled robot,** not a scale and not a hand. A scale the robot
  pushes on is fixed to something solid, and read from the side or on a phone's video.
- **Wear safety glasses** whenever the robot is powered or a tool is running, as at events.
- **Batteries** are heavy and can deliver hundreds of amps:
  [battery safety](#battery-safety) below, and the FIRST Safety Manual.

## Camera noise

**What it measures:** how far a camera's view of a tag scatters with everything still, how far it
is off, and how both grow with distance. It sets vision's trust in a camera:
`VisionConfig.linearStdDevBaseline`, `angularStdDevBaseline`, `distanceExponent`,
`headingConstrainedFactor`, the simulator's corner noise (`VisionSim`), and the camera's sight
range (`CameraConfig.Sensor.sightRange`).

**Equipment:**

- The camera, on the coprocessor the robot runs, with the same PhotonVision version, resolution,
  exposure, and pipeline settings, and its lens calibrated at that resolution.
- A Systemcore running the robot program, to log the camera: it logs every tag a camera sees, with
  the tag's pose from the camera (`Vision/<camera>`). The robot itself is easiest; a camera on a
  tripod, on the robot's network, works as well.
- A tag of the field's size and family, printed flat and stiff, on a stand whose height you can set.
- A tape measure, a plumb line or a square, and tape to mark the floor.
- Light like an event's: bright, even, no sunlight on the tag.

**Safety:** the robot stays disabled throughout.

**Steps:**

1. Raise the tag until its center is at the height of the lens's center, so the tag is straight
   ahead of the camera and the distance along the floor is the distance the camera measures. (If
   it can't be, measure the height difference `h`, and use `sqrt(d² + h²)` for every taped
   distance `d` below: at 1 m, a 0.6 m difference reads 1.17 m.)
2. Mark the floor straight below the front of the lens, with the plumb line. Set the tag square to
   the camera, 1 m from that mark, measured along the floor to the tag's face, to the millimeter.
3. Log 30 s, with nothing moving.
4. Do the same at 2, 3, 4, 5, and 6 m, then 7 m and on until the camera stops finding the tag in
   most images. Note each distance and the time you logged it.
5. At 2 m and 4 m, log 30 s more with the tag turned 30° from square, and 60°.

**Working out the numbers:** for each spot, take the tag's pose from the camera
(`bestCameraToTag`) in every image.

- **Spread:** the standard deviation of its position, along the line of sight and across it, and
  of its rotation about the vertical (its yaw), across the images.
- **Scale and offset:** over the square spots, plot the average distance the camera measured
  against the taped one, and fit a straight line. Its slope should be within about 1% of 1: more
  means the lens calibration's focal length is off, which puts far tags off by the same share
  (recalibrate the lens before going on). Where it crosses zero is how far behind the front of
  the lens the camera measures from, its optical center, a few millimeters: add it to every taped
  distance below. A single spot can't tell the two apart: at 1 m, 1% is 1 cm, about what the
  offset and the tape's own error are.
- **Error per spot,** for vision's trust: the root mean square, over the images, of each image's
  position less the taped one (corrected as above), combining both directions (the estimator uses
  one number for both). It includes the spread and what's left of the bias.

Plot the error per spot against distance, square spots only, on a spreadsheet's power trendline
(`y = b · x^k`). `k` is **`distanceExponent`**. `b`, the error at 1 m, is the least
**`linearStdDevBaseline`** can be (below). Do the same with the rotation's spread for the least
**`angularStdDevBaseline`** can be.

**`headingConstrainedFactor`:** a solve that takes its heading from the gyro has no error from the
tag's rotation; a full solve's position moves sideways by the distance times that rotation's error.
At each square spot, with `s` the position's spread, `r` the rotation's spread in radians, and `d`
the distance, the factor is `s / sqrt(s² + (d · r)²)`. Take the average over the spots.

**Corner noise, for the simulator:** PhotonVision's simulation moves each tag corner a distance
drawn from a normal distribution, in a random direction; `VisionSim`'s two numbers are that
distance's mean and standard deviation (today 0.25 px and 0.08 px). To measure them at one still
spot: for each corner in each image, its distance in pixels from that corner's average position
over all the images. The mean of all those distances, and their standard deviation, are the two
numbers. The sight range (`sightRange`) is the farthest spot where the camera found the tag in
most images.

**What good looks like:** the exponent near 2, since a tag's distance error grows with the distance
squared (its sideways error only with the distance). A still camera's error at 1 m is a few
millimeters. The configured baseline (0.02 m today) is larger on purpose: on the robot it also
covers motion, timing, and the mount, which the bench doesn't see. So take the exponent from here,
and compare cameras with `b`; set the baseline on the robot, starting from `b` and raising it until,
in clean driving, the gate rejects about 1 in 1000 of a camera's measurements
(`Vision/<camera>/Decisions`). Far more, and the baseline is too small.

### While moving

`speedStdDevFactor` and `yawRateStdDevFactor` say how much worse an image is while the robot moves.
It needs the robot program running on the robot. **Safety:** it drives:
[the driving rules](#safety-first).

1. On a practice field with tags at known places, calibrate the robot's camera mounts.
2. Stand still where the camera sees tags 2 to 4 m away, and log 20 s. Then drive back and forth
   past the same tags at a steady 1, 2, and 3 m/s, and spin in place at 1, 2, and 3 rad/s, logging
   each.
3. Each camera logs how far its tags land from their real places, through the estimate
   (`Vision/<camera>/TagResidualMeters`). Take its root mean square at each speed, over the
   still one: the ratio is `1 + factor × speed`, so the factor is `(ratio - 1) / speed`. Average
   over the speeds.

The estimate's own error grows while moving too, so this is an upper bound. Measure it with the
cameras the robot will use, at the exposure it will run: a shorter exposure blurs less.

## Camera latency and time sync

**What it measures:** how long a camera's result takes from the image's capture to reaching the
robot (`CameraConfig.Sensor.latency`), how many results arrive per second (`frameRate`), and
whether the capture time the robot is given is right. It also checks `EstimatorConfig.historySeconds`
and `futureToleranceSeconds`, and sets the simulator's latency spread (`VisionSim`).

**Why the capture time matters:** the estimator applies each camera measurement at the moment its
image was captured, from the history it keeps. Latency itself costs little: a late measurement is
still applied where the robot was. A wrong capture time costs accuracy: 10 ms off, and every
measurement lands where the robot was 10 ms earlier or later, 3 cm off at 3 m/s, along the way it
drives. PhotonVision stamps each image on the coprocessor's clock and converts the stamp to the
robot's, with its time sync keeping the two clocks in step over the network. On Systemcore, with
WPILib 2027's nanosecond clock, check that it does.

**Equipment:** the robot, running its program on its Systemcore, with its cameras on their
coprocessor, on the network it runs at events (the radio, or the same switch); tags in view; for
the second part, a practice field with tags at known places, and room to spin.

**Safety:** the loaded run and the turning drive the robot: [the driving rules](#safety-first).

**Steps, standing still (latency and frame rate):**

1. Robot disabled, a camera facing a tag 2 m away. Log 60 s.
2. Again with the robot loaded as in a match: every camera running, the robot enabled and driving
   in a clear space, a dashboard open. Latency grows when the coprocessor or the network is busy
   ([coprocessor capacity](#vision-coprocessor-capacity) measures how much).

**Working out the numbers:** the robot logs each image's latency, from its capture to the loop
that read it (`Vision/<camera>/LatencySeconds`). The robot reads its cameras once a loop, so an
image also waits for the loop, anywhere from 0 to 20 ms: that wait averages 10 ms, and spreads
with a standard deviation of 20/√12 = 5.8 ms. So from the loaded run:

- **`latency`** is the latencies' mean less 10 ms (`latency` is an average, as the simulator uses
  it);
- the camera's own spread, for the simulator, is `sqrt(s² - 5.8²)` in milliseconds, where `s` is
  the latencies' standard deviation;
- **`frameRate`** is the images logged in a minute, over 60; PhotonVision's dashboard shows the
  same.

With `latency` measured, set `VisionConfig.latencyMarginSeconds` a little above the latencies'
spread: images later than both are left out.

**Steps, turning (the capture time):**

1. On the practice field, where a camera sees two or more tags 2 to 3 m away, spin the robot in
   place at a steady 1 rad/s one way for 10 s, then the other way for 10 s. Log it.
2. For each image with a multi-tag solution, the camera's heading on the field comes from the
   solution (`multiTagFieldToCamera`'s yaw, from its quaternion as above). The gyro's heading at the image's capture time comes from
   its 250 Hz readings (`Drive/Gyro/OdometryYawsRad`, at `OdometryTimestampsNanos`): interpolate
   between the two readings either side.
3. Each image's difference: the camera's heading, less the gyro's, less the camera's mounted yaw.

**Working out the number:** a capture time off by `t` makes a robot turning at `w` rad/s read
`w × t` off, one way spinning left and the other way spinning right; a constant difference (the
gyro's offset, the mount) is the same both ways. So the capture time's error is
`(average difference spinning left - average difference spinning right) / (2 × w)`, in seconds.

**What good looks like:** latency steady within a few milliseconds, with no long tail; a result
100 ms late or more means a busy coprocessor or network. The capture time's error under about
5 ms, and under about 2 ms if every fix corrects the heading (`EstimatorConfig.headingCorrection`
`EVERY_FIX`): while the robot turns, each fix then leans on its capture time ([the consistency
study](testing.md#the-simulators-late-stamps)). More than a loop (20 ms) means the time sync isn't
working: check that the coprocessor runs the PhotonVision version the robot's PhotonLib expects,
and that the robot's clock and PhotonVision's agree. Results applied "too late" or "from the
future" in `Vision/<camera>/Decisions` say the same.

## Vision coprocessor capacity

**What it measures:** how many cameras one vision coprocessor can run before their frame rate
drops or their latency grows. It sets the frame rate and latency each camera gets on the robot
(`CameraConfig.Sensor.frameRate` and `latency`, for that many cameras on one coprocessor), and how
many coprocessors the robot needs. Measure it on the model of coprocessor the robot will use.

More images is more measurements, closer together: when it comes to a choice, keep the frame rate
and add a coprocessor.

**Equipment:**

- One coprocessor as the robot runs it: the same model, cooling, PhotonVision version, and power,
  through the regulator the robot uses.
- Three cameras of the robot's model, each with its lens calibrated at the resolution in use, and
  the AprilTag pipeline set as the robot runs it (resolution, exposure, and the rest).
- Several tags in each camera's view, 2 to 3 m away, so detection does real work.
- A Systemcore running the robot program, to log the results
  ([camera latency](#camera-latency-and-time-sync) says how); or, with no Systemcore, PhotonVision's
  dashboard on a laptop on the coprocessor's network.

**Safety:** a coprocessor gets hot over ten minutes of full load: let it cool before handling it.

**Steps:**

1. One camera on the coprocessor. Let it run two minutes to warm up, then log two minutes.
2. Plug in the second camera, set the same; log two minutes, both cameras.
3. Plug in the third; log two minutes, all three.
4. With all three, log ten minutes more. A coprocessor that heats up slows itself, and its frame
   rate falls with it. Note its temperature and load at the end, if its dashboard shows them.

**Working out the numbers:** for each step and each camera, its frame rate (results logged per
second) and its latency, worked out as in [camera latency](#camera-latency-and-time-sync). Write
them in a table: cameras on the coprocessor, against each camera's frame rate and latency.

**Where it goes:** each camera's `CameraConfig.Sensor` takes the frame rate and latency from the
step with as many cameras as one coprocessor carries on the robot. Cameras whose coprocessors carry
different numbers of cameras need a `Sensor` each.

**What good looks like:** every camera holding its own frame rate as cameras are added, and its
latency within a few milliseconds of the single camera's. The step where any camera's frame rate
drops (by more than about a tenth) or its latency climbs is past the coprocessor's capacity: put
that camera on another coprocessor. So is a rate that sags over the ten minutes.

## Gyro drift and noise

**What it measures:** how far the heading drifts per minute with the robot standing still, and how
far each reading scatters, for the gyro and for the backup IMU. It sets
`EstimatorConfig.headingDriftDegreesPerMinute`, and `SensorErrors.TYPICAL`'s gyro and backup drift
and noise; it checks `ImuCrossCheck.Settings.toleranceDegrees`.

**Equipment:** the robot, running its program on its Systemcore (which logs both IMUs), with the
IMUs mounted as they'll run; a floor or a bench nobody will bump; half an hour.

**Safety:** the robot stays disabled while it logs; the hard driving before the third run, and the
spins of the scale's repeatability below, follow [the driving rules](#safety-first).

**Steps:**

1. Turn the robot on and leave it still for the first minute, as before a match.
2. Log 10 minutes, robot disabled, nobody touching it or what it stands on.
3. Do it twice more, once right after ten minutes of hard driving: drift changes as the electronics
   warm.

**Working out the numbers:** fit a straight line to each IMU's heading against time
(`Drive/Gyro/Yaw`, `Drive/BackupGyro/Yaw`). The **drift** is the line's slope, in degrees per
minute. The **noise** is the standard deviation of the heading about the line: from the gyro's
250 Hz readings (`Drive/Gyro/OdometryYawsRad`), and the backup's, read once a loop. Those readings
are in radians: multiply by 180/π (57.3) for `SensorErrors`' `noiseDegrees`.

**Where it goes:** `headingDriftDegreesPerMinute` takes the largest drift of the runs, rounded up,
since the estimator should never be surer of the heading than the gyro deserves.
`SensorErrors.TYPICAL` takes a typical run's drift and noise, for the gyro and the backup.
`ImuCrossCheck.Settings.toleranceDegrees` should be well above how far the two drift and scatter
apart in its window.

**What good looks like:** a still gyro drifting well under a degree a minute, about the same every
run. More, or a drift that changes from run to run, means a loose mount, vibration, or a failing
gyro. The simulator assumes 0.2° a minute for the gyro and 1° for the backup IMU; how Systemcore's
onboard IMU drifts is unknown until measured.

### Repeatability of the gyro's scale

`EstimatorConfig.headingStdDevDegreesPerTurn` is how far the gyro's heading may still be off after a
full turn, once its scale is calibrated: the same error every turn, so five turns are five times as
far off. Until it's calibrated, keep the template's 1°.

1. Run [the gyro's scale](calibration.md#the-gyros-scale) calibration five times in a row, from the
   same spot. Each reports a scale.
2. The standard deviation of the five scales, times 360°, is the error per turn. Scales spread by
   0.0005 give 0.18° per turn.

**The backup IMU's scale:** during those spins, the log has both IMUs. The backup's turn over the
gyro's, across all five turns, is the backup's scale (`SensorErrors.TYPICAL`'s `backupGyro.scale`).
How far it is from 1 is what `ImuCrossCheck.Settings.scaleTolerance` must stay above. The
headings wrap at ±180°: count the whole turns.

## Tread friction

**What it measures:** the tread's grip on carpet, new and worn: the coefficient of friction,
`DrivetrainGeometry.ModuleType.wheelCoefficientOfFriction`. The robot's drive moves plan their
acceleration on it and its slip check judges by it, the simulator's grip comes from it, and with the
center of gravity's height it sets where the wheels slip under acceleration, which the drive's
current limit is chosen to stay under. Until it's measured, it's PathPlanner's default, 1.2, and
the robot says so at startup.

**Equipment:**

- The robot, as it runs (bumpers and battery on), or a sled with four of the tread's wheels,
  weighted to the robot's weight.
- Carpet like the field's, or the field's own.
- A force gauge or digital hanging scale that reads to at least 1.5 times the robot's weight, and a
  strap.
- The robot powered, its program running on its Systemcore, and disabled: its drive and steering
  motors are then in brake mode, which holds the modules' angles while you pull.

**Safety:** pull from the side, standing clear of where the robot goes when it lets go, with the
strap's loop on the scale's hook, never around a hand. The robot stays disabled.

**Steps:**

1. Weigh the robot ([as calibration says](calibration.md#the-robots-mass-center-of-gravity-and-inertia)).
2. Point every wheel straight ahead (drive forward a little, then disable).
3. Strap around the frame, low and on one side. Pull sideways, level with the floor, slowly and
   steadily, watching the scale. Note the highest reading, just before the robot slides.
4. Five pulls each way, on different spots of carpet.
5. Again with worn tread: the end of a season's, or a worn set kept for this.

**Working out the number:** the coefficient is the pull over the weight, in the same units (pounds
over pounds). Average the highest readings: the grip before it lets go is what a hard launch uses.

**Where it goes:** `wheelCoefficientOfFriction` takes the worn tread's, the worst the robot will
play with, unless the team retreads before every event. Then redo the drive's current limit (the
weight-transfer estimate in `Robots`) with it and the center of gravity's height.

**What good looks like:** 1.0 to 1.3 for common treads on carpet. Over 1.5, the pull wasn't level
(pulling up lightens the robot) or the carpet slid. A sideways pull drags the tread across its
grain; a tread may grip differently rolling. To measure the driving direction, drive the robot
slowly into a scale fixed to a wall, raising the current until the wheels spin, and divide the
highest reading by the weight. That one drives: [the driving rules](#safety-first), with nobody
near the scale while the robot is enabled, and the reading taken from the side or on video.

## Feedforward

**What it measures:** the drive's feedforward: `DrivetrainControl.Gains.driveKs`, the volts to
overcome friction, and `driveKv`, the volts per meter per second. These belong to a drivetrain
design (modules, gearing, tread): robots built alike share them, measured on one. Measure again when
any of those changes.

**Equipment:** the robot, running its program on its Systemcore, and a long clear floor.

**Safety:** SysId's tests drive the robot on its own, the dynamic ones at full voltage: lots of
room, and [the driving rules](#safety-first). For a module lifted off the floor, set the robot
securely on blocks so it can't drive off them, and keep hands, hair, and cords away from the
spinning wheels.

**WPILib's SysId** measures them: it records the drivetrain under slowly rising voltage and under
voltage steps, and fits kS, kV, and kA from the log. Follow
[WPILib's SysId guide](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html);
the template doesn't have a SysId routine yet. The same tests on one module, lifted off the floor,
give the inertia of its wheel, gears, and motor, which the template estimates today
(`DrivetrainGeometry.ModuleType.driveInertia`) and the simulator adds to the share of the robot's mass
each wheel carries (`Body.mass`).

**Units:** SysId reports in the units the routine logged; the template's gains are in volts per
meter per second.

- Logged in wheel rotations: divide its kV by the wheel's circumference in meters
  (`2π × wheelRadius`).
- Logged in motor rotations: multiply by `driveGearing`'s ratio first, then divide by the
  circumference.

A 2026 drivetrain's 0.7237 V per wheel rotation per second, with a 0.049 m wheel radius, is
`0.7237 / (2π × 0.049)` = 2.35 V per m/s.

**Without SysId,** the template's own tools get close:

- `driveKv` starts at `DrivetrainGeometry.ModuleType.idealDriveKv()`, the motor and gearing's ideal
  (what the template's robot uses).
- Then tune both live on the Pitwall page: with `driveKp` at zero, drive at a steady speed and
  compare each module's speed with its setpoint in AdvantageScope (`Drive/ModuleVelocities`
  against `Drive/SetpointsOptimized`). Raise kV until they match at speed; raise kS until slow
  moves start without lagging. Then put `driveKp` back, and commit what works.

**What good looks like:** kV a few percent above the ideal (2.54 V per m/s on the template's
robot); kS a few tenths of a volt or less (the 2026 code's was 0.145 V). The steering's gains are
tuned, not characterized: tune them live, watching each module's angle
(`Drive/Module<name>/SteerAngle`) against its setpoint.

## Odometry error

**What it measures:** how far odometry alone drifts per meter driven and per radian turned, and how
far the heading the wheels give drifts per turn. Like feedforward, these belong to a drivetrain
design and its tread: robots built alike share them. It sets
`EstimatorConfig.translationStdDevPerMeter`, `translationStdDevPerRadian`, and
`wheelHeadingStdDevDegreesPerTurn`.

**Equipment:** the robot, running its program on its Systemcore, calibrated first (the gyro's
scale and the wheel radius); a clear floor of field carpet at least 6 m long; tape and a tape
measure.

**Safety:** it drives at match speeds: [the driving rules](#safety-first).

**Steps:**

1. Turn every camera off (Vision's switch for each, a tunable), so the estimate is odometry and the
   gyro alone.
2. Tape a start mark at one front corner of the frame, and a finish mark 5 m straight ahead of it,
   measured with the tape. Mark the robot's heading at each with a line along its side: the same
   heading at both.
3. **Driving:** start on the start mark, drive a route like a match's (straights, curves, strafing,
   about 30 m in all, at match speeds) and stop exactly on the finish mark. Log it. Do it five
   times.
4. **Spinning:** on the start mark, spin in place ten turns one way and ten the other at driving
   speed, and stop on the mark. Log it. Do it five times.

**Working out the numbers:** the estimator assumes each direction's variance grows by the square of
the per-meter number with every meter. So for each run, take `d`, the distance driven (how far the
wheels rolled, either way: the changes in `Drive/Module<name>/DrivePositionRad`, without their
signs, added up, times the wheel radius, averaged over the modules), and `e`, how far the
estimate's travel is from the real one. From the estimate at the start
(`Localization/Pose`: `x₀`, `y₀`, heading `θ₀`) and at the end (`x₁`, `y₁`), its travel ahead is
`(x₁ - x₀) cos θ₀ + (y₁ - y₀) sin θ₀` and to the left `-(x₁ - x₀) sin θ₀ + (y₁ - y₀) cos θ₀`. The
robot really traveled 5 m ahead and none to the left (none at all, spinning), and `e` is the
distance between the two.

- **`translationStdDevPerMeter`** = `sqrt(average of e² / (2 × d))` over the driving runs.
- **`translationStdDevPerRadian`** = the same over the spinning runs, with `d` replaced by the turn
  in radians (twenty turns is 126 rad).
- **`wheelHeadingStdDevDegreesPerTurn`:** spinning in place, each wheel rolls around a circle whose
  radius is its distance from the robot's center. Over each direction's ten turns, the wheels'
  average travel over that radius is the turn they report, in radians. Compare it with the gyro's
  turn: the difference in degrees, over the ten turns, is the error per turn. Take the root mean
  square over the runs and both directions.

**What good looks like:** a calibrated swerve drivetrain on carpet ends within about 1 to 2% of the
distance it drove. Much more means a wheel radius or the gyro's scale needs calibrating again, or
the wheels slip: the log's `Localization/SlipMetersPerSecond` and `ResidualMeters` show where.

## Battery internal resistance

**What it measures:** how far a battery's voltage drops for each amp drawn: its internal
resistance, in milliohms. It belongs to each battery, not to a model: batteries age differently, so
measure every one, and label each with its own name. It sets how far the voltage sags in a hard
acceleration, and so how much current the drivetrain may draw before the robot browns out:
`DrivetrainControl.Limits.driveSupplyCurrent` is chosen for the weakest battery the team plays with.

**Equipment:** a battery analyzer that measures internal resistance (no Systemcore needed); or the
robot, running its program on its Systemcore to log, and a clear floor.

**Safety:** [battery safety](#battery-safety), below; the log method also drives:
[the driving rules](#safety-first).

**Steps, with an analyzer:** charge the battery fully, let it rest an hour, and measure it as the
analyzer's instructions say.

**Steps, from the robot's log:**

1. A fully charged battery in the robot, on carpet with room to drive.
2. Log: five seconds standing still, then a full-stick launch from rest for two meters. Rest, and
   launch twice more.

**Working out the number:** in the log, the battery's voltage (AdvantageKit records it under
`SystemStats`) and the current drawn, which in a launch is nearly all the drive motors': the sum
of their supply currents (`Drive/Module<name>/DriveSupplyCurrentAmps`). Fit a straight line of the
voltage against the current over each launch: its slope, without the minus sign, is the resistance
in ohms, and where it meets zero current is the battery's voltage at rest. This counts the robot's
wiring, breaker, and connectors too, so it reads higher than an analyzer: it's what the robot
really sees, and a loose connection shows in it.

**Where it goes:** on the battery's label and in the team's battery log, with the date, not in the
code. For the drive's supply limit: with four drive motors at their limit, the voltage is about the
rest voltage less `4 × limit × resistance`. For the weakest battery, that must stay above both
Systemcore's brownout voltage (still to measure on a real Systemcore) and the [vision coprocessor's](#coprocessor-brownout-ride-through).

**What good looks like:** batteries of one model, bought together, reading alike. One that reads
well above the rest is aging or damaged: keep it for practice, or retire it. Measure again before
each event.

### Battery safety

A robot battery is heavy, and if its terminals are shorted it delivers hundreds of amps: enough to
weld a wrench across them and start a fire. Before you handle one:

- Look it over. A cracked or bulging case, a leak, or a loose terminal: set it aside, labeled, and
  don't charge or use it.
- Carry it by its case or handle, never by the cables, and keep its terminals covered: nothing
  metal across them.
- Charge only with the team's battery charger, in a ventilated place, with someone nearby.
- Follow the battery rules in the FIRST Safety Manual, which every team member should read.

## Coprocessor brownout ride-through

**What it measures:** the lowest input voltage at which a vision coprocessor keeps running, powered
the way the robot powers it. Below it, the coprocessor resets, and its cameras are gone for its
whole reboot, far longer than the sag that caused it. No configuration value holds it; it
sets a floor the drive's supply current limit must keep the battery above
([battery internal resistance](#battery-internal-resistance)).

**Equipment:** an adjustable bench power supply with a current limit, that can give what the
coprocessor and its cameras draw at the lowest voltage you'll test (below); the coprocessor, its
cameras, and the regulator or power module it runs from on the robot, wired as on the robot; a
multimeter on the regulator's input; a laptop on its network, to watch PhotonVision. No
Systemcore needed.

**Safety,** with a mentor there:

- Wire everything with the supply's output off.
- Check the polarity before switching it on: the supply's positive (red) to the regulator's
  positive input. Reversed, many regulators are destroyed at once.
- Set the voltage and the current limit before connecting, and never set the voltage above the
  regulator's maximum input (its datasheet).
- Keep loose wire ends apart, and touch nothing while the output is on.

**Steps:**

1. Wire the supply to the regulator's input, as the robot's battery would be. Set 12.5 V. A
   switching regulator draws constant power, so as its input voltage falls its input current
   rises: at 6 V it draws about twice what it does at 12.5 V. So set the current limit to about
   `(12.5 / the lowest voltage you'll test) × the current it draws at 12.5 V` (measure that first,
   with the coprocessor running), or the supply limits the current instead of the voltage.
2. Start the coprocessor with every camera running its pipeline as on the robot: a regulator drops
   the most at full load.
3. Lower the input in half-volt steps, then tenth-volt steps as you near the end, ten seconds at
   each, watching the cameras' frame rates and the supply's constant-current (CC) light. The
   voltage at which the coprocessor resets (its stream stops, and the dashboard loses it) is its
   ride-through voltage. A reset while the CC light is on doesn't count: the supply, not the
   coprocessor, gave out (its voltage collapsed). Raise the current limit and run it again.
4. Do it three times. If the supply can, also drop it suddenly from 12 V to a low voltage for a
   tenth of a second, as a brownout does, and find the lowest dip it survives.

**What good looks like:** the coprocessor riding through to below Systemcore's own brownout
voltage, so a sag costs the robot its motors before it costs its cameras. If it resets higher,
power it through a regulator that keeps its output up as its input sags (a buck-boost), and measure
again.
