# Testing

Every test runs on every build, locally and in CI, so a test is the cheapest way to keep a bug
fixed. The simulator makes most of the robot testable without a robot: a test can drive it, knock
it, fool its sensors, and check what it believes against where it really is.

## Running tests

These are the commands in a robot program that has the runtime as `runtime/`, as the template does.
The runtime, its libraries (localization and the simulation), the model and Pitwall are builds of
their own there (included builds), run with their path in front; in the runtime's own repository
they are the build's own, and `:runtime:` goes away.

| Job | Command |
|---|---|
| Rerun the fast tests on every save while you work | `./gradlew test -Pquick --continuous` |
| Rerun only the tests you're working on, on every save | `./gradlew test -Pquick --continuous --tests '*Arm*'` |
| One of your test classes | `./gradlew test --tests frc.robot.ArchitectureTest` |
| One of the runtime's, whatever its kind | `./gradlew :runtime:test --tests com.michaelgrundvig.frc.runtime.RobotContainerSimTest` |
| A package's tests | `./gradlew :runtime:test --tests 'com.michaelgrundvig.frc.runtime.subsystems.drive.*'` |
| One project's tests: your robot program's, the runtime's, the simulation's | `./gradlew :test`, `:runtime:test`, `:runtime:simulation:test` |
| The model's, localization's and Pitwall's tests | `./gradlew :model:test`, `:localization:test`, `:pitwall:test` |
| Your robot program's tests, with the coverage report | `./gradlew test` |
| Every check CI runs, in every project | `./gradlew ci` |

`--tests` finds what it names in the build it is run in: your robot program (`frc.robot`) from the
program's root, and in front of `:runtime:test`, the runtime (`com.michaelgrundvig.frc.runtime`),
and its simulation's (`:runtime:simulation:test`, `com.michaelgrundvig.frc.simulation`).
`frc.robot.ArchitectureTest` is yours, `com.michaelgrundvig.frc.runtime.ArchitectureTest` the
runtime's. A project with none of it passes; if no project has any, the build fails, naming what
you asked for, so a misspelled name can't pass unnoticed. A project's name in front
(`:runtime:simulation:test`) runs only its tests. Localization is a build of its own, like the
model: `:localization:test --tests ...` runs its tests (`com.michaelgrundvig.frc.localization`).

`-Pquick` leaves out the slow tests (below) unless `--tests` names what to run, and skips the static
analysis and coverage that `ci` runs. Each project has its own coverage report, your robot
program's at `build/reports/jacoco/test/html/index.html`, the runtime's under `runtime/` and each
library's under its folder (`runtime/lib/localization/build/reports/...`), and each its own floor: a
project's own tests must cover it. The runtime's are on the guide's robot (`ExampleRobot`).

### The fast loop

While you work on something, leave one command running in a terminal, naming the tests that
cover it:

```
./gradlew test -Pquick --continuous --tests '*Arm*'
```

(In the Windows Command Prompt: `gradlew test -Pquick --continuous --tests *Arm*`, without the
`./` or the quotes.)

Every save reruns them, in a second or two, because Gradle stays running between saves. What keeps
it quick:

- **`--tests`** runs only what you name. Without it, quick mode runs every fast test in every
  project.
- **`-Pquick`** leaves out what only `ci` needs: Error Prone and NullAway, formatting, JaCoCo's
  coverage agent and report, the simulation and end-to-end tests unless `--tests` names one, and
  the build's time in `BuildConstants`. Without the time, a program that hasn't changed compiles
  to the same classes, and Gradle skips tests whose inputs haven't changed. So a change to a
  comment reruns no test, as long as it adds or removes no line (the classes record each line's
  number) and the repository already had uncommitted changes (the first change after a commit
  turns `BuildConstants.DIRTY` from 0 to 1, which the tests see).
- **Gradle's caches** (`gradle.properties`): the configuration cache skips configuring the
  projects after the first build, and the build cache hands a task the outputs of an earlier run
  with the same inputs, such as after undoing an edit.

From saving a file to the result, measured on a 16-core Linux desktop while other builds were
running on it, so a quiet computer may do better and a laptop worse:

| You save | What runs | Save to result |
|---|---|---|
| A word in a comment, in a robot class, its lines unmoved, with uncommitted changes already | The robot program's compile; no test, since its classes come out the same | 1.5 s |
| Code in a robot class | The compile, then the tests | 1.8 s |
| Code in a library (`localization/`) | The library's compile, then the tests; the robot program compiles again only when the library's public API changed | 1.1 s |
| Anything, with a one-off `./gradlew test -Pquick --tests ...` instead | Gradle starting and finding its cached configuration, then the above | about 7 s |

About 0.9 s of a robot edit is compiling the whole robot program again. Gradle compiles only the
classes an edit affects unless an annotation processor can't say what it generates from what, and
three on the robot program's path can't: AdvantageKit's `@AutoLog` processor, and the processors
for WPILib's Epilogue and avaje-jsonb that GradleRIO adds. At under a second, it's left as it is.

`ci` takes four to five minutes on the same computer, most of it the robot program's simulation
tests: `EstimatorConsistencySimTest` (80 to 105 s) and `VisionSimTest` (40 to 50 s), more as other
builds share it. Run it before pushing, not on every save.

### Memory

Each project's tests run in a Java process (a JVM) of their own, with up to 1.5 GB of heap. The
robot program's simulation tests use more outside the heap (PhotonVision's camera simulation,
WPILib's native libraries); a small young generation keeps that in check ([the consistency
study](#is-the-estimator-honest-the-consistency-study), *Memory*). Its tests also start a fresh JVM every 20 test classes,
which is faster as well as smaller: 124 s with each JVM near 1.2 GB, against 265 s and 2.2 GB for
one JVM running them all (2026-10-02). Projects build in parallel, and
at most two test JVMs run at once, the robot program's and another project's. On a computer with 8
GB, make it one: put `testJvms=1` in the `gradle.properties` of your Gradle user home (`~/.gradle/`
on macOS and Linux, `%USERPROFILE%\.gradle\` on Windows), or pass `-PtestJvms=1` to one build. The
limit doesn't count build-logic's plugin tests, which `ci` runs with Gradle TestKit (a Gradle of
their own), or the robot programs `ReplayEndToEndTest` starts, one at a time, beside its test JVM.

## Where a test goes

The runtime ships what a team tests its own robot with, its test fixtures
(`runtime/src/testFixtures/java`): `RobotSim`, `Monkey`, `AutoCheck`, the joint, drive and
localization invariants, `GoldenFile` and the rest of the harness. Your tests (`src/test/java`,
`frc.robot`) use them as any test uses a library; the runtime's own tests
(`runtime/src/test/java`) are on the guide's robot, and a robot of yours is tested in yours.

Test classes mirror the package of what they test, in the same project: the tests for the world
model's `com.michaelgrundvig.frc.world.field.Zone` are in
`model/src/test/java/com/michaelgrundvig/frc/world/field/ZoneTest.java`. Pick the lowest level
that can show the bug; the lower the level, the faster the test and the more exactly a failure says
what broke.

| Level | What it runs | Example to copy |
|---|---|---|
| A component | Plain JUnit on a class in a library (the model in `model/`, localization in `localization/`) or the robot's own `com.michaelgrundvig.frc.runtime.lib`. No simulator, no hardware, no clock: these run in milliseconds, and most tests belong here. | `ZoneTest`, in `model/`; `LocalizationServiceTest`, in `localization/`, for localization fed by hand a loop at a time |
| A mechanism | A subsystem on simulated hardware, stepped loop by loop. The class carries `@ExtendWith(RobotSim.class)`. | `subsystems/drive/DriveBasicsTest`, with `sim/DriveSimHarness` |
| The whole robot | The robot program as `Robot` builds and runs it, Pitwall and all, driven from a simulated Driver Station and gamepad. | `RobotContainerSimTest` |
| The program itself | The robot program started as a program, in its own process: a session recorded, then replayed. A recorded session runs on a clock of its own, which moves one loop at a time (`SessionClock`), so it records the same log on any computer, however busy; `SessionRepeatsEndToEndTest` records each session twice and compares the two logs to the bit (with `SESSIONS_UNDER_LOAD=1` it keeps every processor busy for the second, which is a check to run alone). | `ReplayEndToEndTest` |
| The structure | The architecture rules (what may depend on what), checked against the compiled code, each for a bug Gradle's boundaries between projects can't see: the runtime's, which hold its components and the libraries to the same answer every run too, and keep it from naming any of your code; yours, the rules that hold for every class of a robot program (`ProductionRules`, one definition for both) and what keeps your code and the runtime apart; the world model's packages; Pitwall's. | `ArchitectureTest`, the runtime's and yours |

Name a test class after what it tests: `ZoneTest`. A test class that runs the simulator for
seconds of robot time is a `...SimTest` and carries `@SimulationTest`; one that starts the program
as a process carries `@EndToEndTest`. Quick mode leaves both out.

## A whole-robot test

`RobotSim` gives each test the same start, whatever ran before it: the clock stopped, the Driver
Station attached and disabled on the blue alliance's station 2, and fresh dashboards. It builds the
robot exactly as `Robot` does (with no controls bound, unless the test gives it yours:
`sim.build(robot, Rebuilt::game, Controls::new, errors)` is your `Main`'s call), and `loop()` runs
exactly the loop `Robot` runs. Afterwards it
closes and resets everything, so nothing leaks into the next test.

```java
@SimulationTest
class DrivingSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void theLeftStickDrivesAwayFromTheDrivers() {
    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-1.0); // pushed forward
    sim.driver().notifyNewData();

    sim.run(1.5); // seconds of robot time, loop by loop

    assertThat(sim.drive().getPose().getX()).isGreaterThan(5.0);
  }
}
```

What it offers:

- **The robot:** `sim.robot()`, built on first use as the simulator's robot (`ExampleRobot.SIM`,
  the template's drivetrain and cameras) with sensors exactly as configured. To test another, build it first: `sim.build(SensorErrors.TYPICAL)` for a
  season-worn robot, `sim.build(RobotSim.WITHOUT_CAMERAS, SensorErrors.NONE)` for odometry alone,
  or `sim.build(RunMode.REAL, robot, SensorErrors.NONE)` for the CTRE code on CTRE's
  simulated devices.
- **Time:** `sim.loop()`, `sim.loops(n)`, and `sim.run(seconds)`. The clock moves only when the test
  steps it, so a test never waits and never depends on how fast the computer is.
- **The Driver Station:** `RobotSim.setEnabled(true)`, `RobotSim.enable(RobotMode.AUTONOMOUS,
  true)`, `RobotSim.station(AllianceStationID.RED_1)`. Change it through these, or call
  `DriverStationSim.notifyNewData()` after changing it yourself, or the change never arrives.
- **The drivers:** `sim.driver()` and `sim.operator()`, the gamepads on ports 0 and 1, which the
  robot's controls are bound to; `sim.bindings()`, every binding it made, by button and command.
  A class shaped as `Controls` (a `ControlsMaker`: yours, or a fixture's) is bound to them by
  `sim.controls(Controls::new)`.
- **Autos:** `AutoCheck.check(sim, robot, Controls::new)` runs every auto a robot declares, and "Do
  nothing", on each alliance from its own start for the 15 s of autonomous, and fails, saying where
  and why, if one throws, has a move `REFUSED`, leaves the field, or can't start ([autos](autos.md#test-them)).
- **The truth:** `sim.simulation()` knows where the robot really is (`truePose()`), and can move
  it, tilt it, or hit it.
- **Faults and scenarios:** what can fail in the simulated world while it runs (the catalog,
  `FaultCatalog` in `lib/simulation`, bound to the robot's parts in `com.michaelgrundvig.frc.runtime.sim.Faults`),
  applied as the Pitwall page applies them: `sim.components().apply(Setting.of(FaultCatalog.GYRO,
  FaultCatalog.UNPLUGGED))`, and `undo` for one that lasts. A scenario is a named set of them
  (`com.michaelgrundvig.frc.runtime.sim.Scenarios`), the ones `./gradlew run -Pscenario=<name>` starts.
  `sim.build(Scenarios.named("camera-low"))` builds the robot as `./gradlew run` does, with the
  scenario's sensor faults on a season-worn robot's sensors (`SensorErrors.TYPICAL`, where
  `sim.robot()` starts from `NONE`), and `sim.start(scenario)` makes what happens in it happen (a
  collision, the gyro unplugged) at the moment the test chooses. `FaultsTest` and
  `CameraFaultsSimTest` show each fault changes what the robot's code reads, and undoing it puts
  that back; `ScenarioSimTest` runs every scenario on the robot. A new fault goes in
  `FaultCatalog`, bound in `Faults`, with such a test; a new scenario goes in `Scenarios.ALL`,
  with a test in `ScenariosTest` that it does to the simulated world what it says.
- **What dashboards see:** `sim.telemetry()`; and the Pitwall page, `sim.pitwall()`: its status,
  actions, tuning, and experiments, as the page uses them. The page's own requests go over HTTP to
  `sim.pitwall().port()` (use `127.0.0.1`); `SteeringExperimentSimTest` drives an experiment the
  way the page does, its event stream and all. Status items are read only when a page watches or
  asks, so ask first (`sim.pitwall().status().requestRead()`). The experiments' deadman runs on
  the robot's clock, so it steps with the loops: send the page's heartbeats every so many loops,
  not on a timer.

### The monkey

`MonkeySimTest` puts a monkey at the controls (`Monkey`): the guide's robot and a team's two 2026
robots (Beta and Delta), each with its own controls, play a 153 s match of random input from a
seed. It presses, holds, taps and releases every button the robot's bindings record, wanders both
gamepads' sticks, schedules and cancels each joint's moves and outputs (now and then a target that isn't a number) and the
drivetrain's moves (to a pose, a heading, along keyframes, at a speed until something holds),
disables the robot as a match, a brownout and the drive team do, at any moment, and switches it
between autonomous and teleop.

Its faults come from the robot's definition, so a robot handed to it gets chaos for its own parts
with no test code: each sensor tripped, blinded, silenced or its wire cut; each joint's motor
controllers silenced or restarted, its encoder frozen, slipped or unplugged, the joint jammed, each
follower off the CAN bus alone, and each motor's PDH breaker tripped (its controller off the bus
without power, a leader's taking its joint with it); and everything the simulator's controls can fail, the drivetrain's modules, the gyro
and the backup IMU, each camera and every camera at once, the robot shoved. Each is mended later.
What the simulator can't fail yet, a module's motor off the bus or the battery sagging, it doesn't
([simulator](simulator.md#what-the-simulator-cant-make-fail-yet)).

After every loop it checks what must always hold, and nothing may escape the robot's loop. Of the
joints (`JointInvariants`): nothing that isn't a number reaches a joint's IO, a joint that doesn't
know where it is isn't run to a position, nothing drives a joint on toward a tripped limit switch,
no joint goes into a closed keep-out region past its band, every move ends by its deadline, a
jammed joint run at an output is stalled by its gate, and a motor that draws nothing while its
joint works is flagged, and none that draws. Of the drivetrain (`DriveInvariants`): every drive
move ends by its deadline. Of where it thinks it is (`LocalizationInvariants`): odometry turns on
the gyro, the backup IMU while only it reads, or the wheels, and the robot says which; with every
camera unplugged it's soon not ready to score, nor tracking tags; and the estimate moves in a loop
no farther than its wheels' travel and what the images it accepted can move it by their gate.

It runs three seeds a robot, about 4 s each for the guide's robot (7 s for the first, as the JVM
warms) and 1.2 s for Beta's and for Delta's. A failure names its seed, its loop, the monkey's last
doings and the test that ran it, and the seed runs the same every time:
`./gradlew test --tests com.michaelgrundvig.frc.runtime.MonkeySimTest -PmonkeySeed=17` runs it again, alone, and
`-PmonkeySeeds=50` runs seeds 1 to 50, for a longer hunt.

## Rules that keep tests trustworthy

- **The same run every time.** The clock moves only when stepped, and the simulator's IMUs' noise
  and its cameras' (one stream per camera) come from fixed seeds, each its own stream of the
  world's seed (`Seeds`), so a run repeats exactly. A test with cameras still checks against a margin, never an exact
  value: a change anywhere else moves the noise it draws. Never `Thread.sleep` to wait for robot
  time; step the loop. To wait for the network (an answer, an event on a stream), wait on it with
  a timeout, a loop at a time; to show nothing else arrived, send a change and check it's the next
  thing to.
- **Recorded behavior stays recorded.** Some behavior is recorded, row by row, and must come out
  the same, numbers within a millionth. Each recording guards its own part, and none another's:
  - `EstimatorGoldenSimTest` (`estimator-golden.csv`) guards the pose estimator. It's fed
    measurements already solved (driving, a misread tag, a collision, an unfelt shove, a tilt), so
    it never reaches the camera path.
  - `CameraGoldenSimTest` (`camera-golden.csv`) guards the camera path up to the estimator: four
    cameras' raw tag images, each solved by `LocalizationService` as the robot was when it was taken
    (its heading, attitude and turning then), with a camera's mount doubt and a task's tags, and
    every image's solve and every measurement's verdict recorded.
  - The joints' recordings guard what each joint's motor controllers are told and what the joint
    does in the simulator (`JointRecordings`): `JointsRecordedSimTest` for the runtime tests'
    joints, `ExampleRobotRecordedSimTest` for the guide's robot, `BetaRecordedSimTest` for the Beta
    robot and `DeltaRecordedSimTest` for the Delta robot each record their joints' doubts and
    refusals, every setting their TalonFXs, CANcoders and SPARKs are told (wired as their robots have them, and otherwise), and a session
    in the simulator, every second loop. `SparkRequestsRecordedTest` records what a joint on a
    SPARK asks of it each loop: its setpoint, its slot, and the feedforward the robot sends with it.
    `JointStagesRecordedSimTest` records what holds a joint beneath its commands and how it finds
    where it is, in scenes of the runtime tests' joints: its interlocks, its limit switches, its
    homing, and an absolute encoder the robot reads, with what each joint's IO is told, what holds
    it and its state, every second loop.

  When a change is meant to move one, run its test with the environment variable `GOLDEN_UPDATE=1`
  to record it again, and commit the recording with the change, saying why it moved.
- **No test runs past two minutes.** A hung test fails instead of hanging the build. A test that
  really needs longer says so with `@Timeout`.
- **Tests run in `build/test-work`.** What simulated devices write (CTRE's `ctre_sim/`, for one)
  stays out of the project. To read a project file, start from the `frc.projectDir` system
  property, as `DashboardLayoutTest` does, and declare the file an input of the test task in
  `build.gradle`, so Gradle knows the tests depend on it. A quick build (`-Pquick`) skips tests
  whose inputs haven't changed, so after a change to an undeclared file it would call them up to
  date, and skip them.
- **Coverage has a floor.** The build fails below the floors in each project's
  `coverage-floor.properties`. They catch code with no tests at all; they are not a target to
  chase.

## Would the tests notice? Mutation testing

Coverage says which lines the tests ran. It doesn't say whether a test would fail if a line were
wrong. Mutation testing asks that directly: PIT changes the compiled code a little at a time (a `<`
to a `<=`, a `+` to a `-`, a returned value emptied, a call removed) and runs the tests that cover
each change. A change the tests catch is killed. One they don't has survived, and marks a line the
tests run without checking.

It runs on the robot model, in its own build (`model/`), which everything else is built from:

```
cd model
./gradlew pitest
```

It takes about six minutes, so nothing runs it but that task: not `check`, not `ci`. While working
on one class, name it, and a run is much shorter:

```
./gradlew pitest -PpitClasses='com.michaelgrundvig.frc.world.robot.Interlock*'
```

The report is
`model/build/reports/pitest/index.html`, each class's source with its surviving changes marked
on their lines. As of this writing: 4,048 changes, 87% killed, on 97% line coverage. Most survivors
are a boundary (`<` for `<=`) or a sign in geometry, where a test checks a value well inside a
range and none checks the edge.

Read it for what to test next, not as a number to raise: a survivor in a tolerance or a log message
is often not worth a test, and one in a limit or a comparison usually is.

## Is the estimator honest? The consistency study

The pose estimator reports how uncertain it is, and the χ² gate, the scoring gate's uncertainty
limit, and any later bound on its error all trust that number. A claim of 3 mm while the robot is
10 cm off is worse than no claim at all. The simulator knows where the robot really is, so it can
check: `./gradlew consistencyStudy` drives the whole simulated robot through ten runs, twelve
seeds of each, and compares every loop's estimate with the truth (`frc.robot.studies`:
`ConsistencyStudy`, `ConsistencyRuns`, `Consistency`), one robot at a time (WPILib's simulation is
one per program). The report is in `build/studies/consistency/`.

**Memory.** PhotonVision's camera simulation allocates its frames in native memory, which only
Java's garbage collector frees. Two settings keep that in check:
- **The simulated cameras render no video** (`VisionSim`). Their raw and processed streams are off,
  and what they detect doesn't depend on them.
- **The build gives every test and simulating process a 32 MB young generation** (`-Xmn32m`,
  `build.gradle`), so the collector runs often enough to free those frames.

Measured on three driving runs at twelve seeds (2026-10-02):

| | Wall time | Peak memory |
|---|---|---|
| Before | 145 s | 4.8 GB |
| With video off | 40 s | 4.8 GB |
| With both | 38 s | 1.0 GB |

Every loop came out bit-identical (fingerprint `26ff4b4fd6ee4fc1`).

The runs, on the simulator's season-worn robot (`SensorErrors.TYPICAL`) with four cameras:

| Run | What happens |
|---|---|
| `driving` | The route in view of the tags, there and back, then 3 s standing |
| `standing` | The route once, then 20 s standing still |
| `turning` | 5 s standing, two turns in place, 5 s standing |
| `gyro-scale` | The gyro-scale procedure (five turns), then standing: 40 s |
| `collision` | The `collision` scenario partway through the route |
| `gyro-lost` | 5 s standing, a turn in place on the gyro, the gyro unplugged (`gyro-unplugged`), a turn back on the backup IMU, 5 s standing |
| `camera-low` | The `camera-low` scenario (the first camera 4 in low), the route four times, then 3 s standing |
| `camera-turned` | The `camera-turned` scenario (the first camera turned 2°), the route four times, then 3 s standing |
| `camera-turned-4` | The first camera turned 4° on its mount, the route four times, then 3 s standing |
| `tag-moved` | Tag 26, on the blue hub's face toward the route, moved 4 in along the field (the simulator's Field fault), the route four times, then 3 s standing |

A scenario's sensor faults are applied through the simulator's controls before the robot's first
loop, as `./gradlew run -Pscenario=<name>` applies them, and its events when the run's script
says; a knocked camera is knocked from where the seed mounted it.

Each seed draws its own: noise in every camera and IMU, an error for each camera's mount (along
and about each axis), one for each tag (along each axis: the cameras see a tag where it is, the
robot believes the layout), and the gyro's and the backup IMU's scales. Each kind of error, and
each camera and tag, draws from its own stream of the seed, so no two seeds share a draw. They're
drawn from Gaussians at the spread the estimator's configuration claims (`EstimatorConfig`'s
`cameraTurnStdDevDegrees`, `tagPlacementStdDevMeters`, `headingStdDevDegreesPerTurn`,
`backupHeadingStdDevDegreesPerTurn`), each mount 3 mm off along each axis, within the 5 mm of
`cameraShiftStdDevMeters` (a stop-gap that also covers a single tag's solve coming out short:
[below](#what-the-simulated-cameras-repeat)), and the vision noise model halved, a little wider than
the simulated cameras' noise: an estimator honest about its own claims reads a NEES near its number
of dimensions, or a little below. Wider or narrower errors than it claims show it over- or
under-confident, which is what the options are for.

| Option | Default | What |
|---|---|---|
| `--runs=A,B` | all | Which runs |
| `--seeds=N` | 12 | Runs of each, each with its own draws |
| `--seed=N` | 1 | The first seed |
| `--mount-error-mm=M`, `--mount-error-degrees=D` | 3, `cameraTurnStdDevDegrees` | Each camera's mount off along and about each axis (standard deviations) |
| `--tag-error-mm=M` | `tagPlacementStdDevMeters` | Each tag off the layout along each axis; 0 for an exact field |
| `--gyro-scale-error=F`, `--backup-scale-error=F` | `headingStdDevDegreesPerTurn` / 360, `backupHeadingStdDevDegreesPerTurn` / 360 | Each IMU's scale off by this share |
| `--noise-scale=F` | 0.5 | The vision noise model scaled; 1 is the model as shipped. The simulated cameras are quieter than half: at a quarter, the calibrated runs read a position NEES of 1.3 to 2.5, and still a heading NEES within its band |
| `--heading=W` | `measurements` | What corrects the heading beside the gyro (`EstimatorConfig.headingCorrection`): `measurements`, `doubt`, or `every`; see [three ways to correct the heading](#three-ways-to-correct-the-heading) |
| `--out=DIR` | `build/studies/consistency` | Where the report goes |

The console prints each seed's draws and numbers, so a run's result can be traced to the seeds
behind it, and ends with a fingerprint of every loop's error and claim, to the bit: two studies
with the same options print the same one, whenever and wherever they run. (The simulator's
clock stays paused between runs, and its cameras keep the robot's poses by the time since their
world began, not by the clock's own count, which starts wherever the computer's uptime stood and is
too large for a double to keep to a nanosecond: before, two identical studies differed in their
last digits.)

### Reading the report

`summary.md` has a row a run. Each column answers a question:

- **NEES** (normalized estimation error squared): the error weighted by the inverse of the
  covariance the estimate claimed. An honest estimator averages its number of dimensions: 2 for
  position, 1 for heading. Far above, it's overconfident: sure of an estimate that's off. Below,
  it's timid: it trusts its measurements less than it could, which costs accuracy but misleads
  nobody.
- **Above, below:** at each loop, the NEES averaged over the seeds falls inside a 95% band (1.03
  to 3.28 for position with 12 seeds) if honest, so about 2.5% of loops land above it and 2.5%
  below. Many above is overconfidence at those moments, even when the average looks fine.
- **Outside the ellipse:** how often the position error falls outside the claimed covariance's
  99.9% ellipse (its NEES past 13.8, χ² for two dimensions). Honest is a thousandth of loops.
- **Past 3.72 σ:** how often the error goes past 3.72 times the claimed spread along its long
  axis: the circle a bound on the error, such as a scoring gate's, would use. It holds the ellipse,
  so it's past less often; the ellipse also catches an estimate too sure along its short axis.
- **Allowed:** the most of either an honest estimator shows, 999 times in 1000. A seed's draws
  last its whole run, so a seed that drew a gyro 2.4 σ off can spend a quarter of its loops outside
  the ellipse: the seeds, not the loops, are the sample, and twelve allow 2.9%.
- **Worst:** the most long-axis spreads the error ever was.
- **Mean error, worst error, heading error, claimed σ:** accuracy beside honesty. An estimator
  that got honest by getting worse is no improvement.

Below it, two more tables:

- **The mount check:** for each camera, how many seeds' mount checks doubted it at some loop (its
  measurements counting for less), the most it was doubted, and how many seeds ended with it out of
  calibration (the alert to recalibrate). The camera runs knock the first camera, front-left; a
  good mount check doubts it and leaves the others, front-right beside it, alone.
- **The gates:** how often each check the robot's code makes on the claimed uncertainty would
  pass: calibrating a camera (the pose tracking and sure within 5 cm), judging a camera's mount
  (sure enough, position and heading at the tag's distance, to tell a turned camera from the
  estimate's own error), scoring (`LocalizationGate.SCORING`, sure within 15 cm), solving with the
  gyro's heading (`VisionConfig.maxHeadingStdDevToConstrain`, 5°), and the 2° a robot placed on its
  start claims. An honest uncertainty that's large closes them; the [table
  below](#the-gates-on-the-claimed-uncertainty) shows how often.

`loops.csv` has each run's NEES, error, and claimed spread loop by loop, averaged over the seeds,
for a spreadsheet: it shows when a run goes wrong (during the turns, or standing after them).

What to expect, today (12 matched seeds, the estimator as configured, a camera at each corner of
the robot's frame facing 45° out):

| Run | NEES (2) | Heading NEES (1) | Outside the ellipse | Past 3.72 σ | Mean error | Heading error |
|---|---|---|---|---|---|---|
| driving | 0.86 | 0.56 | 0 | 0 | 1.4 cm | 0.17° |
| standing | 0.92 | 0.26 | 0 | 0 | 1.5 cm | 0.14° |
| turning | 1.02 | 0.60 | 0 | 0 | 2.1 cm | 0.29° |
| gyro-scale | 1.19 | 0.71 | 0 | 0 | 2.3 cm | 0.31° |
| collision | 1.38 | 0.43 | 0.29% | 0.29% | 1.6 cm | 0.16° |
| gyro-lost | 1.09 | 0.57 | 0 | 0 | 2.1 cm | 0.30° |
| camera-low | 0.73 | 0.68 | 0 | 0 | 1.2 cm | 0.17° |
| camera-turned | 3.47 | 6.23 | 5.35% | 4.73% | 2.6 cm | 0.52° |
| camera-turned-4 | 9.39 | 13.73 | 23.43% | 21.22% | 4.6 cm | 0.77° |
| tag-moved | 0.74 | 0.65 | 0 | 0 | 1.3 cm | 0.16° |

The calibrated runs are timid, as before: none is outside its ellipse but in the loops of the
collision. A camera turned 2° or 4° still leaves the estimate surer than it is. The mount check
doubts the camera turned 4° in 9 of 12 seeds (by 5.6 cm) and its front-right neighbour in 4 (1.7
cm), and neither back camera in any run; turned 2°, it doubts the turned camera in none and its
neighbour in one.

The rest of this section is the record of how the estimator came to be honest, and its tables are
from the runs of the time: an earlier estimator, with the robot's four cameras in the rear half
of its frame, two facing forward and two straight back. Where it names the tags a camera saw, or
the camera a check blamed, it is that robot's. That run's table, with the heading corrected by
measurements with a heading of their own:

| Run | NEES (2) | Heading NEES (1) | Outside the ellipse | Past 3.72 σ | Mean error | Heading error |
|---|---|---|---|---|---|---|
| driving | 1.30 | 0.87 | 0.58% | 0 | 1.7 cm | 0.29° |
| standing | 2.17 | 0.68 | 0 | 0 | 1.6 cm | 0.29° |
| turning | 1.52 | 0.31 | 1.83% | 0 | 1.5 cm | 0.45° |
| gyro-scale | 1.61 | 0.33 | 3.55% | 0 | 1.4 cm | 0.66° |
| collision | 1.58 | 0.95 | 0.43% | 0.43% | 1.9 cm | 0.28° |
| gyro-lost | 1.60 | 0.28 | 3.62% | 0 | 1.4 cm | 0.36° |
| camera-low | 1.10 | 1.15 | 0 | 0 | 1.6 cm | 0.28° |
| camera-turned | 3.64 | 5.65 | 2.87% | 1.58% | 2.7 cm | 0.62° |
| camera-turned-4 | 10.70 | 21.48 | 29.25% | 27.55% | 4.8 cm | 1.18° |
| tag-moved | 1.69 | 1.47 | 0.93% | 0.67% | 1.7 cm | 0.28° |

The calibrated runs are near their NEES or below it, timid rather than overconfident: the floor
for what a camera repeats is sized for the worst of what it allows, and the noise model is a
little wider than the simulated cameras. Standing reads 2.17, inside its band. Standing still, both
front cameras see tag 23 and both back cameras tag 28, so two cameras' fixes share each tag's
place on the field: when each camera's shift held the tags' placement as well as its mount, the two
averaged it down as if each had its own, and standing read 3.42, above its band in 72% of its
loops. The estimator now considers each tag's placement on its own (`tagPlacementStdDevMeters`),
the same for every camera that sees it and every time it's seen, and each camera's shift
(`cameraShiftStdDevMeters`) is what's its own: its mount, and for now a stop-gap for its solve
coming out short ([below](#what-the-simulated-cameras-repeat)).

One seed is still overconfident where the robot stands at its start, and it's its tags. Seed 2's
gyro reads 0.38% high and its two front cameras are turned the same way (0.32° and 0.48°), but
drawn exact, neither changes much: its turning run's NEES 8.0 and 6.9, against 7.8. Its tags
drawn exact, it reads 1.2, and none of its loops is outside the ellipse. From the start, only tags
31 and 32 are in view, both back cameras see both, and seed 2 drew tag 31 3.4 cm off along the
field (3.4 times what `tagPlacementStdDevMeters` allows) and tag 32 1.6 cm: every fix from there
sits 2.5 cm off, 3.5 times what two tags each 1 cm off allow. The simulated cameras' own 0.5 to
0.8 cm at that distance ([below](#what-the-simulated-cameras-repeat)) and its mounts' 0.3 add to
it, 3.6 cm in all against a claimed 0.9, and the mounts are what tip it out: drawn exact, it stays
inside its ellipse (a NEES of 5.4). Outside it in 22% of the turning run's loops and 43% of the
gyro-scale procedure's and the lost gyro's, it puts those two past the 2.9% twelve seeds allow;
the study reports it, and no test pins it ([keeping it honest](#keeping-it-honest)). No estimator honest about tags 1 cm off can hold a pair 3.5 times that inside its 99.9%
ellipse; fifty seeds, below, show it's the draw and not the model.

Fifty seeds (`--seeds=50`, run as four studies of 12 or 13 seeds and their reports combined) put
every calibrated run but one within the 1.48% fifty allow: driving 1.05 (0.14% of loops outside
the ellipse), standing 1.28 (none), turning 1.27 (0.44%), collision 1.03 (0.28%), gyro-lost 1.37
(0.87%), camera-low 0.84 (none), tag-moved 1.56 (1.07%); the 2° camera reads 3.47 (3.41%) and the
4° 10.26 (26.55%). Seed 2 is one draw in fifty there. The gyro-scale procedure reads 2.68, with
3.03% outside: beside seed 2, seeds 31 and 38, whose gyros read 0.55% and 0.66% low (2.0 and 2.4
times what `headingStdDevDegreesPerTurn` allows), end its five turns 6.7° and 11.2° off, claiming
4.9°. No heading of a multi-tag solve comes to correct it from the start's two tags (drawn exact,
one does, and the heading comes back), and a fix's lever on the heading is linear in it: 11° off
at 3.6 m, what the line leaves out is 7 cm along the tags, about what the estimate is off (10 cm)
where it claims a centimeter. The estimator before this table does the same on those two seeds (a
NEES of 16.8 and 55.8): it isn't the tags' doing, and it isn't fixed here. Without them, the
procedure has 0.89% of its loops outside.

Standing reads 1.28 over fifty, below its band (1.48 to 2.59): timid, as the other calibrated runs
are. At its end, along its tags, the estimate is off by the tags' mean placement and 3.5 mm more,
while it claims the tags, its cameras' shifts and 6 mm of its own: the doubt the route left, from
wheels better than `translationStdDevPerMeter` and `translationStdDevPerRadian` claim, that fixes
sharing two tags can't wear down (at a third of each, it claims 3.7 mm of its own, and standing
reads 1.75). The estimator before read 2.02 there, the tags counted at half their weight (each
camera's shift, averaged four ways) beside that same doubt.

The collision's exceedances are the loops of the hit, before the jolt is read, and the few after
it while the cameras pull the estimate back across a knock of 0.6 m, more than the 25 cm
`collisionStdDevMeters` allows. A tag 4 in off the layout, ten times what
`tagPlacementStdDevMeters` allows, stays within the allowance: one tag among several in view. A
camera turned 2° is eight times what `cameraTurnStdDevDegrees` allows, and 4° sixteen: the
estimator can't tell either from a heading error, so its heading grows sure of a wrong value.
Catching that is the mount check's job; how well it does depends on what may correct the heading,
below.

#### What the simulated cameras repeat

With every error drawn at zero (no mount, tag or gyro scale off), the simulated cameras still place
their tags short, the same in every image: 5 to 8 mm from the start's tags 31 and 32, at 3.6 m,
and 1 to 3 mm from the standing run's, at 2 m. It's a single tag's solve from noisy corners: with
noiseless corners it's gone. PhotonVision's own `solvePNP_SQUARE`, on the template's sensor with
the simulator's corner noise, shows how it hangs on the view:

| Tag seen | 2 m | 3.6 m | 6 m |
|---|---|---|---|
| Head-on | 4.5 mm short | 14.6 mm | 40 mm |
| 30° off its face | 0.2 mm | 0.8 mm | 2.9 mm |
| From 0.5 m above its axis | 0.2 mm | 4.0 mm | 30.8 mm |

It grows with the corner noise and about with the distance squared, and looks like a single tag's
ambiguity near face-on. It's a real error the estimator doesn't model: one-sided, hanging on range
and angle, and shared by every camera that sees the tag face-on. `cameraShiftStdDevMeters` covers
it for now, as a **stop-gap**: 5 mm, against the 3 mm of mount the study draws, a constant for each
camera that models it poorly. At 3 mm, the mount's alone, the turning run sat at exactly its 2.92%
allowance outside the ellipse, the driving run had 1.42%, and the 2° camera rose to 3.80% (2.83%
before tags were considered on their own); at 5 mm, 1.83%, 0.58% and 2.87%. Part of what the extra
2 mm absorbs is seed 2's tag 31, 3.4 times `tagPlacementStdDevMeters` off, seen from the start where
those runs stand. Modelling the short solve itself (by range and angle, for each tag face) is left
to do; until then, measure it on the robot's own cameras ([camera
noise](characterization.md#camera-noise)).

On the same draws, before the estimator considered each tag's placement on its own, the table
read: driving 1.37 (0.09% of loops outside the ellipse), standing 3.42 (above its band in 72% of
loops), turning 1.57 (2.09%), gyro-scale 1.87 (4.72%), collision 1.68 (0.45%), gyro-lost 1.83
(3.94%), camera-low 1.19 (none), the turned cameras 3.78 (2.83%) and 10.71 (29.68%), tag-moved 1.82
(1.23%); its errors were the same to a millimeter, and its gates' pass rates to 0.2%.

Numbers from before that, in older commits, aren't comparable: the simulated drivetrain
carried a robot of about 39 kg, not the configured 68, so it sped up 75% faster and drove the
runs' sticks along another path ([stand-ins](stand-ins.md)); before that, the simulated cameras
stamped each image up to a loop late ([below](#the-simulators-late-stamps)), and before that the
seeds shared draws (seed 2's first camera was seed 1's second, through `Seeds.mix` adding the seed
to what it mixed) and the mounts and tags were drawn narrower than the configuration claims.
Since, each seed's IMUs and cameras draw from streams of their own (they were offsets from the
seed: seed 1's gyro drew what its first camera did, and each seed's gyro what the seed before it
gave its gyro's accelerometer), and so do the study's own draws (each seed's camera mounts were
the seed before it's tag offsets, its kinds of draw mixed in as 1, 2, and 3), and the table moved
with them: drawn the old way, it read driving 1.08, standing 1.83, turning 1.44, gyro-scale 2.10
(0.09% outside the ellipse), collision 1.40, gyro-lost 1.71, camera-low 0.99, the turned cameras
3.28 and 9.83, tag-moved 1.71. The rest of this section's numbers were drawn the old way, and
with each camera's shift (then 1 cm) holding the tags' placement too.

On the same simulator, with the old draws, the estimator before that table doubted a quarter of its wheels' travel
whenever a drive motor was at its current limit, for most of a second each time the robot got up to
speed or changed course. In NEES, it and that table: driving 1.01 and 1.08, standing 1.80 and 1.83,
collision 1.28 and 1.40, camera-low 0.90 and 0.99, tag-moved 1.56 and 1.71, the turned cameras 2.87
and 3.28, 9.00 and 9.83; the runs that turn in place didn't move. The doubt it took off fell where
it wasn't needed: in the tenth of the driving run's loops where the estimator before claimed more
than 5 cm, averaged over the seeds, it claimed 7.6 cm (11.0 at most) and this one 4.2 cm (5.4), 1.5
cm off there; over the whole run, 2.7 and 2.2 cm. In the collision run, the doubt the drive motors'
limit added after the hit was what covered the knock's last 15 cm: without it the estimate is past
its ellipse for a few loops more, 0.65% of them in all, under the allowance. The turned cameras,
eight and sixteen times what the configuration allows, grow a little more overconfident: the doubt
at the current limit had covered some of their error too. For a while, the wheels' disagreement
counted as slip only where one wheel alone explained it (driving 1.13, the 4° camera 12.56, the
mount check blaming its healthy neighbour), until two wheels slipping together, which that counted
as noise, showed it wrong; it was reverted.

### Three ways to correct the heading

The gyro owns the heading; what else may correct it is `EstimatorConfig.headingCorrection`, and
the study runs each (`--heading=`):

- **`measurements`** (`HEADING_MEASUREMENTS`, the default): only measurements with a heading of
  their own, multi-tag solves, with all three of their rows. A position solved with the gyro's
  heading carries the heading's doubt but never turns it.
- **`doubt`** (`HEADING_MEASUREMENTS_AND_MOUNT_DOUBT`): those, and a camera the mount check doubts
  has the doubt added to what it repeats (its turn, at its tags' distance, and its shift), so its
  headings pull the heading only so far, however many it sends.
- **`every`** (`EVERY_FIX`): every fix corrects the heading too, through its tags' distance and
  how the position's error has come to depend on the heading's: a full Kalman update.

Twelve matched seeds of each (a, b, c in that order), run while the estimate still doubted a
quarter of the wheels' travel whenever the drive motors were at their current limit. The default
(a) has been run again since, in the table above, with the estimate doubting its wheels only on
evidence that they slid: the calibrated runs moved by at most 0.15 in NEES (the tag-moved run's)
and 0.1 cm in mean error, and the turned cameras' NEES rose by 0.4 and 0.8. The mount check still
finds the 4° camera first (11 seeds, 7.4 cm, against its neighbour's 12, 5.7 cm; were 7.0 and
5.2), and at 2° doubts it in 3 seeds (0.5 cm), its neighbour in 1. (b) and (c) weren't run again:
the comparison below is as it stood.

| Run | NEES (2) | Heading NEES (1) | Outside the ellipse | Past 3.72 σ | Mean error | Heading error |
|---|---|---|---|---|---|---|
| driving | 1.01, 1.01, 0.94 | 0.54, 0.54, 0.81 | 0, 0, 0 | 0, 0, 0 | 1.5, 1.5, 1.4 cm | 0.20, 0.20, 0.17° |
| standing | 1.80, 1.80, 1.75 | 0.37, 0.37, 1.17 | 0, 0, 0 | 0, 0, 0 | 1.2, 1.2, 1.2 cm | 0.16, 0.16, 0.17° |
| turning | 1.44, 1.44, 1.56 | 0.68, 0.68, 0.71 | 0, 0, 0 | 0, 0, 0 | 1.5, 1.5, 1.6 cm | 0.68, 0.68, 0.21° |
| gyro-scale | 2.10, 2.10, 1.87 | 1.05, 1.05, 0.54 | 0.09%, 0.09%, 0 | 0.00%, 0.00%, 0 | 1.8, 1.8, 1.4 cm | 1.36, 1.36, 0.19° |
| collision | 1.28, 1.28, 1.14 | 0.64, 0.64, 0.62 | 0.41%, 0.41%, 0.42% | 0.41%, 0.41%, 0.42% | 1.7, 1.7, 1.6 cm | 0.21, 0.21, 0.15° |
| gyro-lost | 1.71, 1.71, 1.75 | 0.46, 0.46, 0.54 | 0, 0, 0 | 0, 0, 0 | 1.5, 1.5, 1.6 cm | 0.46, 0.46, 0.20° |
| camera-low | 0.90, 0.90, 0.79 | 0.78, 0.78, 0.91 | 0, 0, 0.00% | 0, 0, 0 | 1.4, 1.4, 1.3 cm | 0.22, 0.22, 0.16° |
| camera-turned | 2.87, 2.84, 3.18 | 4.99, 4.97, 8.41 | 1.43%, 1.43%, 2.83% | 0.58%, 0.58%, 2.23% | 2.6, 2.6, 2.6 cm | 0.51, 0.51, 0.52° |
| camera-turned-4 | 9.00, 8.95, 10.62 | 19.56, 19.62, 30.70 | 25.54%, 25.52%, 28.62% | 24.76%, 24.74%, 28.00% | 4.8, 4.7, 5.0 cm | 1.00, 1.01, 1.01° |
| tag-moved | 1.56, 1.56, 1.40 | 1.97, 1.97, 1.10 | 1.25%, 1.25%, 1.23% | 1.01%, 1.01%, 1.11% | 1.8, 1.8, 1.6 cm | 0.31, 0.31, 0.18° |

And whether the mount check finds the turned camera (front-left) and leaves its healthy neighbour
(front-right) alone: seeds that doubted each, the most each was doubted on average, and seeds that
ended with each out of calibration. The back cameras were never doubted.

| Run | Variant | front-left (turned) | front-right (healthy) |
|---|---|---|---|
| camera-turned | `measurements` | 5/12, 1.4 cm; out 4/12 | 1/12, 0.4 cm; out 1/12 |
| camera-turned | `doubt` | 5/12, 1.4 cm; out 4/12 | 1/12, 0.4 cm; out 1/12 |
| camera-turned | `every` | 4/12, 1.3 cm; out 2/12 | 7/12, 1.8 cm; out 6/12 |
| camera-turned-4 | `measurements` | 11/12, 7.0 cm; out 11/12 | 12/12, 5.2 cm; out 6/12 |
| camera-turned-4 | `doubt` | 11/12, 7.1 cm; out 11/12 | 12/12, 5.2 cm; out 7/12 |
| camera-turned-4 | `every` | 11/12, 7.1 cm; out 11/12 | 12/12, 10.1 cm; out 12/12 |

The next twelve seeds (`--seed=13`) agree. Under `every` the healthy camera is doubted in 7 seeds
at 2° (by 1.8 cm) against the turned one's 2 (0.3 cm), and by 9.4 cm at 4° against 7.4; under
`measurements` the turned camera is doubted in 3 seeds at 2° (0.6 cm) against its neighbour's 2
(0.3 cm), and both by 6.0 cm at 4°.

What it shows:

- **On a calibrated robot** all three are honest, `measurements` and `doubt` identically (only the
  healthy front-right is doubted, in 2 seeds of the standing run, by 0.3 cm). `every` is as
  accurate in position, and more accurate in heading wherever the gyro's scale matters: 0.21° off
  after turning against 0.68°, 0.19° after the gyro-scale procedure against 1.36°, 0.20° on the
  backup IMU against 0.46°. Its heading NEES is 0.54 to 1.17, inside the 95% band (0.37 to 1.94)
  in every calibrated run; `measurements` reads 0.37 to 1.05, timid standing still.
- **With a turned camera**, none singles it out. `measurements` lets the turned camera's headings
  turn the robot's, and then the healthy cameras' positions disagree with it: the mount check
  doubts the turned camera a little more than its neighbour (at 4°, 7.0 cm against 5.2), though
  it calls the neighbour out of calibration in half the seeds. `doubt` adds the doubt where the
  mount check puts it, so it's barely different (8.95 against 9.00 at 4°). `every` lets all four
  cameras' positions hold the heading, and the turned camera's pull the healthy neighbour's off
  more than its own: the mount check doubts the neighbour more, at 2° in 7 seeds against 4, at 4°
  by 10.1 cm against 7.1, out of calibration in every seed. Its estimate is no closer on average
  (5.0 cm off at 4°, against 4.8) and less honest (a NEES of 10.6 against 9.0, 29% of loops
  outside the ellipse against 26%), but its worst is smaller (17.3 cm against 27.0). None is honest
  with a turned camera: one turned eight or sixteen times its allowance is an error the
  configuration says can't happen.
- **With a moved tag**, the three are within the allowance, and no camera is doubted.
  `measurements` and `doubt` grow surer of the heading than they should (a heading NEES of 1.97,
  above its band in 23% of loops); `every` stays honest (1.10).

On the lighter drivetrain before, `every` singled out a turned camera (at 4°, 16.5 cm against its
neighbour's 4.2) and `measurements` blamed the neighbour (15.2 cm against 6.6). The heavier robot
drives the same sticks along another path, and the comparison turned over: which camera the
mount check blames depends on how the robot drives as much as on what corrects the heading.

`VisionSimTest`'s mount-check tests run under each, on one route with one seed. For a camera
turned 4°, all three keep three of the original promises: it is out of calibration and counts for
less, the back cameras are never doubted, and it is doubted by more than 8 cm (19 cm under each).
`measurements` and `doubt` keep the fourth too, the last lap a fifth closer than trusting it as
before: 0.5 and 0.4 cm off, against 0.7. `every` breaks it: 0.5 cm off, against 0.5 trusted as
before. Every one of those laps is within a centimeter, because the estimator's own watch on its
cameras takes off a turned camera's miss whatever the mount check is told to allow.
A camera turned 2° stands out in its tag residuals under all three, most under `measurements` and
`doubt` (2.2 times its worst neighbour's, against 1.5 under `every`). Each test says which, and
changes when one does.

One route is a narrow look, and where the cameras sit counts as much as what corrects the heading.
The numbers above are with a camera at each corner of the frame, facing 45° out. With all four in
the rear half of the frame, two facing forward and two straight back, and before the estimator
watched its cameras itself, all three kept the first two promises and broke the other two: `every`
doubted the camera by 6.5 cm, its last lap 3.7 cm off against 4.5 cm trusted as before;
`measurements` and `doubt` by 3.0 and 3.1 cm, their last laps 4.1 and 4.3 cm off against 4.2. With
the seed's noise drawn as offsets from the seed (before its streams were its own), `measurements`
and `doubt` never doubted it, and `every` by 5.7 cm: on one seed, whether they call it out at all
hangs on its draws. There the 2° camera stood out most under `every` (2.5 times its worst
neighbour's, against 1.7). Before the simulated drivetrain carried the robot's mass, the robot drove
the same sticks 75% quicker off the line, and `every` doubted the 4° camera by 9.2 cm,
`measurements` by 5.5, and the 2° camera stood out only under `every`; on the same route, while the
estimate still doubted a quarter of the wheels' travel whenever the drive motors were at their
current limit, `every` doubted it by 2.6 cm, and `doubt` brought the last lap to 5.4 cm; and while
the wheels' disagreement counted as slip only where one wheel alone explained it (since reverted),
`doubt` kept the first two promises and `every` none.

#### The simulator's late stamps

On the simulator's lighter drivetrain, before it carried the robot's mass, an earlier run of this
comparison found `every`'s heading claiming more than it knew: a heading NEES of 1.7 to 6.3 where 1
is honest, the most after the gyro-scale procedure. The simulator was the cause, not the estimator.
PhotonVision's simulation takes each image from where the robot was when the image fell due (less
the camera's latency), but stamped it as of the loop that rendered it, up to a loop (20 ms) later. A
position solved with the gyro's heading at a stamp late by `t` is off just as if the heading were
off by the turn rate times `t`, the same way in every image while the robot turns one way: 1.1° at 2
rad/s and 10 ms. A measurement with a heading of its own isn't taken while the robot turns faster
than `VisionConfig.maxYawRateForHeading`, so `measurements` never let it near the heading; `every`
took it for a heading error, image after image, and grew sure of a heading 0.7° off after the
procedure's spins. `VisionSim` now stamps each image with when it was taken, as a real camera's time
sync does (`CameraTimingSimTest`). `measurements` gained too: a stamp up to a loop late put every
fix where the robot was up to 20 ms earlier, along the way it drove, so stamped right its position
was closer (1.3 cm off driving, against 1.6) and its NEES lower (the collision run's 1.14, against
2.75).

How it was found, under `every`, four seeds of the turning run and the gyro-scale procedure, each
kind of error taken away in turn:

| What the seeds drew | Turning: heading NEES | Gyro-scale: heading NEES |
|---|---|---|
| Every error, as configured | 3.57 | 6.86 |
| Tags exactly on the layout | 4.08 | 7.72 |
| Cameras exactly on their mounts | 3.00 | 6.86 |
| The IMUs' scales exact | 2.81 | 6.45 |
| The noise model doubled (`--noise-scale=1`) | 2.43 | 3.75 |
| No tag, mount, or scale error at all | 3.05 | 7.08 |
| No error, images stamped when taken | 0.01 | 0.02 |
| Every error, images stamped when taken | 1.55 | 0.92 |

With no error drawn, the heading's error averaged the same in every seed, 0.35° over the turning
run and 0.7° over the procedure's: it came from the turning itself. Each fix's error against the
truth, regressed on what a late stamp does to it (the robot's velocity, plus its turn rate times
the lever to its tags), put the stamps 8 to 11 ms late, and within 0.3 ms once stamped when taken.

The innovations couldn't show it. Each kind of fix's normalized innovation (NIS) averaged under its
degrees of freedom, before and after: under `every`, a single tag's position 0.12 to 0.25 of its 2
before and 0.06 to 0.13 after, several tags' 0.36 to 0.83 and 0.10 to 0.59, a multi-tag heading
(given its position) 0.39 to 0.92 of 1 and 0.17 to 0.62. An error of a centimeter or two in each
fix hides inside the noise model's spread; only taken as new in each of hundreds of fixes does it
make the heading sure.

Nor was it what was first suspected. The tags' errors, the same for every camera that sees a tag:
taking them away didn't lower the heading NEES (above), and with a noise model half as wide
(`--noise-scale=0.25`, twelve seeds), `every`'s heading NEES on a calibrated robot is 0.61 to 1.19
with them (`measurements`' 0.48 to 0.99) and 0.33 to 0.62 without: each camera's considered shift
covered them then (each tag's placement is considered on its own since). A lever wrong in sign or
frame: with no error drawn and the images stamped right, `every`'s heading is 0.02° off. The
gyro's scale coupling with the fixes during the procedure's turns: exact scales left it at 6.45.

**What `every` leans on:** images stamped right while the robot turns. With every camera's stamps
off by the same amount, drawn for each seed with a spread of 2 ms, its heading NEES after the
gyro-scale procedure is 0.69 (0.54 stamped right); with a spread of 5 ms, 1.45; `measurements` reads
0.92 either way (on the lighter drivetrain, as is this section's table). So before choosing it,
check the capture times on the robot to within about 2 ms ([camera latency and time
sync](characterization.md#camera-latency-and-time-sync)).

### The gates on the claimed uncertainty

The robot's code acts on the claimed uncertainty in five places, and the report counts how often
each would pass (`Gates`; see [reading the report](#reading-the-report)). Before is the estimator
from before this study (sure of millimeters while centimeters off: a NEES of 17 to 476), on the same
draws, with its configuration as it was, the simulated images stamped up to a loop late, and the
simulator's lighter drivetrain; after is the estimator as it was before it stopped doubting its
wheels at the drive motors' current limit (below), the heading corrected by measurements (a) and
by every fix (c). With `doubt` (b) they're as (a) but for the turned cameras: calibrating 89.0%
and 88.3%, judging a mount 85.4% and 85.1%. Each cell is before → a, c, in percent of loops:

| Run | Calibrating (σ ≤ 5 cm) | Judging a mount (3 m) | Scoring (σ ≤ 15 cm) | Constraining (σθ ≤ 5°) | As placed (σθ ≤ 2°) |
|---|---|---|---|---|---|
| driving | 97.8 → 89.6, 90.1 | 97.1 → 83.8, 85.5 | 99.95 → 100, 100 | 100 → 100, 100 | 100 → 100, 100 |
| standing | 99.3 → 96.6, 96.6 | 99.1 → 93.4, 93.5 | 100 → 100, 100 | 100 → 100, 100 | 100 → 100, 100 |
| turning | 99.9 → 100, 100 | 99.9 → 32.5, 73.5 | 99.97 → 100, 100 | 100 → 100, 100 | 100 → 100, 100 |
| gyro-scale | 97.0 → 100, 100 | 74.4 → 57.6, 95.1 | 99.8 → 100, 100 | 100 → 99.99, 100 | 100 → 73.6, 100 |
| collision | 97.5 → 90.3, 90.3 | 97.2 → 82.7, 82.8 | 99.6 → 99.7, 99.7 | 100 → 100, 100 | 100 → 100, 100 |
| gyro-lost | 99.99 → 100, 100 | 99.99 → 38.3, 73.5 | 100 → 100, 100 | 100 → 100, 100 | 100 → 100, 100 |
| camera-low | 97.1 → 88.8, 89.3 | 96.2 → 84.7, 86.0 | 99.95 → 100, 100 | 100 → 100, 100 | 100 → 100, 100 |
| camera-turned | 96.9 → 89.1, 89.3 | 95.8 → 85.5, 86.7 | 99.95 → 100, 100 | 100 → 100, 100 | 100 → 100, 100 |
| camera-turned-4 | 95.6 → 88.5, 89.1 | 94.4 → 85.3, 86.6 | 99.8 → 99.9, 99.9 | 100 → 100, 100 | 100 → 100, 100 |
| tag-moved | 96.9 → 88.7, 89.3 | 95.9 → 85.3, 86.7 | 99.9 → 99.99, 100 | 100 → 100, 100 | 100 → 100, 100 |

Scoring and solving with the gyro's heading barely change: an honest position is sure within 1.5 to
2.5 cm on average, well inside 15 cm, and the heading's doubt stays under 5°. Calibrating passed in
only 88 to 90% of the loops of the runs that keep driving the route: while the robot got up to speed
or changed course, its drive motors at their current limit, the estimate doubted a quarter of its
wheels' travel and claimed more than 5 cm. Doubting them only on evidence that they slid, (a) passes
in 97.1 to 99.6% of those runs' loops now (driving 97.9, standing 99.6, collision 97.1, camera-low
97.9, camera-turned 98.0, camera-turned-4 97.5, tag-moved 97.7), and judging a mount in 89.1 to
96.5% (92.0, 96.5, 89.1, 93.5, 94.1, 94.0, 94.1); the runs that turn in place are as in the table.
What still fails calibrating is the estimate changing course, its wheels disagreeing with each other
as each reaches its new speed. (c) wasn't run again. Judging a mount is where honesty costs. It
needs the position and heading together within the mount tolerance (5 cm) at the tag's distance, so
the heading within about 0.95° at 3 m, and the gyro's scale allowance
(`headingStdDevDegreesPerTurn`, 1° a turn) puts it past that after a turn or two, until a multi-tag
heading brings it back: with (a), the mount check can judge a third of the turning run's loops and
fewer than three in five of the gyro-scale procedure's. Every fix brings the heading's doubt back
sooner (74% and 95%), and honestly: its heading is 0.2° off on average over either run, against
(a)'s 0.7° and 1.4°. The estimator before passed nearly always because it was sure of a heading up
to 2° off. And after the procedure's five turns, (a)'s heading claims more than a placed robot's 2°
for a quarter of the run. None of these gates is retuned here: a lower allowance once the gyro is
calibrated, a judging distance, or a tolerance are the team's to choose with these numbers.
Considering each tag's placement on its own moved none of them by more than 0.2% of loops, on
today's draws: scoring passes in all of the calibrated runs' loops but the collision's (99.71%,
as before).

### What it proves, and what it doesn't

It proves the estimator's arithmetic keeps its claims honest against the errors the simulator
makes: wheel wear, a gyro's scale and drift, the cameras' noise, mounts and tags within their
tolerances, a collision. It can't prove the real robot's numbers: that the noise model fits the
real cameras, that the gyro's scale is as configured, that the field's tags are within half an
inch. Those are measured ([characterization](characterization.md)), and on a robot the check is
the same idea without the truth: a camera's normalized innovations (NIS), and the error at
surveyed spots. A configured repeated error smaller than the real one shows here too: with
`cameraShiftStdDevDegrees` at 1°, half what tags 1 cm off put in a heading, the gyro-scale run read
a heading NEES of 2.05 and had 4.9% of its loops outside the ellipse (with the images stamped up to
a loop late). Nor can it prove the cameras' capture times: the simulated ones are exact, and a real
camera's are only as good as its time sync, which `every` leans on while the robot turns ([the
simulator's late stamps](#the-simulators-late-stamps)).

None of its runs slides the robot over the floor, and slides are where the estimate is least
honest. `LocalizationSimTest` drives them (pushed sideways, a launch at half grip, the camera blind
and then seeing again), and the estimate recovers in each. What's known not to hold:

- **Wheels spinning past the floor.** The wheels' disagreement with each other (the kinematic
  residual) says how far one wheel slipping put their fitted travel off; two slipping together put
  it off by about twice what it says (2.2 times on a square drivetrain), and neither the gyro nor
  the accelerometer sees a pair either side of the robot spinning slower than the speed
  comparison's tolerance. So a pair spinning is doubted about half as far as it put the estimate
  off: the front pair at 0.5 m/s for a second, 25 cm off, claims 12.6 cm; at 1 m/s, 50 cm off,
  56 cm, the accelerometer seeing the rest. One wheel spinning claims about what it put the
  estimate off. Nor can the simulator say more: its chassis moves as the wheels' best fit says
  (`KinematicPhysics`, a known gap in its [truth suite](simulator.md#how-far-to-trust-it-the-truth-suite)),
  so in it, wheels that disagree can't put the robot anywhere their fit
  doesn't, and a rule for telling which disagreement is slip, tuned against it, would fit the
  simulator's own guesses. Wheel slip needs a simulator whose physics makes it (one modelling the
  robot as a rigid body on its tread), and logs from a real robot, before it can be judged better.

- **A slide's first tenth of a second.** The slip check sees a slide only once the wheels and the
  accelerometer are 0.3 m/s apart, so the doubt starts behind it. With the camera's last fix still
  recent, the scoring gate lets through up to 3 loops while the estimate is 15 to 20 cm off as a
  push or a launch at half grip gets going.
- **Worn tread, far without a camera.** Tread worn a little unevenly puts the wheels' reckoning
  off by a share of every meter, where the estimator allows for an error growing with the root of
  the distance (`translationStdDevPerMeter`). Three seconds at full stick with no camera, the
  season-worn simulated robot is 16 cm off claiming 10. Before the slip check, the estimate doubted
  a quarter of the wheels' travel at the drive motors' current limit, which covered it in a launch
  (and nowhere else).
- **The end of a long slide.** What the speed comparison let go of while a slide held (0.15 m/s
  for each second) it under-reads as the slide slows. Pushed at 3 m/s for two seconds, the camera
  seeing, the last 0.6 m/s of the slowdown reads as no slide, and for 4 loops the estimate is 9 to
  11 cm off claiming 2 cm, trailing the camera's fixes.
- **After a long slide.** When it stops, the comparison finds what it let go of the other way,
  and reads it as a slide while the robot stands still, until it's let go of again, a few
  seconds: the estimate claims 13 cm while 3 cm off, and the scoring gate stays shut longer.
  Honest, but slower.
- **A slide that goes on.** The doubt a slide adds keeps its direction for
  `slipCorrelationSeconds` (0.5 s), and then grows as if each half second's were apart from the
  last, while the robot's error goes on growing one way. While the slip check took its tolerance
  off every slide, that showed: pushed for seconds with a camera seeing, the estimate cycled from
  tracking, sure of a few centimeters while 15 to 40 cm off, to lost and recovering, about every
  0.3 s, and pushed at 1.5 m/s for four seconds, a 10 fps camera seeing again halfway through, the
  scoring gate let through 24 loops with it more than 15 cm off, 41 cm at worst. Counting a slide
  in full, the same runs don't cycle (3 loops, at the push's start, 20 cm), but nothing keeps the
  claimed uncertainty from shrinking below what the cameras' disagreement says after a recovery,
  or holds a slide's doubt for as long as it lasts: neither is built.
- **A push that keeps the robot speeding up, standing.** The slip check learns the accelerometer's
  bias from a reading that holds steady for 2 s with the wheels and the gyro still, whatever its
  size, so gravity leaking into an IMU tilted on its mount is learned, not added up as a slide:
  unchecked, tilted 10°, standing 3 s and then driving 5 s, the estimate claimed 4.0 m and then
  10.2 m (7.4 and 13.4 m on a vertical plate); now 1 cm and then 13 cm, as with a level IMU, and an
  alert says to check the orientation. A push the wheels don't see that holds the robot speeding up
  steadily for 2 s reads the same, and is learned too, after which the slide the comparison holds
  stops growing with it: pushed at a steady 0.8 m/s² (which the simulator's accelerometer reads at
  about 0.45, the push stepped once a loop), 0.45 m/s² was taken in two seconds on. One that builds
  isn't: pushed from standing at 0.5, 1 and 2 m/s³ up to 2 m/s², the bias took in 0.05 m/s² at most,
  the noise, where learning any reading within 1 m/s² of the bias took in 1.38, 0.65 and 0.29. At
  worst the estimate was 10, 7 and 4 times as far off as it claimed disabled (34, 22 and 13
  enabled), against 105, 20 and 7 (352, 67 and 20): still far off, since the comparison sees a slide
  only past its tolerance (above), and the simulator's accelerometer reads these stepped pushes at
  about half their size.
- **Before the bias is learned.** For the first 2 s standing after the robot starts, or after its
  IMU comes back, the accelerometer's bias isn't learned. A checked one's is taken as none: a bias
  up to `speedBiasMetersPerSecSq` can't build up, and the check passes only one within 0.2 m/s², but
  a push reads that much smaller (pushed half a second after starting, the camera blind, the scoring
  gate let one more loop through off, 3, than once the robot had stood long enough), and a bias past
  0.3 m/s² shows as a slide until it's learned (0.6 m/s², standing: 0.9 m/s, claiming 1.5 m). An
  unchecked one isn't compared with the wheels at all until then: launched at half grip half a
  second after starting, the robot was 2.4 m off claiming 8 cm. On the field, a robot stands far
  longer than that before the match.

### Keeping it honest

`EstimatorConsistencySimTest` runs four of the runs (driving, standing, turning, the gyro-scale
procedure), six matched seeds each, on every build, about 80 s. It's a guard, not a verdict: it fails
only when the estimate is grossly wrong about itself.
- **Position NEES:** between 0.2 and 10, where honest is 2.
- **Heading NEES:** between 0.1 and 5, where honest is 1.
- **Long axis:** past 3.72 σ in at most 10% of loops.
- **Accuracy:** a mean error under 2.5 to 3.5 cm, as today.

The estimator before this study read 17 to 476 on these draws. The test doesn't pin an exact NEES
or a share of loops outside the ellipse, and nothing else does.

**Why not tighter.** The draws behind every number here, such as camera noise, tag placement and
gyro scale, are guesses until real cameras, IMUs and fields are measured. A test held to a number
those guesses set fails whenever a guess moves, in either direction, and judging changes that way
made work swing back and forth. So the study is the closer look, as a report.

**Before and after.** Run the whole study before and after a change to the estimator, its
configuration, or the vision pipeline, and keep the tables here up to date. A change that moves the
report is a question to look into, not a failed build.

## Configuration is tested too

A bad configuration fails the build, never a robot. Every robot in `Robots.all()` is built and
checked by the tests `deploy` runs first:

- **Its values:** each record checks its own when built, and reports every problem at once
  (`RobotConfigTest` and each record's own test).
- **Its tuning:** every value a mechanism lets the pit tune starts within its limits
  (`RobotTuningTest`).
- **Its register:** every configuration value, and every value a mechanism tunes, has a row in
  [docs/values.md](values.md) saying what kind it is and where its number came from, and no row
  names one that's gone (`ValuesRegisterTest`).
