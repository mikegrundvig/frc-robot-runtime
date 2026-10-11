# FRC Robot Runtime

The robot program every robot shares, which no team writes: the loop, the drivetrain, the
mechanisms built from a robot's definition, vision and localization's wiring, autos, the venues,
the simulator's adapters, and the harness a team tests its own robot with. A team writes its
robots' definitions and what they do; this builds the rest and runs it, on a robot, in the
simulator, and replaying a log. It is part of the FRC robot template, and a repository of its own
so the template stays what a team edits.

It targets WPILib 2027 (Systemcore, Java 25), Commands v3, AdvantageKit and Phoenix 6. It isn't
published anywhere: a robot program takes it as source, as a git submodule built as part of its
build (below).

## What's in it

| Path | What |
|---|---|
| `src/main/java/` | The runtime (`com.michaelgrundvig.frc.runtime`, `com.michaelgrundvig.frc:frc-robot-runtime`). At its top: `RobotProgram` (the entry point), `Robot` (logging, Pitwall, and the loop), `RobotContainer` (builds every mechanism on the hardware for the mode: real, simulated, or replayed from a log), `RobotConfig`, `RunMode`, and `ExampleRobot`, the guide's robot, which the runtime is tested on, with `ExampleControls`, its buttons and autos. Its packages: `subsystems` (`drive`, `localization`, `vision`, `mechanisms`), `hardware`, `behavior`, `autos`, `commands`, `dashboard`, `views`, `venues`, `sim` (the simulated world's adapters), and `lib` (loop timing, Systemcore's CAN ports, the Phoenix and REV helpers). |
| `src/testFixtures/java/` | What a team tests its own robot with: `RobotSim`, `Monkey`, `AutoCheck`, the invariants, `GoldenFile`, and the architecture rules a robot program shares (`ProductionRules`). A robot program asks for them as `testFixtures("com.michaelgrundvig.frc:frc-robot-runtime")`. |
| `src/test/java/` | The runtime's own tests, in the packages of what they test, from single classes to the whole robot in the simulator, on the guide's robot, with its architecture rules (`ArchitectureTest`), the values register's test (`ValuesRegisterTest`) and the guides' examples' test (`DocExamplesTest`). |
| `lib/localization/` | Localization, a project of its own (`com.michaelgrundvig.frc.localization`): the service the robot runs each loop, the pose estimator and everything that feeds it, and the calibration math. Reads the world model and nothing else of the robot's. |
| `lib/simulation/` | The simulation, a project of its own (`com.michaelgrundvig.frc.simulation`): the physics, the IMUs' models, how the simulated sensors err, the catalog of what can fail, the seeds, and the truth suite that checks them against physics. Reads the world model and nothing else of the robot's. |
| `build-logic/` | The Gradle plugin that writes AdvantageScope's robot assets from a robot program's `Robots.java` (`frc.advantagescope-assets`), an included build of its own. |
| `model/`, `pitwall/` | The robot model (`com.michaelgrundvig.frc:frc-robot-model`, with the documentation processor) and Pitwall (`com.michaelgrundvig.frc:frc-pitwall`), each a repository of its own, here as git submodules. |
| `docs/` | The guides, below. |
| `vendordeps/` | The vendor libraries the runtime is built and tested with, as in any WPILib project. A robot program that uses the runtime has the same files at the same versions. |
| `gradle/` | The build's supporting rules: `quality.gradle` (checks, for every project), `wpilib-repositories.gradle` (where the libraries find WPILib), the version catalog, and every downloaded library's checksum. |
| `licenses/` | The licenses of what files here derive from: AdvantageKit's template and WPILib. |

## The entry point

A robot program's `Main` is one call, handing the runtime what is the team's: which robot this is,
this season's field, and what the robot does. The runtime never names a team class, and
`ArchitectureTest` keeps it so.

```java fragment
RobotProgram.start(
    () -> ExampleRobot.COMPETITION, // which robot this is
    Rebuilt::game, // this season's field and venues
    ExampleControls::new); // what it does
```

That is the guide's robot with the guide's controls (`ExampleControls`, beside `ExampleRobot`). A
team passes a method that picks its robot (`Robots::current`) and its own `Controls::new`; a
fourth argument, `BuildInfo`, tells the log which code this is.

## Using it

A robot program includes it as a git submodule, built as part of its own build (a Gradle composite
build), so there is no version to bump:

```
git submodule add https://github.com/mikegrundvig/frc-robot-runtime.git runtime
```

```groovy
// settings.gradle
includeBuild("runtime")

// build.gradle
dependencies {
    implementation "com.michaelgrundvig.frc:frc-robot-runtime"
    testImplementation testFixtures("com.michaelgrundvig.frc:frc-robot-runtime")
}
```

The program also includes the robot model and Pitwall, its own `includeBuild("model")` and
`includeBuild("pitwall")`: the runtime includes them only when it is the build that runs
(`settings.gradle`), because Gradle refuses two builds that provide one module. A program that
checks out the runtime does not need to initialize its submodules (`git submodule update --init`
without `--recursive`).

GradleRIO, WPILib and the vendor libraries are the program's own: the runtime compiles against them
and never bundles them. So the program must use the versions this repository's build and tests
use: GradleRIO's version in `gradle/libs.versions.toml` and the files in `vendordeps/`. The
template checks it in its build.

To draw AdvantageScope's robot assets from `Robots.java`, a program applies the plugin that lives
in `build-logic/`, through its own plugin management:

```groovy
// settings.gradle
pluginManagement {
    includeBuild("runtime/build-logic")
}

// build.gradle
plugins {
    id "frc.advantagescope-assets"
}
```

## Building it

You need git and any Java installation (to start the Gradle wrapper); Gradle downloads the JDK 25
the code needs. Clone it with its submodules:

```
git clone --recurse-submodules https://github.com/mikegrundvig/frc-robot-runtime.git
```

| Command | Does |
|---|---|
| `./gradlew ci` | Everything CI checks, for the runtime, its libraries, the model, Pitwall and the plugin: formatting (Spotless), static analysis (Error Prone, NullAway), the tests, coverage (JaCoCo; the floors are in each `coverage-floor.properties`), and the Javadoc. |
| `./gradlew test --tests '*RobotContainerSimTest'` | Some of the tests, in whichever project has them. |
| `./gradlew test -Pquick --continuous` | The fast edit-test loop: skips formatting, static analysis, coverage and the slow tests. |
| `./gradlew spotlessApply` | Fixes formatting. |

[docs/testing.md](docs/testing.md) says how the tests are written and run. The build applies
GradleRIO, which gives it WPILib, the vendor libraries and the native libraries the simulator's
tests load, as it does a robot program. There is nothing to deploy here, so no team number.

## The guides

The guides are written for a robot program that has this repository as `runtime/`, as the
template does: a path such as `runtime/src/main/java/...` is from that program's root (drop
`runtime/` in this repository), and `./gradlew run` is the program's.

- [Start here](docs/guide.md): how a robot is written, a twenty-minute read before the rest
- [Coming from WPILib](docs/coming-from-wpilib.md): for experienced command-based teams, what maps to what and what you give up
- [Mechanisms](docs/mechanisms.md): describe a mechanism in one block, use its commands
- [Driving](docs/driving.md): the drivetrain's moves
- [Autos](docs/autos.md): declaring, choosing, running and checking autos
- [Testing](docs/testing.md): the tests, and how to write one against the simulator
- [The simulator](docs/simulator.md): its faults and scenarios, and how far to trust it
- [Values](docs/values.md): every configuration value, what it means, and how to find yours
- [Calibration](docs/calibration.md) and [characterization](docs/characterization.md): measuring the real robot
- [Camera placement](docs/camera-placement.md): where to mount the cameras
- [Venues](docs/venues.md): where a match is played, and what the robot knows of it
- [Stand-ins](docs/stand-ins.md): what waits on a library's release
- [Rule requests](docs/rule-requests.md): architecture rules that blocked a natural solution

## License

[BSD 3-Clause](LICENSE). `Robot.java`, `RunMode.java` and `ModuleIOSim.java` derive from
AdvantageKit's templates (BSD 3-Clause, Littleton Robotics); WPILib is BSD 3-Clause (FIRST and
other WPILib contributors). Their licenses are in [`licenses/`](licenses/). The libraries the
runtime builds with come under their own licenses, which Gradle downloads, so sharing this source
shares none of them; see the robot template's license table.
