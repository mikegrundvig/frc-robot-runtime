# FRC Robot Runtime

The robot program every robot shares, which no team writes: the loop, the drivetrain, the
mechanisms built from a robot's definition, vision and localization's wiring, autos, the venues,
the simulator's adapters, and the harness a team tests its own robot with. A team writes its
robots' definitions and what they do; this builds the rest and runs it, on a robot, in the
simulator, and replaying a log. It is part of the FRC robot template, and a repository of its own
so the template stays what a team edits.

It targets WPILib 2027 (Systemcore, Java 25), Commands v3, AdvantageKit and Phoenix 6. It is
published to the FRC libraries' Maven repository,
[mikegrundvig.github.io/frc-maven](https://mikegrundvig.github.io/frc-maven/), as dated releases
(below) that are pre-release, with no promise of compatibility from one to the next.

## What's in it

| Path | What |
|---|---|
| `src/main/java/` | The runtime (`com.michaelgrundvig.frc.runtime`, `com.michaelgrundvig.frc:frc-robot-runtime`). At its top: `RobotProgram` (the entry point), `Robot` (logging, Pitwall, and the loop), `RobotContainer` (builds every mechanism on the hardware for the mode: real, simulated, or replayed from a log), `RobotConfig`, `RunMode`, and `ExampleRobot`, the guide's robot, which the runtime is tested on, with `ExampleControls`, its buttons and autos. Its packages: `subsystems` (`drive`, `localization`, `vision`, `mechanisms`), `hardware`, `behavior`, `autos`, `commands`, `dashboard`, `venues`, `sim` (the simulated world's adapters), and `lib` (loop timing, Systemcore's CAN ports, the Phoenix and REV helpers). |
| `src/testFixtures/java/` | What a team tests its own robot with: `RobotSim`, `Monkey`, `AutoCheck`, the invariants, `GoldenFile`, and the architecture rules a robot program shares (`ProductionRules`). A robot program asks for them as `testFixtures("com.michaelgrundvig.frc:frc-robot-runtime")`. |
| `src/test/java/` | The runtime's own tests, in the packages of what they test, from single classes to the whole robot in the simulator, on the guide's robot, with its architecture rules (`ArchitectureTest`), the values register's test (`ValuesRegisterTest`) and the guides' examples' test (`DocExamplesTest`). |
| `lib/simulation/` | The simulation, a project of its own (`com.michaelgrundvig.frc.simulation`, published as `com.michaelgrundvig.frc:frc-robot-simulation`): the physics, the IMUs' models, how the simulated sensors err, the catalog of what can fail, the seeds, and the truth suite that checks them against physics. Reads the world model and nothing else of the robot's. |
| `build-logic/` | The Gradle plugin that writes AdvantageScope's robot assets from a robot program's `Robots.java` (`frc.advantagescope-assets`), an included build of its own, published with the runtime as `com.michaelgrundvig.frc:frc-gradle-plugins`. |
| `docs/` | The guides, below. |
| `vendordeps/` | The vendor libraries the runtime is built and tested with, as in any WPILib project. A robot program that uses the runtime has the same files at the same versions. |
| `gradle/` | The build's supporting rules: `quality.gradle` (checks, for every project), `wpilib-repositories.gradle` (where the libraries find WPILib), `frc-maven.gradle` (where the robot model, localization and Pitwall come from), `publishing.gradle` (the published artifacts), the version catalog (with `frc-libs`, the release's version), and every downloaded library's checksum. |
| `licenses/` | The licenses of what files here derive from: AdvantageKit's template and WPILib. |

## The entry point

A robot program's `Main` is one call, handing the runtime what is the team's: which robot this is,
this season's field, and what the robot does. The runtime never names a team class, and
`ArchitectureTest` keeps it so.

```java fragment
RobotProgram.start(
    () -> ExampleRobot.COMPETITION, // which robot this is
    Rebuilt.GAME, // this season's field and venues
    ExampleControls::new); // what it does
```

That is the guide's robot with the guide's controls (`ExampleControls`, beside `ExampleRobot`). A
team passes a method that picks its robot (`Robots::current`) and its own `Controls::new`; a
fourth argument, `BuildInfo`, tells the log which code this is.

## Using it

A robot program takes it from the FRC libraries' Maven repository by its version. It brings the
libraries it is made of at the same version: the robot model
([frc-robot-model](https://github.com/mikegrundvig/frc-robot-model)), localization
([frc-localization](https://github.com/mikegrundvig/frc-localization)), the simulation
(`frc-robot-simulation`, from `lib/simulation`) and Pitwall
([frc-pitwall](https://github.com/mikegrundvig/frc-pitwall)).

```groovy
// build.gradle
repositories {
    exclusiveContent {
        forRepository {
            maven { url = "https://mikegrundvig.github.io/frc-maven/" }
        }
        filter { includeGroup "com.michaelgrundvig.frc" }
    }
}
dependencies {
    implementation "com.michaelgrundvig.frc:frc-robot-runtime:2026.10.22"
    testImplementation testFixtures("com.michaelgrundvig.frc:frc-robot-runtime:2026.10.22")
}
```

GradleRIO, WPILib and the vendor libraries are the program's own: the runtime and its libraries
compile against them and bring none of them. So the program must use the versions this
repository's build and tests use: GradleRIO's version in `gradle/libs.versions.toml` and the files
in `vendordeps/`. The jar says which they were, in `META-INF/frc-robot-runtime/built-with.json`
(written by the `builtWith` task), and the template's build reads it and fails when they differ.

To draw AdvantageScope's robot assets from `Robots.java`, a program applies the plugin that lives
in `build-logic/`, published with the runtime at its version:

```groovy
// settings.gradle
pluginManagement {
    repositories {
        gradlePluginPortal()
        exclusiveContent {
            forRepository {
                maven { url = "https://mikegrundvig.github.io/frc-maven/" }
            }
            filter {
                includeGroup "com.michaelgrundvig.frc"
                includeGroup "frc.advantagescope-assets"
            }
        }
    }
}

// build.gradle
plugins {
    id "frc.advantagescope-assets" version "2026.10.22"
}
```

## Building it

You need git and any Java installation (to start the Gradle wrapper); Gradle downloads the JDK 25
the code needs.

```
git clone https://github.com/mikegrundvig/frc-robot-runtime.git
```

| Command | Does |
|---|---|
| `./gradlew ci` | Everything CI checks, for the runtime, its simulation and the plugin: formatting (Spotless), static analysis (Error Prone, NullAway), the tests, coverage (JaCoCo; the floors are in each `coverage-floor.properties`), and the Javadoc. |
| `./gradlew test --tests '*RobotContainerSimTest'` | Some of the tests, in whichever project has them. |
| `./gradlew test -Pquick --continuous` | The fast edit-test loop: skips formatting, static analysis, coverage and the slow tests. |
| `./gradlew spotlessApply` | Fixes formatting. |
| `./gradlew publishToFrcMaven` | Every check, then this release (the runtime, the simulation and the plugin) into `../frc-maven` (below). |

[docs/testing.md](docs/testing.md) says how the tests are written and run. The build applies
GradleRIO, which gives it WPILib, the vendor libraries and the native libraries the simulator's
tests load, as it does a robot program. There is nothing to deploy here, so no team number.

The robot model, localization and Pitwall come from the FRC libraries' repository, at the version
the catalog's `frc-libs` names (`gradle/libs.versions.toml`), which is also this release's own. The
guides' examples test (`DocExamplesTest`) reads their published sources and test sources (the
`-sources` and `-test-sources` jars). Two switches change where they come from, both off by
default:

- `-PlocalLibs=<folder>` builds those found in `<folder>/model`, `<folder>/localization` and
  `<folder>/pitwall` (clones of their repositories) from source, in place of the published
  versions, to try a change to one here before it's published.
- `-PfrcMaven=<url>` reads the published libraries from another copy of the repository, such as a
  local clone holding a release not pushed yet: `-PfrcMaven=file:///home/you/src/frc-maven`.

**Publishing:** `./gradlew publishToFrcMaven` writes into `../frc-maven`, a clone of
[frc-maven](https://github.com/mikegrundvig/frc-maven) beside this one (`-PfrcMavenDir=<folder>`
names another). The model, localization and Pitwall must be published at that version first;
frc-maven's `release.sh` publishes every library in order and commits, and its README says how a
release goes.

## The guides

The guides are written for a robot program built on the runtime, as the template is: a path such
as `runtime/src/main/java/...` is this repository's (drop `runtime/`), and `./gradlew run` is the
program's.

- [Start here](docs/guide.md): how a robot is written, a twenty-minute read before the rest
- [Coming from WPILib](docs/coming-from-wpilib.md): for experienced command-based teams, what maps to what and what you give up
- [Mechanisms](docs/mechanisms.md): describe a mechanism in one block, use its commands
- [Driving](docs/driving.md): the drivetrain's moves
- [Autos](docs/autos.md): declaring, choosing, running and checking autos
- [Testing](docs/testing.md): the tests, and how to write one against the simulator
- [The simulator](docs/simulator.md): its faults and scenarios, and how far to trust it
- [Bench](docs/bench.md): drive real CTRE hardware from a laptop, before the robot has its Systemcore
- [Values](docs/values.md): every configuration value, what it means, and how to find yours
- [Calibration](docs/calibration.md) and [characterization](docs/characterization.md): measuring the real robot
- [Camera placement](docs/camera-placement.md): where to mount the cameras
- [Localizers](docs/localizers.md): choosing WPILib's pose estimator or your own, and the contract one keeps
- [Venues](docs/venues.md): where a match is played, and what the robot knows of it
- [Stand-ins](docs/stand-ins.md): what waits on a library's release
- [Rule requests](docs/rule-requests.md): architecture rules that blocked a natural solution

## License

[BSD 3-Clause](LICENSE). `Robot.java`, `RunMode.java` and `ModuleIOSim.java` derive from
AdvantageKit's templates (BSD 3-Clause, Littleton Robotics); WPILib is BSD 3-Clause (FIRST and
other WPILib contributors). Their licenses are in [`licenses/`](licenses/). The libraries the
runtime builds with come under their own licenses, which Gradle downloads, so sharing this source
shares none of them; see the robot template's license table.
