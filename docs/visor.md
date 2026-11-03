# Visor on the coprocessors

[Visor](https://github.com/mikegrundvig/frc-visor) is an AprilTag vision program for the
coprocessors: it captures each camera's frames, finds tags with NAPA, solves each tag's two
candidate poses, and sends every frame to the robot over UDP, stamped with its capture time on the
robot's clock. It's the other choice to PhotonVision, per camera.

The robot program and Visor deploy together: `./gradlew deploy` puts Visor's program and a
configuration written from the robot's definition on each coprocessor, beside the robot program on
the Systemcore. So a coprocessor never runs another version than the robot's client, or cameras the
robot doesn't have. The robot reads a camera on Visor through `CameraIOVisor`, by its place in the
robot's list of cameras, and answers Visor's clock sync while any camera is on it.

**Visor hasn't run on robot hardware yet.** It's built and tested against rendered frames and the
simulator.

## Putting a camera on Visor

In `Robots.java`, the camera's `.software(...)` says which board it plugs into and which device
it is there, with its settings and its lens:

```java
Software.visor()
    .coprocessor("vision-a")
    .device("/dev/v4l/by-id/usb-Thrifty_ThriftyCam_0001-video-index0")
    .controls(
        List.of(
            Software.Control.of("auto_exposure", 1),
            Software.Control.of("exposure_time_absolute", 10)))
    .lens(
        Lens.builder()
            .width(1600)
            .height(1304)
            .fx(1229.712)
            .fy(1229.954)
            .cx(801.183)
            .cy(650.427)
            .distortion(List.of(-0.3117, 0.1204, 0.00041, -0.00022, -0.0208))
            .build())
    .build()
```

`Software.VISOR` alone is enough for the simulator, which sends each simulated image through Visor's
frames, its client and `CameraIOVisor`, as a real one arrives; a deploy needs the board and the
device. [values.md](values.md#cameraconfigsoftwarevisor)
says what each value is and how to find it.

In `build.gradle`, the `visor` block says how to reach each board, by the name the cameras give:

```groovy
visor {
    team = deploy.targets.systemcore.team
    coprocessors {
        "vision-a" { address = teamAddress(11) }   // 10.TE.AM.11
        "vision-b" { address = teamAddress(12) }
    }
}
```

A camera on a board the block doesn't name fails the deploy, saying which. With more than one robot
on Visor, name the one you're deploying to: `./gradlew deploy -Probot=competition`.

## What a deploy does

For each board with a camera on it:

1. `visorConfigs` writes its configuration from the robot's definition (`build/visor/configs/`):
   each camera's index, device, frame size and rate, controls, time offset and lens.
2. It takes Visor's program for the Visor client this build has, from the FRC libraries'
   repository (`com.michaelgrundvig.frc:visor`, built for arm64), or `-PvisorProgram=<file>` for one
   built from Visor's source.
3. It copies both beside the running ones, as `visor.new` and `visor.toml.new`, and has the new
   program check the new configuration (`visor check`). Only if it passes do both move into place;
   the old program is stopped, and systemd starts the new one. A refused configuration fails the
   deploy with Visor's reason, and the board keeps running what it had.

A board none of the robot's cameras is on gets nothing and isn't contacted. A board that can't be
reached fails the deploy, as the Systemcore does: a coprocessor left on the last deploy's
configuration would run cameras the robot no longer has. Frames from a board a deploy missed carry
another version ID: the robot refuses them, and its camera alert names both versions as they
arrive.

## What the board's image needs

The deploy logs in over SSH and needs nothing else of the board's system:

- a `visor` user in the `video` group, who owns `/opt/visor` (on a read-only root, a kept path),
  and logs in over SSH with the password the build gives (`-PvisorPassword=...`, or in
  `~/.gradle/gradle.properties`; none unless given);
- Visor's systemd unit, enabled: `deploy/visor.service` in Visor's repository, which runs
  `/opt/visor/visor` and keeps trying until a first deploy puts it there;
- chrony, with `deploy/chrony.conf` from Visor's repository, synced to the robot program.

`visor { directory = ...; user = ... }` changes the first two for an image laid out otherwise.

## Calibrating a camera's lens

Without a lens, Visor sends tags' corners and no poses. Calibrate each camera once, at the frame
size it runs, with Visor's `tools/calibrate.py` (its README): print the board, record about 40
frames on the coprocessor with `visor record`, and solve on a laptop. It refuses a session it can't
trust, and prints the lens to paste into the camera's `.lens(...)`. The build refuses a lens
calibrated at another frame size than the camera's `Sensor`.
