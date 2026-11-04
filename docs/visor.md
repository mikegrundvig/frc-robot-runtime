# Visor on the coprocessors

[Visor](https://github.com/mikegrundvig/frc-visor) is an AprilTag vision program for the
coprocessors: it captures each camera's frames, finds tags with NAPA, solves each tag's two
candidate poses, and sends every frame to the robot over UDP, stamped with its capture time on the
robot's clock. It's the other choice to PhotonVision, per camera.

The robot program carries Visor with it and puts it on its coprocessors itself. As the robot
starts, it asks each coprocessor's [Spotter](https://github.com/mikegrundvig/frc-spotter) agent
what it has, and, while disabled, installs this robot's Visor release on a board that has another,
and writes each board the configuration for the robot's cameras plugged into it. So a coprocessor
never runs another version than the robot's client, nor cameras the robot no longer has; a spare
board swapped in, or a camera moved to the other board, is put right the next time the robot
starts. The robot reads a camera on Visor through `CameraIOVisor`, by its place in the robot's
list of cameras, and answers Visor's clock sync while any camera is on it.

**Visor hasn't run on robot hardware yet.** It's built and tested against rendered frames, the
simulator, and Spotter's real agent on a laptop.

## Putting a camera on Visor

In `Robots.java`, the camera's `.software(...)` says which camera it is, by its device, with its
settings and its lens:

```java
Software.visor()
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

The device's `/dev/v4l/by-id/` name is the camera's own (its USB serial), so the robot finds which
coprocessor it's plugged into from what each board reports, and nothing says which board a camera
is on. `Software.VISOR` alone is enough for the simulator, which sends each simulated image
through Visor's frames, its client and `CameraIOVisor`, as a real one arrives.
[values.md](values.md#cameraconfigsoftwarevisor) says what each value is and how to find it.

The robot's entry also lists its coprocessors, each by its Spotter agent's address and the packs
the robot pushes it, from the deploy folder's `spotter-packs/`: `visor`. A pack the board's image
installs, such as Spotter's `debian` pack for its own health (the Paddock starter's image has it),
isn't listed: one listed that the deploy doesn't carry is a standing Spotter warning.

```java fragment
.coprocessors(
    List.of(
        new BoardDefinition("10.12.34.11", List.of("visor")),
        new BoardDefinition("10.12.34.12", List.of("visor"))))
```

## What the robot does

The build carries the Visor release for the robot's Visor client in the deploy (the `frc.visor`
plugin, `build.gradle`): `visor/visor-<version>-linux-arm64.tar.gz`, and its Spotter pack in
`spotter-packs/visor/`. On the robot, Spotter's manager pushes each board its packs, and for each
board that reports any of the robot's Visor cameras, while the robot is disabled and off the field:

1. If the board's Visor isn't this robot's version, it's sent the release (`visor.install`): the
   board unpacks it beside the running one, has the new program say its version, and only then
   moves it into place and restarts Visor. One that won't run there leaves the board as it was.
2. If the board's configuration isn't the one for its cameras, it's sent it (`visor.configure`):
   Visor checks it (`visor check`), and only then is it moved into place and Visor restarted. One
   Visor refuses leaves the board as it was.

Each step is tried once for what's wanted. One that fails is said by an alert, with the board's
reason, and isn't tried again until what's wanted changes or the program restarts. A camera none of
the boards has plugged in is said once every board has reported. A board that reboots keeps what
it has, so it's back on its own in seconds, mid-match included.

Spotter's own alerts are the robot's too, in the group `Coprocessors`, each naming its board: one
that's missing or on another protocol (high), one whose packs differ from the robot's or that
reports problems in them, each value of its packs at warning (medium) or failing (high), and the
robot's own Spotter setup, such as a packs folder it can't read. So a board that never answers says
so, even while the camera alert above waits for it. The keys and texts are in Spotter's
[robot guide](https://github.com/mikegrundvig/frc-spotter/blob/main/docs/robot.md).

## What the board's image needs

The board's image provides, once ([Visor's pack README](https://github.com/mikegrundvig/frc-visor/blob/main/spotter/visor/README.md)):

- Spotter's agent, with the team named (`/etc/frc-spotter/agent.json`), so it takes the robot's
  pushes and actions;
- a `visor` user in the `video` group, Visor's systemd unit (`deploy/visor.service` in Visor's
  repository), enabled, and `/opt/visor`, writable by the agent, kept on a read-only root;
- the polkit rule that lets the agent restart Visor, and only that;
- chrony, with `deploy/chrony.conf` from Visor's repository, synced to the robot program.

## Calibrating a camera's lens

Without a lens, Visor sends tags' corners and no poses. Calibrate each camera once, at the frame
size it runs, with Visor's `tools/calibrate.py` (its README): print the board, find the camera's
device with `visor cameras` on the coprocessor, stop Visor's service, record about 40 frames with
`visor record --device ... --width ... --height ...` (no robot needed yet; once the robot has
configured the board, `--camera <name>` instead), copy them from `/tmp/visor-frames/`, and solve
on a laptop. It refuses a session it can't
trust, and prints the lens as `Lens.builder()...build()`, to paste into the camera's
`.lens(...)`. The build refuses a lens calibrated at another frame size than the camera's `Sensor`.
