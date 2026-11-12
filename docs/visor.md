# Visor on the coprocessors

[Visor](https://github.com/mikegrundvig/frc-visor) is an AprilTag vision program for the
coprocessors: it captures each camera's frames, finds tags with NAPA, solves each tag's two
candidate poses, and sends every frame to the robot over UDP, stamped with its capture time on the
robot's clock. It's the other choice to PhotonVision, per camera.

The robot program carries Visor with it and puts it on its coprocessors itself. As the robot
starts, it asks each coprocessor's [Spotter](https://github.com/mikegrundvig/frc-spotter) agent
what it has, and, while disabled off the field, installs this robot's Visor release on a board
that has another, and writes each board the configuration for the robot's cameras plugged into it. So a coprocessor
never runs another version than the robot's client, nor cameras the robot no longer has; a spare
board swapped in, or a camera moved to the other board, is put right the next time the robot
starts. The robot reads a camera on Visor through `CameraIOVisor`, by its place in the robot's
list of cameras, and answers Visor's clock sync while any camera is on it.

**Visor hasn't run on robot hardware yet.** It's built and tested against rendered frames, the
simulator, and Spotter's real agent on a laptop. Before a Systemcore, a laptop can stand in for the
robot with a real board ([bench.md](bench.md#standing-in-for-the-robot-with-its-coprocessors)).

## Putting a camera on Visor

In `Robots.java`, the camera's `.software(...)` says which camera it is, by its device, with its
settings and its lens. `Software.visor()` is a camera on Visor with nothing said yet, and each
setting is a verb on it:

```java fragment
Software.visor()
    ...
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
```

The device is the camera's name as `visor cameras --names` prints it on its coprocessor, the one
rule for naming a camera. A ThriftyCam has no USB serial, so it is named by the USB port it's
plugged into, its `/dev/v4l/by-path/` name, such as
`/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0`, and must go back in that port
(label the ports and cables). A camera that does have a serial is named by its `/dev/v4l/by-id/`
name, which follows it to any port. Either way the robot finds which coprocessor the camera is
plugged into from what each board reports, and nothing says which board a camera is on.
`Software.VISOR` alone is enough for the simulator, which sends each simulated image
through Visor's frames, its client and `CameraIOVisor`, as a real one arrives.
[values.md](values.md#cameraconfigsoftwarevisor) says what each value is and how to find it; the
camera's exposure and gain have [a section of their own](#exposure-and-gain). A camera with a lens takes its field of view
from it, so its sensor says only its size and rate, `ImageSensor.of(1600, 1304, Hertz.of(60))`, and its
latency is Visor's typical 20 ms until the Vision log measures it.

The robot's entry also lists its coprocessors, each by its Spotter agent's address. While any of
the robot's cameras is on Visor, the robot pushes each of them Visor's pack, from the deploy
folder's `spotter-packs/visor/`, itself. A pack the board's image installs, such as Spotter's
`debian` pack for its own health (the Paddock starter's image has it), comes with the board.

```java fragment
.withCoprocessors("10.12.34.11", "10.12.34.12");
```

The robot takes Visor's frames only from these boards' addresses ([below](#which-frames-are-believed)).
A board listed by a hostname is looked up off the loop, as the robot starts and every 5 s until
it's found; until then, frames from an address no found board has aren't refused for it.

## What the robot does

The build carries the Visor release for the robot's Visor client in the deploy (the `frc.visor`
plugin, `build.gradle`): `visor/visor-<version>-linux-arm64.tar.gz`, and its Spotter pack in
`spotter-packs/visor/`. On the robot, Spotter's manager pushes each board Visor's pack, and for
each board that has it, while the robot is disabled and off the field and no push of the pack is
under way:

1. If the Visor on the board's disk isn't this robot's version, it's sent the release
   (`visor.install`), whether or not it can name its cameras yet (a board with no Visor, or an
   older one, names none): the board checks the archive's SHA-256 against the one the robot names
   with it, unpacks it beside the running one, has the new program say its version, and only then
   moves it into place and restarts Visor. One that won't run there leaves the board as it was.
2. If the Visor running isn't the one on its disk (a restart after an install failed), it's
   restarted (a configure restarts it), then installed again.
3. If the configuration Visor runs isn't the one for the robot's cameras plugged into it, it's sent
   it (`visor.configure`): Visor checks it (`visor check`), and only then is it moved into place
   and Visor restarted. One Visor refuses leaves the board as it was. A board that runs a
   configuration but has none of the robot's cameras (one taken out of `Robots.java` but left
   plugged in) is sent a configuration with no camera, which Visor accepts (it then sits idle), so
   it stops sending their frames.

What a board runs is believed over what's on its disk: Visor's pack reports both. A board whose
image is older than that report, or whose Visor isn't running, can't say what runs, and is judged
by its disk; once that's right, it's left alone.

A step that gets no answer (the board rebooted or dropped off mid-step, the run timed out, or a push
of the pack restarted its agent) is tried again 3 times, 30 s apart, and its alert says so; then
it stops, and starts over when the board reconnects or reboots. A step that worked but didn't take
(the board still reports what it had) counts the same, so nothing is tried in a loop. A step the
board refused (Visor refused the configuration, or the release won't run there) is said by a high
alert with the board's reason, and isn't tried again until what's wanted changes or the program
restarts; a reboot doesn't clear it. A camera none of the boards has plugged in is said once every
board with Visor's pack has said what it has: a board without the pack, such as one that refuses
pushes, doesn't hold it back. A board that reboots keeps what it has, so it's back on its own
in seconds, mid-match included.

Every board the robot lists gets Visor's pack while any of its cameras is on Visor, one running
PhotonVision too, and so the release; with none of the robot's Visor cameras plugged into it, it's
given no configuration, and Visor opens no camera there. A robot with no camera on Visor pushes no
board the pack (one pushed before is taken away) and leaves every board's Visor alone.

**The release is checked twice.** The Visor client carries the SHA-256 of the release archive it
was published with (`VisorVersion.RELEASE_SHA256`). The build stages an archive only when it has
that hash, and fails saying so otherwise: an archive replaced in the libraries' repository, or on
its way, never reaches the deploy. The robot checks the archive in its deploy folder against the
same hash as it reads it, and names the hash to each board with the archive; the board's install
refuses an archive whose hash isn't that one before anything in it runs. Either refusal is a high
alert naming the board: deploy again.

Nothing is installed, configured or restarted while the robot is enabled or the field is attached.
A step already under way then finishes (stopping it halfway helps no one), and a medium alert,
`Visor/landing/<board>`, says its cameras restart as it does. The configurations are worked out
again only when the cameras or a board's report change, and the release is read once, off the
loop, the first time a board needs it.

Visor's configuration names the robot by the team number set on Systemcore. With none set, the
robot installs and configures nothing of Visor's, and a high alert, `Visor/noTeam`, says how to set
it.

Each configuration also gives the field's tag size, `tag_size_m`, from the field's tag layout (the
game's 6.5 in for `Rebuilt`), so Visor solves every camera's poses at the size of the tags it
sees; Visor's own default is there for files written by hand. Visor detects 36h11 tags only, so a
robot with a camera on Visor is refused as it starts on a field whose tags are another family,
naming the camera. The robot writes every Visor camera's entry as it starts too, whether or not it
names its device yet, so a setting Visor refuses stops the program then, in Visor's words, rather
than at the first board: a control the exposure or gain sets (`auto_exposure`, `exposure_auto`,
`exposure_time_absolute`, `exposure_absolute`, `gain`), which would undo them and their live tuning,
is one.

Spotter's own alerts are the robot's too, in the group `Coprocessors`, each naming its board: one
that's missing or on another protocol (high), one whose packs differ from the robot's or that
reports problems in them, each value of its packs at warning (medium) or failing (high), and the
robot's own Spotter setup, such as a packs folder it can't read. So a board that never answers says
so, even while the camera alert above waits for it. The keys and texts are in Spotter's
[robot guide](https://github.com/mikegrundvig/frc-spotter/blob/main/docs/robot.md).

## Which frames are believed

Visor's frames arrive over UDP, and a frame names its camera only by its place in the list. So the
robot checks each one, and refuses it only for what it knows is wrong. It keeps a frame for a
camera when:

- a camera on Visor reads that index;
- it's the camera's configured size (its `ImageSensor`'s width and height);
- its number wasn't seen before: the same datagram twice is taken once;
- it comes from where the camera can be. A robot that lists no coprocessors takes frames from
  anyone. One that lists them takes frames only from their addresses, and once a board has
  reported the camera plugged in (through Spotter), only from that board. A camera no board has
  reported yet takes them from any listed board, so a Spotter agent that's slow to start, or
  down for a match, never costs the robot a working camera. A camera, once reported, stays on its
  board while that board is out of reach;
- it names a configuration the robot gave the board it came from ([below](#the-configuration-a-frame-names)).

Anything else is left out and counted, by why, under `Vision/Visor/Refused/`: `NoReader`,
`NotAListedBoard`, `NotFromItsBoard`, `OtherConfig`, `OtherSize`, `SeenBefore`, and `WaitingFull` (more than 64
frames waiting for a camera between its reads, which a camera read every loop never reaches). So a
board still running a camera taken out of `Robots.java`, or anything else on the network, can't put
frames into localization or fill the robot's memory. `Vision/Visor/CamerasUnplaced` counts the
cameras no board has reported yet, normal for the robot's first seconds, and
`Vision/Visor/BoardsUnfound` the listed names not yet looked up; neither raises an alert.

When a camera's frames are all being left out (none taken in the last 2 s) for a reason the team
can fix (not from a listed board, from another board than its own, another configuration, another
size), its card and its unreadable alert say which. A coprocessor running another Visor than the
robot's is said on every Visor camera, since its frames can't be read for which camera they are,
naming the camera index its frames carry.

A frame that arrives after a newer one (the network reordered them) is believed, and counted as
arrived late, not lost. A camera's numbers going back while its frames are taken later is Visor
restarting.

The receiver's own counts are logged beside them: `Vision/Visor/Dropped` (the loop fell behind and
its ring was full), `Malformed` (datagrams that weren't frames), `Unsynced`, `Gaps` and
`OtherVersion`.

In the simulator the frames come from no board, so neither where they come from nor their
configuration is checked.

### The configuration a frame names

A frame's camera index and its tags' poses mean what the configuration its board runs says: which
camera has which index, and the lens the poses were solved with. So every frame names that
configuration, by an ID made from its SHA-256 (the first eight hex digits of the hash the board
reports, Visor's [docs/wire.md](https://github.com/mikegrundvig/frc-visor/blob/main/docs/wire.md#the-configuration-id)).
The robot takes a board's frames only from a configuration it gave that board:

- the one it wants for the board now, with each camera's exposure and gain as tuned;
- the committed one, from `Robots.java` as it is;
- the last one it saw the board run while it was the one wanted;
- what it kept of these before the robot program last restarted (below).

A tuning only changes a camera's exposure and gain, which don't change what a frame means, so a
tuned value waiting for the robot to be disabled, a configuration on its way (the board runs the
old one until Visor restarts with the new), and a tuning kept for the match all cost no frames.
What's refused, under `OtherConfig`, is a configuration from before the robot program started that
the robot didn't put there this time: the cameras reordered or a lens calibrated again in
`Robots.java`, with the program booting on the field, where nothing is configured; or one the board
refused, so it runs on with its old one. Disabling the robot off the field configures it, and its
frames are taken from the moment Visor restarts with the new one.

Which configurations a board may run is known once it has reported with this robot's Visor on its
disk. Until then (the robot's first seconds, a Spotter agent that's down, an install under way,
whose old Visor's frames are refused for their version anyway), a board's frames aren't refused for
their configuration: as with where frames come from, a quiet Spotter never costs a working camera.
Once known, a board out of reach keeps what it was given.

**Across restarts.** A restart must not forget what the robot gave a board: a brownout or a crash
mid-match, or the power-on at the field, restarts the program with the field attached, where
nothing is configured, and a board still running a tuning would be left out for the match. So the
robot keeps each board's set on its own disk, beside the venues: on Systemcore,
`/home/systemcore/visor/configurations.properties` (on a laptop, the program's `build/visor/`). It
writes the file whole, off the loop, whenever a set changes, and reads it once as it starts. A
board's set is at most 4 configurations, newest first, kept by the board's address, with the ID
of the committed configuration it was kept with:

- what was kept counts only while that's still the committed configuration for the board's
  cameras, so a robot whose cameras were reordered or lenses calibrated again in `Robots.java`
  forgets it, and a board running the old layout is refused as before;
- it counts until the board is seen running a configuration wanted since the restart; then the
  set is this run's own again;
- a board not heard from since the restart keeps what was kept for it.

A file that's missing or can't be read is nothing kept, said in one line of the program's output,
and the robot goes on as it would without one. A write that fails raises a medium alert,
`Visor/forgetting`, and the robot goes on: after the next restart, a board running a tuning is
left out until the robot is disabled off the field. `Visor/uncommitted` still names each board
running another configuration than the committed one: put tuned values in `Robots.java`.

## Exposure and gain

Visor has no opinion on a camera's exposure or gain: it sets what the robot's configuration gives,
every time it opens the camera, after a replug too. The defaults live with the camera's
description in `Robots.java`, so a camera written as above runs them:

- **Exposure: a fixed 1 ms, the camera's automatic exposure off** (as `Software.visor()` starts). Blur
  from the robot turning is the main error in a tag's corners, and a short exposure cuts it: at 1 ms
  a robot turning at 360°/s moves a tag 4 m away about 2.5 cm across the image. A top team runs the
  ThriftyCam (OG02B1B) at 1 ms with its automatic exposure off. A camera's own automatic exposure
  lengthens it in dim light, which blurs exactly when a tag is hardest to read.
- **Gain: the camera's own** (`Gain.CAMERAS_OWN`). A `gain` control's units are the camera's, so no
  one number suits every camera. A top team runs the ThriftyCam at an analogue gain of 2 on a range of
  1 to 4 (4 is noisy), but that's libcamera's scale, not V4L2's: the ThriftyCam's default here waits
  until its `gain` range is read off a real one (`visor cameras` on its coprocessor lists it).

Any camera can give its own, as dialed in at a venue:

```java fragment
.exposure(Milliseconds.of(1.5))
.gain(40)
```

The exposure is from 100 µs, V4L2's step, to 1 s; the gain is a whole number, 0 to 65535.
`.camerasOwnExposure()` hands the exposure back to the camera's automatic exposure, which Visor
turns back on. `.controls(...)` stays for any other V4L2 control, by name; it refuses the exposure
and gain controls, which these own.

**Tuning at a venue.** A camera whose exposure is fixed, or whose gain is given, has a tuning of its
own on Pitwall's Tune view, `<camera> camera`, beside the drivetrain's and the joints': its
`exposure.time` (from 0.1 ms, V4L2's step, to one frame's time, past which the frame rate drops) and
its `gain.value` (0 to 65535, UVC's range: Visor holds it to the camera's own and says so). Tune it
while the robot is disabled and off the field (no FMS attached): the robot writes the board its
new configuration, as it does a new deploy's, and Visor restarts with it, so frames stop for a
second or two. A value tuned while enabled, or on the field, waits until then, and a low alert,
`Visor/waiting`, names the cameras whose configuration hasn't reached their board. When the field
connects, the tuned values revert to the committed ones, but the board keeps what it has for the
match (restarting the cameras on the field would cost more), and its frames are still taken
([above](#the-configuration-a-frame-names)), so a medium alert, `Visor/uncommitted`, names each
board running another configuration than the committed one. Watch
the camera's brightness in the log (below), and write the value that works into `Robots.java`: the
Tune view gives its builder call. A camera on its
own automatic exposure and gain has nothing to tune; give it `.gain(...)`, its default
from `visor cameras`, to tune its gain. Camera settings don't go in a venue's file: a venue is its
tag layout.

**What the log shows.** Every frame says the exposure and gain it was taken at, read back from the
camera, and how bright it is. The robot logs each camera's latest, every loop:

| Key | What |
|---|---|
| `Vision/<camera>/ExposureMs` | The exposure, in milliseconds; -1 while the camera chooses it, or without an exposure control |
| `Vision/<camera>/Gain` | The `gain` control, in the camera's units; -1 without one |
| `Vision/<camera>/AutoExposure` | Whether the camera's own automatic exposure is on |
| `Vision/<camera>/BrightnessMedian` | The image's median brightness, 0 to 255 |
| `Vision/<camera>/BrightnessHigh` | Its 99th percentile, about a tag's white where one is in view |
| `Vision/<camera>/ClippedShare` | The share of its pixels at full white, 0 to 1 |

A well-exposed frame has its high brightness between about 100 and 200, and nothing clipped. The
simulator's images have no pixels: there, the brightness reads 0 and the exposure and gain -1. Visor's
own log warns, every 10 s, when most frames were too dark to read tags far (the brightest 1% under
64 of 255) or had their whites clipping (over 1% of the frame at full white), naming the exposure
and gain, and what to change. A camera missing a control, or given a value outside its range, is a
warning there too, said once, and the camera keeps capturing at what it has.

## What the board's image needs

The board's image provides, once ([Visor's pack README](https://github.com/mikegrundvig/frc-visor/blob/main/spotter/visor/README.md)):

- Spotter's agent, with the team named (`/etc/frc-spotter/agent.json`), so it takes the robot's
  pushes and actions;
- what Visor's release carries for the board's root, its `os/` folder (packed by Visor's
  `tools/package.sh`): the `visor` account in the `video` group (sysusers), the `/opt/visor` folder
  the agent writes (tmpfiles), Visor's systemd unit, which the image enables, the agent's drop-in
  that lets it write there, and the polkit rule that lets the agent restart Visor, and only that.
  The image copies `os/` to its root and runs `systemd-sysusers` and `systemd-tmpfiles` on it
  ([Visor's pack README](https://github.com/mikegrundvig/frc-visor/blob/main/spotter/visor/README.md#what-the-boards-image-gives-it)
  lists each file);
- the parts that stay the image's own: keeping `/opt/visor` across boots on a read-only root, and
  a drop-in pinning Visor to the board's fast cores;
- chrony, with `deploy/chrony.conf` from Visor's repository (also in the release), synced to the
  robot program.

Visor's unit tells systemd when it's ready and then that it's alive every 1.5 s while every
camera's detection runs (`Type=notify`, `WatchdogSec=3`), so systemd restarts one that hangs, and
it limits the files and memory Visor may use. A board whose unit is older than the release's, with
no `/run/visor` for it (`RuntimeDirectory=visor`), never reports a running Visor until its image is
newer.

**Checking the clock.** Visor stamps each frame in robot time through chrony, which the robot
program answers on UDP 5806. On the coprocessor, with the robot program running, `chronyc sources`
shows the robot's line starting `^*`, and `chronyc tracking` names `10.TE.AM.2` with a System time
under a millisecond ([Visor's README](https://github.com/mikegrundvig/frc-visor#checking-the-clock)).
Until then each frame arrives marked unsynced: the robot leaves it out, counts it in the camera's
verdicts (`Vision/<camera>/Verdicts`: "image(s) without a capture time, left out"), and raises the
camera's high alert, `Vision/<camera>NoCaptureTime`, while they keep coming. The camera still
counts as connected, since its frames arrive.

## Calibrating a camera's lens

Without a lens, Visor sends tags' corners and no poses. Calibrate each camera once, at the frame
size it runs, with Visor's `tools/calibrate.py` (its README): print the board, find the camera's
device with `visor cameras` on the coprocessor, stop Visor's service, record about 40 frames with
`visor record --device ... --width ... --height ...` (no robot needed yet; once the robot has
configured the board, `--camera <name>` instead), copy them from `/tmp/visor-frames/`, and solve
on a laptop. It refuses a session it can't
trust, and prints the lens as `Lens.builder()...build()`, to paste into the camera's
`.lens(...)`. The build refuses a lens calibrated at another frame size than the camera's `ImageSensor`.

Until it has one, a camera on Visor with its `device` given places nothing, and says so: the robot
names it among its doubts as it starts, the camera's high alert `Vision/<camera>NoLens` stays up,
and its card on the Pit view has a **Lens** line. Each of its images is counted under
`Vision/<camera>/Images/NO_LENS`, its outcome naming the tags it saw ("saw tags 25, 26 and has no
lens calibration to solve them"), since Visor still sends those tags, without poses, as unsolved.

## Reading its frame rate and load

**Visor's log.** Every 10 s, Visor writes one line per camera to its journal. Read it on the
board with `sudo journalctl -u visor -f`, or through Spotter's agent, whose Visor pack serves it as
the log `visor.log`:

```text
front: 59.8 fps, 0 dropped, detect 4.2 ms mean, 9.8 ms worst, 1.50 tags per frame, brightness 41 median and 187 at 99%
```

- **fps:** frames Visor processed, each a datagram to the robot. It should be the camera's
  `fps` in its configuration (`CameraConfig.ImageSensor`'s `frameRate`).
- **dropped:** frames the camera delivered that Visor never sent: replaced by a newer one before
  detection got to them, or broken (marked bad by the driver, or short). Above 0, detection can't
  keep up with the camera, or its USB link is short of bandwidth.
- **detect, mean and worst:** how long detection and the pose solves took, per frame. Under the
  frame's period (16.7 ms at 60 fps), with room to spare, is what keeps `dropped` at 0.

- **brightness:** the last frame's median and 99th percentile, 0 to 255 ([exposure and
  gain](#exposure-and-gain)).

It also warns, in the same 10 s, about frames it couldn't send (the robot program isn't running,
or the network), frames stamped with the clock not synced, and frames too dark or clipping.

**The board's load**, from Spotter's `debian` pack on the image:
`curl -H 'Accept: application/json' http://10.TE.AM.11:5808/v2/values` from a laptop on the
robot's network. `debian.cpu.busiest` is the busiest core's share, `debian.cpu.capped` whether the
board slowed itself for heat or power, and `debian.thermal.margin` how far it is from slowing for
heat. [Characterization](characterization.md#vision-coprocessor-capacity) uses these to find how
many cameras one board carries.

## Health in the log and on Pitwall

Every frame also says how long its detection took and how many of the camera's frames Visor has
dropped so far, and Spotter's `debian` pack on the board's image says how the board is. The robot
logs both every loop and judges them, so after a match the log, and Pitwall's Pit view, say whether
a camera dropped frames, or a board ran hot, slowed itself, dropped out or rebooted: last season's
argument, settled by the record.

**Each camera**, under `Vision/<camera>`. The raw values are the camera's inputs, so a replay judges
them the same:

| Key | What |
|---|---|
| `Sequence` | The highest frame number heard since Visor last started; -1 before any (and on PhotonVision) |
| `Dropped` | Visor's count of the camera's dropped frames from its start, as the newest frame said; -1 before any |
| `DetectMs` | Each frame's detection and pose-solving time this loop, ms |
| `VisorAddress` | The board the camera is plugged into, as the boards report it, or else the listed board its frames come from (any sender, for a robot listing none), where its [live view](#the-live-view-on-pitwall) is; empty before either (and in the simulator) |
| `Health/FramesPerSecond` | Frames that arrived per second over the last 10 s; NaN in the first second |
| `Health/DroppedLately` | Frames Visor dropped in the last 10 s; not logged until a frame says (and on PhotonVision) |
| `Health/LostLately` | Frames Visor sent in the last 10 s that never arrived (gaps in the numbers, less frames that arrived late); not logged until a frame says (and on PhotonVision) |
| `Health/DetectMeanMs`, `Health/DetectWorstMs` | Detection time over the last 10 s; NaN without frames |
| `Health/VisorRestarts` | How often Visor restarted this session (its numbers went back): each new configuration restarts it |
| `Health/DroppingFrames`, `Health/LosingFrames`, `Health/DetectionSlow` | Each warning, as of the last 10 s |

The warnings, each over the last 10 s (Visor's own report's period, and the robot loop's "lately"):

- **Dropping frames:** any frame Visor dropped. Detection didn't keep up, or the camera sent broken
  frames.
- **Losing frames:** any frame sent that never arrived: the network lost it.
- **Detection slow:** the mean detection time is longer than the camera's frame period, from its
  `ImageSensor`'s `frameRate`, so it can't keep up. The mean, not the worst: one slow frame now and then
  costs that frame alone, and shows as a dropped one.

**Each board**, under `Coprocessors/<address>`, its address as the robot's configuration gives it.
The inputs are read from Spotter's manager in the robot loop, from the `debian` pack's values:

| Key | What |
|---|---|
| `Connection`, `Why` | Whether Spotter reaches its agent (`CONNECTING`, `CONNECTED`, `MISSING`, `OTHER_PROTOCOL`), and why not |
| `Hostname` | Its hostname, once it's described itself |
| `CpuBusiestPercent`, `CpuCapped` | `debian.cpu.busiest`, and `debian.cpu.capped`: whether it slowed itself for heat or power |
| `HottestCelsius`, `ThermalMarginCelsius`, `ThermalMarginLevel` | `debian.thermal.hottest`, `debian.thermal.margin`, and the margin's level by its limits |
| `UsbDevices`, `UsbDisconnects` | `debian.usb.devices`, and `debian.usb.disconnects`, since its boot |
| `UptimeSeconds` | `debian.uptime.seconds`: how a reboot shows |
| `AgentDescribes`, `PackPushes`, `PackPushesSinceDescribed` | Its agent's connects, the robot's pushes of its packs this session, and those since its agent last connected (Spotter's recorder, which tells them in order) |
| `Unavailable` | Each of those it can't give, and why: a board without the `debian` pack has none |
| `Health/Missing`, `Health/Throttled`, `Health/LowThermalMargin` | Each, now |
| `Health/Dropouts`, `Health/Reboots`, `Health/UsbDisconnectsThisSession`, `Health/PackRestarts` | Each, counted this session |

The rules, which set no limit of their own:

- **Missing:** nothing heard from its agent for Spotter's missing threshold (1 s). A **dropout** is
  missing after Spotter first reached it this session: a board still booting as the robot program
  starts isn't one, nor is one away while its agent restarts with packs the robot just pushed it
  (counted as a pack restart instead).
- **Throttled:** `debian.cpu.capped`, while it's reached.
- **Low thermal margin:** the margin's level, by the pack's limits: a warning below 10 °C from the
  nearest trip point that slows it, failing below 5. A board's own pack, or robot code's Spotter
  limits, may set others.
- **Rebooted:** its uptime went back. Without the `debian` pack there's no uptime, and a reboot
  shows only as a dropout.
- **USB disconnects:** its count since boot rose: a camera dropped off and came back, or didn't.

**On Pitwall,** the Pit view has a card for each camera (kind Camera, named by its id) and each
board (kind Coprocessor, named by its address, dots as hyphens), beside the autos' and the
venue's. A camera's card says whether its frames arrive, its frame rate against its configured
one, its dropped and lost frames and its detection time against its frame period, Visor's
restarts, and the exposure and gain its latest frame was taken at, beside its live view
([below](#the-live-view-on-pitwall)). A board's says whether it's
reached, how long it's been up, its busiest core and whether it slowed, how hot it runs and how
far from slowing, its USB devices, and what it doesn't report. A warning stays on its card, ▲, for
the rest of the session, with how often, how long and how long ago, so a drop in a match shows in
the pit after it; a card is ✗ while its camera isn't sending or its board is missing, and while a
board's margin is failing. Restarting the robot program clears them.

Visor's frames carry what the camera did; whether the board browned out isn't among the `debian`
pack's values (it has no undervoltage reading for an Orange Pi), so a power dip shows as a reboot
or a dropout, not by name.

## The live view on Pitwall

Each camera on Visor has a live view on its card in Pitwall's Pit view: its frames at half size, 10
a second, each tag outlined with its ID, and a line with the frame rate, exposure, gain and
brightness. Beside it, the card's health lines, and the camera's [tuning](#exposure-and-gain) where
it has one: its exposure and gain, edited and applied there as in the Tune view, so a camera is
dialed in while watching it. A tuned value restarts the camera, as from Tune, so the view pauses a
second or two.

The browser loads the view straight from the coprocessor, Visor's MJPEG stream on port 5809
(`http://<board>:5809/stream/<camera>`, Visor's docs/live-view.md), the board being the one the
camera is plugged into as the boards report it, or until then the listed board its frames come
from, logged as `Vision/<camera>/VisorAddress`: never an address the robot doesn't list. The video never passes through
the robot program or NetworkTables. The page's Content-Security-Policy lets the browser load
pictures only from port 5809 on the listed coprocessors (`CameraPit.pictureOrigins`), so a robot
that lists none has no live view in the browser. Visor makes pictures only while a card shows them: the page
opens the stream only while the Pit view is shown and in sight, and closes it otherwise.

Where there's no view, the card says why, in its place: before a frame from a listed board or a
board's report of the camera; in the simulator and in a replay, which have no pictures; for a camera on
PhotonVision, whose own page shows it. Where the browser can't reach the board (it's off, unplugged,
or the laptop isn't on the robot's network), it says so, and tries again every 5 s. Pitwall isn't
for the field, so the view has no field mode.

The view's size, rate and JPEG quality are the camera's `stream` when it gives one
([values.md](values.md#cameraconfigsoftwarestream)): its size as a share of the camera's, its rate,
and its JPEG quality. A camera that gives none has Visor's own view, half size, 10 a second (or the
camera's rate, if slower) and quality 60, and the robot writes none of it:

```java fragment
.stream(Percent.of(25), Hertz.of(5), 60)
```

A camera's card is named by its id and a board's by its address, so a camera can't be named as
another card on the Pit view is (`auto`, `venue`, a listed board's), nor two cameras alike but for
dots and underscores, nor a board listed twice: the robot's configuration refuses one, saying
which, by Pitwall's rule for card names.
