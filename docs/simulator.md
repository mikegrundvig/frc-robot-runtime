# The simulator

`./gradlew run` runs the robot program on your laptop, in WPILib's simulation, with this template's
simulated world (the simulation library, `lib/simulation`, and its adapters to the robot's IO,
`com.michaelgrundvig.frc.runtime.sim`: [how it's built](#how-its-built)): a drivetrain that really moves, as heavy as the
robot's configuration says, cameras that render what they'd see, and sensors that err as a
season-worn robot's do. [The truth suite](#how-far-to-trust-it-the-truth-suite) says how far it
can be trusted, number by number. AdvantageScope shows the field, the robot, and the truth beside the estimate
([below](#see-it-in-advantagescope)), and so does Pitwall's
[World view](#seeing-it-the-world-view).

Pitwall's page drives the simulator from its *Sim* view (http://localhost:5820/#sim): the Driver
Station, joysticks from the keyboard or a gamepad, pausing and stepping, Stop, and failing the
robot's parts live, which WPILib's simulator GUI can't do. The view is a client of the simulator's
API (http://localhost:5820/api/simulator), below, which anything else that speaks HTTP can use too.

| Job | Command |
|---|---|
| The simulator, driven from *Sim*: in teleop on a start, disabled until you press Enable | `./gradlew run` |
| With WPILib's simulator GUI too, enabled and drivable from its keyboard | `./gradlew run -Pgui` |
| Disabled, with no alliance station, like a real robot | `./gradlew run -PstartDisabled` |
| In a scenario: a fault, or something that happens to the robot | `./gradlew run -Pscenario=collision` |
| List the scenarios | `./gradlew scenarios` |
| Beside another simulator, with Pitwall on another port (a page open on 5820 would drive whichever answers there) | `./gradlew run -Ppitwall.port=5920` |
| Not the simulator: a robot's real CTRE devices, driven from this laptop ([bench.md](bench.md)) | `./gradlew run -PhwSim -Probot=chassis` |

**WPILib's simulator GUI is off unless you ask for it** with `-Pgui` (and `-Pgui=false` is the
default spelled out). *Sim* does its job. A real Driver Station can connect to the simulator
either way (`halsim_ds_socket` stays loaded). The GUI's default settings in `sim/` are copied
into the project folder only when it runs, and only when missing. Without it you lose its HAL
device views, its NetworkTables browser and plots, its `Field2d` window (AdvantageScope and
Elastic show the same), its notifier table, keyboard remapping, and its joystick reading, which
works whatever window has focus.

**The robot starts ready to drive, but disabled:** in teleop, on the blue alliance's station 2 (the
center start) unless one was chosen, with a Driver Station attached, so it stands on its start and
*Sim*'s Enable is one press from driving. It doesn't enable itself. To drive you come to *Sim*
anyway, to switch the keyboard or a gamepad on, and Enable is there beside them; a robot that
enabled itself would be enabled before anyone was watching, and a scenario's events, which happen
once it's enabled, would be over before the page or AdvantageScope was open. With the GUI it starts
enabled, as it always has: the GUI's keyboard drives it at once, and the GUI is on screen to
disable it. `-PstartDisabled` starts it as a real robot starts, before its Driver Station connects:
disabled, with no station or mode.

**The GUI and the API's joysticks don't mix.** Read in alpha-7's `DriverStationGui.cpp`
(`DriverStationExecute`): while the Driver Station is attached and not in autonomous, the GUI
writes all six joysticks every frame, zeros for a slot it has no device for, and announces new
data every 20 ms. Its *Zero disconnected joysticks* setting doesn't stop that: the check tests the
setting's pointer, not its value, so it always zeros. The GUI's own match time counts down while
enabled too. So with the GUI on, the API's joystick input is overwritten within a frame, and its
match time fights the GUI's. So drive from *Sim* without the GUI, the default, or with `-Pgui` turn
the GUI's Driver Station off (its *Disable DS* setting), which makes it only read.

## See it in AdvantageScope

The simulator logs what AdvantageScope needs to show the estimate against the truth, and the build
draws the robot for it from `Robots.java`. A layout in the repository puts it all on screen.

**Once, on each laptop:**

1. Install AdvantageScope v27.0.0-alpha-6, the release that pairs with the AdvantageKit in
   `vendordeps/` (27.0.0-alpha-6).
2. **App > Use Custom Assets Folder** (**AdvantageScope >** on a Mac), and choose the
   repository's `ascope_assets/` folder. The robot's model arrives with every `git pull` from then
   on; AdvantageScope rereads the folder every five seconds.
3. **File > Import Layout...**, and choose `layouts/advantagescope.json`. AdvantageScope warns
   when the layout came from another version; it was exported for this one.

**Then:** `./gradlew run`, and **File > Connect to Simulator > NetworkTables (AdvantageKit)**.
AdvantageScope remembers the choice, so from then on **File > Connect to Simulator > Default**
(Ctrl+Shift+K, Cmd+Shift+K on a Mac) connects the same way. It matters: connected as plain
NetworkTables, the default, every value's name starts `NT:/AdvantageKit/`, and every view in the
layout stays empty. A log opens in the same layout (**File > Open Log(s)...**, from `logs/`), since
a log and the simulator connected this way name every value alike.

| Tab | What it shows |
|---|---|
| 3D: estimate and truth | The robot where it believes it is, as a schematic of its configuration: bumpers, the frame (see-through, as tall as the robot), an orange arrow forward, the wheels turned as their modules measure, a small block where the gyro's IMU is mounted, and each camera with its field of view out to its sight range. The truth as a magenta ghost. A line from the robot to each tag the cameras saw this loop, green where its measurement was accepted and red where it was refused. The estimate's 99.9% ellipse on the floor, in yellow. |
| 2D: the ellipse | The same from above, where the ellipse is easiest to read: the truth inside it is the estimate being honest. |
| Swerve | Each module's measured velocity (red) and its setpoint (blue). |
| Localization | How far the estimate is from the truth (meters and degrees) beside how far it claims it could be (a standard deviation), and what it rests on. |
| Slip | The slip check: how fast the wheels may be sliding, as the gyro and as the accelerometer see it; how far the wheels' turn and the gyro's part; how much of a collision the IMU felt; and which IMU turns odometry. |

**What it reads.** These names stay as they are, so old logs open in the layout too:

| Value | Logged as |
|---|---|
| The estimate | `Localization/Pose` (2D); `Localization/Pose3d`, raised to the ground under it and tilted as the IMU says |
| Its uncertainty | `Localization/Ellipse`: the 99.9% ellipse (3.72 standard deviations, where the consistency study's "outside the ellipse" begins), a closed outline of 17 poses; nothing while it's wider than the field |
| The truth (simulator only) | `Sim/TruePose`, and `Sim/TruePose3d`, raised and tilted as the robot really is |
| Each camera's tags | `Vision/<camera>/AcceptedTags` and `RejectedTags`: this loop's tags where the field's layout has them, by what became of their images; `RejectedReasons`, why each refused one was, in the same order |
| What Pitwall's World view is drawn from, at any loop | `Localization/Pose3d`, `Covariance` (the position's covariance and the heading's variance: xx, xy, yy, heading), `Mode`, `Accepted`, `Rejected`, `VisionAgeSeconds`, `Recoveries`, `Placements` and `PlacedBecause`; `Drive/ModuleVelocities`; `Sim/TruePose3d`; and for each camera `Vision/<camera>/Sightings`, its tags as facts (the tag, which image, what became of it, and its verdict and solver by their numbers in localization's lists), written when they change, with `SightingNumbers` beside them: seven numbers for each tag of this loop (the verdict's value and its limit, the innovation, χ², and the measured pose's x, y and heading), empty in a loop without an image |
| The wheels | `Drive/ModulePoses`: each wheel's center in the robot's frame, turned to its measured angle, which turns the model's wheels |
| The slip check | `Localization/SlipMetersPerSecond`, `TurnSlipMetersPerSecond`, `SpeedSlipMetersPerSecond`, `SlideMetersPerSecond`, `TurnApartDegrees`, `CollisionShare` |

**Where the robot's model comes from.** Every build (and so `ci`) writes `ascope_assets/` from the
robot's configuration in `Robots.java`, through a Gradle plugin of the runtime's
(`frc.advantagescope-assets`, in `runtime/build-logic/`, which `settings.gradle` names in its
`pluginManagement`): `Robot_<name>/`, the schematic with its
cameras' fields of view, and `Robot_<name>_NoCones/`, the same without them, for the truth's
ghost. Commit the folder with the change to `Robots.java` that made it.

- **One plugin per tool.** A team that doesn't use AdvantageScope removes its line from
  `build.gradle`'s `plugins`, and nothing of it runs: not the plugin, and not its own tests in
  `ci` (`runtime/build-logic/`'s, which run only while one of its plugins is applied). A path tool's
  plugin, once one is chosen, will be another line, and work the same way.
- **Which robots:** `advantageScope { robots = ["competition", "practice"] }` in `build.gradle`,
  one folder each; by default every robot in `Robots.all()`.
- **Part of the build, never of `run` or `test`.** The simulator never waits for it, and it never
  starts the robot program to draw it: it builds only the robots' configuration records, in a Java
  process of its own on the project's Java, with a 32 MB heap. It can't tell `Robots.java` from
  the rest of the program, so it runs again whenever the compiled program or its libraries change
  (any change to the main code, to a library in `runtime/lib/`, where the writer is, or to a vendordep):
  0.3 s on the template's robot. With the code unchanged it's skipped as up to date (0.025 s).
- **Don't edit the folder:** the next build writes over an edit, from `Robots.java`. Nobody edits
  these files with a tool, so the build simply keeps them in step: after a pull, a `clean`, or a
  merge alike. **A merge conflict in `ascope_assets/`:** take either side
  (`git checkout --theirs ascope_assets`), build, and commit what the build writes. (A plugin for
  a tool whose GUI rewrites its own file, such as a path tool's, would instead fail the build on an
  edit made outside it, naming the difference: the plugins' shared base in `runtime/build-logic/` can.)
- **What the model leaves out:** everything it doesn't draw, such as the mass, the motors, the
  gains, and each camera's noise and latency (the writer's Javadoc lists them, and a test keeps
  the list whole).
- **Your own assets go in `ascope_assets/` too:** a CAD robot, a joystick, a field.
  AdvantageScope takes one custom assets folder, and the build leaves alone every folder it didn't
  write: it removes only a folder it wrote itself whose robot is gone (renamed, or dropped from
  `advantageScope { robots }`).

**Known limits.**

- A tag's line starts at the robot's center, half a meter up, not at the camera that saw it:
  AdvantageScope draws vision targets from the robot's pose (its `RobotManager.ts`). Which camera
  saw a tag is in the line's source, one pair of sources per camera.
- A field of view is drawn flat at the sight range along the camera's axis; the simulator's limit
  is a distance from the camera, so the cone's corners reach a little past it.
- The cameras' fields of view are fixed to the frame: a camera on a mechanism would need its cone
  moved with the mechanism, which the model can't do yet.
- The layout names AdvantageScope's 2026 field as `2026 Field`, a name not checked against the
  field AdvantageScope downloads. If it's wrong, AdvantageScope opens the first field it lists
  instead: the newest season's, once its fields have downloaded.

## Pitwall's simulator controls

Everything the *Sim* view does to the simulator goes through an API Pitwall serves only in
WPILib's simulation, asked of the HAL itself (its runtime type is simulation; anything else counts
as a robot), and not while replaying a log. On a robot none of it exists: every route answers 404
and the event stream has none of its sections, so nothing can write the Driver Station's state
there (`PitwallRobotTest` checks this with WPILib told it's on a Systemcore). The controls are
exempt from the field lock: in the simulator they *are* the field, so attaching the field system
locks tuning and actions exactly as a real field does, and the controls stay usable to detach it
again.

### In the event stream

Three sections, beside the page's others, sent as JSON merge patches when they change:

- **`simulator`**: `enabled`, `mode` (`AUTONOMOUS`, `TELEOPERATED`, `UTILITY`, or `UNKNOWN` before
  a Driver Station set one), `allianceStation` (`RED_1` to `BLUE_3`, or `UNKNOWN`), `fmsAttached`,
  `dsAttached`, `matchTime` (whole seconds left, counting down while enabled; -1 for none),
  `paused`, and `joysticks`: for `"0"` and `"1"`, `fromPage` (a page drives it),
  `centeredByDeadman` (the page went quiet, and it was centered), and `stream` (the event stream
  of the page whose input it took last, or empty).
- **`components`**: each part that can fail, by name: `{"kind", "faults": {name: {"does",
  "lasting", "undo", "parameters": [{"name", "unit", "min", "max", "initial"}], "on", "values"}}}`.
  A lasting fault has `on`, and while on, the `values` it was applied with; an event has neither.
  `undo` says what undoing a lasting fault does, or for an event, how the robot recovers.
- **`scenarios`**: each by name: `{"description", "faults": [{"component", "fault", "values"}]}`.

### Requests

Every request names the page's event stream in the `Pitwall-Stream` header, as the page's other
requests do; a joystick's input without one is refused (400), since the joystick is let go when
that stream closes. Errors are problem details, as elsewhere: 400 for a body of the wrong JSON
type or shape, 422 for one that names what isn't there or gives a value out of range (with the
reason), 409 for one that can't be done now.

| Request | Body | Answer |
|---|---|---|
| `GET /api/simulator` | | `{"simulator", "components", "scenarios"}`, read by the next loop; while paused, at once, as the last loop left it |
| `PATCH /api/simulator` | any of `enabled`, `mode`, `allianceStation`, `fmsAttached`, `dsAttached`, `matchTime` (0 to 600 s, or `null` for none), `paused` | the `simulator` section, once the loop has applied it all; 202 if it didn't within a second (while paused, it waits for a Step) |
| `POST /api/simulator/step` | | one loop of a paused simulator, then the `simulator` section; 409 while it runs |
| `POST /api/simulator/stop` | | 202, then the program ends: the stream closes |
| `PUT /api/simulator/joysticks` | `{"0": {"axes": [...], "buttons": [...], "povs": [...]}, "1": null}` | 204 at once |
| `POST /api/simulator/faults` | `{"component", "fault", "values": {name: number}}` to apply; `{"component", "fault", "undo": true}` to undo | the `components` section |
| `DELETE /api/simulator/faults` | | every lasting fault undone: the robot as built |
| `POST /api/simulator/scenarios` | `{"apply": name}` | the `components` section |

A `PATCH` applies its changes in one loop, in the order given. Enabling also attaches the Driver
Station, as a real one would be, and detaching it disables the robot. Pausing while paused answers
at once; a resume drops any pause asked before it, and a Stop any pause at all. A parameter a fault
request leaves out takes its `initial` value; a lasting fault whose values leave the part as it
was built changes nothing, and is refused (422, "changes nothing: as built"), never shown as on.

**Joysticks.** Each joystick's whole input, by WPILib's indices (for a gamepad,
`org.wpilib.driverstation.Gamepad`'s): axes from -1 to 1, a stick negative pushed up or left, at
most 12; buttons pressed or not, at most 64; POVs in degrees clockwise from up, in steps of 45, or
-1 centered, at most 8. A joystick is declared a gamepad, with at least a gamepad's 6 axes, 26
buttons, and 1 POV, the first time the page drives it. `null` lets it go: centered, and no longer
the page's. Only the page that drives it now can: when two pages drive one joystick, the latest
input wins, and `null` from the other is ignored, so a page that stops driving never lets go of a
joystick another page drives. A joystick the page never drove is never touched, so the GUI's
keyboard or a real Driver Station still can. The page sends a joystick's input when it changes, at most once a frame,
and again every 200 ms while anything is pressed; after 500 ms of silence on the robot's clock the
robot centers it (`centeredByDeadman`), and when the page's event stream closes it lets it go.

### How it works, and why

Like the rest of Pitwall, only the robot loop changes anything. A request waits in a queue, and
Pitwall's `periodic()`, first thing in `robotPeriodic()`, applies it through WPILib's own
simulation (`DriverStationSim`) and announces it at the HAL alone, as a real Driver Station's
packet arrives. WPILib reads it as the next loop begins, so that loop's mode methods
(`utilityInit()`), the robot's code, and AdvantageKit's log, which captures the Driver Station as
it was read, all see it in the same loop, as they do a real Driver Station's. (WPILib's
`DriverStationSim.notifyNewData()` would refresh WPILib's copy partway through a loop: the rest of
that loop would see a change its mode methods hadn't, and the replay, which hands each loop its
logged Driver Station as the loop begins, would see it a loop early.)

Nothing new needs logging: AdvantageKit logs the Driver Station's state and the joysticks on every
robot and replays them, and a fault changes the simulated hardware, whose readings the IO layer
logs. `ReplayEndToEndTest` sets the Driver Station and drives a joystick through these controls,
and the replay matches loop for loop, `utilityInit()` included. A replay has no simulator controls
(its Driver Station and hardware come from its log), so what they record to say why is never
replayed: the faults on (`Pitwall/Simulator/Faults`) and every sentence each loop said
(`Pitwall/Simulator/Said`) are outputs of the simulator's own run alone.

Three things can't wait for the loop, because a paused simulator runs no loops: resuming and
stepping happen at once on the server's thread (`SimHooks`), and so does asking to stop, which
resumes a paused clock so a loop runs to end the program. Pausing goes through the loop, so its
last patch tells every page. A step moves the clock to the next alarm of any notifier, as the GUI's
Step does: between loops that's the robot loop's own, so one loop runs and applies whatever was
asked while paused, but a team's own `Notifier` due sooner would fire instead, and Step would need
pressing again. Stop ends AdvantageKit's log, so its last loops are kept, then the program; it
refuses anywhere but WPILib's simulation.

## What can fail, live

The simulation's catalog says what can go wrong with each part, as data (`FaultCatalog`, in
`lib/simulation`), and the robot program binds each fault to its part and declares them to Pitwall
(`com.michaelgrundvig.frc.runtime.sim.Faults`); Pitwall knows nothing of any robot's parts. A fault that lasts says
whether it's on by asking the world, and undoing it puts the part back as the world was built: a
season-worn robot's sensors (`SensorErrors.TYPICAL`), not perfect ones. An event happens once, and
nothing undoes it. The robot's code is never told: it finds out from its sensors, as on a robot.

| Part | Fault | Lasts or happens | Takes |
|---|---|---|---|
| Each camera (`Camera front-left`) | Unplugged: no results, and it reads disconnected | lasts | |
| | Frozen: its last image's tags again and again, stamped as new | lasts | |
| | Moved on its mount, from where it was mounted | lasts | forward, left, up (in) |
| | Turned on its mount | lasts | roll, pitch, yaw (°) |
| | Late: images arrive later than its configuration says, still stamped when taken | lasts | extra latency (ms) |
| | Clock offset: its clock off the robot's, so each image arrives on time but is stamped that much after it was taken (before, if negative): the stamps are wrong | lasts | offset (ms) |
| Gyro, and the backup IMU | Unplugged | lasts | |
| | Frozen: the same yaw every time | lasts | |
| | Errors: its scale, drift, and noise from now on, carrying on from what it reads | lasts | scale, drift (°/min), noise (°) |
| | Reset: its yaw jumps to zero, as a gyro that restarts does | happens | |
| Each module (`Front left module`) | Stale encoder: its encoders' readings stop changing while the wheel turns and steers, then catch up | lasts | |
| | Zeroed off: its absolute encoder's zero | lasts | offset (°) |
| | Worn tread: its wheel's true radius over the configured one | lasts | scale |
| | Slips: spins without moving the robot | happens | turns |
| | Spinning: its wheel keeps spinning faster than the floor moves it, as one losing its grip does | lasts | speed (m/s) |
| Robot | Shoved, without its wheels turning; only its gyro feels a turn. It stops at a wall or an element | happens | forward, left (m), turn (°) |
| | Pushed, its wheels skidding: they never see it, and its IMU feels the push get going and stop (3 m/s²) | lasts | forward, left (m/s) |
| | Hit, with a 4 g jolt its IMU feels | happens | forward, left (m) |
| | Tilted by hand, whatever the ground | lasts | pitch, roll (°) |
| | Slipping: only a share of its wheels' rolling moves it | lasts | grip |
| | Placed, as the drive team carries it. Inside an element, it is set down beside it | happens | x, y (m), heading (°) |
| | Carried to the nearest bump, facing across it | happens | |
| Field | Tag moved: one tag, from where the field was built; moving another puts it back. The robot's code and the cameras' multi-tag solves still believe the layout | lasts | tag (its ID), x, y, up (in), in the field's frame |

Carrying the robot to a bump is declared only on a field with bumps. A world can also be built
with its tags a little off the layout (`SensorErrors.tagErrors`), as the consistency study builds
it; undoing a moved tag puts it back there. A world's cameras can scatter their tag corners wider than
PhotonVision's example (`SensorErrors.cornerNoise`, a quarter pixel by default) and be mounted off
their configuration (`SensorErrors.cameraMountErrors`), for a study to see whether a result hangs on
the simulator's kind cameras.

### What the simulator can't make fail yet

The one list of what's not possible yet; the stand-ins and the code point here. Each waits on the
simulator, not on Pitwall. Add each to the catalog (`FaultCatalog`), bound to its part in `Faults`,
and a scenario for it, when the simulator can make it.

- One wheel off the floor: `wheel-lifted` tilts the robot and scales every wheel's grip alike, so
  its wheels never disagree as a lifted one's would, and the robot isn't raised.
- One wheel slipping on its own under acceleration: the robot keeps to its tread's grip as a whole,
  and every wheel carries the same share, so they slip all alike or not at all (maple-sim,
  [stand-ins](stand-ins.md)).
- A CAN device dropping off the bus, or browning out: the simulated modules have no bus. A joint's
  motor controllers can be silenced or restarted, one of its followers taken off the bus alone
  (neutral, drawing nothing, its leader driving the joint without it, the drag of one neutral in
  brake left out), its absolute encoder frozen or slipped on its shaft, and the joint jammed, its
  motors stalled against what holds it, but only by a test (`JointSimulation.silence`, `restart`,
  `silenceFollower`, `freeze`, `slip`, `jam`), not yet from the page.
- The battery sagging: the simulated world models no battery.
- A camera's axes swapped in the configuration, the wrong field variant, or one camera with the
  wrong tag layout.
- Ramp and platform crossings: the field model describes ramps and platforms, but 2026's field
  has bumps only, so nothing has checked the simulator or localization on one.

## Scenarios

A scenario is a named set of these faults (`com.michaelgrundvig.frc.runtime.sim.Scenarios`). `./gradlew run
-Pscenario=<name>` applies its sensor faults before the robot's first loop and makes what happens
in it happen once the robot is enabled; the API applies all of it at once, to a running
simulation (`POST /api/simulator/scenarios`), and shows its lasting faults as on, each undone on its
own or all together. A fault in a scenario that changes nothing (the part is that way as built) is
passed over. A robot without cameras isn't offered the camera scenarios, nor a field without bumps
the bump one. `-Pscenario` in a program without the simulator's controls stops it, saying so.

## The *Sim* view

The page's *Sim* view, offered only in the simulator, does what the GUI did and what it couldn't,
through the API above. Each of its sections is a module of the page (`sim-enable.js`,
`sim-station.js`, `sim-auto.js`, `sim-time.js`, `sim-driving.js` over `sim-drive.js`, and
`sim-faults.js`), and `sim-state.js` is what they read and ask.

**You don't have to be in *Sim* to use them.** The same sections, compact, are the *Sim panel*,
which opens beside any view (its button in the state bar, or P) and stays open as the views
change: the mode and station, driving, the auto to run, the simulator's clock, and what fails with
a filter and the faults used last first. Enable and Disable are in the state bar on every view.
And in the World view a click on a camera, the gyro, a wheel or the bumpers ends in that part's
faults, with "Apply where I click" to set the robot down at a place on the field. The
[guide](guide.md#running-it-with-no-robot) walks through it, with the page's keys.

**The Driver Station** is at the top, and stays there as the view scrolls: whether the robot is
enabled (filled in) or disabled (an outline), its mode, station, and match time, with Enable and a
large Disable. Enable needs a mode, as a real Driver Station always has one. Below are the mode,
the alliance station, Driver Station attached, FMS attached, and the match time (set, cleared, and
counting down while enabled, in whole seconds as a Driver Station shows it). The state bar's mode
is filled in while the simulated robot is enabled, and leads here; beside it, in the simulator
only, are the same Enable and Disable, on every view. Attaching the field asks first
when values are tuned, listing them, since the field returns them to committed; the field's band
then points back to the switch, its way out in the simulator.

**Esc disables the robot** from any view, whatever has the focus, a field or an open dialog
included: while the simulated robot is enabled, Esc disables it and stops whatever is running (an
experiment's run, an action), and means nothing else. A safety action wins over a view's. Only
with the robot disabled does Esc keep its other meanings: it stops a run or an action, drops what
was typed in a value's field, or returns the time strip to live. No key enables. When a disable
fails, the page says so at once, wherever Esc was pressed.

**Keyboard driving is switched on explicitly** ("Drive with the keyboard", in *Sim* or the Sim
panel), and then drives from whatever view is showing. The state bar says so on every view while
it's on: "⌨ Keys drive the robot", or why not now, with Switch off beside it. One fixed layout for
this template's driving, by physical key, so it's where it is on any keyboard layout:

| Keys | Joystick 0 |
|---|---|
| W, S | the left stick's Y, axis 1: W is -1 |
| A, D | the left stick's X, axis 0: A is -1 |
| J, L, or ← → | the right stick's X, axis 2: turning |
| R | Back, button 4, which resets the heading |

The keys pause, and the joystick is centered, while the focus is in a field (typing `w` into a
value never drives), when the window loses the keyboard, and when the tab is hidden; a key held
then counts as released, since its release can't be heard. So do Cmd, the Windows key, and Alt,
pressed or let go, and a context menu: a key let go while one is down may never say so (on macOS,
never while Cmd is down), and a key held that the page thinks is held forever would keep the
robot's deadman from ever stopping it. Hold it again to drive on. Switching driving off lets the
joystick go.

**Gamepads** drive through the browser's Gamepad API, switched on in *Sim* too, with the standard
mapping to WPILib's `Gamepad`:

| Standard gamepad | WPILib |
|---|---|
| axes 0-3 | axes 0-3, the same sign |
| face buttons 0-3 | buttons 0-3 |
| bumpers 4-5 | buttons 9-10 |
| triggers 6-7 | axes 4-5, from 0 to 1 |
| back 8, start 9 | buttons 4 and 6 |
| stick clicks 10-11 | buttons 7-8 |
| the D-pad 12-15 | buttons 11-14 |
| home 16 | button 5 |

The first gamepad drives the joystick chosen in *Sim* (0 or 1, remembered in this browser), and a
second one the other. A gamepad this browser doesn't know the standard layout of can't drive, and
*Sim* says so. Axes go in a Driver Station's steps of 1/127.

**What's sent:** each joystick's whole input when it changes, at most once a frame, and again every
200 ms while anything on it is pressed, each joystick by its own clock, naming the page's stream.
The keyboard's joystick is centered on blur (a gamepad keeps driving a visible page); every
joystick the page drives is centered when the tab is hidden; and `null` lets one go when driving
is switched off. Input the robot didn't answer (it timed out, or the robot was busy) is sent again
shortly, a let-go included, and *Sim* says so meanwhile. *Sim* shows each joystick the page drives,
its axes and the buttons held, and says when the robot's deadman centered it, or when another page
drives it too.

**An unfocused or hidden page.** Keys reach only a focused page, as they reached only a focused GUI
window. The page polls gamepads each frame only while the tab is visible (the W3C spec also hides a
gamepad until a button is pressed, and browsers throttle hidden tabs), centers its joysticks when
the tab is hidden, and says so in the tab's title ("⏸ Not driving"), then in *Sim* on return,
suggesting side-by-side windows. Whether a gamepad keeps reaching a page that's visible but
unfocused, beside AdvantageScope, is unverified: test it in Chrome and Firefox before calling the
view a strict improvement for gamepad users.

**Simulator clock:** Pause and Resume; Step one new loop, only while paused; and, in *Sim*, Stop,
after asking (it lists the tuned values not committed, which end with the program). The names say
"simulator" and "new" because the time strip's Loop buttons are another thing: they look back over
loops already run. A paused simulator runs no loops, so
the stream goes quiet but for heartbeats: a request made while paused says it waits for a Step or
Resume (one the loop was slow to answer, that it waits for the next loop), and that note goes once
a loop has run. A page that opens while the simulator is paused gets no snapshot; after a second
and a half it asks `GET /api/simulator`, answered at once while paused, and if the simulator says
it's paused, offers *Sim*, says why the rest is empty, and offers Resume and Step. The rest fills
in when a loop runs.

**Stop** ends the program, and the stream with it: the page that stopped it says "Stopped", in the
state bar, its band, and *Sim*, not that the link was lost, and reconnects by itself when the
simulator starts again. Another page open on the simulator sees only the link lost.

**What fails:** a card for each part (each camera, the gyro, the backup IMU, each module, the
robot, and each coprocessor), each fault with what it does and what undoing it does, as the
catalog has them (`FaultCatalog`); each parameter with its unit and limits, checked on the page
before it's sent; Apply, and Undo for a fault that's on. A lasting fault that's on is marked as a
tuned value is, an amber bar, ●, and "on", with the values it was applied with; an event is a
button that makes it happen once.
What's on is listed above the cards, with Undo all. The scenarios are listed there too, each with
what it does and its faults, those it put on marked as on while they are; Apply says which faults
it put on, which happened, and which it passed over, the part being that way as built. A refusal
(400, 409, or 422) is shown where it was asked, in the simulator's words, and read out when the
reader is free.

**Changes show a loop later:** the robot program sees a Driver Station change as its next loop
begins, as with a real Driver Station; *Sim* shows the simulator's answer.

## The robot stops at things

A season's field has walls and elements (the hub, the trenches' columns, the tower, an outpost's
face), and the simulated robot stops at them. Each physics step, the robot's bumper outline is
checked against the field's outline and against the footprint of every element that reaches into
the height of its bumpers. Where the bumpers would be inside one, the robot is moved straight back
out of it and loses the part of its speed that went in. So it stops against what it meets head on
and slides along what it meets at an angle. It never bounces, and it never ends up inside
anything. A corner that swings into a wall as the robot turns pushes the robot off the wall.

Its wheels feel the wall through the carpet. The part of a wheel's rolling that goes into the
surface would have to slide on the carpet, and the carpet resists that with as much force as it
takes to stop it, up to its grip on that wheel: the wheels' coefficient of friction times the
weight the wheel carries (less on a slick floor, the *Slipping* fault). So there are two cases, and
the drive's current limit picks one:

- **The wheels stall.** A motor that pushes its tread with less than the grip can't slide it. The
  wheel stops with the robot, the motor sits at its current limit, and the encoder counts nothing.
  Odometry stays right. The template's robot is here: 30 A through a Kraken at 6.75:1 on a 2 in
  wheel pushes 77 N, and the carpet grips each wheel with 200 N. It takes 78 A to break loose.
- **The wheels break loose.** A motor that pushes harder spins its tread against the grip as a
  steady drag. It turns slower than it would free, draws the current that drag takes, and its
  encoder counts travel the robot never made. The estimator then has only the IMU and the cameras
  to tell it, and the *World* view shows the estimate driving on into the hub while the dashed
  truth sits against its face.

A wheel rolling at an angle to the surface is loaded by the share of its rolling that goes in, and
one rolling along it or away from it is not loaded at all. The IMU reads the truth either way: a
sharp stop at contact, then no travel. Driving away from the wall is an ordinary start.

What is left out: the wall's own friction (the robot slides along it freely), the bumpers' give,
and a sliding tread's lower friction (the one coefficient serves for holding and sliding). Only the
bumpers touch, so a robot taller than a trench's bar still drives under it. Something low enough to
drive over is terrain, not an element, and the robot rides over it as before. Away from a wall the
wheels still never feel the floor: past the grip limit they spin up as if they carried the robot,
and a push from outside neither turns nor loads them.

**Elements hide tags.** Before a simulated camera takes an image, each tag is checked against the
field's elements, from the camera's lens to the tag's middle and to each of its four corners. A
tag with any of the five behind an element isn't in the image at all, because a real detector
finds a tag by its whole border and reports nothing of one it sees part of. So the hub hides the
tower's tags from a robot beyond it, a trench's column hides the trench's tags from the side, and
the tower's own uprights hide a tag on its wall from some angles. PhotonVision's simulation
already refused a tag seen from behind, and still does. What still hides nothing: the robot's own
frame and mechanisms, game pieces, other robots, and terrain. On a field with no elements, images
are taken exactly as before. The check costs about 12 µs an image on the 2026 field (32 tags
against 28 blocks), measured once on a development laptop.

A robot can't be set down inside something. Placed inside the hub, it is set down against the
nearest face. Shoved or hit into a wall, it ends against the wall.

A field with no elements is open space, with no walls either. The tests' harness (`RobotSim`)
plays on the season's field with nothing standing on it (`FieldModel.withoutElements()`), so a test
drives across the floor without minding where the hub is. A test of what the robot runs into
builds with the season's own game (`sim.build(robot, Rebuilt.GAME, controls, errors)`). What an element is, and how a season's field writes
them: the model's README.

## Seeing it: the *World* view

Pitwall's *World* view (http://localhost:5820/#world) draws what the robot program believes, beside
what the simulator knows: the robot where localization puts it, its uncertainty as a dashed
ellipse, and the truth as a dashed ghost, so the gap a fault opens shows as it opens. Each camera's
sight lines to the tags it saw are solid where the estimator accepted the measurement and dashed
where it refused it; a click on one says why (the verdict, the innovation, χ², the distance). The
view follows the page's cursor, as the Robot view does: the time strip under it goes back through
the last five minutes, loop by loop or event by event (a refused measurement, the estimate lost, a
recovery), so a line that came and went is found again and clicked. The legend beside the drawing
says what each mark means and switches each layer. What stands on the field is drawn solid, and each
AprilTag shows the side it is read from: a dark face with a tick standing out of it, and a faint
back. Where a robot may start is drawn too, for both alliances: its bumpers as a dashed
outline on the floor, with the start's name. Click a start for its pose, and *Place the robot here*
sets the simulated robot down on it, as the *Placed* fault does (a robot's code is not told: its cameras find it). *Follow* keeps the robot in view; *Top* is the
map, *3D* turns around it. The same view works on a robot, without the truth. How it's built, and how to add to it: [Pitwall's README](https://github.com/mikegrundvig/frc-pitwall/blob/main/README.md).

## How it's built

The simulated world is in two places, apart so that nothing the robot runs can reach it.

- **The simulation library** (`lib/simulation`, `com.michaelgrundvig.frc.simulation`) holds what
  doesn't speak the robot program's language: the physics, each driven joint's plant (`JointPlant`:
  its motors through its gearing, held to a current limit by `CurrentLimitedMotor`, its inertia,
  gravity and hard stops, in closed form), the IMUs' models (`GyroModel`, `AccelerometerModel`),
  how the simulated robot's sensors and wheels err (`SensorErrors`), the catalog of what can fail
  (`FaultCatalog`), and where each random stream starts (`Seeds`). It reads
  the world model and WPILib's math alone: no hardware layer, robot program, localization, or
  Pitwall, and no clock, unseeded randomness, threads, or static state (the robot program's
  `ArchitectureTest`), so worlds run side by side and a run repeats to the bit.
- **The robot program's adapters** (`com.michaelgrundvig.frc.runtime.sim`) hold what does: the modules' motor controllers
  and encoders (`ModuleIOSim`, a TalonFX's loops and limits and the CANcoder's timing, with the
  steering in closed form), each joint's motor controller (`JointIOSim`, a TalonFX's slot or a
  SPARK's loop as its block wires it, `TalonFxEmulator` and `SparkEmulator`, each running the
  joint's `ControllerPlan` where that controller does, on its plant, and stopping on its limit
  switches each millisecond, from the limits the robot's IO is given), the switches and
  rangefinders (`SensorSimulation`), each built from the robot's definition with the world, the
  IMUs' IO (`GyroIOSim`),
  the cameras (`CameraIOSim`, over
  PhotonVision's simulation in `VisionSim`), the coprocessors, the faults' bindings to Pitwall
  (`Faults`), the scenarios, and `DriveSimulation`, which steps them together and takes the
  sensors' readings at 250 Hz. Only these adapters and the program's top level see the library: the
  robot's code sees its IO interfaces alone (the robot's `ArchitectureTest`).

**The physics** sits behind one interface, `Physics`: commands go in, truth comes out.

```
 robot code ──IO──► ModuleIOSim: TalonFX loops and limits, the battery, the steering (closed form)
                         │ each sub-step, each wheel's WheelDrive: its motor's line, its steering angle
                         ▼
 Disturbance ──────► Physics ──► ChassisTruth, WheelTruth ──► IMU models, encoders, cameras ──IO──►
 (placed, pushed, tilted by hand, a slick floor, worn tread, the ground)
```

`step` moves the world on, in sub-steps of the physics' own choosing, asking the actuators at the
start of each what drives each wheel over it; `chassis()` and `wheel(i)` are the truth (pose, attitude,
velocities, every turn the robot made, and each wheel's turn, spin, angle, radius, current, load,
and contact); `apply` does something to the world from outside it. Its physics,
`KinematicPhysics`, is the simulator as it has always run: the robot moves as the best fit of its
wheels' travel, stopped at the field's walls and elements (`Contact`), each wheel turning its share of the robot's mass on its own motor (WPILib's
`DCMotorSim`, worked to the bit, whose matrix exponential, worked once for each length of step, is
WPILib's native math: the library's tests load it as the robot program's do), within the tread's
grip, on the plane through its wheels' ground. Putting it behind the interface changed no recorded run by a bit.

**What it keeps simple.** The simulator is for testing the robot's code against simple plants: the
robot as its wheels roll, and each mechanism's joint in closed form. Traction, weight transfer and
power it leaves out, and the truth suite's known gaps (below) say by how much
([stand-ins](stand-ins.md#the-simulators-physics)).

- **The drive as a motor's line.** Each wheel's controller gives the voltage it asks of its
  motor, its velocity loop's gain, and its current limit (`WheelDrive`), and the physics applies
  the limit: the motor's torque against the wheel's speed, capped at the limit's torque. It
  applies the limit at each 4 ms step's start and holds the voltage that gives through the step,
  which is the F = m a gap below.
- **The steering in closed form.** The steering settles in under a millisecond, so its controller
  turns it (`CurrentLimitedMotor`) and gives the angle; the physics follows it, and hands back the
  carpet's scrub as a torque on the steering (`WheelTruth.steerLoadNewtonMeters`) for the next
  step. Nothing reads it yet: the physics gives none, and the closed form takes it once a real
  module's scrub is measured, as a torque against the motion that can stop the module
  (`CurrentLimitedMotor` says how).
- **The joints in closed form.** Each driven joint is its plant (`JointPlant`): its motors through
  its gearing, held to their current limit, its inertia, gravity and hard stops, stepped every
  millisecond as its motor controller runs. No joint pushes on another, or on the chassis.
- **Determinism.** The simulator steps the physics at 250 Hz, its odometry's rate, with no clock,
  no unseeded randomness, and no order that hangs on identity hash codes, so a run repeats to the
  bit. The actuators keep time, not count steps: the steering's loop runs every millisecond, and
  the CANcoder sends every 10 ms.
- **Disturbances declared.** A push, a slick floor, worn tread, a tilt by hand and a carry are
  overrides of the physics, and say so (`Disturbance`).

A change to the physics moves every recorded run, so it re-records the goldens
(`EstimatorGoldenSimTest`, `CameraGoldenSimTest`) and reruns the [consistency study](testing.md):
the robot moves differently, which is the point. The truth suite says whether it moves better.

## How far to trust it: the truth suite

The simulator is checked against physics and geometry as hard as the estimator is checked against
the simulator. Each truth test compares the simulator with an answer worked out in the test
(Newton, Coulomb friction, a rigid frame's geometry, a pinhole camera, the configuration), never
with the simulator's own earlier output, and says in its Javadoc why its tolerance is what it is.
They run with the rest, in `ci`; `./gradlew test --tests '*TruthTest' --tests '*TruthSimTest'`
runs them alone (`'*Truth*'` would run four other tests too, whose names say they hold the
estimate to the truth).

A check today's simulator fails is a **known gap**: a test marked `@KnownGap`, with its reason, that
asserts today's number. It passes while the gap stays as recorded, and fails loudly when it closes
(promote it to a plain truth test, and take it off the list here) or moves (find out what moved it).

A check today's simulator passes by construction, since it compares the simulator with itself, is
**consistency**, not truth: a test marked `@Consistency`, with its reason. It guards the agreement,
and is truth only for an engine that could fail it. Three are, below: the accelerometer during a
launch (it differences the physics' own velocity), the cameras' latency (the stamps' gap is
`VisionSim`'s own subtraction), and odometry (today's physics moves by odometry's own fit).

| What | Its pass line | Today |
|---|---|---|
| F = m a at the drive's stator limit, 30 and 60 A, from rest (`DynamicsTruthSimTest`) | within 1% of 4 Kt I G / (r (m + 4 J / r²)) | **known gap:** 2.25% short at either (4.35 against 4.45 m/s² at 30 A): the controller's limit holds only at the start of each 4 ms step |
| Yaw against inertia: tangential pushes below slip | the configured yaw inertia within 2% | **known gap:** spins as 11.1 kg m² against 6.7, as if its mass sat at its wheels |
| Momentum and energy: spinning and coasting free, braking, standing a minute, locked in an X on a 15° ramp | 0.1%; no gain past 0.01 J a loop; under 1 mm and 0.01° | passes (the ramp only for want of a force: nothing pulls down a slope today) |
| Braked on a 15° ramp, its wheels straight: the creep its motors' back EMF allows, I = ω / (Kv R) holding the slope's pull | 0.210 m/s across the field, within 2% | **known gap:** stands still, as nothing pulls down a slope |
| Coasting, wheels steered a quarter turn | energy never rises | **known gap:** rises 35 J back as the robot turns to follow its wheels |
| A launch's energy: the motors' work (each wheel's at its own current) against kinetic energy, and friction where the wheels slip (each at its own load) | within 2% | **known gaps:** 3.5% short gripping (the step's limit, and the robot moving each step half a step behind its wheels), 31% short slipping (a slipping wheel still carries its share of the robot) |
| Grip in 16 directions, on a slick tread (μ 0.5) where every wheel slips however weight transfer loads it (`TractionTruthSimTest`) | μ g within 3%, every direction within 5% | passes |
| Each wheel's slip onset against μ times its own load, and the loads under weight transfer | within 3% | **known gaps:** every wheel slips at once, at 209 N, against 124.7 N for the front ones (their motors' push: 122.9 N at the tread, the rest spinning the wheel up); every wheel carries a quarter of the weight |
| One module pointed 10° off the other three: the robot turns toward it (going straight would take 214 N across the other front wheel, past its 199.5 N of grip), no faster than the odd wheel rolls with the rear two | more than 0.05° and less than 16.4° a second, at 1 m/s | passes, at 4.8° a second, as the best fit of the four, not by friction |
| Lift-off at a crest, square and at 45° (`TerrainTruthTest`, in the library) | three touch and one hangs by the ground's twist, within 2 mm, in 99% of places | square passes; at 45°, **known gap:** every wheel a quarter of the twist off the ground |
| Camera geometry from surveyed poses, level and tilted (`CameraTruthSimTest`) | corners against a pinhole within 0.01 px; each tag's solve within 1 mm and 0.05°; seen just inside the image, not just outside | passes |
| Latency and rates (`CameraTruthSimTest`, `CameraTimingSimTest`, `OdometryTruthSimTest`) | stamps within 2 ms on average (4 mm at 2 m/s), and off by a clock offset within as much; latency within 0.5 ms and 10%; frame, CANcoder, and gyro rates within 1% | passes (the latency as consistency only), but the frame rate, a **known gap:** 29.3 frames a second against 30, as PhotonVision's simulation holds a frame back behind a slow image |
| Odometry against kinematics, exact sensors, no slip | within 0.1% of the distance, 0.05° a turn | consistency today: the physics moves by the same fit odometry makes, so it can't fail; an engine whose tread rolls on a compliant contact can |
| The accelerometer (`AccelerometerTruthTest`, in the library) | errors off, physics' specific force exactly (gravity on a tilt, the lever arm, a launch); its noise and bias as configured; an exact clip; a sample every 10 ms | passes (the launch as consistency only) |
| Seeds and independence (`SeedsTruthSimTest`, and `DeterminismTruthTest` in the library) | no stream shared over a hundred seeds; draws uncorrelated within 3 / √n, within a world and between neighboring seeds' first draws; bit-identical alone, repeated, beside seven others, and in another JVM | passes: each stream is the seed mixed with its kind, then its index (`Seeds.stream`) |

What it doesn't check yet: the frame meeting a crest (the robot's definition doesn't say how high it
rides), dynamic events (a hop at speed, a landing, a bumper's impact), which only a real robot's logs
can check, and how Phoenix's own simulated devices behave under the simulator's stepped time.
