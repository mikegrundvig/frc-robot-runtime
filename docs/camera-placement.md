# Camera placement

Where to mount the robot's AprilTag cameras, and how many, so the robot knows where it is anywhere
on the field. The CAD team needs that answer before the robot is designed, which means at kickoff.

The camera placement study answers it in simulation. It tries many camera layouts on the robot and
scores each one all over the field: how often its cameras see tags, and how close the robot's own
vision pipeline puts the robot from what they see. It knows nothing about any game, so it runs on
any field: last season's until the new one is published, then the new one the same day.

## What it answers

- **How many cameras**, and what each one more buys.
- **Where:** which corners or sides of the frame, how high, turned which way, and tilted how far up.
- **How the robot's own layout compares** with the others.

It doesn't know the robot's mechanisms. A mount it likes still needs a clear view past every
mechanism through its whole travel, and a rigid mount; that's for CAD to check.

## How it works

**Where the robot might be.** Positions on a grid over the whole field, every 0.5 m, kept half a
robot's length off the walls, each at 8 headings. Each position's headings start at their own random
fraction of the 45° step, so they don't all line up with the walls and with cameras splayed 45°,
which would favor the layouts that happen to match them. Every spot counts the same, because the
study can't know what the next game rewards; a team that does can say so with weights (below). A
spot the robot can't reach, inside a field element, counts too until a weights file leaves it out.

**The candidates.** A candidate is a set of cameras, each placed and pointed as a `CameraConfig`
describes it: position, tilt up, and yaw. Every candidate uses the robot's own camera sensor. The
study tries:

- the robot's own cameras, as `Robots` configures them, always;
- every layout CAD's places allow, when CAD has said where cameras could go (`--mounts`, below);
- otherwise, a family built around the robot's frame: one to four cameras at its corners and the
  middles of its sides, facing out (ahead, back, sideways, or splayed 15°, 30°, or 45° from ahead or
  back), at 8, 16, and 24 in off the floor, tilted up 0°, 10°, 20°, or 30°.

Every camera must be within the robot: inside the bumpers, and between the floor and the robot's
height (`Body.contains`), the check the robot's own configuration makes. A candidate with a camera
outside is refused, naming it. The family leaves out heights the robot doesn't reach.

**One set of images at each pose.** At each pose, every camera takes an image as PhotonVision's
simulation takes it, with the camera model the simulator uses (`VisionSim`): the sensor's resolution
and field of view, about a quarter pixel of noise on each tag corner, and tags seen up to 7 m. Each
image goes through the robot's own vision pipeline (`VisionPipeline`), as on the robot: solved with
the gyro's heading, correcting heading from a multi-tag solution when it qualifies, with the same
filters and uncertainties. The robot stands still and level, and its gyro's heading is off by a
small random amount (a quarter degree, typically), as a real one is. The measurements from all the
cameras are combined as the estimator combines them, each weighed by its uncertainty. Each camera
takes two images at each pose, each with its own noise.

**PhotonVision's simulation, without the pictures.** PhotonVision's simulation draws every image
into native memory that only the garbage collector frees: fine at the simulator's 30 images a
second, but the default run takes about a million and a half, and that exhausts a computer's
memory. So the study takes its images step for step as PhotonVision's does (`CameraImageSim`): the
same test of which tags are in view, the same projection, the same model of pixel noise, the same
detection limits, and PhotonVision's own solves, without drawing anything. Its test
(`CameraImageSimTest`) gives the two the same seeds, so they draw the same noise, and compares
them image for image over a sample of 2026's field: they must see the same tags and make the same
multi-tag solves, with corners within 0.01 px and solves within a millimeter and a milliradian. In
the last run, 160 images from the template robot's four cameras saw the same 274 tags and made the
same 75 multi-tag solves, with corners within 0.001 px and solves within 0.03 mm.

**The ranking: the error 95% of the field is within.** For each candidate, the study finds the
position error that 95% of the field (of the poses, weighted, and every set of images at them) is
within; a spot with no measurement at all counts as missed by any distance. Smaller is better. It
puts coverage and accuracy in one number that keeps telling layouts apart: a share within a fixed
distance stops doing that once every good layout reaches nearly all of a field rich in tags, as
2026's is. A layout that leaves more than 5% of the field without a measurement has no such number
("none"); those rank after the rest, by how much of the field they place within 5 cm. Errors that
round to the same millimeter count as a tie, about the sampling's own noise, and any tie goes to
fewer cameras.

**The other scores**, each a share of the field:

| Score | The share of poses where | Why it matters |
|---|---|---|
| within 2 cm, within 5 cm | the combined measurement is that close to where the robot really is | How much of the field is placed well, at two standards |
| one tag | a camera sees at least one tag | Without a tag, the robot runs on its wheels alone |
| two tags | one camera sees two or more tags at once | A multi-tag solve needs two, and so does correcting the gyro's heading |
| two cameras | two or more cameras see tags | A blocked or knocked camera doesn't leave the robot blind |
| measured | the pipeline makes a measurement: tags in range, and the solve checks out | What reaches the estimator |
| heading | a measurement corrects the gyro's heading | What keeps the gyro honest over a match |

And the accuracy where there's a measurement: the position error's root mean square, and the
heading error's.

**Losing any one camera.** A camera gets blocked by a mechanism or another robot, knocked out of
calibration, or unplugged mid-match. For each layout of N cameras, the study scores it again with
each camera in turn left out, and reports the worst: its 95% error and the share of the field it
still measures with N - 1 cameras ("one lost"). That's the layout's tolerance of one fault. The two
are each the worst on its own, so they may come from losing different cameras. A layout of one
camera, lost, has nothing left: "none" and 0%. It's a
column, not part of the ranking: the ranking says how well a layout does as built, and whether a
fifth or sixth camera is worth its cost, weight, and wiring as insurance is the team's call, which
this column informs. Layouts share their N - 1 subsets, so this costs little.

**Deterministic.** Every random draw comes from the seed, so the same run gives the same report, to
the digit, with any number of threads, on the same kind of computer (the same operating system and
processor: the native solves may round differently on another). A camera's noise depends only on the
seed, the pose, and where the camera is, so a layout scores the same whatever else is tried beside
it.

## Running it

```bash
./gradlew cameraStudy                                   # the robot's cameras and the family
./gradlew cameraStudy --args="--weights=weights.csv"    # some spots count more
./gradlew cameraStudy --args="--layout=2027-field.json" # a field WPILib doesn't have yet
./gradlew cameraStudy --args="--robust"                 # how layouts hold up when cameras err more
```

The default run takes about a minute on two threads (45 to 60 s on the computer it was written
on, for 3,840 poses and 157 layouts), and about 1 GB of memory. A robustness run (`--robust`)
scores the field 13 times: 7.5 to 9 minutes there, depending on what else the computer was doing.
A finer grid costs the square: `--grid-meters=0.25` takes four times as long.

| Option | Default | What |
|---|---|---|
| `--robot=NAME` | `Robots.current()` | Which of `Robots.all()` to place cameras on |
| `--field=NAME` | the season's, `Fields.DEFAULT_FIELD` | One of WPILib's fields, by its name in `Fields`: `FRC_2026_REBUILT_ANDY_MARK` |
| `--layout=FILE` | none | A tag layout file (JSON), as WPILib or FIRST publishes it, for a field WPILib doesn't have yet |
| `--weights=FILE` | none | Regions of the field that count more, or not at all |
| `--mounts=FILE` | none: the family around the frame | Where CAD could put cameras, and the ways each could face (below) |
| `--cameras=N` | 4 | The most cameras in a layout |
| `--grid-meters=M` | 0.5 | Spacing of the positions |
| `--headings=N` | 8 | Headings at each position |
| `--frames=N` | 2 | Images each camera takes at each pose, each with its own noise |
| `--corner-noise-pixels=P` | the simulator's, 0.25 | How far each tag corner lands from where a perfect camera would put it, on average |
| `--corner-spread-pixels=P` | the simulator's, 0.08 | That distance's spread |
| `--sight-meters=M` | the robot's cameras' `sightRange`, 7 | Tags farther than this aren't detected |
| `--gyro-error-degrees=D` | 0.25 | The gyro heading's typical error (one standard deviation) |
| `--mount-error-mm=M` | 0 | How far each camera really sits from its configured place, at most, along each axis |
| `--mount-error-degrees=D` | 0 | How far each camera is really turned, at most, about each axis |
| `--other-robots=N` or `N-M` | 0 | Other robots in the cameras' way around each pose, five at most (below) |
| `--other-robots-height-meters=M` | as tall as this robot | How tall the other robots are |
| `--trials=N` | 0 | Robustness trials (below), each drawing all of these errors |
| `--robust` | | Robustness with the usual 12 trials |
| `--seed=N` | 1 | The seed every random draw comes from |
| `--threads=N` | 2 | How many threads to run on (the computer may be shared) |
| `--out=DIR` | `build/studies/camera-placement` | Where the report goes |

## Reading the result

The report is in `build/studies/camera-placement/`, written over on each run:

- `summary.md`: what ran, the top of the ranking, the best layout at each camera count, where the
  robot's own layout ranks, and a map of the field for the robot's layout and the best one.
- `candidates.csv`: every candidate, ranked, with every score, for a spreadsheet.
- `poses.csv`: every pose for the robot's layout and the best one, for a closer look.

A map shows, for each spot on the field, the share of headings where the robot is placed within
5 cm: `.` for none, `1` to `9` for tenths, `#` for all. Blue's alliance wall is on the left.

To turn it into requirements for CAD:

1. Take the fewest cameras whose 95% error is within a few millimeters of the best's. Errors a
   millimeter or two apart, or shares a point apart, are the sampling's own noise: choose among
   them on something else, such as easier mounts.
2. Among those, prefer a high "two cameras" share and a small "one lost" error: one camera
   blocked by a mechanism, a robot, or a knock still leaves the robot seeing tags. If the best
   layout at that count falls apart with one camera lost, weigh one more camera.
3. Read off what the top layouts at that count share (corners or sides, height, splay, tilt).
   Those are the requirements, in words like these: "four cameras, one at each corner, facing out
   at 30° to 45°, 16 to 24 in up, tilted up 10° to 20°".

Rerun with weights once the strategy says where the robot must know where it is.

## Other robots in the way

On the field, other robots stand between the cameras and the tags. With `--other-robots=3-5` (five
at most: an FRC field has five other robots), each set of images has 3 to 5 other robots around the
robot, as many as the seed draws. Each is a solid box with the outline of the robot being designed
(its bumpers') and as tall as it, unless `--other-robots-height-meters` says otherwise. Each stands
at a random place near the robot, evenly over the area within reach (the farthest a camera sees a
tag, 7 m, plus how far a camera and another robot's outline reach from their centers), turned at
random, anywhere its outline fits: clear of the robot, of each other, and of the walls, and not in
a region the weights leave out. They're placed anew for every set of images. A tag is hidden, and
not detected, when the line from the camera to any of its corners, the middles of its edges, or
its center passes through one.

It's a comparison under one crowded case, not a prediction. What it leaves out:

- **Real robots aren't solid boxes:** they're open frames, which hide less, and their mechanisms
  can reach higher than the robot's starting height. Use `--other-robots-height-meters` for taller
  ones.
- **Where robots really go:** a game puts robots in some places (scoring, loading, defending) and
  not others; here they're anywhere near the robot, evenly.
- **Field elements:** they block views too, and other robots can stand where an element is unless
  a weights file leaves those spots out. Nothing about any game goes in; after kickoff, elements
  could be added from the new field's drawings.
- **Room:** the count is the same however crowded the spot, short only of robots that don't fit.

On the example (2026's field standing in), 3 to 5 other robots barely move the ranking (rank
correlation 0.996 to 0.997 with the clear field; the top tens share 8 to 10 layouts), but they cost
every layout, and they cost a layout with few cameras most, so each camera is worth more:

| Cameras | 95% within, clear | 95% within, robots in the way | One lost, clear | One lost, robots in the way |
|---:|---:|---:|---:|---:|
| 3 | 3.3 cm | 4.7 cm | none | none |
| 4 | 2.4 cm | 2.8 cm | 3.6 cm | none (94.97% measured) |
| 5 | 2.1 cm | 2.3 cm | 2.9 cm | 3.5 cm |
| 6 | 1.8 cm | 2.0 cm | 2.2 cm | 2.5 cm |

(The best layout of each count at the eight places in "More than four cameras", below.) With other
robots in the way, the best four cameras can't lose one and still measure 95% of the field; five
can. The family's best four cameras go from 2.5 to 2.9 cm, and with one lost, from 3.7 cm to none
(94.7% measured). Robots in the way cost no measurable time.

## Robustness

The camera errors above are guesses until the robot is built: the simulator's own corner noise and
sight range, a gyro a quarter degree off, cameras mounted exactly where they're configured. A layout
that's best only when those guesses are right is a risky one to build. So a robustness run
(`--robust`, or `--trials=N`) scores the field in the nominal conditions, then again in each of 12
trials, each drawing every error from a plausible range, and ranks the layouts by their 95% error
in the worst trial: what the robot lives with when its cameras turn out worse than hoped. Every
layout faces the same trials, so they're compared fairly; the median across trials is beside it.
More trials make the worst a little worse, mostly for fragile layouts, whose worst trial is far
from their median. A layout that leaves more than 5% of the field unmeasured in any trial has no
worst 95% error ("none"): those rank after the rest, by the least of the field they placed within
5 cm in any trial, then fewer cameras.

| Error | Drawn from | Why that range |
|---|---|---|
| Tag corners off | 0.1 to 0.6 px on average (spread in the simulator's proportion) | A sharp, still image from a global-shutter camera locates corners to about a tenth of a pixel; defocus, a softer lens, or a little blur puts them half a pixel off or more. The simulator's 0.25 px sits low in the range. |
| Tags seen to | 5 to 7 m | A 6.5 in tag 7 m from a 1280 x 800 camera with an 80° lens is about 20 pixels across, near where detection fails; exposure, blur, and detection settings bring it in. Single tags count to the sight range; the pipeline throws out an image whose tags average farther than 6 m. |
| Gyro heading off | 0.1° to 0.5°, typically | The estimator holds heading to a few tenths of a degree while cameras correct it; after a hit, or a stretch without two tags in view, half a degree. |
| Each camera off its mount | Per trial, a bound from 0 to 5 mm and one from 0° to 1°; each camera's offset along and about each axis drawn evenly within plus or minus those bounds | A mount measured from CAD and checked with the camera calibration (docs/calibration.md) lands within a few millimeters and a fraction of a degree; a bumped camera, or one never calibrated, a degree or more. Each camera's offset is its own, fixed for the trial: the same over the whole field, as a mount's error is, and the same in every layout the camera is part of. |

The report's robust tables come first: each layout's worst-trial and median 95% error, its nominal
95% error and rank, the least of the field it placed within 5 cm in any trial, and the rank
correlation between the two rankings (1 is the same order). Each trial's draws are listed, and
`candidates.csv` has every trial's 95% error.

## Designing with CAD

Once the robot is taking shape, the family's guesses give way to CAD's own places: where on this
robot a camera could actually go, clear of the mechanisms, on something rigid. CAD lists them in a
CSV file, one place a line, with the ways a camera there could face, and the study scores every
layout of one camera up to `--cameras` (four, unless you say otherwise) at different places, each
facing any of its place's ways:

```csv
name,        forward (in), left (in), up (in), yaws (deg),     tilts (deg)
front-left,  12.25,        12.25,     20,      0 30 45,        10 20
front-right, 12.25,        -12.25,    20,      0 -30 -45,      10 20
back-left,   -12.25,       12.25,     20,      180 150 135,    10 20
back-right,  -12.25,       -12.25,    20,      180 -150 -135,  10 20
shooter-top, 0,            0,         28,      0 180,          15
```

The first line names the columns and their units, so a file in another unit can't be read as
inches by mistake: lengths in `in`, `mm`, `cm`, or `m`, angles in `deg` or `rad`, whatever CAD
works in. Positions are from the robot's origin (the frame's center, on the floor; forward, left,
and up, as `CameraConfig` measures them); a yaw of 0 faces ahead and positive turns left, and a
tilt is how far up from level. Lists of yaws and tilts are separated by spaces, each angle once.
Lines starting with `#` are comments. A place must be within the robot and at least an inch off
the floor (a lower one was measured in another unit), or the study names it, in its own units,
and stops. `--cameras` above the number of places means a camera at every place.

Every way of facing multiplies the layouts: five places with six ways each make over 9,000 layouts
of up to four cameras, and the study scores at most 5,000 in one run. When there are more, it says
how many, and the most cameras that would make few enough, if any would. Offer the ways that are
really in question, and narrow them run by run.

How CAD uses it while designing:

1. Early: list the places the design leaves room for, with the ways each could face. Run the study
   and read the top layouts at each camera count.
2. Settle the count, the places, and roughly the angles; narrow the file to those, with finer
   steps of angle, and run again.
3. When the design changes (a mechanism moves into a camera's view, a place goes away), change the
   file and run again. Each run takes a minute or two.
4. Commit the chosen layout as the robot's cameras in `Robots`: from then on it's "the robot's own"
   in every run, the baseline the alternatives are measured against.

## Weights

A CSV file of regions, each written for the blue alliance, in meters in the field's frame (see
`com.michaelgrundvig.frc.world.field`). The study adds each region's image on red's half by the
field's symmetry.
A pose counts as the product of the weights of the regions it's in, and 1 in none: 0 leaves a spot
out (inside a field element), 3 counts it three times (where the robot scores).

```csv
# name, x min, y min, x max, y max, weight
Field element, 7.5, 3.5, 9.0, 4.5, 0
Scoring, 1.0, 2.0, 3.0, 6.0, 3
```

## An example: the template's robot on 2026's field

2026's field stands in until 2027's is published, so this says nothing about next season's robot;
it shows what a run says and how to read it. The default run, seed 1:

| Cameras | The best layout of that many | Rank | 95% within | Within 2 cm | Two cameras |
|---:|---|---:|---:|---:|---:|
| 1 | ahead, 16 in up, tilted 20° | 139 | none (65% measured) | 44.9% | 0.0% |
| 2 | ahead and back, 24 in up, tilted 10° | 70 | none (93% measured) | 72.1% | 47.2% |
| 3 | front corners splayed 30°, and back, 16 in up, tilted 20° | 29 | 3.3 cm | 84.4% | 81.8% |
| 4 | ahead, back, left, right, 16 in up, tilted 20° | 1 | 2.5 cm | 91.3% | 96.3% |
| 4 | the template robot's own | 19 | 2.9 cm | 87.6% | 94.7% |

One or two cameras leave more than 5% of the field without a measurement. The third brings the 95%
error to 3.3 cm, and the fourth to 2.5 cm, with nearly every spot seen by two cameras. The top ten
are all four cameras, 2.5 to 2.6 cm: cameras on the four sides, or at the corners splayed 30° or
45°, at any of the three heights, tilted up 10° to 20°. The template robot's own layout is 4 mm
behind them. REBUILT has 32 tags, many in mid-field on the hubs; a field with fewer tags spreads
the layouts further apart.

With `--robust` (12 trials, 8 to 9 minutes), the errors grow and the layouts spread apart: the
most robust, four cameras on the sides, 16 in up, tilted 10°, has 5.5 cm in its worst trial (4.5
cm median), and the tenth 6.8 cm. The ranking keeps its shape (rank correlation 0.94 with the
nominal one): the robust top ten all ranked in the nominal top 21, the nominal best is 12th, and
the template robot's own moves from 19th to 25th (7.9 cm in its worst trial). Three cameras' best
is 9.8 cm; one or two leave too much of the field unmeasured in every trial.

**How far to trust the robust ranking.** Twelve trials are a small sample of the conditions, so
the robust ranking moves more between seeds than the nominal one. Over seeds 1, 2, and 3, the
rankings correlate at 0.93 to 0.95, but their top tens share only 5 to 7 layouts, and each seed's
best ranks between 2nd and 23rd in the others. A layout's worst-trial error moves 8 mm between
seeds, typically (the nominal 95% error moves 0.4 mm). Ranking by a percentile across trials
instead of the worst doesn't help: the second worst, the 75th percentile, and the median each move
7 to 10 mm between seeds too, and their top tens share no more. So read robust differences under a
centimeter as ties, and the robust ranking for what it agrees on across seeds. In this example:
four cameras, on the sides or at the corners splayed 30° to 45°, tilted up 10° to 20°, at any of
the three heights; three cameras about 3 cm behind; one or two blind too often.

**More than four cameras.** The family stops at four. To weigh a fifth or sixth, give the study
places: here, eight places 16 in up, tilted 20°, at the four corners (splayed 30°) and the middles
of the four sides, every layout of up to six cameras (247 with the robot's own, 3 s), with
`--mounts=eight.csv --cameras=6` and this file:

```csv
name,        forward (in), left (in), up (in), yaws (deg), tilts (deg)
front-left,  12.25,        12.25,     16,      30,         20
front-right, 12.25,        -12.25,    16,      -30,        20
back-left,   -12.25,       12.25,     16,      150,        20
back-right,  -12.25,       -12.25,    16,      -150,       20
front,       12.25,        0,         16,      0,          20
back,        -12.25,       0,         16,      180,        20
left,        0,            12.25,     16,      90,         20
right,       0,            -12.25,    16,      -90,        20
```


| Cameras | 95% within | Two cameras | One lost: 95% within | One lost: measured |
|---:|---:|---:|---:|---:|
| 3 | 3.3 cm | 83.7% | none | 88.8% |
| 4 | 2.4 cm | 96.2% | 3.6 cm | 97.5% |
| 5 | 2.1 cm | 99.2% | 2.9 cm | 98.3% |
| 6 | 1.8 cm | 100.0% | 2.2 cm | 99.8% |

Each camera past four buys 3 mm as built, but more when one is lost: the fifth brings the worst
one-lost error from 3.6 to 2.9 cm, and the sixth to 2.2 cm. That's the case for them, if any.

## At kickoff

The study needs only the field's size and tags, never the game's class (`docs/kickoff.md`), so it
runs the day the field is published.

1. Download the new field's tag layout (JSON), from WPILib or FIRST.
2. Run `./gradlew cameraStudy --args="--layout=<the file>"`. The field's symmetry is read from its
   tags.
3. Add weights once the strategy says where the robot must localize well, and run it again.
4. Once WPILib is updated for the season, its default field is the new one: no `--layout` needed.

To try a layout of your own (CAD's first draft, say), give it a robot of its own in `Robots.all()`
with those cameras, and run with `--robot=<its name>`: its cameras are "the robot's own".

## What it leaves out

- **Field elements and the robot's own mechanisms blocking a tag:** never modeled; where they
  stand is the game's and the design's. Other robots are modeled only as boxes, and only when
  asked (`--other-robots`, above). So every layout's scores are an upper bound.
- **Motion:** the robot stands still, so there's no blur and no latency. Motion costs every layout,
  more so for far tags.
- **Tilt:** the robot is level.
- **Time:** the estimator combines many images over time, and the wheels between them, so the
  robot does better than one set of images. The study compares layouts; it doesn't predict the
  robot's accuracy on the field.
- **Errors beyond the robustness ranges:** a camera knocked far off its mount, or a lens never
  calibrated, is worse than any trial; calibrate cameras (docs/calibration.md).
- **Small and failed tags:** a tag smaller than 100 square pixels in the image isn't detected,
  PhotonVision's default; a real camera's limit depends on its lens, exposure, and settings. A tag
  whose own solve fails is left out, as the robot leaves out a pose that isn't a number
  (PhotonVision's simulation would stop with an exception); none failed in the example run.
- **The lens:** a perfect lens, with PhotonVision's model of corner noise. Measure a real camera's
  noise before trusting the absolute numbers.

## Later: a tool for any team

The study runs the template's own vision pipeline on the template's own robot configuration. Once
the vision pipeline is a library teams pull in, the study could become a standalone tool any team
runs on its own robot and field, whatever its code. It isn't one yet.
