# Checking localization on the field

This is the last check, once [calibration](calibration.md) is done. It shows whether the robot
knows where it is on a real field: whether its estimate matches spots taped on the carpet, both
standing and just after it drives. It measures the whole chain at once: the venue, the lenses, the
mounts, the capture times, the gyro and the wheels. So a failure says something is off, not which
part. [What a failure points at](#what-a-failure-points-at) says where to look first.

**When:** once calibration is done on a new robot. Then after a camera is moved or rebuilt, and at
each event's practice field, against that event's venue.

**Equipment:** the robot, calibrated, running its program on its Systemcore. A field with its tags
up, and its [venue](venues.md) chosen on Pitwall (the official layout counts as one). Tape, a tape
measure, a square, and a laptop with AdvantageScope, logging.

**Safety:** part 2 drives at match speeds, so follow
[the driving rules](characterization.md#safety-first).

## The spots

Tape four to six spots, each the robot's frame outline drawn on the carpet. The robot's origin, the
frame's center, sits at the outline's center, and the outline's sides give its heading. Spread
them where the robot plays: some 1 to 2 m from tags, some 3 to 4 m away, facing different tags,
and one where only a single tag is in view.

Measure each spot from a tag, not from the field's walls, so the spot is in the venue's own
coordinates.

1. Pick a tag in view of the spot, and look up its pose in the venue: `x`, `y` and its yaw `θ`,
   the way its face points.
2. Measure from the wall under the tag, square to it, to the outline's center: `d` straight out
   from the tag's face, and `s` sideways, positive to your right as you stand facing the tag.
   Measure each twice, to 5 mm.
3. The spot's center is at `x + d cos θ − s sin θ`, `y + d sin θ + s cos θ`.
4. The robot's heading in the outline is the angle its front makes with the field's x axis. Facing
   the tag, it's `θ + 180°`.

Write each spot down: its number, its x, y and heading, and the tag it was measured from.

## 1. Standing

For each spot:

1. Put the robot in the outline by hand, disabled, square to the tape, within 5 mm.
2. Leave it still for 5 s, its cameras on.
3. In the log, average `Localization/Pose` over those 5 s, and note
   `Localization/PositionStdDevMeters`.

**Passes when** at each spot, position is within **3 cm** and heading within **0.5°**, and at all
but one spot the error is under twice the standard deviation the robot reported. That last part
checks the robot's doubt is honest: a robot that's 6 cm off while claiming 1 cm trusts itself too
much, and the gates that keep bad images out rely on that number.

Why these numbers: 0.5° of heading puts an aim 3.5 cm sideways at 4 m, and 3 cm is about what
scoring at range tolerates. The tape measure is good to about 1 cm over a few meters, which leaves
the robot 2 cm. They're starting points, not measured limits: tighten or loosen them once a real
robot shows what's reachable, and say why beside them.

## 2. Just after driving

A robot can read well standing and still be off while it drives: images taken on the move, stamped
late, or blurred. The cameras then pull the estimate back once it stops. So the check here is how
far the estimate moves once the robot is at rest, which needs no tape:

1. From a spot, drive a route like a match's for about 10 s at match speeds: straights, curves,
   strafing and a spin, past tags and away from them.
2. Stop anywhere with tags in view, and hold still for 3 s.
3. In the log, find the first loop at rest. Take the distance between `Localization/Pose` there
   and its average over the last second.
4. Do it five times, from different spots.

**Passes when** each move is under **8 cm**, and heading moves under **1°**. That's what aligning
and shooting on the move tolerate.

A move past 8 cm that happens only after fast driving points at capture times or motion blur. One
that happens after a spin points at the gyro's scale.

## What a failure points at

| What you see | Look first at |
|---|---|
| Every spot off the same way, by about the same amount | The venue: the tags aren't where it says. Check one tag's place with a tape measure |
| Off only facing one tag, or one camera's spots | That camera's [mount](calibration.md#a-cameras-mount), or that tag in the venue |
| Worse the farther the tags are | [Lens calibration](calibration.md#done-elsewhere), then [camera noise](characterization.md#camera-noise) |
| Heading off by the same angle everywhere | The camera mounts' yaw, or `minTagsForHeading` ([values.md](values.md)) |
| Standing passes, part 2 fails after fast driving | [Capture times](calibration.md#a-cameras-capture-time) |
| Part 2 fails after spins only | [The gyro's scale](calibration.md#the-gyros-scale) |
| Error well past twice the reported standard deviation | Camera noise characterization: the estimator trusts its images too much |

Fix the part, then run the whole check again, standing first.

## Keep the results

Write each run down with its date, the venue, the robot, and each spot's error. A result to beat
is how a team knows a change helped.
