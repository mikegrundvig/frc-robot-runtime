# Venues: the field as measured

A venue is a field as it was measured: the official AprilTag layout corrected where the tags were
found, such as WPIcal's output for an event's field. Fields aren't built exactly to the drawings, and tags on a practice
field are printed on paper and hung on plywood. A robot that knows where the tags really are
localizes better.

Venues are data, never code. Nothing about one needs a rebuild or a deploy at an event.

## The file

A venue file is a small JSON document that embeds WPILib's layout unchanged (`VenueFile` reads it):

```json
{
  "format": 1,
  "name": "Practice field, October",
  "kind": "practice",
  "base": "2026 FRC Rebuilt Welded",
  "layout": { "...": "WPILib's layout, as WPILib or WPIcal writes it" },
  "moved": {"14": "the panel was rehung after the survey"},
  "method": "hand",
  "date": "2026-10-12"
}
```

| Key | What |
|---|---|
| `kind` | `practice`, `shop` ([below](#a-shops-tags)), `event`, or `official` |
| `base` | The official layout the survey was measured against |
| `layout` | Either of WPILib's formats: the field format (`field-tags`) or the earlier AprilTag layout (`tags`, which WPIcal and PhotonVision use) |
| `moved` | Optional. Why a tag sits more than 10 cm or 5° from its official place |
| `method`, `date` | Optional. How and when it was measured |

A key the format doesn't have is an error, so a misspelling can't pass silently.

**A bare WPILib layout uploads too,** such as WPIcal's output. The robot wraps it as an event
venue, with method "unknown", and says so.

## A shop's tags

A shop rarely has room for a field, or a wall where each tag officially goes. A `shop` venue hangs
some of the game's tags wherever they fit. The upload checks it less: its tags aren't judged
against their official places, and their partners aren't checked. The robot still pairs tags by
their official places, so places on the field still come out. It still needs the base's tags, its
field size, and every tag inside the field's walls. Like a practice field's, it's never used with
the field attached.

1. **Pick the tags to hang**, four to eight, from the game's layout, by number. Print each at the
   game's size, the black square's edge measured to the millimeter, flat on something rigid. The
   robot solves images at the game's size, whatever the venue says.
2. **Lay the shop out in the field's coordinates.** Pick a corner of the shop's floor as the
   origin, with x along one wall and y along the other, both pointing into the shop. The shop has to
   fit inside the field's length and width, since the robot refuses an image that puts it off the
   field. A shop bigger than the field uses the part that fits.
3. **Hang each tag** flat, level, and square to the wall it's on. Measure its center: x and y from
   the origin's walls, and z, its height above the floor, each twice to 5 mm. Note which way its
   face points.
4. **Write the venue.** Copy the game's layout, the file the base names. It's in WPILib's fields
   library, such as `org/wpilib/fields/frc/2026-rebuilt-welded.json`. Wrap it in a venue file with
   `"kind": "shop"`, `"method": "hand"` and the date. Then change each hung tag's `translation` to
   what you measured, and its `quaternion` to the way its face points:

   | Face points | W | X | Y | Z |
   |---|---|---|---|---|
   | Along +x | 1 | 0 | 0 | 0 |
   | Along +y | 0.7071068 | 0 | 0 | 0.7071068 |
   | Along -x | 0 | 0 | 0 | 1 |
   | Along -y | 0.7071068 | 0 | 0 | -0.7071068 |

   Leave the tags you didn't hang as they are. The robot never sees them, so where it believes
   they are doesn't matter.
5. **Upload it and choose it** on Pitwall, then commit it to `src/main/deploy/venues/`, below.
   **Check:** with the robot still and a tag in view, `Localization/Pose` is where the robot is,
   measured from the same origin. [The localization check](localization-check.md) then works in
   the shop too, with the spots measured from the shop's tags.

A tag moved since is a new venue: measure it again, and upload the change.

## In the pit

Pitwall's **Pit** view has a **venue** card.
- **Upload venue:** the robot checks the file before keeping it, and lists every problem in words:
  - the field it was measured against, and the field's size;
  - the same set of tags;
  - any tag more than 10 cm or 5° from its official place without a reason;
  - every tag still pairing with its partner on the other half;
  - for a shop, in place of the two above: every tag inside the field's walls;
  - an official venue matching WPILib's layout to 1 mm.
- **Choose:** the robot takes the chosen venue at the start of a loop while disabled, never while
  it drives. The choice stays across reboots and deploys.
- **Download venue file:** gives the file to commit.

The robot keeps uploaded venues outside the deploy folder, so a deploy can't wipe them: on
Systemcore in `/home/systemcore/venues/`, and in the simulator in `build/venues/`. It keeps up to
32; past that, an upload is refused until you delete ones you don't need from that folder.

## What the robot does with it

**The robot's belief changes.** Localization solves images against the venue's tags, and the
cameras' mount checks judge against them, as do places derived from tags. In the simulator, the
simulated world keeps the official field: a venue changes what the robot believes, not where the
tags are.

**What it falls back on.** With no venue chosen, or one the robot can't use, it uses the official
layout, and in the second case a HIGH alert says why.

**A practice field's or shop's venue is never used with the field attached.** If one is chosen
when the field system connects, the robot falls back to the official layout and raises a HIGH
alert until the pit chooses the event's venue.

**Logged in full.** The chosen file is logged as an input, so a replay uses exactly that venue.

## Keep it in Git

A venue uploaded at an event lives only on the robot. The card and a MEDIUM alert say **not in
Git** until a build carries the same file. Download it and commit it to
`src/main/deploy/venues/`: a venue there ships with every deploy and can be chosen like an
uploaded one.
