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
| `kind` | `practice`, `event`, or `official` |
| `base` | The official layout the survey was measured against |
| `layout` | Either of WPILib's formats: the field format (`field-tags`) or the earlier AprilTag layout (`tags`, which WPIcal and PhotonVision use) |
| `moved` | Optional. Why a tag sits more than 10 cm or 5° from its official place |
| `method`, `date` | Optional. How and when it was measured |

A key the format doesn't have is an error, so a misspelling can't pass silently.

**A bare WPILib layout uploads too,** such as WPIcal's output. The robot wraps it as an event
venue, with method "unknown", and says so.

## In the pit

Pitwall's **Pit** view has a **venue** card.
- **Upload venue:** the robot checks the file before keeping it, and lists every problem in words:
  - the field it was measured against, and the field's size;
  - the same set of tags;
  - any tag more than 10 cm or 5° from its official place without a reason;
  - every tag still pairing with its partner on the other half;
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

**A practice field's venue is never used with the field attached.** If one is chosen when the
field system connects, the robot falls back to the official layout and raises a HIGH alert until
the pit chooses the event's venue.

**Logged in full.** The chosen file is logged as an input, so a replay uses exactly that venue.

## Keep it in Git

A venue uploaded at an event lives only on the robot. The card and a MEDIUM alert say **not in
Git** until a build carries the same file. Download it and commit it to
`src/main/deploy/venues/`: a venue there ships with every deploy and can be chosen like an
uploaded one.
