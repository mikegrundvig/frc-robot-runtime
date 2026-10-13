# Design decisions

This page says why. The [guide](guide.md) shows what a team writes, and
[coming from WPILib](coming-from-wpilib.md) maps it onto code you know. Neither says much about the
reasons. This page takes each choice that differs from WPILib's defaults, or from what most teams
do, and gives the reasoning, the evidence, the cost, and what would make us reverse it.

Every section has the same six parts: what most teams do, what this does, why, the evidence and how
strong it is, what it costs, and what would change our mind. Some end with a line of what is not
built yet.

## Read this first

**Nothing here has driven a real robot.** The stack targets WPILib 2027 alpha on Systemcore, and no
such robot has run it. Every test is in the simulator, or against a vendor's simulated device. A
section says so again only where it changes what the evidence is worth.

**This is pre-release, and parts of it are unfinished.** "Not built yet" marks work that is
intended. It is not a cost of the decision it sits under, and it is not an argument against it.
"What it costs" holds only what a finished design choice trades away.

**The code is open so that you can add to it.** Where the runtime lacks something your robot needs,
such as a kind of joint or a driver for a device, the usual answer is to add it, and the sections
say how.

**Each piece of evidence has a grade.** *Measured* means we ran something and have a number.
*Counted* means a figure from the census below. *Experience* means we did it the other way first and
changed. *Argued* means a design argument with no measurement behind it. *Precedent* means several
top teams' public code does the same thing.

**The census.** We read 100 public FRC robot codebases from the 2025 and 2026 seasons, from 74
strong teams. The reading was automated, and each finding was checked by a second pass whose only
job was to prove it wrong. 2,252 bugs survived, and 253 were rated serious. A serious bug crashes
the code, loses a scoring cycle under ordinary conditions, or can break something. The census
sorts bugs into twelve classes, and this page cites them by name. Read it with three cautions.

- These are strong teams that publish their code. The counts show these bugs happen at the top,
  not how often they happen everywhere.
- A count is of bugs. Whether this stack would have stopped one is a separate judgment that we
  made for each serious bug. By it, 162 of the 253 would not have survived, with no extra work
  from the team. No judgment has been proven by writing the bug into a robot built on this stack.
- The census is not published, because it quotes other teams' code. You cannot check it yet.

Where a section says a mistake is ruled out, it says which way: it is **impossible to express**,
it is **refused at startup**, or it is **caught by a test the stack ships**.

## The decisions

They run from the most fundamental to the least.

| # | Decision | In five words |
|---|---|---|
| 1 | [Describe the robot](#1-describe-the-robot-do-not-write-subsystems) | Typed description replaces subsystem classes |
| 2 | [Refuse at startup](#2-refuse-at-startup-and-doubt-out-loud) | Wrong numbers stop the program |
| 3 | [Interlocks in the description](#3-interlocks-live-in-the-description) | Safety rules sit below commands |
| 4 | [Named places](#4-named-places-and-a-practice-robot-as-differences) | Names, not angles, in behaviour |
| 5 | [Values in code](#5-values-in-code-one-owner-per-fact) | One owner for every fact |
| 6 | [Commands v3](#6-commands-v3-coroutines) | Coroutines replace lifecycle and groups |
| 7 | [Outcomes and gates](#7-outcomes-and-gates) | Every move says how it ended |
| 8 | [Disabled means disabled](#8-disabled-means-disabled) | Nothing carries over a disable |
| 9 | [Places for blue](#9-field-places-are-written-once-for-blue) | Nobody flips a pose by hand |
| 10 | [A driver contract](#10-a-written-contract-for-hardware-drivers) | Real and simulated drivers agree |
| 11 | [The simulator](#11-a-simulator-built-from-the-description) | Simple physics, real faults, repeatable |
| 12 | [Whole-robot tests](#12-whole-robot-tests-on-a-stepped-clock) | Run the robot, not mocks |
| 13 | [Localization](#13-localization-sits-behind-a-seam) | Two estimators behind one seam |
| 14 | [No SysId routine](#14-no-sysid-routine) | Seeded feedforward plus step tests |
| 15 | [Paths in code](#15-paths-and-autos-in-code) | In code first, not finished |
| 16 | [Pitwall](#16-pitwall-instead-of-dashboard-tuning) | Tune live, commit in code |
| 17 | [Separate repositories](#17-separate-repositories) | One boundary for each owner |
| 18 | [The platform](#18-java-25-wpilib-2027-alpha-systemcore-only) | 2027 only, no roboRIO |

## 1. Describe the robot, do not write subsystems

**What most teams do.** You write a subsystem class for each mechanism and a constants file beside
them. With AdvantageKit you add an IO interface, a real IO and a simulated IO for each. You
configure each motor controller by hand. Strong teams keep a base class for "a motor that goes to
positions" and reuse it for years.

**What this does.** You write one block of typed constants for each joint: its range, motors,
gearing, wiring, sensors and control. The runtime builds the mechanism, its motor controller
settings, its simulated plant and its pit page entries from that. Most values carry a unit type.

**Why.** Most of a subsystem class is plumbing: ratios, unit conversions, controller settings,
setpoints and a check for "has it arrived". That plumbing is where several of the census's classes
sit. If it is generated from one checked description, a team writes only behaviour, and the real
robot, the simulator and the pit page cannot disagree about a gear ratio.

**Evidence, and how strong it is.** Counted, for the plumbing. *Units, ratios, signs and angle
wrap* has 193 bugs on 49 robots, 43 serious. A value with a unit type cannot be read in the wrong
unit, so that mistake is impossible to express for those values. We judged 20 of the 43 stopped
that way or by the description's checks. Fifteen are in a team's own aiming math, which is still
the team's, and eight need the real robot. *Feedforward, gravity and holding* has 35 bugs on 12
robots, none serious. The gravity term's kind comes from the joint's kind, so an arm given an
elevator's term is impossible to express.

Argued, for the claim that students do better with a generated layer than a hand-written one. We
found no evidence either way, and users of the public mechanism libraries we looked at complain that
they are opaque.

**What it costs.** Students learn less about configuring a motor controller. Code you did not write
is harder to debug at an event, and the joint's log is the help you get. Each joint is its own
`Mechanism`, which is finer than students are used to.

**Adding a kind of joint.** Today the runtime drives a motor, or several together, turning one
shaft: a pivot or arm, an elevator or slide, a turret, a roller or flywheel. A joint moved by
servos, a differential pair, and an arm moved through a linkage are not built yet. You add one the
way the existing kinds are made. The model's `Joint` and `MechanismControl` say what the joint is
and how it is driven, a `JointIO` driver in the runtime turns requests into what the hardware takes,
and `RobotHardware` picks the driver from the block's wiring. A new driver is checked by giving the
conformance suite a bench for it (section 10), and `JointMechanism` then gives the joint its moves,
gates and interlocks as it does for any other. For one odd mechanism there is a lighter route. Keep
the joint in the description, leave out its control, and drive it with a v3 `Mechanism` of your own.
[mechanisms.md](mechanisms.md#8-when-the-template-cant-drive-it) has a worked hood on linear servos.

**What would change our mind.** A robot whose mechanisms take more work to describe than they would
take to write as subsystems. Or students who cannot work out from a joint's log why it will not
move, without the mentor who wrote the runtime.

## 2. Refuse at startup, and doubt out loud

**What most teams do.** A setpoint past a hard stop is sent anyway, or clamped without a word. A
failed configuration is printed to the console. The mistake shows up when the mechanism moves.

**What this does.** The description is checked as it is built. A value that cannot work stops the
program, on the robot and in the simulator, with a message that names the value and its limit.
The template's `RobotsTest` builds every robot a team has, so the same refusal fails the build on
a laptop. A value that is allowed but looks wrong is a doubt. The robot starts, and the doubt goes
to the log under `Robot/Doubts` and to the Driver Station's console.

**Why.** Nobody reads a console at an event, so a warning that scrolls past is no warning. A
program that will not start cannot be ignored.

The danger is a robot that will not start in the queue. Three things keep that small. The checks run
in a test before any deploy. A refusal is kept for what cannot work, such as a place outside its
joint's hard stops or a speed past the motor's free speed, and a matter of judgment is a doubt. And
a move that cannot be, built while the robot runs, ends `REFUSED` and the robot carries on.

**Evidence, and how strong it is.** Counted, and modest. Of the 253 serious bugs we judged 11
refused at startup. The plain case is a scoring position set past the mechanism's travel. The
checks catch arithmetic and bookkeeping, which is a small part of the whole. The rest is argued.

One risk only a real robot can show. A check works from the description's own figures, such as a
motor's free speed. If a figure is wrong, the check refuses a robot that works.

**What it costs.** One wrong number means no robot until it is fixed. A check can be wrong. Doubts
can pile up until nobody reads them, which is the problem warnings had.

**What would change our mind.** A refusal that stops a working robot at an event because the check
was wrong. That check becomes a doubt the same day. If it happens with several checks, doubting is
the right default and refusing is not.

## 3. Interlocks live in the description

**What most teams do.** A safety rule between two mechanisms is an `if` inside a command, written
again in the next command. Strong teams route every request through a superstructure state
machine, which they rewrite each season.

**What this does.** The rule is written once, on the joint it restricts, as `runsOnlyWhen` or
`keepsOut(...).when(...)`. It is applied every loop, after the command's request and before the
output. If the joint it reads does not know where it is, the rule assumes the worst and holds. At
startup the rules are checked against each other and against every configuration, and two that would
leave joints holding each other for good are refused.

**Why.** The class of bug is a check that one command has and the next one forgot. Often it was
added for teleop and missed in an auto. A rule under every command cannot be forgotten by a new
command, a pit action or a test. It also removes ordering code. You tell two joints to move at
once and the rule produces the right order.

**Evidence, and how strong it is.** Counted, and the class is small. *Limits and interlocks* has
91 bugs on 35 robots, 9 serious. We judged four of the nine impossible to express when the rule
sits under every command, three refused at startup, and one caught by static analysis. The ninth
is an interlock nobody thought to write, and nothing fixes that.

We also looked for reports of forgotten interlocks at events and found design threads but very few
incidents. So this failure is rare. It is worth stopping because it bends metal. And the band a
keep-out holds has only been checked in the simulator. How far a real joint overshoots its edge
depends on sensor delay and momentum, and nobody has measured it.

**What it costs.** Behaviour is in two places. The command says go, the description says wait, and a
student reading the command cannot see why the roller is not spinning. The answer is in the log
under `HeldBy`. A held joint keeps its command, so a move can time out while it waits.

**Not built yet.** A bench bypass for interlocks, and moving between configurations by a route.

**What would change our mind.** A robot whose real collision rules cannot be written as
conditions, so the team writes a state machine anyway and has two sources of truth. Or drive teams
confused by held joints more often than a forgotten check would have hurt them.

## 4. Named places, and a practice robot as differences

**What most teams do.** Setpoints are constants in a file, or literals in a command. A practice
robot is an `if (isPractice)`, a second constants class, or a branch.

**What this does.** A joint lists its places in its own block, each checked against its hard stops,
and behaviour says `pivot.goTo(PIVOT_OUT)`. A practice robot is the competition robot's description
with its differences: a CAN ID, an encoder offset, a place two degrees away.

**Why.** "The intake is out at 130 degrees" is a fact about the intake, the same kind of fact as its
range. Next to the range, it can be checked, and the name shows up in every outcome line and on the
pit page.

A practice robot differs in a few numbers. Two copies of the constants drift apart, and a fix made
on one robot is missing on the other. A list of differences cannot drift, because everything not
on the list is shared.

**Evidence, and how strong it is.** Mostly argued. One case is counted: a named place past its
joint's travel is refused at startup, which is among the refusals in section 2. Precedent, for the
practice robot. Several top teams' public code runs one code base on several robots, chosen when
the program starts. Ours has only been tried on robots described in tests, never on a real pair.

**What it costs.** A literal still works. `goTo(Degrees.of(130))` compiles, so this is a habit and
not a guarantee. A place has to be given a name before it can be used, which is ceremony for a quick
test.

**Not built yet.** Tuning a named place live, and a practice robot that differs in motor count or
drivetrain gearing.

**What would change our mind.** A practice robot whose list of differences grows as long as the
description. Or teams that write literals anyway because naming a place first gets in their way.

## 5. Values in code, one owner per fact

**What most teams do.** Robot facts are spread over a constants file, a swerve generator's file, a
path tool's settings, values saved on the robot from a dashboard, and settings stored in the motor
controllers. The same fact is often in several of them.

**What this does.** Every fact about the robot is in its Java description, in the block of the thing
it describes. The runtime reads no configuration file for them. Where another tool needs a file, it
is generated from the description. A venue's measured tag layout is the one kind of file the robot
reads, because it is data measured at an event. The [values register](values.md) lists every value
and where the template's number came from, and a test fails when one has no row.

**Why.** The class of bug is a number written in two places that disagree, usually the code and a
tool's file. Code also gets what a file does not: the compiler, a rename that reaches every use, a
diff in review, and a test that builds every robot. A value saved on the robot is worse than a
copy. It survives a redeploy, so a tweak from a month ago can override today's code and nobody can
see it in the repository.

**Evidence, and how strong it is.** Counted for how common it is, and weak for how much it hurts.
*The same number written in two places* has 249 bugs on 25 robots, and only 5 are serious. We
judged one of the five refused at startup. The other four need a test that a team writes. So this
decision is about numbers nobody can explain, not crashes.

The register is honest about the template's own numbers. About 130 of its rows are marked as
guesses, and none is marked as measured on a robot.

**What it costs.** Changing a number means a rebuild and a deploy, unless it is one of the values
the pit page tunes live. Someone who does not write Java cannot edit a number. A tool that expects
its own hand-edited file gets none.

**What would change our mind.** A tool the team needs whose file cannot be generated from the
description. That file then needs an owner, and the rule needs an exception that says who.

## 6. Commands v3 coroutines

**What most teams do.** Commands v2. A command is a class with `initialize`, `execute`,
`isFinished` and `end`, or it is built from factories such as `sequence`, `parallel`, `race` and
`until`. WPILib 2027 ships v2 and v3 side by side, and v2 is what the documentation, the examples
and most vendor tools use.

**What this does.** v3 only. A command's body is a coroutine, so you write steps in order with plain
`if` and `while`, and `co.fork`, `co.await` and `co.awaitAny` replace the groups. Every driven joint
is a v3 `Mechanism`.

**Why.** A sequence reads top to bottom. In v2 the same logic is a tree of factory calls, and a
group that waits on the wrong member is hard to see. A forked command ends when its parent ends,
so "do this while that runs" cannot outlive its parent.

A coroutine's state is local variables, made fresh on every run. A v2 command object keeps its
fields between runs, and a timer that was started and never reset breaks the second run.

A command can also branch on how a move ended with a plain `if`, which the next section needs.

**Evidence, and how strong it is.** Argued. We do not claim a census class for this decision. The
nearest is *timing and leftover state*, with 192 bugs on 54 robots and 16 serious, and we judged
only 3 of the 16 impossible to express here. The biggest command class is about waits with no time
limit, and a coroutine does not fix that. `co.await` on a command that never ends still waits
forever. Nobody has measured whether students write fewer bugs in coroutines, and v3 is an alpha.

**What it costs.** v3 has traps of its own, and the guide for WPILib teams lists them. A forked
command dies with its parent. A command that has run cannot be scheduled again. Every tutorial your
students find will be v2. WPILib's build refuses a project with both the v2 and the v3 vendor files,
so a library written for v2 cannot be added the usual way. The v3 API can still change before
release.

**What would change our mind.** WPILib dropping or sidelining v3. Or a season with students that
shows the coroutine traps bite more often than v2's groups did.

## 7. Outcomes and gates

**What most teams do.** A command finishes or is interrupted, and that is all the code learns. Each
subsystem writes its own `atSetpoint()`. Timeouts are added by hand where someone remembered. A
`Trigger` is a lambda with no name.

**What this does.** Every move, on a joint or the drivetrain, ends as `ARRIVED`, `TIMED_OUT`,
`CANCELED` or `REFUSED`, with one line in the log that says why. A move to a place has a default
deadline of twice its profile's time plus half a second. A gate is a trigger with a name, and when
it is false it logs which of its terms are false. Each joint has its gates built in.

**Why.** Two census classes. In *commands that wait forever or end wrong*, a step waits for
something and nothing says what to do if it never comes, so in autonomous the robot sits still for
the rest of the period. In *readiness checks that are wrong or out of date*, a hand-written "has
it arrived" passes at once, or one condition is left out. Here the arrival check is answered once,
by tested code, for every joint. And "why isn't it shooting?" has an answer in the log that a
student can read when the person who wrote the code is not there.

One consequence. `runUntil` has no default deadline, because nothing says how long a game piece
takes to show up. It had a hidden default of three seconds, and we took it out. A deadline nobody
chose ends moves for reasons nobody can explain. You write `.within(...)`.

**Evidence, and how strong it is.** Counted, with one claim we walk back. *Commands that wait
forever or end wrong* has 309 bugs on 78 of the 100 robots, 28 serious. It is the most widespread
class. *Readiness checks* has 230 bugs, 21 serious.

A move to a place with no deadline is impossible to express. The hand-written arrival check is
gone too, and we judged 6 of the 21 serious readiness bugs stopped that way. The other 15 are game
logic, such as which conditions matter for a shot, and only a test the team writes knows that.

The census's scorecard says 21 of the 28 serious command bugs could not be written here. We now
think that is too generous. By our reading, 13 of those 21 are waits on a sensor, a flag or a
readiness check, and here such a wait has no deadline unless you write one. The stack makes the
deadline one call and makes its absence visible in review. It does not force it.

**What it costs.** A default deadline can end a move that would have finished. The formula is a
guess, and a real mechanism more than twice as slow as its model would time out on a healthy move.
An outcome nobody reads lets an auto carry on with the arm not where it should be, which can be
worse than stalling.

**What would change our mind.** Default deadlines that fire on healthy moves on a real robot. Or
an auto that does damage by carrying on after a time-out. Then an unread `TIMED_OUT` should stop
the auto by default.

## 8. Disabled means disabled

**What most teams do.** Nothing, because v2 does it for them. The v2 scheduler cancels a command
when the robot is disabled unless the command says it runs while disabled. The v3 scheduler, as of
the 2027 alpha, has no such rule. It runs every command in every mode.

**What this does.** On the loop the robot goes from enabled to disabled, the runtime cancels every
running command, and each move's outcome says "the robot was disabled". While disabled, no joint
and no wheel is driven, whatever asks, in the simulator too. A button pressed while disabled is
spent and starts nothing later. When the robot is enabled again, each joint holds where it is, and
only a default command starts by itself.

**Why.** The class is leftover state across a disable. A move part way to an old target carries on
when the robot is enabled. A trigger held through the disable spins its roller up again. A
deadline counts down while the robot sits still. A robot that moves on enable with nobody asking
is a hazard in the pit. We found all of these by writing the tests. The first scenarios showed
that disabling cancelled no command at all, and that the simulated drivetrain drove while
disabled.

**Evidence, and how strong it is.** Argued, and tested in the simulator. `DisablingSimTest` runs
fourteen scenarios on the guide's robot. No census class is cited, because the census's robots run
v2, where the scheduler already does this.

A real robot's motor outputs are cut by the control system while it is disabled, whatever the code
does. So the half of this rule about not driving mostly keeps the simulator honest. The half about
cancelling is what changes a real robot, and it has not run on one.

**What it costs.** No command can survive a disable. v3 has no way for a command to say it should,
and we did not add one. A press made while disabled is dropped, so a driver holding a trigger as
the robot is enabled gets nothing until they let go and press again. If the field disables the
robot for a moment between autonomous and teleop, as fields have done, that happens every match.

**What would change our mind.** A mechanism that needs its command to live through a disable. Or
drivers who lose their first teleop action to a spent press often enough to notice.

## 9. Field places are written once, for blue

**What most teams do.** Field poses are constants. A flip function is called wherever someone
remembers to call it. The alliance is read from the Driver Station, often once, in a constructor.

**What this does.** A `Place` is written once, for the blue alliance, and resolved for the robot's
own alliance when a move starts. The field's symmetry is declared once, in the season's game. An
architecture test fails the build when code reads the alliance as a class loads or as an object is
built.

**Why.** The class is alliance mistakes. The alliance is read at startup, before the Driver
Station has said which it is. A pose is flipped twice and lands back on the wrong side. A rotated
field is treated as mirrored. These bugs show up in half your matches and never on a practice
field set up for one alliance.

**Evidence, and how strong it is.** Counted, and it is the strongest count on this page. *Where
the robot is, and which alliance* has 182 bugs on 66 robots, 44 serious. Alliance mistakes alone
are 114 of those bugs, on 63 robots.

We judged 35 of the 44 serious ones ruled out here. A caller cannot flip a place twice, because no
caller flips. That is impossible to express. An alliance read at startup is caught by a test the
stack ships, the architecture test. Three more need a test on both alliances, and `AutoCheck` runs
every declared auto on both. The last six are in how a path tool flips, and there is no path tool
here.

It is not airtight. `drive.goTo` also takes a raw `Pose2d`, and a team's own math on a raw field
coordinate is the team's.

**What it costs.** You think in blue, always. Someone has to write each season's field and its
symmetry into the model on kickoff day.

**What would change our mind.** A game whose field is not symmetric, so that most places need both
alliances written out by hand.

## 10. A written contract for hardware drivers

**What most teams do.** With AdvantageKit, a mechanism has an IO interface, a real implementation
and a simulated one. Nothing checks that the two behave alike, and a difference between two vendors'
controllers is found on the robot.

**What this does.** The joint driver's promises are numbered clauses in `JointIO`'s Javadoc, J1 to
J40. The swerve module driver's are M1 to M18, in `ModuleIO`'s. One conformance suite runs against
every joint driver (TalonFX, TalonFXS, SPARK MAX and SPARK Flex), each on its vendor's simulated
device, and against the simulator's two emulated controllers. So the simulator is held to the
contract the real drivers are held to. A test fails if the clauses in the Javadoc and the clauses
the suite checks are not the same list. Where a driver is known to differ, the clause says so.

**Why.** The simulator is worth something only if a simulated controller behaves like the real
driver in the ways robot code relies on. A contract says which ways those are. It also makes the
vendors agree, so a joint acts the same on a TalonFX and on a SPARK.

Writing it turned up questions nobody had decided. Does kI apply to a speed request? Is a current
signed? Does a freely turning joint take the short way round? The drivers and the simulator gave
different answers. Each now has one answer in a clause. A current is a size, never negative (J23).
A position is taken as its number says (J40). The half second before a silent device reads "not
connected" is kept in whole nanoseconds, so it ends on the same loop every run (J27).

**Evidence, and how strong it is.** Measured, on vendors' simulated devices only. The suite has
23 tests for a joint driver and 7 for a module driver. Every driver passes them, with a few
skipped where a simulated device cannot do what the test asks. Two known differences are written
into their clauses and left open for a real device (J29 and M15).

Counted, for how common device trouble is. *Devices: wrong setup, or failures nobody notices* has
148 bugs on 42 robots, 15 serious. The clauses about a silent or restarted controller are aimed at
that class.

The caveat is large. A vendor's simulated device is not the device. REV's simulation ignores several
things a real SPARK does, and [stand-ins.md](stand-ins.md) lists them. A driver that passes may
still break a clause on hardware.

**What it costs.** There are 58 clauses to keep true. The contract is a lowest common denominator. A
feature that only one vendor's controller has is left unused so the drivers agree.

**Adding a driver.** A driver for another motor controller is a new `JointIO`. You give the suite a
bench for it (`JointBench`), and it must pass each clause or write down where it differs.

**Not built yet.** The module suite checks some of its clauses, and `ModuleIO`'s Javadoc says which.
A gyro and a camera have no contract.

**What would change our mind.** The first bench session with real devices. If real controllers
break many clauses that their simulated versions pass, the suite proves little, and running it
against real devices on a bench becomes the requirement.

## 11. A simulator built from the description

**What most teams do.** Many teams do not simulate. Those who do write a simulated IO for each
mechanism with WPILib's motor and mechanism models. Some add a physics library such as maple-sim
for the drivetrain and the game pieces.

**What this does.** The simulator is built from the robot's description, and you write nothing for
it. Each joint is a closed-form plant. The drivetrain is kinematic: the robot moves as the best fit
of its wheels' travel, within the tread's grip. The simulator keeps the robot's true pose apart from
what the code believes, its sensors err in ways you configure, and faults can be made while it runs.
There is no rigid-body engine, and a game piece is a sensor reading that a test sets.

**Why.** We asked what simulation pays for. Testing the robot's logic against simple plants pays,
and strong teams do it every season. We looked for a team reporting a payoff from high-fidelity
drive physics or from game-piece physics, and found none.

Simple physics also repeats. A run gives the same result to the bit, and many worlds run side by
side in one process. Tests and studies rest on that. When we evaluated maple-sim in 2026 it was 2D
and kept state shared between every robot in the process, and neither fits.

Faults matter more than fidelity. Much of what goes wrong in a match is a sensor that lies or goes
quiet. And the hardware for 2027 is not here yet, so the simulator is the only place to work.

**Evidence, and how strong it is.** Precedent, for simple plants. Several top teams' public code
tests its superstructure against simulated plants, and one models its game pieces as a state
machine that drives simulated beam breaks. The search for a payoff from fidelity found nothing,
which is absence of evidence and weaker than a measurement.

Measured, for how wrong the physics is. The truth suite compares the simulator with answers worked
out from physics and records each shortfall as a known gap. A launch accelerates at 4.35 m/s²
where the physics says 4.45. The robot spins as if its yaw inertia were 11.1 kg m² when its body
is configured as 6.7. All four wheels slip at once, where a real robot's front wheels slip first.
[simulator.md](simulator.md#how-far-to-trust-it-the-truth-suite) has the full table. No simulated
number has been compared with a real robot's log.

**What it costs.** An auto that works in the simulator shows the logic is right. It does not show
the robot will do it. Traction, weight transfer, battery sag and contact with game pieces are left
out on purpose. Gains tuned there are starting points. The sensor error models are guesses, and the
values register marks them so. There is no arena of game pieces and other robots, as maple-sim has.

**Not built yet.** Some faults, which
[simulator.md](simulator.md#what-the-simulator-cant-make-fail-yet) lists, and failing a joint from
the pit page. A test can already do the second.

**What would change our mind.** The first real logs. If a real joint or drivetrain responds far
from its simulated one, the fidelity has to go up. If field failures keep tracing back to traction
or power, those get modelled. If a physics library can run many independent, repeatable worlds, we
look at it again.

## 12. Whole-robot tests on a stepped clock

**What most teams do.** No tests, or a few unit tests of math. Teams with IO layers test a
subsystem against a fake IO. CI checks that the code compiles.

**What this does.** The common test builds the whole robot in the simulator and steps the clock.
It presses buttons, runs loops and reads outcome lines. The stack ships more of the same kind.

- `AutoCheck` runs every declared auto on both alliances.
- The monkey plays a match of random input and faults from a seed, and checks invariants after
  every loop.
- Recorded sessions must come out the same on every run, numbers within a millionth.
- Loop-cost tests put a ceiling on the bytes a loop allocates.
- Architecture tests fail the build when a component reads the clock, draws an unseeded random
  number, or keeps static state. Static analysis fails it on a possible null.

A flaky test is a bug. It is fixed when it is found, not retried.

**Why.** A test against a fake IO checks what its author believed the IO does. The bugs between
parts need the whole program running: a mode change at the wrong moment, a command run twice, a
move interrupted half way. A clock that moves only when the test steps it gives the same answer on
every laptop, so a red test means something. A test that fails one run in five teaches everyone to
ignore red.

That rule reached into the runtime. WPILib's `Debouncer` keeps time as seconds in a double, and a
debounce of a whole number of loops landed a loop early or late from one run to the next. It
showed up as a flaky test. The runtime's `Debounce` keeps whole nanoseconds.

**Evidence, and how strong it is.** Counted, by our judgment of where each serious bug would have
been caught. Of the 253, we judged 16 caught by static analysis, 11 by the monkey, and 7 by a test
that starts the robot and runs an auto. Nobody has written those bugs into a robot and watched the
monkey find them. And the largest remainder points the other way. We judged that 63 of the 253
need a test that somebody writes, because they are game logic. The stack gives you the harness. It
does not give you those tests.

Measured, for repeatability: the truth suite checks that a run is bit-identical alone, repeated,
beside seven others, and in another JVM. Measured, for allocation: the drive's loop allocates
about 4.5 to 5.4 KB a loop in the three situations its test runs. That is not zero, and whether it
matters on a Systemcore is not known.

**What it costs.** These tests are slower than unit tests. A recording has to be made again when
behaviour is meant to change, and someone has to read the diff. A test can pass on physics that is
wrong. The static analysis is friction for a student who wants to try something.

**What would change our mind.** Bugs at events of the kinds the monkey and the simulator tests are
meant to catch. Or recordings made again so often that nobody reads the diffs.

## 13. Localization sits behind a seam

**What most teams do.** WPILib's `SwerveDrivePoseEstimator`, fed by PhotonVision or a Limelight.
In front of it sits a filter of threshold rules: ambiguity, height off the floor, inside the
field, distance, turn rate. Standard deviations are scaled by distance and tag count. Several top
teams solve each image with the gyro's heading.

**What this does.** Localization is behind one interface with two implementations, in a repository
of its own, and a team picks one. The first is the runtime's own estimator. It applies a late
camera measurement at the time its image was taken, gates each measurement statistically against
the estimate, recovers when several rejected measurements agree with each other, and lowers its
trust in the wheels when they slip. It solves nearly every image with the gyro's heading, and it
estimates what the gyro gets wrong: its scale and its steady drift, from the cameras as the robot
drives and from the gyro itself while the robot stands still. It watches each camera against the
others, and takes off what a camera knocked on its mount or stamped late puts its measurements
off by. The second is WPILib's estimator, handed the same measurements from the same
image-solving pipeline. You choose it with one line in the robot's configuration, and
[localizers.md](localizers.md) has the contract a localizer of your own is checked against.

**Why.** We built the custom estimator on a design argument. Then we compared it with WPILib's,
and the first result did not justify making ours the only choice: ours lost wherever the gyro or a
camera mount was off. So the decision is the seam, not the estimator. We then changed the
estimator against that result and measured again. The seam stays, because the new result is still
a simulation's.

One design choice explained most of the first table. Solving an image with the gyro's heading
removes the ambiguity of a single tag, which helps against bad camera data. But a heading error
then becomes a position error, the same in every image, and all the cameras agree on the wrong
place. The estimator now uses that: two cameras whose tags lie in different directions disagree
by the heading's error, and that disagreement corrects the heading and tells the estimator how
its gyro misreads.

**Evidence, and how strong it is.** Measured, in simulation only. The study ran 41 scenarios with
20 seeds each, and both estimators saw the same simulated robot in the same loop. WPILib's was the
2027 alpha's, behind a filter whose rules came from several top teams' public code, tuned once and
left alone. That is WPILib's estimator as a good team runs it, which is the fair opponent. It is
not the second localizer as it ships: that one sits behind our pipeline, and a robot running it
was not what the study measured. The figures are position error, RMS over the run. "Before" is
ours before it corrected its gyro and watched its cameras.

| Scenario | Ours before | Ours | WPILib's |
|---|---|---|---|
| Clean driving, short route | 0.6 cm | 0.5 cm | 0.8 cm |
| Clean driving, full match | 0.9 cm | 0.7 cm | 1.4 cm |
| A camera misreading a tag, repeatedly | 0.6 cm | 0.6 cm | 2.2 cm |
| A camera frozen on an old image | 0.6 cm | 0.5 cm | 13.7 cm |
| Low grip | 2.0 cm | 2.0 cm | 6.2 cm |
| A stale wheel encoder | 1.9 cm | 1.9 cm | 3.9 cm |
| Gyro reading 3% high | 9.1 cm | 0.6 cm | 0.8 cm |
| A camera knocked on its mount | 2.2 cm | 1.0 cm | 1.0 cm |
| A camera's clock 20 ms off | 2.3 cm | 0.8 cm | 0.8 cm |
| A match with several faults in a row | 5.9 cm | 4.3 cm | 4.0 cm |

In clean driving the two are the same for any practical purpose, since a centimeter is below what
the simulator can resolve. Ours does better against bad camera data and wheel slip. With the gyro
or a camera mount off, the two are now within a centimeter of each other, where ours lost before.
With the gyro 3% high its heading error is 0.1° against 0.5°, where it was 2.0°.

A second set of 19 scenarios was written before the estimator was changed and looked at only
twice, before and after. It varies what the first set tested: other sizes of gyro and mount
error, two faults at once, fewer cameras. Over its 87 comparisons with WPILib's, ours was behind
in 25 before the changes and in 1 after. The
[estimator study](../../docs/estimator-comparison.md#what-it-found) has both tables.

WPILib's is still ahead in two ways. Ours moves the estimate in one step when it decides the
estimate is wrong, 60 cm after a shove where WPILib's largest step is 11 cm, and an auto-align
that follows the estimate feels it. And with a camera turned 1.5° on its mount, in the second
set, ours was 1.7 cm off against 0.8 cm: too small a turn for its watch to act on.

Read these with care.

- The slip and collision models are the simulator author's, and the custom slip handling was
  developed against them. Its wins there are on home ground.
- The simulated gyro's fault is a constant scale or a constant drift, which is exactly what the
  estimator now estimates. A real gyro's errors move with temperature. This is the result we trust
  least.
- The simulated cameras are ideal. Every tag in view is found, with no blur and a perfect lens,
  and they scatter far less than a real camera does, which makes a knocked camera easy to see.
- One family of scenarios, with tags sitting off the field layout, favoured ours by a wide margin.
  The study could not check that size against real data, so we do not quote it.
- When WPILib's filter was fed the measurements our pipeline produces, it came within a centimeter
  of ours in 35 of the 41 scenarios. Most of the difference is in how an image is solved and
  gated, not in the filter.
- Ours is still surer of itself than it should be in some scenarios. With every camera mount off
  by twice what the configuration allows, its heading was outside its own 95% bound in a quarter
  of the loops.

None of this says anything about a real robot.

**What it costs.** There are two implementations to keep working and a contract between them. The
custom estimator is a lot of code that a student will not debug at an event. WPILib's needs its
rules and standard deviations tuned, and the study found no single tuning that is best everywhere.
A team has to choose, and choosing well means understanding this section.

**What would change our mind.** A test on a real chassis. Log the raw detections, the wheels and
the gyro, replay the same log through both, and use taped checkpoints as the truth. Add faults
that cost nothing: a camera loosened and turned a few degrees, the gyro's scale left uncalibrated,
a shove, a wheel lifted. The gyro is the first thing to check: whether the scale and drift the
estimator finds on a real gyro are steady enough to be worth correcting, and whether its heading
stays inside its own bound. If the wins do not hold up, WPILib's becomes the default and ours
shrinks to a gate in front of it.

## 14. No SysId routine

**What most teams do.** WPILib's SysId. A routine in robot code runs ramp and step tests, the
robot logs them, a desktop tool fits kS, kV and kA, and someone copies the gains back. Many teams
skip it for arms and elevators and tune by hand.

**What this does.** A joint's kV, kA and kG start from its description, worked out from the motor,
the gearing and the load's mass if you gave one. You overwrite them when you have measured values.
The pit page's Experiment view runs step tests made from each joint's own places or speeds, and
scores each run for rise, overshoot, settling and steady-state error. There is no SysId routine in
the runtime. Nothing stops you running SysId by hand and writing its numbers in.

**Why.** The method is sound, and the cost is in its seams. You need a routine in robot code, a
log on the robot, a desktop tool, a vendor's high-rate logger for a short mechanism, and gains
copied back by hand. The seeded feedforward and the step tests cover what most teams use SysId for
on an arm or an elevator, and nothing is copied anywhere.

**Evidence, and how strong it is.** Argued. "What most teams use SysId for" is our impression, not
a count. The seeds have only run in the simulator, where the plant is built from the same model
that makes the seed. So their agreement there is circular and proves nothing about a real
mechanism. kS has no seed, because the model has no friction.

**What it costs.** You have no identified model of your mechanism unless you run SysId yourself.
Step tests tune a loop. They do not measure a plant, so control that needs a plant model has
nothing to start from. The drivetrain's feedforward is still best measured with SysId, and
[characterization.md](characterization.md#feedforward) sends you to WPILib's guide for it.

**What would change our mind.** Step runs on a real robot that show the seeded feedforward is
badly off. Then a characterization routine built into the runtime is the next thing to write. One
obstacle is known already: joints are sampled once a loop, which is too slow to fit kA on a short
mechanism.

## 15. Paths and autos in code

Autos and pathing are not finished. This section says what exists, why it was built in code first,
and what is still to come.

**What most teams do.** PathPlanner or Choreo. Paths are drawn in a GUI and deployed as files. An
auto is a tree in a file. Named commands and event markers tie the file to the code by a string. A
`SendableChooser` on the dashboard picks the auto.

**What this does.** Today a path is Java: `drive.goTo(place)`, or `drive.follow(...)` through
keyframes, planned when the move starts. A named keyframe gives a gate a command can wait on. An
auto is a name, a start place and a method that returns a command, declared with `autos.add`, and it
calls the methods your buttons call. "Do nothing" is always in the list and is the default. Before a
match the robot compares where it is with the chosen auto's start. `AutoCheck` runs every auto on
both alliances in the simulator.

**Why.** The join between a file and the code is a class of bug. A name is a string matched at run
time, so one wrong character means a step that never runs. The robot's mass and gearing are typed
again into the tool's settings, and the two copies disagree. In Java the compiler checks every name
and there is no second copy. So the part in code came first, and a file's path can be handed to it
later as one more command.

The path libraries are also Commands v2 for now. As of October 2026, PathPlannerLib's build for 2027
is written against v2, and WPILib's build refuses a project with both v2 and v3.

**Evidence, and how strong it is.** Counted. *Wiring of modes, autos and buttons* has 238 bugs on 44
robots, 24 serious. Of the 253 serious bugs, 14 were in how autos and path tool files are joined to
code, and when we judged them we had no way to stop those. An auto written as Java has no such join,
so those are impossible to express. So is a chooser with no default. The census counts bugs. It does
not count what a path tool saves a team, and nobody has measured how long an auto takes to write as
keyframes against drawing it.

**What it costs.** An auto is Java. Someone who does not write Java needs a path tool's files, once
those are read. A path written as keyframes is not drawn, so you first see it by running it in the
simulator.

**Not built yet.**

- Reading a path tool's files. [stand-ins.md](stand-ins.md#waiting-on) lists it as waited on.
- Choosing an auto from the driver station over NetworkTables. Today it is chosen on the pit page.
- Keeping the chosen auto across a restart of the robot program.
- Steering a path around obstacles.
- Planning at the robot's full limits. Moves plan at half the model's accelerations until the
  robot's yaw inertia is measured.

**What would change our mind.** Bugs in the join coming back once files are read, which would say
the join needs its own startup check. Or a season that shows keyframes in Java are slower to get
right than drawn paths, even for the students who write the code.

## 16. Pitwall instead of dashboard tuning

**What most teams do.** Tunable numbers on a NetworkTables dashboard, or gains set from a vendor's
tuner straight into the motor controller. The values are saved on the robot or in the device. They
are copied back into the code by hand, or they are not. To see what a mechanism is doing they plot
its numbers on a dashboard, or draw it in 3D from CAD.

**What this does.** The robot serves a web page, Pitwall, and the simulator serves the same page.
Its Tune view edits gains and current limits while the robot runs, within limits. Nothing is saved
on the robot. The Changes view lists each difference as the builder call that commits it, such as
`.withKs(0.25)`, and you paste that into the description. When the field connects, every value
returns to what is committed.

Its Robot view draws the robot's joints from the model, live: a card for each joint, drawn by its
kind, with where it is, where it was told to go, who owns it and what holds it. An interlock is
drawn on both joints it is about. The roller that runs only once the pivot is out is a mark on the
pivot's card, and it stands out while the roller waits for it. The view is drawn from the
program's own log, which AdvantageKit hands Pitwall loop by loop, and Pitwall keeps the last five
minutes of it. So Pitwall reads the model: the runtime hands it the robot's definition, and what
holds a joint is the robot's own logged fact, which the page resolves against the model and words
with the model's own sentences.

**Why.** The class of bug is a tuned value that lives somewhere other than the code. The code in
the repository is then not what ran. A swapped motor controller loses the value, and a redeploy
keeps a stale one. Here a match always runs the committed configuration. The page also needs no
install and no layout file, and it is the same in the simulator, so a student can learn it before
the robot exists.

The Robot view is there because a simulator is of little use unless you can see what the simulated
robot is doing, and the model's geometry is boxes, which say little in 3D. A view of what the
program believes and why, drawn from the model, says more: "the roller is waiting for the pivot" is
a fact on the page, where a 3D drawing shows a still roller. Until 2026-10-12 Pitwall knew nothing
of robots, and the runtime handed it generic data. A view that knows what a joint, a place and an
interlock are could not be built from generic data without rebuilding the model beside it.

**Evidence, and how strong it is.** Argued. No census class is claimed. Precedent, for part of it:
several top teams' public code tunes live through logged tunable numbers and saves nothing on the
robot. The page has never been used in a real pit, and what its web server costs the loop on a
Systemcore has not been measured. The Robot view's part is measured in the simulator and nowhere
else: taking a loop of the log costs about 25 microseconds on the log's own thread and allocates
nothing, and the five minutes it keeps take 12 MB for a robot with eight joints.

**What it costs.** It is one more tool, and no other team uses it. The paste step can be forgotten,
and the change is then gone when the field connects. That is safe, and it will surprise someone who
saw it work in the pit. The page is not reachable through the field network, on purpose: nothing is
tuned on the field. Pitwall can no longer be used without the model, and a change to the keys the
runtime logs has to be made in Pitwall's list of them too.

**Not built yet.** Live tuning of a joint's feedforward, its profile and its named places. A
timeline of the session, to scrub and step through the five minutes Pitwall remembers, with the
World view following it: the Robot view shows the newest loop only. Opening a saved log stays
AdvantageScope's job.

**What would change our mind.** A pit crew that keeps losing tuned values at the paste step. Or a
dashboard teams already know that does the same job and shows the change to commit.

## 17. Separate repositories

**What most teams do.** One repository for a season, often copied from last year's. A team library
sits inside it, or in a second repository that drifts.

**What this does.** There are five. The **model** is the vocabulary and the checks. It depends on
WPILib's math and units and nothing else of a robot program, so a robot, a simulator and a replay
read the same model. **Pitwall** is the pit page. It reads the model, to draw a robot's joints as
the model says them, and knows nothing of the runtime or of any team's robot: the program hands it
the robot's definition and its data. The **runtime** is the program every robot shares, and an architecture test fails if
it names a class of the team's. A **team's program** is `Main`, the robots' descriptions and
`Controls`. It takes the other three as git submodules, built from source in one build, with no
version numbers. Localization is a fifth, a library the runtime reads, as section 13 says.

**Why.** The project began as one repository, for its first stretch of development. With everything
in one place the boundaries were conventions, and they were crossed. Splitting it made the
boundaries real. The model cannot see the runtime. Pitwall cannot see the runtime or a team's robot.
The runtime cannot name a team's class. A team's repository holds only what the team edits.

**Evidence, and how strong it is.** Experience. We did it the other way first and unwound it. That
is one project's experience and not a measurement, and no census class bears on it.

**What it costs.** A change that crosses a boundary is two or three commits in order, with a
submodule pin moved each time. A student who wants to change the runtime has to learn what a
submodule is. The team's build checks that its WPILib and vendor versions match the runtime's, and
that is one more thing that can fail.

**What would change our mind.** Paired commits that slow down fixes in a build season more than the
crossed boundaries did.

## 18. Java 25, WPILib 2027 alpha, Systemcore only

**What most teams do.** The current season's WPILib release on a roboRIO, with Commands v2 and
released vendor libraries.

**What this does.** The target is WPILib 2027, currently 2027.0.0-alpha-7, on Systemcore, with a
Java 25 toolchain that Gradle downloads. The vendor libraries are alphas tied to that WPILib alpha,
and PhotonLib is pinned to a development build. It is Java and not Kotlin, because Java is what
students arrive knowing and what nearly every FRC example is written in. What is supported today is
a swerve drivetrain, PhotonVision, and the motor controllers and sensors listed in [coming from
WPILib](coming-from-wpilib.md#supported-today). The drivetrain stands in until AdvantageKit
publishes its 2027 templates.

**Why.** The stack is meant for the 2027 season, and 2027 is Systemcore. Commands v3 exists only
there. Code written for the roboRIO would be rewritten at once. Building on the alphas now means
the work is done before kickoff, when time is shortest.

**Evidence, and how strong it is.** Argued. It is a bet on a schedule, not a finding.

**What it costs.** It cannot run on a roboRIO robot, so it cannot run on any robot a team owns
today. That is why nothing on this page has been checked on hardware. Each new alpha can break the
build.

**What would change our mind.** A 2027 release that slips or changes shape so that the alphas were
a poor guide. Or a first Systemcore robot showing that what we learned in the simulator did not
carry over. If either happens, the description and its checks are the part worth keeping, because
they depend least on the platform.
