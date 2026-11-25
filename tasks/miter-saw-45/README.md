# Set a miter saw to 45 degrees and lock it

A SKIL MS6316-00 sliding miter saw stands in front of the robot, set square, unplugged, its blade taken off. The robot has
to set it to 45 degrees (either side) and lock it: squeeze the miter-detent release lever up toward the lock lever, swing
the miter arm round, let go so that the detent pin drops into the 45 degree notch, and press the lock lever down until it
clicks. That is the procedure of the saw's manual (p. 25): "Hold the miter-lock lever and miter-detent release lever
simultaneously. Lift the miter-detent release lever and rotate the miter arm ... Release the miter-detent release lever.
Press down on the miter-lock lever firmly until you hear a click ... making sure that the miter-detent pin is fully
snapped into the detent."

What is hard: one gripper squeezes two levers, one above the other, 4 cm apart and 5 cm above the table, and keeps them
squeezed while it carries the arm's drag round a 45 degree arc, past the stops at 15, 22.5 and 31.6 degrees; it must let
the pin drop into the right notch (let go less than a degree from it, or nudge the arm on until it drops in), and then press the
lock lever down with about 45 N without disturbing the setting. Let go elsewhere, the arm is loose or sits in a wrong stop,
and the joint does not close.

## Files

| File | What |
|---|---|
| `task.py` | The task: saw, detent pin and clamp rules, score, scripted demo |
| `mechkit.py`, `closedloop.py` | Multi-part objects and the closed-loop demo controller (unchanged copies) |
| `task.json` | Submission text |

## Run

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the miter saw to 45 degrees and lock it." \
  --scorer task_score --max-steps 1850 --store-frames
```

`--policy yam_scripted` runs the demo. The layout is fixed (`object_jitter=0`); there are no seeds to choose. Runtime:
about 1.5 minutes of simulated time a trial (1,850 steps at 20 Hz); wall time with cameras on one GPU was 8 minutes for the demo (730 steps) and 15 minutes for Dream-1 (1,850 steps), start-up included. `MITER_DEBUG=1` prints the joint angles.

## Scoring

Only the state at the end of the trial counts, read with the saw's own pointer and scale, a ruler, a photo and the trial
video. Every limit is inclusive.

| End state | Score |
|---|---|
| Pointer within 0.5 degrees of 45 (left or right), release lever let go (within 1.5 degrees of its rest), lock lever pressed past its click and staying down (30 degrees or more below level in the simulation), every fingertip 30 mm or more from the saw, nothing moved for 0.5 s | 1.0 |
| Pointer within 0.5 degrees of 45 and release lever let go, but not locked, or a fingertip closer than 30 mm to any part of the saw, or something still moving | 0.75 |
| Pointer within 0.5 degrees of 15, 22.5 or 31.6 (either side), release lever let go | 0.5 |
| Any other angle of 10 degrees or more (loose, locked between stops, release lever still squeezed) | 0.25 |
| Under 10 degrees, whatever the levers | 0 |
| Any of the four marked corners of the plywood board more than 10 mm from where it started, in any direction (slid, turned, tipped, lifted) | 0 |

- **Order.** How the robot got there does not count. With the lock lever down the arm cannot be swung (the clamp holds
  40 N m, 133 N at the levers), so the saw itself enforces swing first, then lock.
- **Hands.** Fingertip distance is taken to the box round each part of the saw where that part is now, the pressed-down lock
  lever included; the plywood board does not count. A round part therefore counts as its box: near a corner of the 220 mm
  round guard's box, a fingertip up to 46 mm from the guard itself reads as touching it, and up to 76 mm as closer than 30 mm.
- **Stillness** needs a full half second of history in which the arm moved at most 0.2 degrees, each lever at most 1
  degree and each board corner at most 1 mm. A corner's movement is its largest spread in x, in y and in z over the half
  second, the three combined (root of the sum of squares), so a corner that never strays more than 0.9 mm between any two
  samples can read as 1.19 mm. It is judged by how far things moved, not by reported velocity. **Sampling
  limit:** the simulator is sampled 20 times a second, so a jerk and return between two samples is not seen.
- Score and `success()` use the arm's real angle, not the notch it is near.

## The saw

SKIL MS6316-00, 7-1/4 in (184 mm) sliding compound miter saw, 24.5 lb (11.1 kg), miter range 47 degrees left to 50 right,
stops at 0, 15, 22.5, 31.6 and 45 degrees each side (owner's manual, pp. 13-16 and 25; skil.com product page, $209.99 on
22 Nov 2026; weight: Pro Tool Reviews 24.5 lb, the maker's page "under 25 lbs"). The range is modelled as the manual gives
it. Sizes are read off the manual's Figs. 1, 15 and 16 and the maker's photos (about +-5 mm): table surface 69 mm above its base, miter arm 265 mm from the
pivot, lock lever to 346 mm, release lever's pad 280-296 mm from the pivot, its underside 26 mm above the base and 41 mm
under the top of the lock lever. The lock lever goes down 35 degrees here, which is **not** taken from the saw: the manual's
Fig. 16 shows it at roughly 40-50 (a steeper lever would meet the release pad at these read-off sizes). The red pointer is a
bracket on the turntable's edge, left of the arm, as in the photo. The release lever is lifted straight up, toward the lock lever, as in Fig. 15 and the maker's
photo of a hand on both levers. The saw is bolted to a 19 mm plywood board; the manual asks for a workbench, mounting board
or stand (p. 22).

## Forces

**Not measured.** Nobody on the team has had this saw in hand, and no published release force or table drag was found, for
this or any other miter saw. Each value below is an **estimate**: the table's drag at the upper end of its arithmetic, the
release and lock levers in the middle of theirs. The squeeze (27 N at the pad) and the swing (13 N) are within the about 40 N
a YAM finger can give. The lock lever is pushed by the arm, not by a finger: about 2.5 N m (measured in the simulator;
nominal 2.75) is 30 N at its end, 42 N 6 cm from its hinge and 45 N where the demo presses.

| What | In the simulation | Not measured; estimate from |
|---|---|---|
| Release lever, up at the middle of its pad | 13 N to start, 22 N with the pin out (5.7 mm), 27 N at its stop | The spring in the maker's photo: a coil about 8 mm outside, 1 mm wire, round the lock rod, 0.77 of the way to the pad. A catalogue spring of that size (94125K631: 9 mm outside, 1 mm wire, 13 mm long) has a rate of 5.7 N/mm and a listed maximum of 7.5 lbf, 33 N (Lily Bearing's listing; a McMaster-Carr search excerpt says 9 lbf, 40 N: the two disagree). With that rate and 0 to 2 mm of preload the pad needs 19 to 28 N with the pin out |
| Table, sideways at the levers (0.30 m) | 13 N to start it, about 12 N moving at 10 degrees a second (4.0 N m breakaway, 3.2 N m moving, 2 N m s/rad) | Arithmetic with assumed numbers: turning parts 7 kg on a ring of 0.1 m radius at friction 0.25 is 1.7 N m; doubled for the pivot bolt's preload and rounded up |
| Lock lever, down, 6 cm from its hinge | about 42 N at its peak (about 2.5 N m measured in the simulator; nominal 2.75), then it snaps down | Order of magnitude only. US 5,595,124 describes a different clamp (a spring-held table lock, 350-500 lb, freed by a lever of about 40 to 1, so 39-56 N at that lever); it says nothing about this saw's lever |
| Pin in its notch | 1,000 N m/rad; less than 1 degree from a notch (44 and 46 do not catch), with the release lever let go, it draws the table in | US 2011/0162502 (Bosch): a saw set one degree from 45 is often pulled back to 45 by its detent spring (read as a search excerpt) |

MIL-STD-1472G Table XI (design guidance for hand controls, not a measurement) allows 2.8-110 N for small slide levers.

**Two readings to take** on a real MS6316-00, unplugged, blade off, bolted down, lock lever up and level, with a luggage
scale that reads to 10 g:

1. *Release force.* Arm in the 0 notch. Loop the strap under the middle of the release lever's pad and pull straight **up**,
   slowly, until the arm just swings freely by hand; read the peak. 5 repeats, the lever snapping back each time. Simulation:
   **22 N** (2.2 kgf). Mean above 27 N: the simulation is too light, raise it. Above 40 N: too much for one YAM finger. Below
   11 N: more than twice too heavy, lower it.
2. *Turntable resistance.* Release lever held fully squeezed (a strip of tape round both levers). Hook the scale round the
   lock lever's neck, **300 mm from the pivot** (the arm itself ends at 265), and pull **sideways, level and square to the arm**, steadily from 0 to 45
   degrees in about 5 s; read the steady value and the breakaway peak. 5 repeats each way. Simulation: **12 N moving, 13 N to
   start**. Mean above 15 N: too light, raise it. Above about 40 N: more than the pads can pass on by friction. Below 6 N: more
   than twice too heavy, lower it.

Optional third: scale on the lock lever's handle, 60 mm from its hinge, pulled straight down until it clicks, 5 repeats
(simulation about 42 N peak; wrong if above 60 or below 25 N). The results go into `REL_K`, `REL_PRE`, `DRAG`, `LOCK_K` and
`LOCK_HOLD` in `task.py`.

## What the simulation simplifies

Rigid parts at true size. The pin, its notches and the clamp are rules in the task code, run every control step (20 Hz):
squeezed past 3.7 degrees (5.7 mm at the pad) the pin is out and the table turns against its drag; with the release lever let
go, the pin drops in as soon as the arm is less than 1 degree from a notch (44 and 46 do not catch), whether it was let go there or the arm was pushed on
to it, and stays until the lever is squeezed again; elsewhere the arm stays loose; with the lock lever down the table is
clamped. An arm swung past a notch faster than 40 degrees a second with the lever let go can pass it between two steps. The
lock lever is an over-centre spring. The trial starts with the lock lever up, so unlocking is not part of it. The head is
fixed in its raised position and does not slide; guard, motor and rails are plain shapes. The blade is left off, as in the
real trial.

## Changes after the first simulator run (which scored 1.0)

No force, mass, friction or score tier was changed. Scorer changes are tightenings unless marked.

| What | Old | New | Why, source |
|---|---|---|---|
| Miter arm box | top at the turntable's surface | 0.4 mm lower | coplanar faces flickered; the demo never touches that face; no source |
| Look-only parts | pointer hidden inside the arm | red pointer bracket beside the arm, scale turned with it; grip patches; red inserts 0.2 mm higher | the pointer could not be seen; maker's photo |
| Arm's range | 47 degrees each side | 47 left, 50 right | owner's manual p. 16 |
| Demo | hand 65.5 mm up, 62 mm open, finger target up to -6 mm | 68-72 mm up, 66 mm open, up to -10.5 mm | fingers pressed about 51 N; a YAM finger gives about 40 N. Now 26-34 N |
| Run limit | 2,000 | 1,850 | 2.5 times the demo |
| Limits | exact 44.5 and 45.5 degrees and 10 mm were out by rounding | inclusive, as the text says (looser by rounding only) | text and code disagreed |
| Board shift | horizontal distance of the board's origin | 3-D distance of its four corners | a turned, upside-down or lifted saw scored 1 |
| Fingertip clearance | a fixed box round the levers | the box round each part of the saw where it is now | a fingertip under the pressed lever scored 1 |
| Stillness | 0.44 s; board 2.5 mm and 1.5 degrees; levers 1.5 degrees | a full 0.5 s; board corners 1 mm; levers 1 degree | a slow slide counted as still |

## Sources

| What | Where |
|---|---|
| Procedure, range, mounting, figures | SKIL MS6316-00 owner's manual, skil.com (opened: pp. 12-16, 22, 25) |
| Price, "under 25 lbs", photos | skil.com product page for MS6316-00 |
| 24.5 lb | Pro Tool Reviews, "Skil 7 1/4-inch Miter Saw Offers DIYers a Compact Alternative to Full-Sized Models", 7 March 2025 |
| Carpenters' task | O*NET OnLine 47-2031.00: "Shape or cut materials to specified measurements, using hand tools, machines, or power saws." |
| 889,700 jobs in 2025 | BLS Occupational Outlook Handbook, Carpenters |
| Miter cuts in the trade's curriculum | NCCER Carpentry Competencies / Objectives, module 27210-07 "Window, Door, Floor, and Ceiling Trim", objective 2: "Make square and miter cuts using a miter box or power miter saw." (ccc.edu copy, opened; repeated in the Oklahoma CareerTech carpentry study guide, objective 9) |
| Force comparisons | as given under "Forces" |

## Build it

Buy the saw, unplug it, take the blade off and keep the lower guard on. Bolt it through its four feet to a 350 x 420 mm,
19 mm plywood board, and put the board where `task.json` says, its corners on tape marks. Cover the gripper pads with a
layer of tape. Reset: lift the lock lever, squeeze the release lever, swing back to 0, let go; put the board back on its
marks.

## Credits

Takeover of benchmark-tasks #2579 (TK-K64735): reviewed 19 Nov 2026 01:35 UTC, no revision pushed (last commit 18 Nov 03:17
UTC), so its 48-hour hold lapsed on 21 Nov 01:35 UTC. From #2579 come the idea of setting a miter saw to its 45 degree stop,
the ladder of scores (1, 0.75, 0.5, 0.25, 0) with its 0.5 and 10 degree thresholds, the rule that a pin catches near a stop,
and the 1,000 N m/rad pin. New here: the named saw and its levers, the vertical squeeze, the lock, the forces, the scorer, the
demo and all code and text. `mechkit.py` and `closedloop.py` are copied unchanged from the accepted task
plug-a-night-light-through-a-sliding-outlet-cover (MIT). No third-party assets.
