# Make room in the fridge for a tall bottle by taking the split shelf's front half under its back half

The fresh-food compartment of a French-door refrigerator stands open in front of the robot. Above the lower shelf is a
full-width GE QuickSpace shelf, split into a back half that is fixed and a front half that rides in its own guides. A
full 500 mL bottle of water stands at the front of the lower shelf, in front of the split shelf, where it would stop
the doors closing. The instruction gives only the goal and where the bottle must end up:

> Stand the tall bottle upright on the lower shelf with its whole base behind the front ends of the split shelf's side brackets.

The bottle is 205 mm tall and the space under the shelf's front trim is 147 mm, so the bottle can only stand behind
that line once the front half has been moved out of the way. The front half cannot simply be pushed back: it rests
on two seats behind small lips, level with the back half, and its back edge meets the back half's front trim after
2 mm. It has to be lifted off its seats, drawn forward past them, lowered 30 mm onto a lower track, and only then slid
back under the back half. GE's manual describes the same three moves for this shelf: "Lift the back edge of the front
half of the shelf. Pull slightly forward. Lower the back edge of the front half of the shelf while sliding it
underneath the back half" (GE Monogram ZIFS240 use and care guide, QuickSpace Shelf).

## Why it matters

Putting groceries away is something almost everyone does every week, and a fridge whose doors will not close spoils
food. The QuickSpace shelf exists for this job, and GE fits it across product lines: the owner's manuals for its
French-door and bottom-freezer refrigerators (publication 49-1000472), its side-by-side models (GSS25 series, GE
Profile 26 and 29) and its Monogram built-in column (ZIFS240) all describe it, in the same words: the shelf "splits in
half and slides under itself for storage of tall items on the shelf below". The same work is a core task of food
preparation workers, O*NET 35-2021.00: "Stock cupboards and refrigerators, and tend salad bars and buffet meals."

The skill tested is rearranging a retained, guided part through a constrained path (up, forward, down, back) with one
or both hands (two on a real 760 mm shelf), then using the space it opens. A drawer or an oven rack moves on one level and can be pulled straight out; this half
rides in its own guides and changes height part way, so pushing or pulling it straight gets nowhere.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the fridge and its shelf, the bottle, the trial record, `success()`, `score()` and the scripted `demo()` |
| `parts.py` | A kit for parts built from several coloured shapes, with prismatic joints that can hang from one another (here: the front half's guides) |
| `lots.py` | Runs trial k in lot A or B (k mod 2): the bottle starts left or right of the midline |
| `tilt.py` | Lets the scripted demo lean the gripper (it affects only the demo, never Dream-1 or scoring) |
| `near_misses.py` | Offline checks of the scorer: every tier, its limits, the near misses and the shortcuts below |

`parts.py`, `lots.py` and `tilt.py` patch yam_arena's scene, embodiment step and reset and its demo grip rotation at
run time, inside the task's own process. Nothing on disk is changed. All code is the task authors' own (MIT licence).
No third-party assets are used.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the tall bottle upright on the lower shelf with its whole base behind the front ends of the split shelf's side brackets." \
  --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir ~/yam-eval/fridge-shelf-dream
```

Trial k runs lot k mod 2 (A first); `--epochs 2` runs both and `task_score` prints their mean. `TASK_LOT=B` in front of
the command pins every trial to lot B. Scripted demo: the same command with `--policy yam_scripted` (recorded at
`-E cam_width=640 -E cam_height=360`). Submissions run with `-E object_jitter=0`; no seed changes the scene.

Scores with the submitted instruction, both lots, on this exact `task.py`: scripted demo 1.0 in lot A and 1.0 in lot
B (713 steps each, 35.7 s of the 60 s limit); Dream-1 0.0 (mean of lots A and B; in both trials it ran to the
1,200-step limit: the arms reached toward the split shelf and hovered over it, the shelf did not move and the bottle
stayed at the front). Runtime on one L40S: the two-lot demo took 13.3 minutes of wall-clock time including Isaac
Sim's start-up (about 6.5 minutes a trial); the two-lot Dream-1 run took 29 minutes (about 15 minutes a 1,200-step
trial, most of it waiting for the policy).

## Scene (cm from the midpoint between the arm bases: x forward, y to the robot's left; heights above the table)

The refrigerator is a GE French-door model with a QuickSpace shelf (for example the GFE26JYMFS, 36 in wide, 25.6 cu
ft; retailer listings for it give one Quick Space shelf). GE publishes no drawing of the shelf, so every shelf
dimension below is an estimate from the manual's figures and the 36 in cabinet, stated so it can be checked with a
tape measure on the real fridge.

- **Compartment (fixed).** Doors open; the upper shelves, drawers and door bins are taken out (they block the overhead
  camera and play no part). White liner side walls, inner faces at y ±41.0 (822 mm inside), from x 4.0 to 68.0, shown
  60 cm tall; rear wall at x 66.0 with three dark shelf tracks (at y −39.8, 0, +39.8).
- **Lower shelf (fixed).** 4 mm glass from x 5.2 to 66.0 across the full width, top 0.4 high; a white front trim at
  x 4.0 to 5.2, 1.0 high.
- **QuickSpace shelf, side brackets (fixed).** Grey, y ±39.6 to ±41.0, x 20.0 to 65.2, 9.0 to 16.7 high, hooked into
  the outer tracks. On each bracket's inner face:
  - an upper seat, x 36.4 to 39.0, 13.5 to 14.1 high, with a lip at its front end (x 36.4 to 37.0) rising to 14.6;
  - a lower track, x 34.0 to 60.0, 10.4 to 11.0 high, ending in a stop at x 56.15 to 57.0, 10.4 to 13.0 high.
- **Back half (fixed).** 4 mm glass from x 39.4 to 64.0 (y ±38.0), top 16.5 high, in a light grey frame: front trim
  x 39.0 to 40.2, 15.7 to 17.5 high; side frames y ±38.2 to ±39.6, 15.3 to 16.7 high; rear trim x 62.8 to 64.0.
- **Front half (moves only in its guides; 1.45 kg estimated: 4 mm tempered glass, 1.30 kg, plus trim).**
  - glass 740 × 176 mm (y ±37.0, x 21.2 to 38.8), top 16.5 high, level with the back half's;
  - front trim, dark grey, 760 mm long, 12 mm deep, 24 mm tall: x 20.0 to 21.2, 15.1 to 17.5 high, standing 10 mm
    above the glass; its front face is level with the brackets' front ends;
  - side runners y ±37.0 to ±38.0, 15.3 to 16.7 high;
  - a tab under each back corner (x 37.2 to 38.4, y ±38.0 to ±39.0, 14.1 to 15.5 high) resting on its seat, 2 mm
    behind the lip.

  The space under the front trim is 147 mm high. The guides let the half rise 8 mm (the top of its guide), move 24 mm
  forward (the forward end of its guide), 176 mm back (the stop at the end of the lower track; its front trim is then
  14 mm in front of the back half's) and drop 30 mm (onto the lower track).
- **Bottle (loose).** A full 500 mL PET bottle of water with a polypropylene cap (for example a store-brand
  16.9 fl oz bottle): 65 mm across, 150 mm to the shoulder, 205 mm tall with its cap, 0.52 kg (500 g of water plus an
  estimated 20 g bottle). Clear blue, white label band, blue cap. It stands upright on the lower shelf with its base
  centre at x 12.5, y 6.0 in lot A and y −6.0 in lot B, 18 mm in front of where the front trim can come forward to.

Materials: the liner, brackets and trims are ABS plastic, the shelf panels tempered glass. Friction (static,
dynamic; estimates, PhysX averages the two surfaces): plastic 0.5, 0.4; the lower shelf's glass 0.35, 0.30; the
bottle 0.40, 0.35.

**Staging choices, stated plainly.** The lower shelf reaches 15 cm in front of the split shelf so that the bottle has
somewhere to start inside the compartment (on many fridges the shelves share a front line). The gap under the split
shelf (14.7 cm below its trim) is an estimate; GE fixes the QuickSpace shelf's height.

**Building the real trial.** Use a GE French-door refrigerator with a QuickSpace shelf, with the robot's work
surface level with its lower shelf; or a bench section: the QuickSpace shelf assembly for that refrigerator, ordered
from GE's parts service by the fridge's model number, hung by its brackets on two slotted shelf tracks screwed to the
back of a box of 12 mm plywood painted white, 822 mm wide inside and 620 mm deep, with a 4 mm glass lower shelf at the
robot's table height. Before trials, check the real shelf against the dimensions above (heights, travel, seats, lips):
the scoring limits assume them.

## How the shelf moves (the model)

The front half is a link on two prismatic joints in series, a slide along x and a rise and fall along z, so it is
retained in its guides at all times and stays level. Everything else is contact geometry:

| Move | What happens |
|---|---|
| Push straight back from the start | its glass meets the back half's front trim after 2 mm |
| Pull straight forward from the start | its tabs meet the lips after 2 mm |
| Press down at the start | its tabs rest on the seats |
| Lift it 8 mm, then push back | its glass meets the back half's front trim, which stands 10 mm above the back glass |
| Lift it 8 mm, draw it 24 mm forward | its tabs pass over the lips and clear the seats' front ends |
| Then lower it | it drops 30 mm onto the lower track (the guide's lower limit) |
| Then push it back | its tabs pass 10 mm under the seats, its glass 26 mm under the back half's; it stops after 176 mm |

## Scoring

Read from the video, a top photo and a ruler. "Let go" means no gripper within 5 mm of the front half or the bottle;
"still" means no visible movement for 1 s.

**Score 0 for the rest of the trial** if the bottle ever leaves the fridge, carried or falling: its middle (90 mm up
its axis) passes in front of the lower shelf's front edge (x 4.0), or any part of it ends up more than 20 mm below the
lower shelf's top. Also 0 if the front half is taken out of its guides (possible on the real shelf only).

Otherwise, with the front half and the bottle both let go and still for the last 1 s, and the front half **down on its
lower track** (ruler: its glass top at least 27 mm below the back half's), the first matching level:

| Score | State | How it is read in a real trial |
|---|---|---|
| 1.0 | The bottle **stands** (within 5° of upright, base on the lower shelf with no gap over 3 mm) with its **whole base behind the line** between the brackets' front ends | Top photo against a straightedge laid across the brackets' front ends: no part of the base in front of it. Phone level on the bottle for the 5°; feeler or ruler under the base for the 3 mm |
| 0.75 | The bottle stands with its base centre behind that line but part of its base in front of it | Top photo against the straightedge |
| 0.5 | The bottle anywhere else (still at the front, lying, leaning); the front half **tucked** to its stop | Ruler: its front trim 14 to 19 mm in front of the back half's front trim |
| 0.25 | The bottle anywhere else; the front half down on its track but not tucked | Ruler as above |
| 0 | Anything else: the front half still up on its seats or lifted, held, or moving; the bottle out of the fridge at any time | |

Lots A and B are averaged. The bottle cannot stand with its whole base behind the line unless the lowered front half
has gone back at least 65 mm (33 mm for its base centre), so the top two levels need the shelf moved; they do not need
it pushed all the way to its stop.

**In the simulator** (`task.py`): `lowered()` is the rise joint at most 27 mm below its start (−30 + 3); `tucked()`
adds the slide joint at least 171 mm (176 − 5); `standing()` is the bottle's axis within 5° of vertical and its base
centre between 1 mm below and 3 mm above the lower shelf's top; `inside()` is the base circle's front-most point at or
behind x 20.0 (the brackets' front ends) and within the liner walls; `centre_inside()` is the base centre at or behind
x 20.0. `fallen_out()` uses the bottle's middle, 90 mm up its axis, and its lowest point. `hands_clear()` samples every
collision face of both grippers and both fingers at 3 mm spacing and needs every point more than 8 mm (5 mm plus the
3 mm sampling) from every collision box of the front half and the bottle. The scene is still when the bottle moves
under 3 mm/s and neither the bottle nor the front half drifts more than 0.5 mm or 1° over the hold. The record is
written only by the simulator's step and reset hooks; it fails closed (0) whenever a state or the gripper geometry is
missing or non-finite, and the "left the fridge" flag is reset only by a scene reset. The trial ends at 1.0 once that
has held for 1 s; otherwise it scores the last step.

### Shortcuts and wrong ways (all checked in `near_misses.py`)

| Shortcut | Score |
|---|---|
| Push the bottle straight in with the shelf untouched | 0: the bottle's top meets the front trim; the front half does not move back |
| Push the front half straight back, with or without lifting it first | 0: blocked by the back half after 2 mm |
| Lay the bottle on its side under the extended shelf | 0: front half not moved |
| Lift, draw forward and lower the front half, then stop | 0.25 |
| Lower the front half, push it back part way, stand the bottle in front of it | 1.0: the goal is met, the half rests on its track |
| Tuck the shelf and leave the bottle at the front, or lay it down inside | 0.5 |
| Leave the bottle straddling the front line, upright | 0.75 |
| Leave the bottle leaning more than 5° | 0.5 or 0.25 |
| Carry the bottle out of the fridge to clear the way, then put it back | 0 for good |
| Hold the shelf or the bottle in place at the end, or keep a fingertip within 5 mm | 0 |
| Knock the bottle out of the fridge and then finish the task | 0 for good |

**One hand.** The simulator's guides keep the front half level, so one hand at the middle of the trim can also lift
and move it there. The demo uses two hands, one at each side of the 760 mm trim, as a person does with a wide glass
shelf.

## Scripted demo (open loop from the published layout of each lot)

1. Both hands take the front trim 200 mm either side of the midline, fingers closing front to back across its 12 mm.
2. They lift (aiming 35 mm up; the arms sag under the half's weight and the guides stop it at 8 mm), draw it 24 mm
   forward past the seats, and lower it onto the lower track.
3. They let go, shut their fingers, and push the lowered trim back with the fingertips until it reaches its stop.
4. The hand on the bottle's side moves round the bottle on the midline side, leans its open fingertips 40° forward,
   and slides the bottle upright along the lower shelf by its base (a two-point push that keeps it centred) until its
   base is 37 mm behind the front line. It backs away and parks.

The demo reads nothing from the simulator: all waypoints come from the published dimensions and the lot's bottle
position.

## Fidelity

Approximate, at true size. GE publishes no drawing of the shelf, so the guides, seats, lips, stop, clearances and
masses are estimates (listed above with their values). Bodies are rigid boxes; the bottle is a rigid 24-sided prism
(a real water bottle squeezes a little). The front half rides its guides as two joints and stays level, so "lift the
back edge" is modelled as lifting the whole half 8 mm; the real half can also tip. The seats, lips, stop and the
back half are contacts. Glass is drawn opaque. Frictions 0.3 to 0.5 (above). The lower shelf reaches 15 cm in front
of the split shelf (a staging choice, above). The two layouts are fixed, so a memorised sequence is possible.

## How this differs from close tasks

- **TK-YFUGFS** (accepted): refit a loaded drawer on roller slides, carried in above riding height so the rear hooks
  clear. A detached part is installed; here the part stays in its guides and is taken through a two-level path in
  place, then the space it opens is used.
- **TK-QDX44E** (accepted): refit a fridge door bin on two hidden supports. Also a detached part being hung.
- **TK-3AWYMC** (accepted): slide a sheet pan onto a toaster oven's rails. One level, coplanar rails, no change of
  level.
- **TK-UC4CMD** (accepted): roll a cabinet shelf through a face-frame opening onto four pins. Installation of a loose
  shelf.
- **TK-3DNB4L** (pending): move a whole fridge shelf up a slot, tipping it to hook in. A loose shelf moved to another
  height; here the shelf is retained and split, and the goal is the space it opens.
- RoboCasa and BEHAVIOR open and close drawers and doors and stock fridges with items that fit; their published task
  lists have no task that rearranges a split shelf to make room.
