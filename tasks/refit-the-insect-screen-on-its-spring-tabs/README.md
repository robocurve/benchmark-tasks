# Refit a window insect screen against its corner tension springs

A cropped section of a small window is fixed on the bench in front of the robot: the wall below it with its stool
(the inside sill board), two jambs, the head, a sill channel and a head channel that the screen sits in, and a stop
that frames the opening from behind, with the glass beyond. The window's aluminium insect screen stands on the
stool, leaning back against the head's front face, as left after it was washed. The robot is told:

> Push the screen top into the head channel against its springs, swing the bottom over the sill lip, lower it into the sill channel against the stop, then release.

## How the screen is held

This is the ordinary spring-loaded screen of a sliding or single-hung aluminium window. Two bought corner tension
springs (Prime-Line PL 7776, flat heat-treated steel leaves, 1/4 x 2-3/4 in, sold to hold screens in window channels)
are slipped into the screen's two top corners; each leaf lies along the top edge and bows up out of it. The head
channel's ceiling is 6 mm above the seated screen's top, so it presses both springs 1 mm and they keep the screen down
in the sill channel, behind a 4 mm lip. To fit it, the screen's top is pushed up into the head channel by 4 mm or more
(the springs pressed 5 mm or more; at 6 mm they are flat and the frame meets the ceiling), its bottom is swung in over
the sill lip, and it is let down. To take it out, the same in reverse. The screen is carried by two bought pull tabs
pressed into its spline groove (Prime-Line PL 14621), which are sold to aid screen removal; the springs are
never held and nothing else latches.

This task takes up the idea of [PR 1911](https://github.com/robocurve/benchmark-tasks/pull/1911), "Add refit-window-insect-screen task" (TK-9FXQU5, revised as TK-M74T74),
whose 48-hour hold after its second review lapsed without a revision; the scene, scoring and demo here are written
from scratch.

## Changes in this revision, point by point

Review [TK-PFFG8L](https://github.com/robocurve/benchmark-tasks/pull/3026#issuecomment-7075675043): "The custom pinch latches also act as carrying handles and don't work like real screen latches.
Please use a commercially available screen-latch mechanism (for example spring plungers or pull tabs) and update the
setup, simulation, scoring and demo to match."

| Point | Change |
|---|---|
| "The custom pinch latches also act as carrying handles" | The custom latches are gone. Nothing that retains the screen is grasped: the robot carries the screen by two bought vinyl pull tabs (Prime-Line PL 14621), which the maker sells to aid screen removal; how they bend or slip while carrying and turning the screen has not been measured. The springs are never held (the hand-clearance rule covers the springs too). |
| "...and don't work like real screen latches" | The screen is now retained the way spring-loaded aluminium window screens are: two bought corner tension springs bow up out of the top rail and are pressed by the head channel's ceiling, holding the bottom down in the sill channel. Each spring is used as its maker sells it (slipped into the frame's corner, "used to help hold screens tightly in window channels"). |
| "Please use a commercially available screen-latch mechanism (for example spring plungers or pull tabs)" | Prime-Line PL 7776 corner mount screen tension springs ($7.70 for 6 from the maker) and Prime-Line PL 14621 pull tabs ($10.97 for 25), on a Prime-Line MP8239 frame kit; sources in `fabrication/README.md`. Spring plungers dropping into holes were not chosen: their pins would sit inside the jambs, out of sight (the point of TK-VRBY83), and each must be held back by hand while the screen is placed, which needs more hands than the two that carry it. |
| "...update the setup" | `task.json` setup, this README and `fabrication/` describe the springs; the head channel is 5/8 in (15.9 mm) wide with its ceiling 6 mm above the seated top; the right pull tab is 50 mm above the bottom edge (was 80). |
| "...simulation" | Contact-driven crown sliders: 400 N/m with a separate 0.04905 N proxy-weight correction, 7 mm travel and no nominal preload. Net upright 0.4/2.8 N seated/flat, above the assumed measured-endpoint linear secant. Free-part measurements are recorded; installed curve and grip remain unverified. |
| "...scoring" | The seating and capture thresholds are retained; new end-state guards reject rails through the stop, ceiling or floor. The 0.5 tier's "behind the head lip" now uses the head lip's own back face (the head channel is wider than the sill channel); the hand-clearance rule includes the springs; `near_misses.py` adds the top fallen 4 mm toward the room in the head channel (0.5) and the top pressed to the ceiling with the bottom held up (0.5). |
| "...and demo to match" | The scripted demo lifts the screen by the tabs, turns its top up into the head channel against the springs, swings the bottom in over the sill lip and lets it down. Results below. |

Review [TK-VRBY83](https://github.com/robocurve/benchmark-tasks/pull/3026#issuecomment-7029349281): "Scoring (not met): The required 3 mm bolt penetration is hidden inside each jamb, and the proposed
ruler check does not explain how to measure it. Clarity and reproducibility (not met): The custom squeeze-latch
mechanism has no buildable drawings, fabrication files or identified complete product from which the real fixture can
be reproduced. What to change: Please provide a reproducible real-world screen and latch assembly: identify a
complete suitable commercial product or supply fabrication drawings or printable files, a parts list, and assembly
instructions for the custom latches."

| Point | Change |
|---|---|
| Hidden 3 mm bolt penetration; ruler check | No bolts. Frame seating/capture is read from the room side with a ruler/depth rod (Scoring and sheet 3). Full credit also requires two spring-apex contacts through dimensioned straight room-side feeler ports, and a 1 mm frame hold reconstructed from visible frame landmarks. No bent probe or hidden pad-corner tracking is required. Actual access/camera enrolment remains unperformed; drawings are not a physical demonstration. |
| Buildable drawings, parts list, assembly instructions | The screen is built only from bought parts (frame kit, two springs, two pull tabs); `fabrication/` has four dimensioned sheets, `dimensions.json` (checked against the model), a priced parts list with the source pages, assembly steps and fit checks. Nothing custom latches. |

Earlier reviews of the same idea ([PR 1911, first verdict](https://github.com/robocurve/benchmark-tasks/pull/1911#issuecomment-6889601795); [second verdict](https://github.com/robocurve/benchmark-tasks/pull/1911#issuecomment-6965686920)): "Reseating the insect screen requires aligning with all 4 sides, not just
the bottom" — the screen seats against a stop that laps it on all four sides, in a sill channel and a head channel,
between the frame's sides with 3 mm play; "Insect screens typically come with spring tabs that need squeezing during
installation" — the springs must be pressed (by pushing the screen up against them) before the bottom can pass the
sill lip.

## Real objects (full size)

Inch sizes below are rounded nominal labels. The model uses the stated millimetres; these are not manufacturing tolerances.

- **Screen:** 406 x 356 mm (16 x 14 in), about 0.22 kg (estimate), from a Prime-Line MP8239 frame kit: roll-formed
  aluminium, 7/16 x 3/4 in profile (11.1 mm thick, 19.05 mm face), mill finish, square-cut corners,
  vinyl spline and charcoal fibreglass mesh.
- **Corner tension springs:** two Prime-Line PL 7776 corner mount screen tension springs (heat-treated steel,
  1/4 x 2-3/4 in), one slipped into each top corner, the leaf bowing up along the top edge with its crown about
  35 mm in from the side. Measured on one PL 7776 with calipers, ruler and a kitchen scale: 0.27 mm strip,
  7 mm free bench bow and 161 g / 1.58 N to press its center flat (`fabrication/MEASUREMENTS.md`). The model uses
  that 7 mm bow as its crown above the rail, with 400 N/m, no nominal preload and a separate 0.04905 N
  proxy-weight correction. Upright net 0.4/2.8 N seated/flat; the complete envelope and limitations are below.
  Installed crown, preload and loading/unloading curve have not been measured.
- **Pull tabs:** two Prime-Line PL 14621 vinyl spline channel pull tabs (1-1/16 in overall height, 15/16 in spline-foot width, 5/8 in tongue width), pressed into
  the spline groove on the room-side face of each stile, 15 mm in from the side edge: the left one centred 140 mm and
  the right one 50 mm above the bottom edge. Their stagger resists rotation in the rigid-tab model (two
  pinches at one height act as a hinge). Real loaded vinyl rotation/retention remains unverified. Modelled as a rigid tapered tongue projecting 27.0 mm, 23.8 mm wide at its spline foot and 15.9 mm at its tip, with 2.38 mm thickness (the maker's 3/32 in base dimension used throughout). The foot is 6.35 mm across the stile. The real vinyl bends; the installed projection and bending stiffness remain unmeasured.
- **Window section:** painted wood, fixed to the table (sheet 2 in `fabrication/`). Wall below 70 mm tall, its top
  the stool (projecting 60 mm in front of the jambs) and, behind, the sill channel's floor. Interior jambs (the
  reveal) 40 mm wide (cropped), their room-side faces 45 mm in front of the seated screen's room-side face, their
  inner faces 40 mm out from the screen's sides; behind them the window frame's sides stand 3 mm from the seated
  screen's sides (412 mm apart).
  Stop 12 mm thick, lapping the screen by 10 mm on all four sides; glass 24 mm behind it. Sill lip 4 mm tall, 6 mm
  thick, its back face 12.6 mm in front of the stop (the frame's 11.1 mm plus 1.5 mm); head lip's underside 350 mm
  above the sill channel's floor (6 mm below the seated screen's top), its back face 15.9 mm in front of the stop (a
  5/8 in head channel, room to tip the screen's top in); head channel ceiling 362 mm above the floor. The head's lower 10 mm, the sill lip and the stop are wood-toned so they read from the room. Two 16 x 3 mm straight probe ports run horizontally through the room-side head at the spring centers, top flush with the 362 mm ceiling; the bearing ceiling remains intact (sheet 4).
- **Cost:** verified screen-parts subtotal USD $41.05. A listed 10 N gauge adds USD $124.23; fixture materials are not priced. See
  [`fabrication/README.md`](fabrication/README.md).

![The screen](fabrication/sheet-1-screen.png)

## Run a trial

From this folder, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Push the screen top into the head channel against its springs, swing the bottom over the sill lip, lower it into the sill channel against the stop, then release." \
  --scorer task_score --max-steps 900 --epochs 2 --store-frames --log-dir ~/yam-eval/insect-screen-dream
```

The recorded Dream-1 run uses two epochs (A/B). Use `--epochs 3` to evaluate all three lots.
Trial k uses lot k mod 3 (A, B, C), and `task_score` reports the mean. `TASK_LOT=A|B|C` in front of the command
pins every trial to one lot. Submissions run with `-E object_jitter=0`. The scripted demo is the task file's
`demo()`, run by the stock scripted policy (`--policy yam_scripted`) with `--epochs 3` and the other arguments unchanged.

## Lots

| Lot | Screen's start |
|---|---|
| A | centred on y = 0 (the screenshot) |
| B | 25 mm to the robot's left |
| C | 25 mm to the robot's right |

A replay of one lot's motions reaches 25 mm to the side of the other lots' pull tabs and misses them. The lots stop one fixed replay; a policy that memorised all three layouts is not ruled out.

## Scoring

Judged in the simulator after every control step.

**Score 0 for the whole trial if, at any step:** the hands or the screen could not be read (fail closed), or the
screen was dropped: its origin falling faster than 0.5 m/s while every hand point is at least 20 mm from it.
**Score 0 at the end unless** every sampled point of both grippers and fingertips (each collision hull's vertices and
its faces sampled every 3 mm) is more than 5 mm from the screen, its tabs and its springs, and none of the four frame back corners has moved more than 1 mm in Euclidean distance for 1 s. The rigid frame pose is reconstructed from four visible front frame/mesh intersections; proxy pad corners are not physical landmarks. Drop speed is the finite difference of origin height at adjacent 20 Hz control observations, not continuous peak speed. Physical video must be evaluated at the same 0.05 s sample interval. Doing nothing or refusing scores 0.

| Score | Frame geometry (additional crown/hold checks below) | Tool |
|---|---|---|
| 1.0 | **Seated:** every corner of the screen's back face within 1.5 mm of the stop (its room-side face 43.5 mm or more behind the jambs' room-side faces at each corner); both bottom corners within 1.5 mm of the sill channel's floor (the bottom rail shows 16.55 mm or less above the sill lip at both ends; 15.05 mm when down); within 2 degrees of upright; mesh and tabs toward the room; no side edge more than 0.5 mm past the window frame's side | Ruler or depth rod; a square-on photo with a ruler in view |
| 0.5 | **Top captured only:** not seated, but all four top corners at least 3 mm up behind the head lip (the top rail shows 16.05 mm or less below the head lip at both ends; 13.05 mm seated), within 15 degrees of upright, mesh and tabs toward the room, no side edge more than 0.5 mm past the frame's side: for example the top in the head channel with the bottom left standing on the stool in front of the sill lip | Ruler |
| 0 | Anything else: still on the stool, the top not up in the head channel, turned round or upside down | |

The nominal ruler readings in the table assume an upright screen. For a tilted screen use the projected rail width
and thickness in a square-on photo with a ruler; sheet 3 shows the reading positions. With width-axis lean a, the
lowest top rise in mm is `19.05 cos(a) - 11.1 max(0, sin(a)) - exposed front-rail height`; check both ends (and account
for roll). A depth rod checks that the whole top edge is behind the head lip. The upright 16.05 mm reading alone is
insufficient when the screen leans. Rails through the stop or ceiling by more than 0.5 mm, or the floor by more than
1 mm, receive 0 as impossible simulator states. Full credit additionally requires both spring crowns bearing as described below.

Layouts cycle A/B/C; scores average evaluated trials. The demo covers all three; the recorded two-epoch Dream covers A/B and leaves C untested. The rule that no side edge is more than 0.5 mm past the frame's side
only guards against bodies passing through each other in the simulator. `near_misses.py` checks each limit on both
sides, unintended outcomes, the whole-trial rules, and that the ruler readings above give the scorer's answer.

## Scripted demo

`demo()` plans from the published start layout of the current lot (where the screen stands) and the fixed window;
it reads nothing from the simulator. Both arms follow steps 1–3. During the bottom swing, the left holds its last raised TCP target and pinch while the right continues its tab motion; both complete a 2 s high back-against-stop segment before lowering. The left hand's target is held; the screen can still move a little through arm compliance and tab slip. The plan:

1. draws back clear of the screen from the ready pose, then comes down in front of its pull tab with the fingertips leaned 46 degrees in toward the screen (`tilt.py`, so
   the wrist stays clear of the mesh), and pinches the tab;
2. lifts the screen off the stool, draws it back from the head, centred on the opening, and leans it back 22 degrees
   so that its top passes under the head lip (the hands turn 3 degrees less to compensate for rotation seen
   in diagnostic traces);
3. moves the top in under the head until its back edge is against the stop, then turns the screen up about that
   edge, raising the top into the head channel against the springs, keeping it low while the screen still leans far
   back and pressing it up toward the ceiling as it comes upright;
4. holds the left TCP target and pinch at the last raised waypoint while the right continues the lower-tab swing. The nominal frame path plans 5.3–5.5 mm bottom height over the unchanged 4 mm lip. The actual frame can lag that path through arm compliance and grip slip; it is not a rigid pivot at the left hand. Both arms resume for a 2 s high back-against-stop segment before executing the final lowering segment;
5. eases the press, opens its fingers and draws back; it plans to return to ready, but the runner may stop after success.

The arms run low and short of their targets under load (observed in earlier simulation traces: 5-9 mm low, 5-7 mm short), so
each phase is aimed further (`SAG` in `task.py`, 30 mm final lift compensation): up toward the head channel's ceiling, which stops the screen where a
person's push would, and toward the stop, which the screen presses against. The nominal screen-path template avoids the window (`task.plan_clearance()`, which also reports planned spring compression). With the fixed-left segment, this is a planning template for the right target, not a measured executed clearance or proof that both commands describe one exact rigid-frame pose. GPU state/contact evidence, not the template, establishes the recorded end state.

The demo uses `tilt.py` (a runtime patch of the stock grip rotation, in the task's own process; it affects only the
scripted demo, never Dream-1 or the scoring), `parts.py` (articulated parts: the springs' crowns are sprung sliders) and
`lots.py` (per-trial layouts). `task.plan_clearance()` samples the nominal frame-path template; the offline checker separately verifies both actual arm command plans for IK and fixed-object collision.

Results for the frozen source (task.py SHA-256 `b31f18c7244a18fc3b1a87dfe31fd5263f8c3ec3eba27188006efa624ebfdb5c`):

| Run | A | B | C | Mean |
|---|---|---|---|---|
| Scripted demo | 1.0 | 1.0 | 1.0 | 1.0 |
| Dream-1 | 0.0 | 0.0 | not evaluated (two epochs) | 0.0 |

The direct demo completed three successful trials with zero errors. Same-demo passive records confirm the declared
spring targets, both apex predicates, hands clear and completed frame hold in every lot. Matching Dream-1 used the
same frozen task source, exact ordered instruction and 900-step cap; termination reasons were
`max_steps, max_steps`, with zero errors. Recorded abstentions: `{}`.
C was not evaluated by Dream. The videos show lot A; raw records cover every evaluated epoch. Model force fields are
calculations, not contact-force sensors or physical spring measurements. These are simulation results, not a real
assembly/grip/force/access validation.


## Files

| File | What |
|---|---|
| `task.py` | Scene, scoring, the trial record and the scripted demo |
| `near_misses.py` | Every tier's limits on both sides, unintended outcomes and the whole-trial rules, checked offline; also that the ruler readings agree with the scorer and that `fabrication/dimensions.json` carries the model's dimensions |
| `fabrication/` | The real fixture: parts list with prices, steps and checks (`README.md`), four drawing sheets, `dimensions.json`, and the script that writes them |
| `parts.py` | Kit for articulated parts (the springs) |
| `lots.py` | Per-trial layouts |
| `tilt.py` | Tilted grips for the scripted demo |

## Real-world trials

Build the screen and the window section from [`fabrication/README.md`](fabrication/README.md); clamp the window
section to the table. Frame geometry uses the ruler readings under "Scoring"; full credit also requires the crown contact/load and tracked 1 s hold checks in the fabrication instructions. Those checks have not been demonstrated on a real fixture. The simulated screen mass is an estimated 0.22 kg; a dropped screen ends the trial with 0. Installed spring force, tab retention and nondisturbing probe/video access must pass enrolment before physical trials. This is a reviewable simulation and proposed build package, not a validated real-trial kit.

## Runtime

A trial allows 900 control steps (45 s at 20 Hz). The selected raw evals report wall durations of about
847 s for the A/B/C demo and 869 s for the two-epoch Dream evaluation,
excluding time spent waiting for a simulator. These are eval wall times, not human baselines. Physical trial,
construction and reset times have not been measured. Layouts are fixed per lot (`-E object_jitter=0`); there are no
seeds to choose.

## Spring model and physical limitations

The nominal model uses **400 N/m**, 1.77 times the measured free-part endpoint secant
`1.58 N / 0.007 m = 225.7 N/m`. A separate 0.04905 N drive offset compensates the artificial 5 g crown weight
upright; the constant rest target is -0.122625 mm, applied once after reset. No installed preload was measured;
the nominal model therefore has none. Upright static net axial resistance is `400*q` N with q in metres:
**0.4 N seated, 1.6 N at 4 mm, 2.6 N at 6.5 mm and 2.8 N flat**. Total elastic drive adds 0.04905 N.
At lean a the static axial estimate is `400*q + 0.04905*(1-cos(a))`; it is not a contact-force sensor or a
ceiling-normal measurement. In frictionless static equilibrium with a compressed slider, ceiling-normal reaction
would be axial resistance divided by cos(a), not multiplied by it. Damping and joint-stop reactions are separate.
The equal internal reaction acts on crown and frame; neither gravity nor episode poses are modified.

For every q in 0–7 mm, the nominal linear net resistance is at least the **assumed free-part linear secant**
`225.7*q`; full-compression resistance 2.8 N also exceeds the measured 1.58 N endpoint. This states exactly what
the bound covers. It **does not prove an upper bound on an unknown nonlinear installed curve**, preload or gauge
uncertainty, and it does not assume that a free-part secant measures installed stiffness. Enrol both springs in
their actual corner keys across the executed loading/unloading path. Reject or remodel any assembly whose
measured resistance plus uncertainty exceeds the model, or whose proud height/travel differs materially.

## Room-side spring check and visible-frame hold

Why the springs bear when the screen is seated: each spring stands 7 mm above the top rail, and a seated screen's top is 6 mm below the channel ceiling, so a seated screen with both springs in place presses both. The feeler check below confirms this directly at each spring.

These are the specified fixture dimensions; check installed spring height before real trials. The two straight
16 × 3 mm head ports are scoring aids in the test fixture. Use a straight 0.10 mm feeler with 45/55 mm witness
marks, after empty-port and known-open controls on both sides. Refusal only within that interval confirms the
apex contact; an earlier obstruction is not a pass.

The 1 s hold checks every frame back corner's Euclidean movement throughout the second at 20 Hz. Endpoint ruler
readings alone do not establish that bound. The calibrated camera method uses visible frame landmarks; no hidden
spring-corner tracking is required. Full construction, feeler, uncertainty and camera procedures are in
[fabrication/README.md](fabrication/README.md#room-side-spring-check-and-visible-frame-hold). They have not been
validated on an assembled fixture.

## Hands

Both hands carry the screen by its pull tabs; neither touches a spring. In the scripted demo the left hand holds its raised position while the right swings the bottom in over the sill lip, then both lower it. A coordinated two-hand carry also reaches full score in simulation, so the demo does not show that this split of roles is required. The vinyl tabs are modelled as rigid; how much they bend or slip under this load has not been measured.

Before lowering, both hands complete the 2 s high segment against the stop.

## Related board tasks

Unlike DIN-rail hook, pivot and latch mounting ([PR 37](https://github.com/robocurve/benchmark-tasks/pull/37)) or two-end panel alignment ([PR 943](https://github.com/robocurve/benchmark-tasks/pull/943)), this task inserts a four-sided frame into opposed head and sill channels, lifting it against two springs and swinging it in over a sill lip.
