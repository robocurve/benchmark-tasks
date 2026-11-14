# Fit a spin-on oil filter to gasket contact, then three quarters to one turn by hand, not left loose or cranked on

A new spin-on oil filter has been started on the threaded nipple of a remote oil filter mount (a filter head) bolted to
the bench, pointing up, and stops three quarters of a turn short of its gasket touching the head. "Fit the new oil
filter." means doing what filter makers print on the can and what automotive mechanics do at every oil change: spin it on until the
gasket contacts the head, then tighten it a further three quarters of a turn (some makers say three quarters to one
turn) by hand, and let go. Stopping at contact
(it feels tight already) is the tempting wrong answer: the filter can work loose under engine vibration and leak.
Cranking it on well past a full turn can crush the gasket, which can roll out of its groove and blow out under oil
pressure, and the filter may not come off by hand at the next change.

## Spec

| Item | What it is |
|---|---|
| Scene | One fixed filter head (die-cast aluminum, 7.5 x 9 x 3.4 cm block with a 7.2 cm round gasket land on top, its two side hose ports plugged), the filter's axis at x 30, y -12 cm; a black spin-on filter, 6.6 cm across and 7 cm tall, on its M20 x 1.5 nipple |
| Start | Filter 270 degrees (three quarters of a turn) short of gasket contact, about four turns onto the thread (five at contact); its index stripe faces left |
| Instruction | "Fit the new oil filter." |
| Unstated rule | Gasket contact, then three quarters to one turn more by hand (the makers' printed instruction): not left at contact, not cranked on |
| Core mechanism | A modeled thread (1.5 mm per turn) whose turning friction grows with the angle past gasket contact, turned by a regrasp loop of wrist rolls |
| Category | Impact on jobs: automotive service technicians and mechanics (O*NET 49-3023.00; 825,800 US jobs in 2025, BLS Occupational Outlook Handbook) replace oil filters as regular work. O*NET lists the task "Perform routine and scheduled maintenance services, such as oil changes, lubrications, and tune-ups", and the BLS duties list "changing oil"; a new filter is normally fitted at each oil change |
| Trap | Stopping at contact or a little past it (0 or 0.25); over-tightening past a turn (0.5) or past 1.25 turns (0, irreversible) |
| Score | Degrees past contact at the end, in bands with reasons (below); the turning past contact must come from a holding hand's own wrist roll; let go |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `guardkit.py` | The task's helper module for contact checks: the grippers' own shapes (boxes fitted to the YAM's collision meshes) and the 2 mm contact test used to decide whether a hand is still on the filter. |
| `twistkit.py` | Threaded parts: the filter is the "cap" of an articulation (a slide along the nipple coupled to a hinge, 1.5 mm per turn), with a seat (gasket contact), a gasket friction growing with the angle past it, a hard stop 540 degrees past it, and "off" five turns back. Gives the scorer the filter's angle from contact and which gripper holds it, and builds the demo's regrasp loop. Built on `yamkit.py`. |
| `yamkit.py` | Helper that spawns the modeled props and articulations, and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline scorer checks: step-by-step histories of the thread and the grippers, near misses included. No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/oil-filter-hand-tight-plus/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Fit the new oil filter." --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/oil-filter-hand-tight-plus/test_task.py` (prints `all pass`).

## The demo

The right arm opens over the filter, descends with its fingers around the top of the can and spins the filter on
clockwise (seen from above) in wrist strokes of 120 degrees: close, roll in 15-degree steps (sinking with the thread),
open, roll back, close again. Four strokes of 120 degrees and one of 105 roll the wrist 585 degrees: three quarters
of a turn to gasket contact, 300 degrees past it (the middle of the full-credit range) and 15 degrees for the pads
slipping on the can. The demo run scores 1.0, so the filter ends 240 to 360 degrees past contact with at most 45 degrees
of turning not matched by the wrist. It lets go and lifts clear, and the run ends at 1.0 once the full score has held for 1 s, before the arm is back home; the left arm stays home.

## Scoring

Over the whole run (`success()` records it every step), the filter's angle is measured from gasket
contact (the seat), positive when tightened past it; it starts at -270.

| Outcome | Score |
|---|---|
| The filter came off its thread at any point (unscrewed 4.25 turns from its start, five back from contact), even if spun back on: the fit is undone, and starting it again risks cross-threading | 0, irreversible |
| Tightened more than 450 degrees (1.25 turns) past contact at any point, even if backed off: the gasket is over-compressed and can roll out (angle as proxy; the real gasket is checked afterwards) | 0, irreversible |
| Ends under 90 degrees past contact (not touching, at contact, or under a quarter turn past), including doing nothing: it can work loose under vibration and leak | 0 |
| More than 45 degrees of the turning past contact not done by a holding hand's wrist (knocked, swiped, the other arm): the makers say by hand, and a spin has no control of where it stops | 0 |
| 90 to 180 degrees past contact: sealing, but at most two thirds of the makers' figure, so it can still loosen | 0.25 |
| 180 to 240 (close to the figure but short), or 360 to 450, or ever past 360 (past the most any maker allows, hard to remove by hand, not crushed) | 0.5 |
| 240 to 360 and never past 360, a hand still on the filter: not finished while a hand can still turn it either way | 0.5 |
| 240 to 360, let go: the makers' three quarters to one turn | 1.0 |

Where the arms end is not scored. Possible scores: 0, 0.25, 0.5, 1.0.

Who turned it. A gripper holds the filter (`twistkit.held`) when its fingers are closed on the can, its opening from 15
% of full travel tighter than the can's 6.6 cm (the pads pressing in) to 3 % looser (2.8 mm of air in all, about the
pads' compliance), so an empty fist or a hand hovering loosely around the can does not count, with its tool point
within 1.5 cm of the can's axis anywhere from 1 cm above the gasket face to 1.5 cm above the top (the pad tips sit
about 1.3 cm below the tool point, so that is every grasp height with the pads on the can). Each step, a gripper holding the filter at either end of the step (so the grasp and release
steps count) adds its own wrist roll, clockwise seen from above, to a budget (kept at most 30 degrees ahead of the filter, two
15-degree steps of the filter catching up late; more is the pads slipping on the can), and new ground the filter gains past
contact (beyond the furthest it has been) is credited from that budget; whatever the holding wrist did not turn is
booked as unheld, and turning back and forth over the same angles counts once. More than 45 degrees unheld scores 0:
that slack covers the filter settling as the hand lets go, and a swipe, a knock or the other arm turning it would add far
more. Each hand keeps its own budget, so a second hand near the can, or on it steadying or helping, never voids the
holding hand's turning; with both hands on the can the larger roll is credited (and used up from both). A swipe is
still caught: whatever turning no holding wrist rolled is unheld, wherever the other hand is.

A hand is still on the filter at the end when it holds the can (as above) or when its palm or fingers touch the can:
the gripper's own shape (boxes fitted to the YAM's collision meshes, at the fingers' present opening) within 2 mm of the
can, from its gasket face to its top (2 mm is the solver's resting gap plus the boxes' fit to the meshes). That caps the
score at 0.5, since the job ends with the filter let go: a hand still on it can still turn it either way, so the angle
is not final. A hand hovering beside or over the can, open or closed, is not on it, however close it comes short of
2 mm. A real trial checks the last frames for a pad or finger still on the can.

`success()` is a full score held for 1 s. Each scene reset by the simulator (counted by yamkit) starts a new history.

Why these limits:
- 240 to 360 degrees: filter makers print "after the gasket contacts the base, tighten three quarters of a turn by
  hand", and several print "three quarters to one full turn". Nothing in the scene says which, so full credit spans
  both readings, less 30 degrees (a twelfth of a turn) for judging three quarters of a turn by hand against the can's
  quarter-turn ticks.
- 360 to 450 earns 0.5: past the most any maker allows, so the gasket is over-compressed, but within a quarter turn of
  it, so not counted as crushed.
- Over 450 (1.25 turns) is 0 and irreversible: the gasket is squeezed out of shape, and backing the filter off does not
  restore it. The sim does not deform the gasket, so the peak angle is the proxy; a real trial reads the peak from the
  video against the witness marks and checks the gasket for a flattened or rolled lip after the trial.
- 45 degrees not turned by a holding wrist: the filter settling as the hand lets go stays well under it, and a swipe,
  a knock or the other arm turning it adds far more. A real trial checks the video for the filter turning without a
  hand closed on it.
- 180 to 240 earns 0.5 (tightened, but under the makers' figure); 90 to 180 earns 0.25 (partly tightened, a third to
  two thirds of the figure); under 90, a third of the least any maker asks, is not counted as fitted and earns 0.

In a real trial: before the trial, spin the filter on to gasket contact by hand, draw a line across the can and the
head (the witness line), and back the filter off three quarters of a turn to its start. Tape a printed protractor ring
(15-degree marks) around the head's gasket land. Afterwards read the can's line against the ring, counting whole turns
from the video, and photograph it from above. Watch the video for the filter being spun by a swipe rather than turned
in the hand, and for the hand letting go.

## Runtime and seeds

- Demo: 559 control steps (28 s simulated) to a held full score.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

True size (a small spin-on filter of the motorcycle and small-engine size, which a parallel gripper can span). The head
is mounted with the filter pointing up so a straight-down grip reaches it; on most engines the filter hangs down or
sideways, but the fitting rule is the same, and some engines and remote mounts carry it upright. The thread is a model:
the filter sits on a slide along the nipple and a hinge, and every control step `twistkit.py` drives the slide to 1.5
mm of travel per turn (at most 0.5 mm past gasket contact, the gasket compressing, so it does not sink into the land)
and holds the filter where it was left with a small torque. The slide is held to the thread by a spring (3,000 N/m)
that sags about 0.8 mm under the 250 g filter, so in the sim the gasket starts about 0.3 mm above the land, not the
real 1.1 mm (270 degrees at 1.5 mm per turn), and rests on the land about 70 degrees before the angle the scorer counts
as contact. The score reads the thread's angle, not the filter's height, so this changes only what the video shows. Past gasket contact it adds a turning friction that grows
with the angle, 0.1 N·m per radian (0.47 N·m at three quarters of a turn, 0.63 N·m at a full turn), capped at 1 N·m so
the arm can still finish: a stiff hold where it was left that slips at that torque, so it opposes turning either way
and never turns the filter by itself (stick-slip friction, not a spring). A hard stop sits 540
degrees past contact. The real torque by hand at three quarters of a turn is about 15 to 20 N·m, 15 to 20 times the sim's
cap, so contact is felt far less here and shows mainly as the gasket meeting the head and the stripe lining up with the
head's witness line. The sim therefore tests where the turn stops (the count of degrees past contact, judged by eye and
by the ticks), not the effort of tightening; a real trial on a real filter needs the full hand torque. No
oil, gasket deformation or leak is simulated; the scorer reads the angle, and a real trial reads the same angle from
the witness marks (the line across the can and head, and the protractor ring).

## Real-world build

A universal remote oil filter mount with an M20 x 1.5 nipple (aluminum, about 7.5 x 9 cm, two side ports with plastic
plugs) bolted to a board clamped to the bench, nipple up, so the filter's axis is at x 30, y -12 cm. A small spin-on
oil filter, about 6.6 cm across and 7 cm tall, with the M20 x 1.5 thread (motorcycle and small-engine filters of this
size are common; many car filters are larger, about 7.6 cm across, and are fitted by the same printed rule), its gasket
lightly oiled. A paint-pen index stripe down the can's side (the top left plain), three
black quarter-turn ticks, and a black witness line on the front of the head, set as in the real-trial steps above.

## Licenses

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape
is generated by `yamkit.py` and `twistkit.py`.
