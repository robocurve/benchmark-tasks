# Patch three nail holes in drywall: pack each full of spackle with a putty knife, then strike it flush

`task.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), `station` scene. An offcut of half-inch
drywall leans back 20 degrees in a bench stand with three holes in it (3 mm across and 6 mm deep, 4 mm by 8 mm, 6 mm by
10 mm). The robot takes a 1.5 in. putty knife from the bench, loads it from an open tub of pink spackle, packs each hole
full, strikes each patch off flush with the wall, and sets the knife down. The task's helper modules: `yamkit.py`
spawns the modeled props (panel, stand, tub, knife), `guardkit.py` checks gripper contact, and `pastekit.py` is the
spackle model, a height field on the panel's face worked by the knife's edge (see "The spackle model" below).
`test_task.py` checks the scorer offline: `python3 -m pytest -q test_task.py`.

Requirements: `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its
`setup.sh` (Isaac Lab-Arena `release/0.3.1`, `inspect-robots` 1.0.0, `inspect-robots-yam` 0.38.0), and
`source $ARENA_DIR/yam.env` in the shell.

## Run a trial

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/spackle-nail-holes-flush/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Patch the nail holes in the wall panel with the putty knife so they're ready to paint." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir logs/spackle-demo
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.
Video: `inspect-robots video logs/spackle-demo/*.json`.

## Scoring

`--scorer task_score` gives 1.0 when `success()` holds for 1 s (all three holes filled, nothing proud, the knife set
down) and otherwise `score()` at the end of the trial. Each threshold is defined once in `task.py` (`FLUSH`, `PROUD`,
`PROUD_AREA`, `SET_DOWN_Z`, `PANEL_CLEAR`, `STILL_STEP`, `STILL_TURN`, `HOLD_STEPS`, `GOUGE_CAP`) or in pastekit's
`Law`, set in `task.py` as `LAW`. The paste state (fills, coat, the knife's load, a gouge) is kept for the whole
trial; a gouge latches.

| Rule | Value | Why this much |
|---|---|---|
| Filled | deepest point within 0.3 mm below the face | It is the deepest point, not the average: one unfilled edge shows as a crescent. 0.3 mm (about three sheets of copy paper) casts a shadow in raking light and shows through paint. |
| Partial credit | 0.25 per filled hole | Each hole is a separate patch; filling two of three is two thirds of the job. |
| Proud | over 0.3 mm above the face, at most 10 mm² in all for 1.0 | A ridge or coat left standing is a bump under paint; 10 mm² (about 3 x 3 mm) is what one swipe of sandpaper takes off. Packing and leaving the coat scores 0.75. |
| Set down | no gripper touching (guardkit's contact check, not a distance zone); origin within 6 cm of the table top (on the bench, across or in the tub); every corner of handle, ferrule and blade 2 mm or more in front of the face; under 1 cm/s and 0.5 degree per step; held 1 s | The job is not finished while the knife is in hand or on the wall, where it can still drag across a patch. A knife dropped on the bench or left in the tub counts. The arm's pose is not scored. |
| Corner gouge (proxy) | edge turned more than 15 degrees out of the face, corner dragged 5 mm or more in one stroke: cap 0.5 | Torn or fuzzed face paper needs sealing primer and a second skim, about as much work again. 15 degrees lifts the far corner of a 38 mm blade about 10 mm off the wall, so a small wobble cannot trigger it. |

Doing nothing scores 0 (the holes are open). Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

Scoring a real trial, after the spackle dries (lightweight spackle, about 30 minutes):

- Filled: read each hole's depth with a caliper's depth rod at its deepest point.
- Proud area: zero a dial depth gauge (0.01 mm dial on a flat base wider than the patch) on the bare face beside the
  patch, then step it along a steel rule on a 2 mm grid over the patch and 1 cm around it. Each point over 0.3 mm
  stands for 4 mm²; 3 or more such points (12 mm²) over the three patches is over the limit, 2 or fewer (8 mm²)
  within it. A quick screen first: a straightedge laid across a patch rocks on a ridge.
- Gouge: a raking-light photo (a flashlight held flat to the wall), looking for torn or fuzzed paper.
- Set-down and tool: the video shows the knife released, still and off the wall, and that only the knife touched the
  spackle (a hole packed with a fingertip counts as open, as in the simulation, where only the knife moves spackle).

## The spackle model

The simulator has no paste, so the spackle is a scored model computed from the knife's pose each control step
(`pastekit.py`). The panel's face is a height field of 1 mm cells, each holding the height of the spackle's top above
the face; a hole's cells start at minus its depth.

- **Contact, not proximity.** The knife works the face only while its edge is on it: within 0.6 mm in front of the
  face (about PhysX's contact offset) or pressed in. The knife is rigid and the face is a collider, so the edge cannot
  pass through; a knife held 1 mm off does nothing.
- **Only a held, sliding knife.** A gripper's fingers or palm must touch the handle, and the edge must slide along the
  face at least 1.5 mm in 0.15 s (1 cm/s). A dab, a blade pressed and held still, or a jab straight in and out does
  nothing; a knife knocked across the wall with no hand on it does nothing.
- **The angle sets what is left.** Each cell the edge crosses is left at a level set by the blade's angle to the face:
  0.8 mm at 40 degrees or less, 0.1 mm at 60 degrees or more, linear between (0.3 mm at about 54 degrees). Spackle above that level is scraped onto
  the knife; a cell below it is filled from the knife's load. This is how drywall finishers work a knife: a low blade
  rides up on the wedge of paste under it and lays it on; a steep blade scrapes the surface nearly clean.
- **Packing takes a low angle.** At 40 degrees or less the blade presses the spackle rolled ahead of it into a hole
  and fills it from its floor. Steeper, it only tops up cells within 1 mm of the face and skims over a deeper hole. So
  a steep pass alone leaves the holes open (0), a low pass alone leaves a coat standing proud (0.75), and packing low
  then striking off steep fills each hole flush (1.0).
- **Packing and striking off are different strokes.** Every packing angle leaves a 0.8 mm coat, more than twice the
  0.3 mm limit, and only a blade steeper than about 54 degrees leaves 0.3 mm or less, so the strike-off is at least
  14 degrees steeper than any packing pass. Repeating one stroke at nearly the same angle (50 then 52 degrees, say)
  cannot both pack a hole and leave it flush; `test_task.py` checks pairs from 30 to 64 degrees. The demo packs at
  35 degrees and strikes off at 65. The 40 and 54 degree figures are modeled from drywall finishing practice ("less
  angle lays on, more angle takes off"); on a scrap with a protractor-set knife, a pass at 40 degrees leaves a visible
  coat and one at 60 degrees a bare wall.
- **A finite load.** Drawing the blade through the tub loads it, 0.5 mL per cm of travel with the whole edge in the
  spackle, up to 2 mL; scraped spackle also goes onto the blade. Spackle is conserved: what is laid comes off the
  load. The holes hold 0.43 mL in all; a low pass lays about 0.6 mL of coat around a hole, and striking it off puts
  most of that back on the blade. Smearing the whole area at a low angle from one load runs the knife dry on the wall
  before the holes are reached.
- **Corner gouge.** A proxy for torn paper; see the Scoring section.
- **Friction.** The panel's face has a friction coefficient of 0.3 (static) and 0.25 (sliding) against the knife,
  steel on paper wet with spackle; the other props keep the station's grippy 1.2/1.0.

What it does not model: torn paper (the corner-gouge proxy stands in); the spackle doesn't flow, slump, shrink or dry; the blade is rigid, while a real flexible
putty knife bends under pressure (which is how a worker feels it), and the pressure itself is not read; the direction
of a stroke (handle leading or edge leading) doesn't matter, only the angle; lifting off leaves no ridge at the end of
a stroke. The score checks the outcome a real trial checks: the depth left in each hole, what stands proud of the
wall, and where the knife ends up.

Why the panel leans back 20 degrees: the arm can't hold the knife level and also tip it 55 degrees nose-up at this
reach, and a slight lean also lets the overhead camera see the face. Patching a wall at a 20-degree lean is the same
blade work as on a vertical wall. The holes sit 16 to 17 cm up the face because a packing pass at 35 degrees
holds the handle about 11 cm below the edge: a hole much lower would put the knife's handle and the hand on the bench.

## Real-world build

- Panel: an offcut of half-inch drywall, 36 x 28 cm, paper face out. The 6 mm hole is the size a pulled 1/4 in.
  plastic wall anchor leaves (a pulled #6 screw leaves about 3.5 mm). Drill the holes square to the face with 1/8 in.,
  5/32 in. and 1/4 in. bits (3, 4 and 6 mm), each with a tape flag on the bit at its depth (6, 8 and 10 mm), at 16.0,
  16.5 and 17.0 cm up from the bottom edge and 7.0, 13.5 and 20.0 cm from the panel's right edge.
- Stand: a pine 1x2 (19 x 38 mm) screwed to the bench on edge as a stop in front of the bottom edge, and a pine 2x2
  brace from the bench to the panel's back 22 cm up, screwed so the face leans back 20 degrees (check with a
  protractor or an angle-finder app). Fasten the panel to the brace with two 1-1/4 in. drywall screws, 22 cm up the
  face, 1 cm either side of its center line (y -5.0 and -3.0), heads set just below the paper, so the knife's push
  cannot move or lift it; the stop keeps the bottom edge from sliding.
- Spackle: a half-pint (8 fl oz) tub of pink lightweight spackle that dries white, 9.0 cm inside, lid off, filled to
  5 mm below the rim, set in a plywood ring with a 9.5 cm hole screwed to the bench.
- Knife: a 1.5 in. flexible putty knife with a steel blade about 3.5 in. (89 mm) long and a plastic handle about
  28 mm wide, about 7.75 in. (197 mm) overall; weigh it (70 g modeled) and wrap the handle with tape to 28 mm if it
  is narrower.
- Safety: a blunt putty knife and spackle; nothing sharp, hot or heavy.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes).
- Scripted demo: the plan lasts 97.6 s (1,952 steps); the recorded run reached 1.0 and ended at step 1,900 (95 s). It
  picks up the knife, turns it toward the panel, dips its tip 4 mm into the spackle with the blade tipped 15 degrees
  down and draws it 4 cm, then for each hole a packing pass with the blade at 35 degrees to the face (2.2 cm down the
  face at 1.5 cm/s) and two strike-offs at 65 degrees (4.6 cm each), 3 mm to either side of the packing pass's line so
  they overlap its coat on both sides (the hand sits a millimeter or two differently along the edge at the two
  angles), then lays the knife back and lets go. Holding the knife at the panel, the arm's hand settles about 4
  degrees turned about the blade's normal from where it is sent, so the passes aim the knife 4.3 degrees the other way
  (`EDGE_YAW`) to put the whole edge on the face. The demo is open loop.
- No randomness: the layout is fixed (`-E object_jitter=0`).

## How the file works

- **The knife** is a yamkit `Prop` with box colliders for its handle, ferrule and blade (1 mm thick).
- **Spackle.** After every control step, `_update()` reads the knife's pose and whether a hand touches it, and steps
  a `pastekit.Paste` with the panel's face, the tub and the blade's edge. `success()` and `score()` share one update
  per step. A new trial (yamkit's episode counter) starts a fresh field with empty holes and an empty knife.
- **Visible spackle.** Cells with spackle are drawn as thin pink tiles in 2 mm blocks just in front of the face with
  Isaac Lab's `VisualizationMarkers`, deeper pink where they stand proud, so camera-based policies see them. The
  tiles are only a picture: if drawing fails, they turn off with one warning and scoring carries on.
