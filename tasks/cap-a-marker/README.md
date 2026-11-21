# Cap a dry-erase marker: pull its cap off the back end, turn it round and push it onto the tip until it clicks

An EXPO Low Odor chisel-tip dry-erase marker lies on the table. Its cap is posted on its back end, the way it is left
while writing. The instruction states only the goal: *Take the cap off the back of the marker, put it on the tip until
it clicks, and leave the capped marker lying on the table.*

Capping a marker after writing, so it does not dry out, is something nearly everyone does many times over, at home, at
school and at work. For a robot the work is physical:

- **A posted cap has to be pulled off.** It is gripped on the barrel's narrower back end by the same snap ring that
  later holds it on the tip (about 1.2 N to pull off, an estimate). Pulled on a marker that nothing holds, it only drags
  the marker along.
- **It has to be turned end for end.** Posted, the cap points back along the barrel. To go on the tip it has to point
  the other way: the robot has to reorient one of two small, light parts (a 6.8 g marker and a 2.2 g cap).
- **It has to be lined up and pushed home.** The cap's 13 mm bore has to go over a 5 mm nib and onto an 11.7 mm nose.
  A cap slid on stops against a 0.45 mm ridge with its mouth about 3 mm short of the shoulder. Only a push of about
  5 N along the marker drives the snap ring over the ridge, and the marker has to be braced while that push goes in.
  A cap resting on the ridge scores 0.6, not 1.0.
- **Putting it back on the back end is not progress.** It earns nothing beyond the 0.2 for having pulled it off.

**Nearest tasks, and how this differs.**

- *Cap and sort four pens* (another team's task, not accepted): its caps had a bore with no engagement feature, then a
  liner with no collider. The review asked for a cap whose retention resists insertion, as a real one does. Here the
  retention and the click come from contact between a ridge on the barrel and sprung pads in the cap. Nothing is
  switched on by a rule. The force it takes is set by the springs, and the demo has to supply it. This task also uses
  one named, buyable marker and starts with the cap posted.
- *Load a new lancet into a pen lancing device* (accepted): snaps an end cap on as one step of a medical procedure, at
  2 N, counted as on within 1 mm by a rule.
- *Replace an electric toothbrush's worn brush head* (accepted): pushes a brush head onto its shaft.
- *Unscrew the capped needles from three Luer-lock syringes* (accepted) unscrews caps. The pending 9-volt and rebar
  tasks push loose covers on.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the marker, the cap with its clip and snap ring, `success()` and `score()` |
| `markerkit.py` | Rigid bodies made of several shapes, and the sprung slider joints of the snap ring (adapted from the bodykit module used by other tasks in this repository) |
| `demo.py` | The scripted demo, its calibration mode and control runs |
| `test_scoring.py` | Offline checks (no simulator): the listed sizes, the snap's geometry, click and pull-off forces, every scoring outcome and attempts to score without doing the task, `success()` only at 1.0, the demo's reach, and `task.json` |

`markerkit.py` hooks `yam_arena.scene` and the embodiment's reset and step without modifying yam_arena. The task runs
with the standard `yam_arena` embodiment and the `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the cap off the back of the marker, put it on the tip until it clicks, and leave the capped marker lying on the table." \
  --scorer task_score --max-steps 2400
```

Run limit: 2,400 steps (120 s at 20 Hz). The layout is fixed (`-E object_jitter=0`); there are no seeds. `task_score`
prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Set `CAP_DEBUG=1` to print,
every 10 steps, the gap, the cap's offset and angle, how far each snap-ring pad is pushed out, and the score's
conditions.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 2400 --cam 640x352   # demo_*_cam.mp4, setup.png, result.json
DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record      # measures and prints CAL (closed loop)
DEMO_MODE=nopress python demo.py task.py ctl_nopress             # control: the cap is left on the ridge
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider test_scoring.py   # offline checks
```

1. The right hand picks the marker up by its barrel and turns it upright in the air, tip down. The posted cap is now on
   top.
2. The left hand takes the cap from above and pulls it straight up off the back end, along its own fingers.
3. The left hand stands the cap on its mouth on the table: it is now the right way up for the tip. The right hand lays
   the marker down.
4. The right hand takes the standing cap from above and waits with it. The left hand picks the marker up, turns it
   upright, tip up, and stands it on its back end. It lets go for a moment and takes it again 8 mm deeper in the hand,
   for a firm hold.
5. The right hand brings the cap in line over the tip, lowers it over the nib and onto the nose, and lets go with its
   mouth 5 mm short of the shoulder. The cap slides down onto the ridge (gap 3.0 mm).
6. The right hand turns its closed fingertips down onto the cap's top and presses straight down along the marker. The
   table takes the push through the barrel, and the cap clicks home (gap 0.0 mm).
7. The left hand lifts the capped marker, turns it level, lays it on the table and lets go.

Both hands close only a little past each part, which gives a light, centred hold. In development runs a full-force
pinch pushed the light cap off-centre in the fingers and tipped it. The demo runs open loop: it reads no simulator
state. Where each part ends up at four points (held for the pull, laid and stood, upright, over the tip) was measured
once with `DEMO_CALIBRATE=1` and is fixed in `CAL`. `result.json` reports `score()` on the last step as the trial's
score.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). The cap is **on the
tip end** when its axis is within 10 degrees of the marker's, pointing the same way as the tip, with the centre of its
mouth within 2 mm of the marker's axis. The **gap** is the distance from the cap's mouth to the barrel's shoulder,
along the marker. The cap is **off the back end** when the centre of the barrel's back end is 5 mm or more outside the
cap's mouth plane, or clear of its side.

| Outcome | Score |
|---|---|
| On the tip end with the gap 0.5 mm or less, and the marker lies on the table (within 10 degrees of level; the lowest points of its back end and of the cap's closed end or clip within 3 mm of the table top), no part of either gripper within 2 cm of the marker or the cap, neither moved more than 1 mm over the preceding 1 s; all held for 1 s (20 steps in a row) | **1.0** |
| Otherwise, at some point on the tip end with the gap 0.5 mm or less (clicked home) | **0.8** |
| Otherwise, at some point on the tip end with the gap 4 mm or less (on the ridge; an unclicked cap rests at about 3 mm) | **0.6** |
| Otherwise, at some point on the tip end with the gap 12 mm or less (the nib inside, the cap over the nose) | **0.4** |
| Otherwise, at some point the cap off the back end | **0.2** |
| Otherwise (the cap never taken off the back end) | **0** |

The lying check allows for the pocket clip: a capped marker may rest on its clip. `score()` gives 1.0 from the step
the full-score state has held for 1 s. `success()` returns true only on steps where `score()` is 1.0, and the runtime
ends the episode once `success()` has stayed true for its own 1 s, so an episode can only end with success at 1.0.
Putting the cap back on the back end, on reversed, or beside the tip earns nothing beyond the 0.2.

**In a real trial** everything is measured from outside the parts:

- the gap at the cap's mouth: with a feeler gauge or calipers at the end, and on the close video during the trial,
  using the 16.5 mm cap as the scale;
- the back end clear of the cap, the angles and the 2 cm clearance: by eye on the video;
- the heights of the two ends: with a ruler.

A cap that has not clicked cannot sit within 0.5 mm of the shoulder, because the snap ring holds it on the ridge. Check
this on the marker used: push the cap on gently without clicking it and measure the gap.

## How it is modelled

All parts are rigid bodies at the listed length, width and mass. The marker and the cap are each one body; the snap
ring is three small extra bodies.

**Retention and the click are contact.** Inside the cap's mouth, 1.9 mm in, are three rounded pads (1 mm radius, 3 mm
long, across the bore, at 120 degrees). Each slides radially on a PhysX D6 joint (`Slider` in `markerkit.py`): it is
free to move outward by up to 0.5 mm, everything else is locked, and a spring preloaded to 0.8 N and rising 2000 N/m
pushes it inward against its stop.

- **On the tip.** At rest the pads' inner edges stand 0.35 mm inside the ridge on the nose. A cap pushed on meets the
  ridge with its pads, and the contact between a round pad and the ridge's square edge turns the push into an outward
  force on the pads. Only when the push is enough do the pads ride out and over. The cap then sits home against the
  shoulder, and the pads behind the ridge hold it there.
  - The quasi-static push needed is 5.4 N at a friction coefficient of 0.3: 4.3 N at 0.2, 7.1 N at 0.4
    (`click_force()` in `test_scoring.py`).
  - An unclicked cap rests on the ridge with a gap of 3.0 mm in the demo (3.06 mm from the geometry).
- **On the back end.** Posted, the same pads press 0.2 mm out on the 12.3 mm post, so the cap is held by friction.
  Pulling it off takes about 1.2 N at static friction 0.35 (an estimate).

**Simplifications.**

- A real cap's snap is a moulded inner ring that flexes. Here it is three sprung pads, and their preload and stiffness
  are estimates, to be replaced by measurement (below).
- The barrel collides as a 16-sided tube and the cap as a 12-sided one, because perfectly round simulated bodies creep
  on the level table where real ones lie still.
- The clip is a solid bar with no gap under it.
- The nib is rigid.
- The internal details (bore, lead-in, nose, ridge position, post) are estimates for this marker, not drawings.

## Numbers and where they come from

| Quantity | Value | Source |
|---|---|---|
| Marker, capped | 122 mm long, 19 mm across at its widest (cap over the clip), 9 g | EXPO Low Odor chisel-tip marker, item 80001: the maker's product data, 4.81 in x 0.75 in, 0.02 lb |
| Barrel / cap split | 77 + 45 mm; 6.8 + 2.2 g; barrel 15 mm, cap 16.5 mm across, clip 2.5 mm proud | Estimates consistent with the listed length and width; measure the marker used |
| Nose, ridge, nib, bore, post | 11.7 mm nose, ridge 0.45 mm proud, 5 x 2 mm nib, 13 mm bore, 12.3 mm post | Estimates |
| Click force | about 5 N | **Estimate** (`PAD_PRELOAD`, `PAD_K` in `task.py`), to be replaced by the measured value |
| Pull-off force of the posted cap | about 1.2 N | **Estimate** (same springs, `POST_R` in `task.py`) |
| Friction | 0.35 static, 0.3 dynamic | Estimate; plastic-on-plastic coefficients are commonly 0.2 to 0.4 |

### Measuring the forces

**Click.** Stand the marker on its back end on a digital kitchen scale (1 g resolution). Put the cap on its tip until it
stops on the ridge, zero the scale, and press the cap down slowly with a fingertip until it clicks. The highest reading
before the click, in kg, times 9.81 is the click force in N (500 g is 4.9 N). Take the median of ten.

**Pull-off.** Post the cap, tie a thread loop round the clip, and pull it off along the marker with a luggage scale
while the marker is held. The peak reading is the pull-off force.

Then scale `PAD_PRELOAD` and `PAD_K` in `task.py` by the ratio of the measured click force to 5.4 N, and set `POST_R`
so that the pull-off matches. `test_scoring.py` recomputes both forces from these values.

## Verification

| Run | Score | What happened |
|---|---|---|
| Scripted demo | 1.0 | Cap pulled off the back end by step 390 (0.2); over the tip by step 1,390 (0.4); resting on the ridge at a gap of 3.0 mm by step 1,420 (0.6); pressed home by step 1,520 (0.8, then gap 0.0 mm); laid down with the gap 0.1 mm, 0.4 degrees off line; full score at step 1,846, episode ended by success at step 1,865 (93 s) |
| No press (control) | 0.6 | The cap was pulled off, turned round, lowered onto the nose and let go: it rested on the ridge at a gap of 3.0 mm from step 1,420 and never clicked; the marker was let go of standing |
| Dream-1, 2,400 steps | 0.0 | Both grippers went to the marker and nudged and turned it on the table, the left one closing near the posted cap's end, but the cap was never pulled off. Videos: overhead, left wrist and right wrist cameras side by side |
| `pytest test_scoring.py` | 23 passed | |

In earlier versions of this task (the cap lying apart), a cap brought down 8 degrees off line or 3 mm to the side still
went on: like a real cap, it centred itself on the nib holder and the nose when held lightly. In development runs, a cap
held stiffly 9 to 12 degrees off line jammed with its mouth on the nose's edge.

## Real-world set-up

Everything is bought: one EXPO Low Odor chisel-tip dry-erase marker (item 80001), a table, and tape marks for where it
lies. Use the same marker throughout, and measure its click and pull-off forces as above.
