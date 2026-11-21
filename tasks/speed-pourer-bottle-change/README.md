# Move a speed pourer from the empty liquor bottle to the new one: hold the bottle, draw the pourer, push it home

Instruction given to the policy: `Move the speed pourer from the empty bottle into the new bottle.`

Files: `task.json`, `speed_pourer_bottle_change.py` (scene, `success()`, staged `score()`, the scripted `demo()` and
an offline self-test), `mechkit.py` (multi-box rigid parts, the stopper's friction fit, the tug force, level-grip
demo waypoints and per-episode hooks, added to the stock runtime in memory), `README.md`.

## The scene

Two 750 ml round liquor bottles stand loose on a rubber bar mat, 14 cm apart: the empty one (420 g) on the robot's
left with the pourer in its neck, the new one (full, 1,130 g, cap already off) on the right. Each is 29.5 cm tall
with a 7.6 cm body, a 2.9 cm neck and a 1.9 cm bore. The pourer is the common stainless free-flow type
(Spill-Stop 285-50 pattern): a tapered spout on a 2.6 cm collar, with a finned poly-cork stopper 3.2 cm long under the
collar (16 mm core with five flexible ribbed sealing fins with outer tip diameter 21.5 mm that compress into the 19 mm bottle bore, plus an 11 mm lead); 35 g, 11.8 cm tall overall. Nothing in the scene is powered or pressurised.

## What is modelled

- The stopper's fit. A real stopper's 21.5 mm flexible fins are squeezed by the 19 mm neck and grip it. In simulation the core is rigid
  and 1.5 mm clear of the bore all round, and the fin compression fit is modeled as a prismatic joint along the bore between the pourer
  and the bottle, with 6 N of Coulomb friction (a position drive whose force is capped at 6 N and whose anchor
  follows the stopper once it has slipped; 0.5 mm of elastic give). While the joint is closed the stopper can only
  slide along the bore, from the seat (collar on the lip) outward. Both joints exist from the moment the scene is
  built: the one in the empty bottle starts closed, the one in the new bottle starts open. A fit lets go when less
  than 2 mm of stopper is left in the bore and takes hold when the stopper's lead is 5 mm inside the bore, within
  3.5 mm of its axis and 8 deg of it; closing it does not move the stopper along the bore. Before that, and
  whenever a fit is open, the pourer and the bottle meet by ordinary contact.
- 6 N is a set value: makers do not publish a pull-out force for speed pourers. What matters to the task is that
  it is more than the empty bottle's weight (4.1 N) and less than the full one's (11.1 N), as with real ones: an
  empty bottle comes up off the bar with its pourer unless it is held, a full one does not.
- The bottles are free bodies on the mat. A 30 cm bottle with a 7.6 cm base goes over with about 1.5 N sideways
  at the lip, so a pourer brought down off-centre, or a hand that bumps a neck, knocks it over.
- The tug. Once the pourer is home in the new bottle with both bottles standing on the mat, both grippers and every
  collision piece of both arms (links, palm and finger pads, as posed) are clear of it, and it has been still for
  0.5 s, the scorer pulls the spout base straight up with 3.5 N for 0.5 s. A pourer pushed through the fit holds (it
  gives about 0.3 mm); one left shallow in the mouth without gripping the bore comes out. Missing or non-finite pose
  or robot-clearance readouts fail closed. Drawing the pourer out again cancels a pass.
- The spirit in the new bottle is rigid mass: no slosh, nothing spills. Glass does not break. A bottle more than
  45 deg off upright at any moment (knocked over) latches a score of 0 for the episode.

## Scoring

| Stage | Score |
| --- | --- |
| Pourer out of the empty bottle, that bottle standing on the mat | 0.3 |
| and its stopper started in the new bottle's neck (5 mm in, in line), that bottle standing | 0.5 |
| and pushed home (collar within 2 mm of the lip, within 2 mm and 5 deg of the axis) | 0.7 |
| and let go, both bottles and the pourer at rest, the 3.5 N tug passed | 1.0 |
| Either bottle knocked over at any moment | 0 |

`task.json` gives every threshold. Scored at the last step; success is 1.0 held for 1 s.

## Environment

Isaac Lab-Arena with `yam-arena` and `inspect-robots`; physics at 240 Hz, control at 20 Hz.

## Offline check, no GPU

```bash
python tasks/speed-pourer-bottle-change/speed_pourer_bottle_change.py
```

It asserts the scoring rules on hand-built states (stages, the knocked-over latch, the fit's take-hold and let-go
rule, a finger left on the pourer, non-finite/missing clearance fail-closed, off-mat bottle rejection) and replays
both arms' demo plans through the kinematics at 20 Hz (every joint inside the URDF limits, IK error under 0.5 mm).
Expected output:

```
scoring: 31 cases pass
left: 47.0 s plan, worst IK 0.088 mm / 0.018 deg
right: 45.5 s plan, worst IK 0.096 mm / 0.017 deg
```

## Run and score one trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/speed-pourer-bottle-change/speed_pourer_bottle_change.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Move the speed pourer from the empty bottle into the new bottle." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir runs/speed-pourer-bottle-change
```

For Dream-1 use `--policy dream` with the same arguments. `task_score` is 1.0 when `success()` has held for 1 s,
otherwise `score()` at the last step. A trial takes about 47 s of simulated time for the demo and 60 s at the run
limit.

The demo is `demo()` in the task file, open loop from the fixed layout. Both grippers work level, fingers closing
across a pair of flats. The left hand comes in from the left, takes the empty bottle by its neck under the bead and
holds it down; the right hand comes in at 45 deg, closes on the thick end of the spout and draws the pourer straight
up out of the neck, then carries it across to above the new bottle's lip. The left hand lets go, backs off, turns
and takes the new bottle's neck; the right hand lowers the stopper into the neck, pushes the collar onto the lip,
lets go and backs off; the left hand lets go and backs off. The scorer then makes
its tug. `DEMO_VARIANT` selects a wrong way of doing it: `one_handed` (the empty bottle is not held while the
pourer is pulled), `rest_only` (the pourer is set in the new neck and let go without being pushed home), `none`
(neither arm moves).

## Results (seed 0, no jitter)

| Run | task_score |
| --- | --- |
| Scripted demo (run 1, 942 steps) | 1.0 |
| Scripted demo (run 2, 942 steps) | 1.0 |
| `one_handed` (unheld empty bottle, 1,001 steps) | 0.0 |
| `rest_only` (shallow resting, 1,001 steps) | 0.5 |
| `none` (zero policy actions, 61 steps) | 0.0 |
| Dream-1 (1,200 steps) | 0.0 |
