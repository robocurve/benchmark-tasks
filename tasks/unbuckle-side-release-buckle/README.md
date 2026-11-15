# Unbuckle a side-release strap buckle: squeeze both of its tabs in at once while drawing the halves apart

A 38 mm plastic side-release buckle, the kind on backpack hip belts and chest straps, luggage and camera straps, pet
harnesses and life jackets, lies buckled and flat on the table, each half on a 12 cm tail of nylon webbing: the female
half on the robot's left, the male half on its right. The robot must unbuckle it, taking the two halves apart.

The male half has a centre tongue and two sprung prongs that reach into the female half. The tab at each prong's tip
stands 3 mm out of a window in the female's side and is hooked 1.5 mm deep behind the lip of the female's mouth. The
halves part only while both tabs are pressed in at the same time (by at least 1.5 mm; flush with the female's sides is
3 mm) and the male is drawn out. One tab pressed leaves the other hooked. Pulling with neither pressed only drags the
whole buckle across the table.

What the task tests:

- **Squeezing both tabs at once, in the right place.** The tabs are 16 mm long and 6 mm tall, just behind the female's
  wider mouth. Fingers that land on the mouth stop on it before the tabs are pressed.
- **Keeping them squeezed while the halves are drawn apart.** The squeeze has to hold the female too, and last until the
  tabs are past the lips.
- **Holding the other half back.** Both halves are loose and light. Something has to keep the male half from following
  the female, such as the other hand on its strap.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `bucklekit.py` | Rigid parts built from boxes, with spring-loaded sliders, for yam_arena task files |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Offline checks: every scoring outcome, and the buckle's geometry (clearances, the catch, and that it lets go only with both tabs pressed) (no GPU) |

`bucklekit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with
the standard `yam_arena` embodiment and `task_score` scorer. It follows the part kit used by other tasks in this
repository.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/buckle-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1200              # writes demo_*_cam.mp4, setup.png, result.json
DEMO_DEBUG=1 python demo.py task.py debug_out --no-record      # also prints the halves' poses, the tabs and both TCPs
python -m pytest test_scoring.py                               # offline checks
```

1. The right arm, its fingers closed, presses the male half's webbing tail down on the table near its end and keeps
   pressing.
2. The left arm comes down over the female half with its fingers open 52 mm across the buckle. The pads are centred
   28 mm behind the mouth, over the tabs and clear of the mouth. The hand leans 10 degrees out along the buckle so that
   its wrist stays clear of the right arm. Closing presses both tabs in until the pads stop on the female's sides, and
   holds the female.
3. Still squeezing, the left arm draws the female 75 mm straight off along the buckle, away from the held-down male.

The demo runs open loop. It reads no simulator state while it runs; every target comes from the fixed layout in
`task.py`. With `-E object_jitter=0` there are no seeds. It reaches 1.0 (success) at step 313 (16 s), with the halves
70 mm apart.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). The gap is measured
with a ruler from the centre of the female's mouth to the centre of the front face of the male's base. Buckled, it is
2 mm.

- 0 if either half fell off the table (more than 5 cm below the table top) at any point.
- 1.0 (success, which must hold for 1 s): the gap is at least 52 mm, so the 47 mm tongue is at least 5 mm clear of the
  mouth, for 20 control steps (1 s) in a row. Holding the halves up in the air counts.
- 0.75: the gap was at some point at least 52 mm.
- 0.5: the gap was at some point at least 8 mm: the tabs are past the lips, so the buckle was released.
- 0.25: both tabs were at some point pressed in at least 1.5 mm at the same time.
- Otherwise 0.

## Runtime and seeds

Expect about 5 minutes for a full 1,200-step Dream-1 trial on an L40S, including about 1 minute for Isaac Sim to start. The layout is
fixed (`-E object_jitter=0`), so there are no seeds to choose.

## Fidelity

The parts are rigid boxes at real size. The buckle is 54 mm wide at the female's mouth and 14 mm thick. Each flexing
plastic prong is modelled as a rigid prong on a sprung slider across the buckle. The spring is 1500 N/m with 3.5 mm of
travel, so pressing a tab flush takes 4.5 N. The prongs are 10 g each for solver stability, so the male half (36 g) is
heavier than a real one (about 10 g). The webbing tails are stiff flat strips. The tongue runs between two ribs with
0.75 mm play. The ramps on the tabs' leading faces that let a real buckle snap shut are left off, since the task starts
buckled. Plastic friction is 0.4. Light angular damping is added to the parts. No contact force cap is enforced in
simulation.

## Real-world build

Use any 38 mm (1.5 inch) side-release buckle on two 12 cm lengths of 38 mm nylon webbing. Lay it buckled and flat on a
marked outline. The trial is safe. To measure the gap, use a ruler on the table or a top-down photo with a scale.

## Assets

All geometry is defined in `task.py` from boxes; there are no external assets.
