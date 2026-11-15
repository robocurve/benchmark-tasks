# Seat a switch line card with both ejector levers

The lowest slot of a modular data-center switch chassis is fixed to the table, its mouth facing the arms. A full-width
line card (400 mm wide, 460 mm deep, 4 kg) stands in the slot's guides, pulled out 60 mm, with both of its ejector
levers swung fully open. The robot must seat the card and lock it in.

The card cannot simply be pushed home. Over the last 8 mm its two connectors press onto the backplane's against 80 N,
more than both arms can push together. The ejector levers give the leverage. Each lever pivots on the faceplate, 192 mm
either side of its middle. Swung open, its claw points into the slot beside a steel pin standing in front of the
chassis face, so the card slides in past the pins. Swung shut, the claw comes round behind its pin and draws that end
of the card in. The lever's finger tab is 8.5 times as far from the pivot as the pin is when the lever is shut, so
about 5 N on the tab gives the 40 N at the pin. A lever swung to within 2.5 degrees of shut, with its end of the card
home, snaps into a spring catch on the faceplate that pulls it the rest of the way shut and holds it there against the
connectors.

What the task tests:

- **Leverage instead of force.** Pushed by hand, the card stops at the connectors, about 8 mm short; the levers have
  to finish the job.
- **Both levers together.** The card has only 1.5 mm of clearance each side in its 460 mm-deep guides. A lever closed
  on its own draws one end in and cocks the card; connectors driven in more than 1.5 mm unevenly bend backplane pins,
  and that ends the trial at 0.
- **Driving a lever along its arc.** Each finger tab swings through 85 degrees on a 110 mm radius while the card, and
  so the pivot, moves in by up to 25 mm, and the last few degrees take the most force.
- **Pushing a long card straight.** Pushed off-centre, the card turns in its guides and jams.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `cardkit.py` | Props for yam_arena task files built from USD text: fixed and free parts and mechanisms with sprung hinges and sliders (adapted from the kit used by other tasks in this repository; adds a free frame's inertia and centre of mass) |
| `demo.py` | The scripted full-score demo |
| `test_scoring.py` | Offline checks: every scoring outcome, the layout's geometry and the demo's reach (no GPU) |

`cardkit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with the
standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/line-card-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. A trial takes
about 4 minutes on an L40S, including 1 to 2 minutes for Isaac Sim to start. There is one fixed layout (no seeds;
`object_jitter=0`).

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1200               # writes demo_*_cam.mp4, setup.png, result.json
DEMO_DEBUG=1 python demo.py task.py debug_out --no-record      # also prints the end gaps, yaw, lever angles, connectors
python -m pytest test_scoring.py                               # offline checks
```

The demo runs open loop: it reads no simulator state. Where the card stops after the push (8 mm short, measured once)
is a constant, and the lever arcs follow from the published geometry.

1. Both arms, fingers shut, push the faceplate in 35 mm either side of its middle until the card stops at the
   connectors, then back off.
2. Each arm closes its fingers across its lever's finger tab, above the faceplate.
3. Together they swing both levers shut over 6 s, carrying the tabs round their arcs (which move in with the card) and
   on 10 degrees past shut, so the arms press the levers into their catches. The claws draw the card in square: the
   two end gaps never differ by more than 0.6 mm.
4. They let go and lift clear.

The demo reaches 1.0 (success) at step 421 (21 s).

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). "Home" is where the
card sits with its connectors fully mated: its faceplate 1 mm proud of the chassis face. Each end's gap is how far the
faceplate is short of home, measured 180 mm either side of its middle (a ruler or feeler gauge between the faceplate's
back and the chassis face, less the 1 mm).

- 0 if, at any point while both end gaps are 8 mm or less (the connectors mating), the two end gaps differ by more
  than 1.5 mm (bent backplane pins).
- 1.0 (success, which must hold for 1 s): both end gaps 1 mm or less, both levers within 3 degrees of shut (in their
  catches), and no gripper finger within 3 cm of either lever's finger tab.
- 0.75 if both end gaps were at some point 1 mm or less.
- 0.5 if both end gaps were at some point 8 mm or less.
- 0.25 if both end gaps were at some point 25 mm or less (in far enough for the levers to catch their pins).
- Otherwise 0.
