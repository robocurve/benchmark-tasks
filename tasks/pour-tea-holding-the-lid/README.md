# Pour the tea with one hand on the lid

A teapot of tea stands on a trivet a little left of the middle of the table, its loose lid resting on top; an empty
cup stands to its right. The instruction is only: "Pour a cup of tea from the teapot into the cup, then put the
teapot back on its trivet."

Tea-service guidance says to hold the handle with one hand and keep the lid on with the other while pouring
([English Tea Store](https://blog.englishteastore.com/2014/05/09/a-guide-to-pouring-tea-british-style/): hold the
handle with one hand while gently securing the lid with the other; [Some Tea With
Me](https://someteawith.me/2015/05/19/how-to-pour-a-teapot/): the simplest way is two hands, one on the lid and one
on the handle; [Teasenz](https://www.teasenz.com/chinese-tea/hold-pour-yixing-teapot.html), on Yixing teapots: in the
two-hand method one hand raises the pot while the other's index finger rests on the lid). The lid here is loose, with
no locking tab, and the pot must be tilted 45 degrees or more to pour. The instruction does not mention the lid, so
the task checks whether the robot keeps it on unprompted:

- **A lid rule, not only a goal.** The lid must stay seated on the pot at every step the pot is tilted 10 degrees or
  more. One step with it off scores 0, even if the pour and the return are perfect.
- **Strictly bimanual.** In the recorded `one_arm` variant (the same pour, nobody holding the lid) the lid slid
  16.99 cm off its seat and ended tilted 30.45 degrees to the pot. The pour itself counted 129 steps over the cup and
  the pot went back on its trivet, but the score is 0.
- **A side-handle pot.** The handle sticks out toward the robot at a right angle to the spout, as on a Japanese
  kyusu, so one arm pours by turning the pot about the handle's own axis while the other holds the lid by its knob.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files (a visual-only part for the tea), and per-step readings: finger-pad points and part poses |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure and cheating variants (`democore.py` is shared with our other kitchen tasks) |
| `demo_results.json` | Each variant's score and end-state measurements, from the runs reported below |
| `demo.mp4` (on the task board) | The recorded full-score scripted demo |
| `setup_shot.py` | Writes `setup.png`, the top camera's view of the starting layout |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |

The demo video, the Dream-1 run video and the setup screenshot are uploaded to the task board with the repository's `tools/upload_media.py`; `task.json` names them by media ID.

`kitchenkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pour a cup of tea from the teapot into the cup, then put the teapot back on its trivet." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/tea-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The episode
ends once `success()` has held for 1 s.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Runtime:** the recorded full scripted demo ran 786 steps (39.3 s of simulated time at 20 Hz); its wall-clock time
was not recorded. A model trial runs up to 2,400 steps, plus the policy's own response time. Measured on Nov 15: the Dream-1 run (`dreamlabs/dream-1-beta`, 2,400 steps, `rows_per_request=12`) took 6.6 min of wall-clock time on one NVIDIA RTX PRO 6000 Blackwell GPU, simulator start-up included.

## Scripted demo

```bash
python demo.py task.py demo_out                         # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_one_arm --variant one_arm    # any variant below; --no-record skips the video
python demo.py task.py --plan-only                      # inverse-kinematics check of every variant's plan
DEMO_DEBUG=10 python demo.py task.py demo_out           # also print the pot's state every 10 steps
```

The left arm comes level along the side handle, pinches it top and bottom, lifts the pot off the trivet and holds it
beside the cup; the right arm pinches the lid's knob. Together they tilt the pot about the knob, 10 degrees a second,
to 55 degrees, so the spout goes down over the cup, hold the pour for 3.5 s and tilt it back. The right arm then lets
go of the knob and goes home, and the left arm sets the pot back on its trivet, lets go and backs off. Held by its
side handle, the pot hangs nose-down in the pinch, so the left hand's target is corrected toward the pot's planned
pose; while the right hand holds the knob, its target follows the knob's place on the pot as the pot actually is. In
the recorded run the episode ended at step 786 of the 840-step plan, once `success()` had held for 1 s, so the last
54 planned steps were not run.

| Variant | What it does | Score |
|---|---|---|
| `full` | the left arm pours by the handle while the right arm holds the lid by its knob; the pot goes back on its trivet | **1.0** (ended on success at step 786; 130 pouring steps over the cup; at the end the lid is 2.8 mm from its seat and the pot is home, released and still) |
| `one_arm` | the left arm pours alone; nobody holds the lid | 0.0 (lid lost: at the end it is 16.99 cm off its seat, 6.01 cm below it and tilted 30.45 degrees to the pot; the pour counted 129 steps over the cup and the pot is home) |
| `miss_cup` | both arms pour as in `full`, but the whole pour is shifted 6 cm to the right (-y), so the spout is not over the cup | 0.0 (spilled; 0 pouring steps over the cup; the lid stayed seated and the pot is home) |
| `short` | as `full`, but the pot is tilted only to 50 degrees and held there 1 s | 0.25 (41 pouring steps, under the 60 needed for 0.5; the lid stayed seated and the pot is home) |
| `no_return` | as `full`, but the pot is left held upright beside the cup | 0.5 (130 pouring steps; at the end the pot is not home, not released and not still, hanging 10.44 degrees from upright in the hand) |

The numbers are from `demo_results.json`.

## Scoring

Every step, from poses; the score at the end counts, and the episode ends once `success()` has held for 1 s.

- **Pouring step:** the pot tilted at least 45 degrees from upright, with its spout tip within 2.5 cm (horizontally)
  of the cup's axis and 1-8 cm above the cup's rim. Pouring steps are counted in total, not necessarily in a row.
- **Lid seated:** relative to the pot, the lid is within 8 mm sideways and 5 mm up or down of its seat on the rim,
  and tilted no more than 8 degrees. It must be seated at every step the pot is tilted 10 degrees or more, and at
  the end for 1.0. While the pot is upright a brief nudge is not penalised: a hand closing on the knob of the upright
  pot moved the lid 8 mm for a moment, and it settled back.
- **1.0:** at least 60 pouring steps (3 s at 20 Hz), the lid seated, and the pot back on its trivet (its base within
  6 mm of the trivet's top, within 10 degrees of upright, its centre within 3 cm of the trivet's centre), released
  and still (moving less than 2 mm over 1 s).
- **0.5:** at least 60 pouring steps, but not all of the end conditions for 1.0 met (pot back on the trivet,
  released, still, lid seated).
- **0.25:** 20 to 59 pouring steps (1 to 3 s).
- **0:** otherwise, and from the moment the lid is off its seat while the pot is tilted 10 degrees or more, the pot
  is tilted 45 degrees or more with the spout not over the cup for 10 steps in a row (0.5 s: pouring onto the
  table), or the pot is dropped (it falls 5 cm within 0.5 s with no finger pad within 2 cm of its body or handle).

**Released** means no finger pad within 8 mm of the pot's body (a 10 x 23 x 8 cm box from the pot's base to its rim,
around the body and the spout) or of its 6 x 2 x 1.6 cm side handle, so a hand still closed on the handle counts as
holding the pot: in `no_return`, which ends with the left hand on the handle, the end state reads `released: false`.

`report()` records the lid's offset from its seat (shift, lift, tilt), the pot's tilt, the spout tip's horizontal
offset from the cup's axis and its height above the rim, the pouring-step count, and the over-cup, spilled,
lid-lost, drop, home, released and still flags.

## Real-world trials

Use room-temperature water in place of hot tea, a side-handle teapot whose loose lid has no locking tab, and a cup.
Stand the pot on a trivet with its handle toward the robot and its spout toward the cup, and mark both outlines on
the table. Draw a short line across the lid's edge and the pot's rim: the lid check fails if the two halves of the
line move more than 8 mm apart, or the lid lifts 5 mm or tilts 8 degrees, at any time the pot is tilted 10 degrees
or more. Score the pour from a side camera (the pot tilted 45 degrees or more with the spout tip 1-8 cm above the
cup's rim and within 2.5 cm of its centre line, for 3 s in total); the water itself shows any pour onto the table.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Context.** Tea-service guidance says to hold the teapot's handle with one hand and keep the lid on with the other while pouring: English Tea Store (A Guide to Pouring Tea British Style: hold the handle with one hand while gently securing the lid with the other), Some Tea With Me (How to pour a teapot: the simplest way is two hands, one on the lid and one on the handle, keeping the lid on top) and Teasenz (2 Ways to Pour with an Yixing Teapot: in the two-hand method one hand raises the pot and the other's index finger rests on the lid). Here the lid is loose, with no locking tab, and pouring needs the pot tilted 45 degrees or more. In the recorded one-arm run (the same pour, nobody holding the lid) the lid slid 17 cm off its seat and ended tilted 30 degrees, scoring 0, so one arm tilts the pot by its side handle while the other holds the lid by its knob. The instruction does not mention the lid, so the task checks whether the robot keeps it on unprompted. Done by: restaurant, cafe and hotel servers, tea-house staff, and anyone serving tea at home.

**Simulation fidelity.** Rigid bodies: boxes, with contact forces and friction from PhysX; the lid is a separate loose part on the pot's rim with no locking tab, so it stays on only by friction or a hand. Liquid is not simulated: the tea is a visual block inside the pot that does not collide, and the pot's mass does not change as it is emptied. The scorer stands in for the tea with the spout's pose: the pot tilted 45 degrees or more with the spout tip within 2.5 cm of the cup's axis and 1 to 8 cm above its rim counts as tea going into the cup, and 0.5 s tilted that far with the spout anywhere else counts as tea poured onto the table. A lid that leaves its seat while the pot is tilted 10 degrees or more scores 0 wherever it lands, which stands in for a lid falling into the cup or onto the table. The cup and the trivet are fixed to the table.
