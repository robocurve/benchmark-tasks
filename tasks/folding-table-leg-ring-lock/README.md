# Open a folding table's leg and slide its brace ring over the knee before the table is turned over

Instruction given to the policy: `Open the folding table's leg so the table is ready to be turned over onto its legs, then let go.`

Files: `task.json`, `folding_table_leg_ring_lock.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/folding-table-leg-ring-lock/folding_table_leg_ring_lock.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm, every waypoint at x 0.12-0.40 m and at most 185 mm above
the bench, the arms' reach there). Expected output:

```
scoring: 17 cases pass (strips 112.2 mm; knee bent 142.5 deg folded, 4 deg on the stop; locked from ring travel 72 mm of 85)
left: 43.8 s plan, worst IK 0.086 mm / 0.012 deg, TCP at most 173 mm above the bench
right: 43.8 s plan, worst IK 0.086 mm / 0.013 deg, TCP at most 184 mm above the bench
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/folding-table-leg-ring-lock/folding_table_leg_ring_lock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the folding table's leg so the table is ready to be turned over onto its legs, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/folding-table-leg-ring-lock-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/folding-table-leg-ring-lock/folding_table_leg_ring_lock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the folding table's leg so the table is ready to be turned over onto its legs, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/folding-table-leg-ring-lock-demo
```

The demo is `demo()` in the task file: open loop and deterministic, both arms, 44 s of plan (about DEMO_STEPS steps
including the success hold), inside the run limit of 1,200 steps. The leg starts folded flat, where the brace is at a
dead point (pushing its knee only presses the leg onto the fold stop), so the left hand grips the left tube 10 cm from
the hinge and swings the leg up to 33 deg from upright (past that the tube would run up between the fingers into the
palm); the right hand comes in from the side onto the brace's lower strip just under the knee, the left lets go, and
the right pushes the brace straight, which stands the leg on its stop; then it slides the ring over the knee.

Forces (real world, from the geometry; the sim arm stays inside them): lifting the folded leg 10 cm from its hinge
takes about 23 N (0.6 kg leg, centre of mass about 30 cm up, 0.5 N*m hinge friction); pushing the brace at 33 deg
takes about 19 N, falling to under 4 N near upright; sliding the ring takes about 1 N.

## Videos and screenshot

```bash
for log in ~/yam-eval/folding-table-leg-ring-lock-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/folding-table-leg-ring-lock-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Slide the ring back down to its bracket, fold the leg flat onto its rests (the brace knee folds up), and retract the load pad to its rear stop.

## Response to review

- **TK-DHFSMB (importance):** the category is Societal relatability, and the context explains opening and locking a
  folding table as an everyday job: at home and in school, church and community halls, often by volunteers.
- **TK-FDZ899 ("Table leg should start folded"):** the leg now starts fully folded, flat against the upturned top,
  as tables are stored (its fold stop at 90 deg from upright, the tubes over two moulded rests). The robot must swing
  it up out of the folded position onto its upright stop, which straightens the brace, and then slide the lock ring
  over the knee. Changes that come with it:
  - The leg folds away from the robot (a leg folding toward it would lie across the arm bases), so the scene is
    mirrored: hinge at x 14 cm, the top modelled from x 4 cm to the bench's far edge, the side-load pad now pushes the
    right tube from the robot's side, and its rod runs past the table's side to a column on the bench.
  - Folded flat, the leg's clevis needs room under it: the hinge axis is 2.7 cm above the top, and the brace's lower
    bracket is 20 cm from the hinge (strips 11.2 cm pin to pin), so the folded brace's V clears the ring. The cross
    bar is slotted at the middle and the clevis is two cheeks, so the upper strip passes through the bar when folded.
  - The scoring rules, thresholds, load test and possible scores are unchanged; the self-test adds a leg lifted half
    way and dropped back onto its fold stop (score 0).
