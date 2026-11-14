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
scoring: 20 cases pass (strips 112.2 mm; knee bent 142.5 deg folded, 4 deg on the stop; locked from ring travel 72 mm of 85)
left: 55.1 s plan, worst IK 0.089 mm / 0.012 deg, TCP at most 173 mm above the bench
right: 55.1 s plan, worst IK 0.088 mm / 0.013 deg, TCP at most 184 mm above the bench
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/folding-table-leg-ring-lock/folding_table_leg_ring_lock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the folding table's leg so the table is ready to be turned over onto its legs, then let go." \
  --scorer task_score --max-steps 1400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/folding-table-leg-ring-lock-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/folding-table-leg-ring-lock/folding_table_leg_ring_lock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the folding table's leg so the table is ready to be turned over onto its legs, then let go." \
  --scorer task_score --max-steps 1400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/folding-table-leg-ring-lock-demo
```

The demo is `demo()` in the task file: open loop and deterministic, both arms, 55 s of plan (about 1,010 steps
including the side load and the success hold), inside the run limit of 1,400 steps. The leg starts folded flat, where
the brace is at a dead point (pushing its knee only presses the leg onto the fold stop), so:

1. The left hand grips the left tube 10 cm from the hinge (the fingers on the tube's sides, so the tube turns between
   the pads) and swings the leg up to about 36 deg from upright; past about 30 deg the tube would run up between the
   fingers into the palm.
2. The right hand comes in along the brace's plane from beyond the knee, fingers open, grips the brace's upper strip
   8 cm from the leg's clevis pin, takes the leg's weight, and the left lets go.
3. The right swings the strip along its own path (from the four-bar's geometry); that straightens the brace and stands
   the leg on its stop. It lets go and pushes the lower strip's outer edge just under the knee (the knee push) to
   press the leg onto its stop with the brace straight.
4. It grips the ring by its bracket, slides it up the strip over the knee onto its stop, lets go and parks; the side
   load then runs (it counts only with the leg open and locked and both hands 5 cm clear).

The sim arms are position servos of finite stiffness (shoulder 80, wrist 10 N*m/rad): a 15-25 N load sags a hand by
4-10 cm, so wherever a hand lifts or pushes, the plan aims it past the path by the sag that load causes (the arm's
compliance from its joint gains). The hand's real path stays at x 0.14-0.37 m and at most 185 mm above the bench,
checked offline (the aims are only targets of the servos).

Forces (real world, from the geometry; the sim arm stays inside them): lifting the folded leg 10 cm from its hinge
takes about 23 N (0.6 kg leg, centre of mass about 30 cm up, 0.5 N*m hinge friction); holding it at 36 deg by the
upper strip takes about 12 N and swinging it up about 15-18 N, falling to under 5 N near upright; the closing push
is 6 N; sliding the ring takes about 1 N.

## Videos and screenshot

```bash
for log in ~/yam-eval/folding-table-leg-ring-lock-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/folding-table-leg-ring-lock-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,400 steps (70 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Slide the ring back down to its bracket, fold the leg flat onto its rests (the brace knee folds up), and retract the load pad to its rear stop.

## Response to review

- **TK-DHFSMB (importance: "not shown to be a regular core duty of dining room attendants or hand laborers"):** the
  category is Societal relatability, and the context explains opening and locking a folding table as an everyday job
  people know first-hand: at home for a meal or party, and in school, church and community halls, often by volunteers.
- **TK-FDZ899 ("Table leg should start folded"):** the leg now starts fully folded, flat against the upturned top, as
  tables are stored: 90 deg from upright on its fold stop, the tubes over two moulded rests. The robot must swing it
  up 90 deg onto its upright stop, which straightens the brace, and then slide the lock ring over the knee. In the demo
  run the leg goes from 90 deg to -0.04 deg (on its stop), the brace from 142.5 deg bent to 1.0 deg, the ring from 32
  to 115 mm up the strip (locked from 104 mm), and the side load then folds the leg only 0.11 deg (fail above 3 deg).
  Dream-1 was run on this folded start (one trial, 1,400 steps): it did not open the leg, and scored 0.
  What changed with it:
  - The leg folds away from the robot (a 70 cm leg folding toward it would lie across the arm bases), so the scene is
    mirrored: hinge at x 14 cm, the top modelled from x 4 cm to the bench's far edge (87 cm of it), and the side-load
    pad now pushes the right tube from the robot's side, its rod running past the table's side to a column.
  - Folded flat, the leg's clevis needs room under it, so the hinge axis is 2.7 cm above the top (was 1.6 cm).
  - The brace's lower bracket moved from 17 to 20 cm from the hinge, so the strips are 11.2 cm pin to pin (were
    10.0 cm). With the bracket at 17 cm, the folded brace closes to a narrow V and its upper strip crushes the lock
    ring (6.8 mm of overlap at 81 deg), so the leg could not fold flat; at 20 cm every colliding pair keeps at least
    0.7 mm clear over the whole fold.
  - The cross bar is slotted at the middle and the clevis is two cheeks, so the upper strip passes through the bar
    when folded instead of through solid metal.
  - The side load counts only if it began with the leg open and locked and both grippers stayed 5 cm clear while it
    ran; otherwise it runs again the next time the conditions hold (a robot that let go too early and then held the
    leg through the push could otherwise lock the ring afterwards and score). Thresholds and possible scores are
    unchanged; the self-test has 20 cases.
  - The knee has a stop that keeps the brace at least 3.5 deg bent, as the setup states (4 deg short of straight on
    the stop). Without it, 0.13 mm of give at the pins let the closing push take the brace to 1 deg, where an
    unlocked brace no longer folded under the load (control run: 0.11 deg); with it, the same control (leg opened,
    ring left) folds 3.6 deg within 0.5 s of the load and the leg falls flat: score 0.
  - The kit's slider read its travel from world-frame motion, which drifted once its parent (the lower strip, which
    now turns about 60 deg) rotated and dragged the ring; it now reads travel in the parent's frame.
