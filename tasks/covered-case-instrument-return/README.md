# Covered-Case Instrument Return

Return one 40 mm instrument module to the empty one of three covered cases. The two other cases already contain identical modules. The left arm moves each lid by its central handle; the right arm carries the incoming module and positions its wrist camera to inspect the opened case. A wrong placement into an occupied case scores zero. The fixed setup and exact numerical scoring rule are in `task.json`.

The task uses the calibrated YAM station (`SCENE="station"`) with two stock arms and the stock top and wrist cameras. Its 75 g removable lids are separate USD rigid bodies with a panel, central handle and underside rim groove. The case shells and 80 g modules are simple rigid cuboids. The meshes and contact colliders were authored for this task; no third-party textures or logos are used.

From an environment with `inspect-robots`, `yam_arena`, Isaac Lab-Arena and Isaac Sim installed, run a trial with the official loader:

```bash
inspect-robots run --embodiment yam_arena \
  -E task=tasks/covered-case-instrument-return/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Three opaque cases stand side by side in one row. Use the left arm to grasp each lid's central handle, lift it and slide it horizontally to the left of that case, clear of the right wrist camera's view. The right arm holds the incoming module and uses its wrist camera to check each opened case. If occupied, move the right arm away and replace that lid before checking another case. Case inspection order is unrestricted. Place the module only in a visually confirmed empty case, move the right arm clear, and replace its lid. Keep stored modules undisturbed and leave all three lids seated on their matching cases." \
  --policy yam_scripted --scorer task_score --max-steps 2400 --seed 42 \
  --no-prompt --grader none --log-dir covered-case-run
```

The fixed layout uses no random object displacement, so `--seed 42` is recorded for reproducibility. The 2,400-step limit is 120 seconds of simulation at 20 Hz. On a DGX Spark, the recorded demo took 365 seconds wall time and completed with score 1.0 after 1,633 steps; the recorded Dream-1 beta trial took 456 seconds wall time and reached the 2,400-step limit with score 0.0. `demo()` supplies a scripted bimanual plan. Its motions are open loop and do not prove that a policy can infer occupancy from images. To reproduce the Dream-1 trial, use the same options, instruction, scorer, seed and limit with `--policy dream` in place of `yam_scripted` and a Dream-1 beta credential in the client’s local credential store.

Partial credit is based on opening one case, opening any two distinct cases, and a released correct placement. Full credit requires the incoming module and both stored modules supported, all three matching lids seated and released, and one second of stable placement. The scorer uses end score and latches wrong occupied-case entry and the other violations listed in `task.json`. Physics advances at 240 Hz; scoring samples after each 20 Hz control step, so a brief forbidden crossing between samples can be missed. The model is a simplified rigid-body analogue; it has no real-hardware calibration or contact-force validation.
