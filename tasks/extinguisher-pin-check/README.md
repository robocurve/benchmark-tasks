# Fire-extinguisher check: pull the retaining pin, read the gauge, file the report

A YAM task in Isaac Lab-Arena: a red extinguisher stands on the station table with a steel retaining
pin in a rail bracket above its body and a pressure-gauge dial on its front. Pull the pin out of its
bracket, read the gauge (needle in the green zone or out of range), and file the report by leaving a
white tag on the matching pad - green (in the green) or red (out of range).

- **Task file:** `extinguisher_pin_check.py` (fixture, two gauge states, report pads, scoring,
  scripted demo — one file)
- **Instruction:** "pull the extinguisher's retaining pin, check the gauge and file the report"
- **Scene:** `station` (Dream's calibrated YAM station)
- **Per-trial gauge state:** the `FAULT_MODE` environment variable — `charged` (needle in the green
  zone) or `undercharged` (needle in the red zone). The state is not announced to the policy.

## Run and score a trial

```bash
# scripted demo (reaches full score in both modes; 900-step cap)
FAULT_MODE=undercharged inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=extinguisher_pin_check.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "pull the extinguisher's retaining pin, check the gauge and file the report" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/ext-demo

# Dream-1 run (900 steps = the task's run limit)
FAULT_MODE=charged inspect-robots run --embodiment yam_arena --policy dream \
  -E task=extinguisher_pin_check.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "pull the extinguisher's retaining pin, check the gauge and file the report" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/ext-dream
```

`task_score`: 0.5 for the pin pulled at least 2 cm out of its seat and at rest, +0.5 for the tag on
the pad matching the gauge's actual state, 1.0 for the complete check, 0.0 for do-nothing. Run both
modes for a full picture (`FAULT_MODE=charged`, `undercharged`).

## Runtime and seeds

- Scripted demo: ~17 s of simulated time (about 350 control steps at 20 Hz) plus the tag filing;
  a few minutes wall-clock including Isaac Sim warmup.
- Dream-1: up to 900 control steps (45 seconds at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout for its mode; the
  scripted demo is deterministic waypoints, and the mode it plays is the mode being scored.

## Why this task

Fire-extinguisher checks are a per-unit inspection staple with two distinct physical checks: a
manipulation (the pin is a tamper seal that must be pulled, not observed) and an analog reading
(the gauge). The finding must be filed physically — a tag on the matching pad — so a policy cannot
score by photographing the scene alone; guessing the gauge scores 0.5 in expectation.
