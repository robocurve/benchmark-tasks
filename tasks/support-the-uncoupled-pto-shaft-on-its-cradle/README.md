# Support the uncoupled PTO shaft on its cradle

**Instruction:** "The PTO shaft is uncoupled and on the ground. Set it in the yellow cradle so it sits off the ground, then let go."

HSE Agriculture Information Sheet 40 says that when a machine is not in use, the operator should support the shaft and guard on the cradle provided, and should not rest PTO shafts on drawbars or drop them on the ground. The guard's restraining chain should not be used to support the uncoupled shaft. Weasler, the ADMA driveline safety manual, and ANSI/ASABE AD5673-1 say the same. A shaft left on the ground or hanging from the chain damages the guard that prevents PTO entanglement deaths. Here a short PTO shaft starts on a mark in front of a fixed U-cradle on an implement-tongue stub. The robot has to set the shaft in the cradle so it sits off the table, then let go. Leaving it on the table, or dropping it short of the cradle, is not enough.

## Run a trial

On a machine with the official simulator, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The PTO shaft is uncoupled and on the ground. Set it in the yellow cradle so it sits off the ground, then let go." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/pto-dream

inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The PTO shaft is uncoupled and on the ground. Set it in the yellow cradle so it sits off the ground, then let go." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/pto-demo

# Controls: TASK_DEMO=naive, TASK_DEMO=short, TASK_DEMO=idle
TASK_DEMO=naive inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The PTO shaft is uncoupled and on the ground. Set it in the yellow cradle so it sits off the ground, then let go." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/pto-naive

for log in ~/yam-eval/pto-dream/*.json; do inspect-robots video "$log"; done
```

Runtime is 600 steps (30 s at 20 Hz). Scripted demos use seed 0; Dream-1 flash uses seed 1. Cameras are 320 × 176. Variant is selected with `TASK_DEMO`.

`test_task.py` (offline, no simulator) checks the scoring table (0 / 0.25 / 0.5 / 1.0, including the 1 s hold) and the task.json limits.

## Scoring

From the overhead video, the shaft is seated when it sits in the U-cradle (error under 18 mm in x, 25 mm in y and 10 mm in height, tilt under 15 degrees). At the end, after 1 s at rest with the left gripper clear of the shaft (at least 16 cm from the shaft origin and at least 15.5 cm above the table): 1.0 if seated and clear; 0.5 if it is seated but the arm is still holding it; 0.25 if it is close but not seated (error under 35 mm in x and y and 20 mm in height, tilt under 25 degrees); 0 if it is left on the start mark or dropped short of the cradle. Possible scores: 0, 0.25, 0.5, 1.0.

| End state (held 1 s, at rest, left arm clear) | Score |
|---|---|
| Seated, let go, still | 1.0 |
| Seated, still held or not yet still | 0.5 |
| Close but not seated | 0.25 |
| Left on the start mark, or well short | 0 |

## Parts and real-world counterparts

| Sim part | Real part |
|---|---|
| Yellow guard-tube section, 4 x 14 x 4 cm, dark yokes, top grab bar | Short PTO shaft / guard |
| Yellow U-cradle, floor 3 cm up, inner 7 cm, walls 4 cm | Implement PTO support cradle |
| Short green implement-tongue stub with a gray PIC-style end | Implement tongue (stub) |
| 8 mm start pedestal at (41, 22) cm | Start mark |

## Costed list

USD, 2026 retail, approximate. No special PPE is required for a real trial of this place-and-release job.

| item | cost |
|---|---|
| Implement PTO support cradle | $20–40 |
| Short PTO guard-tube section with yokes (scrap / display length) | $20–40 |
| Short implement-tongue stub | $10–15 |
| Total (2026 retail, approximate) | under $100 |

## Scores

Scripted (yam_arena, 320x176, seed 0; Brev L40S, probe v1): demo 1.0; naive 0.0; short 0.0 (tipped 90 degrees on a 50 mm-short drop, did not luck into the U); idle 0.0. Dream-1 flash (seed 1, 22 Nov 2026): 0.0 at 600 steps (24 requests / 32 attempts, not a refusal).

## Fidelity

The shaft and the cradle are rigid and start on marks. The shaft is a short bench-scale guard-tube section with dark yokes, not a full-length PTO shaft (a real shaft will not fit the envelope). The tongue is a stub, not a full implement. This is a place into an open U-cradle, not a sleeve over a U-joint and not a retry of the failed PTO driveline-shield probe. Full credit is the shaft at rest in the cradle, off the table, left arm clear.

## Policy camera

policy_view.png in the task folder shows the scene in the top camera at the policy resolution (320 × 176), with the yellow shaft, the U-cradle and the tongue stub readable in the middle of the table. The left wrist mostly shows the handled part; the parked right wrist looks at empty table.

## Reach

Straight-down yaw-0 pinch on the left (ipsilateral) side, fixture about mid-table as in the setup field.
