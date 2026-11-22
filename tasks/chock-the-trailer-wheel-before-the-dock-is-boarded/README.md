# Chock the trailer wheel before the dock is boarded: wedge on the ground, slope facing the tread

**Instruction:** "The wheel chock is out. Set the chock against the tire so it sits on the ground and the slope faces the tread."

OSHA 29 CFR 1910.178(k)(1): when a highway truck is boarded with a powered industrial truck, "the brakes of highway trucks shall be set and wheel chocks placed under the rear wheels to prevent the trucks from rolling." 1910.178(m)(7) restates the chock rule for trucks and trailers at the dock. This is a dock-and-yard precaution, not a car-jacking sequence. A known failure is to skip the chock, or to set it backwards or short of the tread, so the trailer can roll when a pallet truck is driven on. Here a yellow wedge starts on a mark beside a trailer-size tire at the back of a short trailer body, with a dock lip in view. The robot has to set the chock on the ground with its slope facing the tread and the tip against the contact patch, then let go. Leaving it on the start mark, or setting it short of the tire, is not enough.

## Run a trial

On a machine with the official simulator, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The wheel chock is out. Set the chock against the tire so it sits on the ground and the slope faces the tread." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/chock-dream

inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The wheel chock is out. Set the chock against the tire so it sits on the ground and the slope faces the tread." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/chock-demo

# Controls: TASK_DEMO=naive, TASK_DEMO=short, TASK_DEMO=idle
TASK_DEMO=naive inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The wheel chock is out. Set the chock against the tire so it sits on the ground and the slope faces the tread." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/chock-naive

for log in ~/yam-eval/chock-dream/*.json; do inspect-robots video "$log"; done
```

Runtime is 600 steps (30 s at 20 Hz). Scripted demos use seed 0; Dream-1 flash uses seed 1. Cameras are 320 × 176. Variant is selected with `TASK_DEMO`.

`test_task.py` (offline, no simulator) checks the scoring table (0 / 0.25 / 0.5 / 1.0, including the 1 s hold) and the task.json limits.

## Scoring

From the overhead video, the chock is seated when it sits on the table, the tip is against the tire's contact patch, and the slope faces the tread (the fat end faces away): lateral error under 20 mm, height error under 10 mm, tip-to-tire gap between -10 mm and 18 mm, tilt under 12 degrees, tip pointing at the tire. At the end, after 1 s at rest with the left gripper clear of the chock (at least 16 cm from the chock origin and at least 15.5 cm above the table): 1.0 if seated and clear; 0.5 if it is seated but the arm is still holding it; 0.25 if it is upright and on the table but 18-50 mm short of the tire (lateral error under 30 mm, height error under 15 mm, tilt under 18 degrees, tip still pointing at the tire); 0 if it is left on the start mark or set down well short of the tread. Possible scores: 0, 0.25, 0.5, 1.0.

| End state (held 1 s, at rest, left arm clear) | Score |
|---|---|
| Seated, let go, still | 1.0 |
| Seated, still held or not yet still | 0.5 |
| Close but not seated | 0.25 |
| Left on the start mark, or well short | 0 |

## Parts and real-world counterparts

| Sim part | Real part |
|---|---|
| Yellow wedge, 10 x 7 x 10 cm, small top handle, tip toward the tire | 4–6 in rubber wheel chock |
| Light-trailer tire, 20 cm OD, 8 cm wide, rubber / rim / hub, axis along y | Trailer wheel |
| Short red-and-white trailer bed, side, black fender | Cut-off trailer body (bench scale) |
| Dock lip with a yellow bumper at the near edge | Dock board lip |
| 8 mm start pedestal at (41, 22) cm | Start mark |

## Costed list

USD, 2026 retail, approximate. No special PPE is required for a real trial of this place-and-release job.

| item | cost |
|---|---|
| 4–6 in rubber wheel chock | $15–25 |
| Light-trailer tire or wheel section (~200 mm OD) | $30–50 |
| Short trailer-bed / fender offcut and a dock bumper | $10–15 |
| Total (2026 retail, approximate) | under $100 |

## Scores

Scripted (yam_arena, 320x176, seed 0; Brev L40S, probe v3d): demo 1.0; naive 0.0; short 0.25 (released 50 mm short of the tire); idle 0.0. Dream-1 flash (seed 1, 22 Nov 2026): 0.0 at 600 steps (24/33 attempts, not a refusal; chock never moved).

## Fidelity

The chock and tire are rigid and start on marks. The tire is a light-trailer size (~200 mm OD), not a 1 m drive tire, so it fits the reach envelope. A short trailer bed, side, fender and a dock lip with a yellow bumper are in the scene; they are cut off at bench scale, and the dock lip is easy to miss in the top camera. The chock is 70 mm wide for a yaw-0 pinch, with a small top bar. Full credit is the wedge at rest, tip against the tread, slope facing it, left arm clear.

## Policy camera

policy_view.png in the task folder shows the scene in the top camera at the policy resolution (320 × 176), with the yellow chock, its top handle, the tire (rubber, rim and hub), and the short trailer body readable in the middle of the table. The left wrist mostly shows the handled part; the parked right wrist looks at empty table.

## Reach

Straight-down yaw-0 pinch on the left (ipsilateral) side, fixture about mid-table as in the setup field.
