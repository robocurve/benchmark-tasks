# Set the header stand under the cutter bar before working underneath

**Instruction:** "The header is raised. Set the yellow stand under the cutter bar so it sits on the ground and the saddle is under the bar."

NIOSH and OSHA fatality investigations (including NIOSH FACE 14MI118 and 01MN054, and OSHA case 68282.015) describe workers crushed under a raised combine header that was not blocked. Case-IH and other OEM manuals require a header stand or equivalent blocking before anyone works under a raised header. The OEM cylinder-rod safety collar is a tight sleeve and is not used here. A known failure is to work under the header with the stand still on the ground, or to set the stand short of the cutter bar so it does not take the load. Here a yellow stand starts on a mark in front of a raised, bench-scale combine header. The robot has to set the stand on the ground with its saddle under the cutter bar, then let go. Leaving it on the start mark, or setting it short of the bar, is not enough.

## Run a trial

On a machine with the official simulator, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The header is raised. Set the yellow stand under the cutter bar so it sits on the ground and the saddle is under the bar." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/header-dream

inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The header is raised. Set the yellow stand under the cutter bar so it sits on the ground and the saddle is under the bar." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/header-demo

# Controls: TASK_DEMO=naive, TASK_DEMO=short, TASK_DEMO=idle
TASK_DEMO=naive inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The header is raised. Set the yellow stand under the cutter bar so it sits on the ground and the saddle is under the bar." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/header-naive

for log in ~/yam-eval/header-dream/*.json; do inspect-robots video "$log"; done
```

Runtime is 600 steps (30 s at 20 Hz). Scripted demos use seed 0; Dream-1 flash uses seed 1. Cameras are 320 × 176. Variant is selected with `TASK_DEMO`.

`test_task.py` (offline, no simulator) checks the scoring table (0 / 0.25 / 0.5 / 1.0, including the 1 s hold) and the task.json limits.

## Scoring

From the overhead video, the stand is seated when its base sits on the table and the saddle is under the cutter bar (error under 15 mm in x, 20 mm in y and 10 mm in height, tilt under 12 degrees). At the end, after 1 s at rest with the left gripper clear of the stand (at least 16 cm from the stand origin and at least 15.5 cm above the table): 1.0 if seated and clear; 0.5 if it is seated but the arm is still holding it; 0.25 if it is close but not seated (20-55 mm short of the bar in x, still on the approach side, error under 30 mm in y and 15 mm in height, tilt under 18 degrees); 0 if it is left on the start mark or falls over short of the bar. Possible scores: 0, 0.25, 0.5, 1.0.

| End state (held 1 s, at rest, left arm clear) | Score |
|---|---|
| Seated, let go, still | 1.0 |
| Seated, still held or not yet still | 0.5 |
| Close but not seated | 0.25 |
| Left on the start mark, or well short | 0 |

## Parts and real-world counterparts

| Sim part | Real part |
|---|---|
| Yellow header stand, 10 cm tall, 6 cm base, cantilevered fork, +x handle | A small implement / header stand |
| Steel cutter bar, underside 14 cm up, 16 cm along y, yellow stripe and pointed guards | Combine cutter-bar section |
| Yellow slatted reel, green side sheets, darker auger trough, two support posts | Bench-scale combine header |
| 8 mm start pedestal at (40, 22) cm | Start mark |

## Costed list

USD, 2026 retail, approximate. No special PPE is required for a real trial of this place-and-release job.

| item | cost |
|---|---|
| Small implement / header stand (shop or OEM-style) | $40–60 |
| Bench-scale cutter-bar / header-section mock-up (reel, sides, trough) | $25–35 |
| Tape, marks, and a short pedestal | $5 |
| Total (2026 retail, approximate) | under $100 |

## Scores

Scripted (yam_arena, 320x176, seed 0; Brev L40S, probe v3vis): demo 1.0; naive 0.0; short 0.0 (tipped on an air drop, did not luck into a seat); idle 0.0. Dream-1 flash (seed 1, 22 Nov 2026): 0.0 at 600 steps (24 requests / 24 attempts, not a refusal).

## Fidelity

The stand and the header are rigid and start on marks. The header is bench-scale, with a yellow slatted reel, green side sheets and an auger trough so it reads as a combine header rather than a box. A designed gap under the bar lets the fork be carried in. This is not the OEM cylinder-rod safety collar (a sleeve-in-bore). Full credit is the stand at rest on the table, saddle under the bar, left arm clear.

## Policy camera

policy_view.png in the task folder shows the scene in the top camera at the policy resolution (320 × 176), with the yellow stand, the reel, the cutter bar and the green side sheets readable in the middle of the table. The left wrist mostly shows the handled part; the parked right wrist looks at empty table.

## Reach

Straight-down yaw-0 pinch on the left (ipsilateral) side, fixture about mid-table as in the setup field.
