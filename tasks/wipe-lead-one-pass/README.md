# Wipe a monitor lead from the plug end to the clip end in one pass

A free grey patient-monitor lead (6 mm PVC, 60 cm) lies straight across the table, its grey plug in front of the
left arm and its red patient clip in front of the right arm. A blue band 14-16 cm from the plug and a red band
54-56 cm from the plug mark the stretch to disinfect. The robot holds the lead by the plug with one hand, closes
the other hand lightly around the lead at the blue band, and wipes along it toward the clip in one continuous
pass: the closed hand slides along the lead and never goes back toward the plug, so the wipe never goes back over
cleaned lead. After reaching the red band it lays the whole lead on a clean white drape and lets go.

The tested ability is continuous sliding contact along a deformable object in one direction. The lead is free, so a
closed hand moving along it just drags it across the table unless the other hand holds the plug: both arms are
needed at the same time (one anchors, one wipes). The wiping hand has to stay closed on the thin lead while
following its live shape, and must not let go and hop forward (hops leave unwiped gaps) or scrub back and forth.

## How this differs from similar tasks

Other tasks wipe a rigid surface or a connector end face with a sponge or pad, slide a rigid part along a rail or
shaft, or regrasp a cable hand over hand. Here the gripper itself is the wipe and the deformable object is the
track: the score depends on which stretch of the lead passed through the closed pinch and in which direction, while
the other hand anchors the lead.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Disinfect the grey monitor lead: hold it by the plug, close the other gripper around the lead at the blue band and wipe along it in one pass toward the clip end, never moving back toward the plug. Wipe all the way to the red band, then lay the whole lead on the white drape and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed-loop (reads the live lead and gripper state): the left arm grips the plug and lifts it 2.5 cm; the right
arm closes lightly around the lead at the blue band (finger target 5 mm apart on the 6 mm lead, so it pinches
without clamping) with its pad tips just above the table. The right arm then steps 1.5 cm at a time toward the
clip, following the live lead sideways and measuring where along the lead its pinch is; the lead slides through
the pads because the left arm holds the plug. It stops once the pinch passes the red band, then both arms carry
the lead to the drape, lay it down, open and move clear. Both wrist cameras look along the lead toward the clip
(the wiping hand sees the stretch still to wipe). Success holds for 1 s and ends the episode at step 366
(18.3 s); task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- A gripper is **closed on the lead** when its finger gap is at most 7 mm (lead diameter + 1 mm) and part of the
  lead lies between its pads. The lead's 1 cm marks from the blue band (15 cm from the plug end of the cord) to the
  red band (55 cm), 41 marks, count as **wiped** when the pinch point of a gripper closed on the lead slides over them
  (within 0.5 cm) while the other gripper holds the plug (finger gap at most 19 mm, plug centre within 2.5 cm of the
  gripper). Only a continuous closed slide counts; opening and regrasping further on covers nothing.
- **0** if, at any time, a gripper closed on the lead goes more than 3 cm back toward the plug from the furthest it
  has reached, measured either along the lead (a backward slide or a regrasp behind the wipe) or as straight-line
  distance from the gripper to the plug (a closed stroke backward, which on a free lead pushes the lead along). Lay
  the lead down without folding it: the drape is long enough for that.
- **0** if the plug is not on the drape at the end, or a gripper is within 3 cm of the lead, plug or clip at the
  end (not released).
- Otherwise, with c = fraction of marks wiped: **0.3** = slid from the blue band to between 35 and 47 cm (c 0.5 to under 0.8), plug and lead laid as required; **0.6** = slid to between 47 and 53 cm (c 0.8 to under 0.95), or all the way to the red band with part of the lead or the clip off the drape; **1.0** = slid to 53 cm or beyond (c >= 0.95) and the whole lead, plug and clip lie on the drape.
- How c is seen: c is not a pose in the final photo. It comes from the path of the slide, so it is read at the end as
  marks clean under a UV torch, and in the video as how far the closed gripper slid from the blue band: c = 0.5 is
  about 35 cm from the plug end, c = 0.8 about 47 cm, c = 0.95 about 53 cm.
- `success()` additionally requires the lead to be at rest. `distance()`: 1 - c (1.0 once the backward rule fired).
- Real trial: dust the stretch with UV tracer powder; a mark counts as wiped if it is clean under a UV torch and the
  video shows the plug held during the wipe; backward slides and the final layout are read from the video and a
  photo of the drape.

## Shortcut variants (test only, `WIPE_CHEAT=<name> python cabledemo.py task.py out`), all score exactly 0

| variant | what it does | why it scores 0 |
|---|---|---|
| `none` | does nothing | plug not on the drape |
| `hold` | full wipe and lay, keeps gripping | not released |
| `hover` | travels along the lead with open fingers, then lays it | gap too wide, c = 0 |
| `onearm` | wipes with no hand on the plug | the whole lead is dragged along, c = 0, plug not on the drape |
| `hops` | opens, moves on and re-closes in hops without sliding (`WIPE_HOP` = 0.10 or 0.025 m) | only the small slips while closed count: c = 0.17 (10 cm hops), 0.44 (2.5 cm hops), both below 0.5 |
| `scrub` | closed stroke 8 cm back and forward again mid-wipe | backward rule (distance to plug) |
| `back` | opens, regrasps 8 cm behind, wipes on | backward rule (along the lead) |

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). One trial takes ~5-10 min wall time on one GPU including the ~1-2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so every trial
  starts from the same state.
