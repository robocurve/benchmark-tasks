# Take a round turn on a bollard

A black mooring bollard (50 mm post, 6 cm tall, 70 mm cap on top) stands on the table's midline between the two
arms. An orange 8 mm line, 68 cm long, is fixed in a low deck eye in front of the bollard and to its right and lies
straight on the table towards the left, passing just in front of the post, ending in a black whipped end
(35 x 16 x 12 mm) on the far left. On the right stands
an open grey tray. The robot must lead the line's end at least one full turn around the bollard post, under the cap
(a round turn: one complete turn of the line around the post), lay the end in the tray and let go with
both grippers. Neither arm can do it alone: the line's end starts on the left, out of the right arm's reach (about
58 cm left of the midline), and the tray is out of the left arm's reach (its reach ends 2.3-6 cm short of the
tray's inner walls), so the end must be handed over; going round the bollard naturally splits the same way.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the line simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Take a round turn with the orange line on the black bollard: lead the line's black end all the way around the bollard post, under its cap, so the line goes at least one full turn around it, then lay the end in the grey tray on the right and let go of the line with both grippers."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop (reads the live line state): the left arm pinches the whipped end, brings it to the left side of the
bollard and leads it clockwise round the far side to the right side (a 7 cm circle about the axis, end ~2 cm above
the table), where it sets it down and moves clear. The right arm picks the end up where it lies, first hauls it out
to the right (so the slack goes to the tail side of the half turn: slack cannot be pulled back through a turn
later, friction round the post holds it), then leads it on round the near side (over the standing part), the left
side and the far side again (about 1.3 turns in all), then carries it out to the tray, lays it in, lets go and moves clear. While leading, each wrist camera looks along the
direction of travel or at the bollard. Success holds 1 s and ends the episode at step 582 (~29 s); task_score 1.0.
`demo/` holds the videos, `setup.png` and `result.json`.

Shortcut variants (`ROUND_CHEAT=...`, each scores exactly 0):
- `nothing`: no motion.
- `half`: the end goes only round the far side (half a turn), is handed over and laid in the tray.
- `boxonly`: the end is carried round the near side of the deck eye, away from the bollard, handed over and laid in
  the tray (no turn).
- `hold`: the full round turn, end in the tray, but the right gripper keeps holding it.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- Winding: following the line from the eye to its end, the signed angle steps about the bollard axis (top view) are
  summed over the parts of the line whose centre line is within 3 cm of the post surface (gap from the post to the
  line surface) and below the underside of the cap; its absolute value is the winding.
- **0** if either gripper's TCP (centre between the finger pads) is within 3.5 cm of the line or its end, if no part
  of the line below the cap is within 1 cm of the post, or if the winding is below 330 deg (a full turn with 30 deg
  tolerance; half wraps and no turn score 0).
- Otherwise **0.5**, or **1.0** if the whipped end's centre is inside the tray's inner walls (x 33-43 cm, y -14 to
  -22 cm) below the 2 cm wall tops.
- Real trials: 0 if the bollard, eye or tray is moved.
- `distance()`: missing winding (turns) plus the end's distance to the tray centre (m).
- `progress()` (analysis only, not the score): end grasped, line touching the post, winding >= 90/180/270 deg,
  full turn (>= 330 deg), end in tray, released. `VARIATION`: seeded start offsets for the line (analysis runs).

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). One trial takes ~5-15 min wall time on one GPU including the ~1-2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the line is never jittered (it always starts straight), so every trial
  starts identically.
