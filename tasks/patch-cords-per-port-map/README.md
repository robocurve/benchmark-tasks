# Patch cords per the port-map card

A network patch panel with six numbered ports (1-6, robot's left to right, ports facing the robot at table level)
stands across the far side of the table. A small switch in the middle of the table has three 6 mm patch cords (blue,
yellow, white) whose plugs lie free on the table: blue on the robot's left, yellow and white on its right. A printed
port map card in front of the switch reads **BLUE 2 / YELLOW 5 / WHITE 3**. The robot must read the card and the port
labels and push each plug fully into its mapped port, leaving every other port empty, then let go.

The layout is a derangement: the port nearest each plug (blue → 1, yellow and white → 6) is never its mapped port, and
the map is not in order (1, 2, 3). A wrong patch breaks or misroutes a live link, so any plug in a wrong port zeroes the score:
position-only or in-order patching cannot earn credit. The task is bimanual: the left arm serves ports 1-3 and the
right arm ports 4-6, and the white plug starts on the right but belongs in port 3, so it is handed over (set down in
the middle of the table by the right arm and taken by the left arm).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cabledemo.py` and `boxfont.py` (block-letter labels) are in this folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Patch the three cords from the switch into the patch panel exactly as the port map card says: push each plug fully into the numbered port listed for its colour. Leave all other ports empty, then let go of the plugs."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed loop on the live plug poses. Plugs are carried pinched at their middle just above the table and turned to
point at the panel; in front of the port the plug is set down flat, squared up, re-pinched 10 mm from its cord end,
lined up with the port and pushed in until its front end is 23.5 mm deep. Sequence: the right arm sets the white plug
down at the hand-over spot while the left arm takes blue aside; then blue → 2 (left) and yellow → 5 (right) at the
same time; then the left arm takes white from the hand-over spot → 3.
task_score 1.0 at 935 steps (~47 s; `demo/` holds its videos, `setup.png` and `result.json`).

Shortcut variants (`PATCH_DEMO=<mode> python cabledemo.py task.py out`), all score exactly 0:
- `nothing`: no motion.
- `nearest`: each plug into the free port nearest to it (blue 1, yellow 6, white 5). In the recorded check blue was
  seated in port 1 and the yellow/white insertions at the panel's right end jammed; the wrong blue patch alone zeroes it.
- `inorder`: ports in card order (blue 1, white 3; yellow untouched): white correctly seated, blue in a wrong port.
- `hold`: blue and yellow patched correctly, but the right arm keeps holding the yellow plug.

## Scoring (`score()`, at the end; `success()` must hold 1 s)

- A plug is IN a port when its front end (tip) is at least 5 mm past the port face inside that port's opening.
- 0 if any plug is in a port other than its mapped one, or a gripper TCP is within 3 cm of any plug centre.
- Otherwise 1/3 per plug whose tip is at least 15 mm inside its mapped port (blue 2, yellow 5, white 3).
- `distance()`: sum over plugs of the tip's distance to 15 mm inside its mapped port.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes ~5-15 min wall time including Isaac Sim start-up.
- Layout fixed with `object_jitter=0`; the cords are never jittered (they always start straight). The card always
  reads BLUE 2 / YELLOW 5 / WHITE 3.
