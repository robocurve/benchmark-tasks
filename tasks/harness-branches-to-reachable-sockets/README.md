# Connect harness branches to the sockets they can reach

An automotive / EV wire-harness assembly step. A black round breakout clamp is fixed in front of the robot, a little
left of centre; three identical grey 7 mm branches leave it through grommets (each pinned at its exit point, free to
swing sideways) and are 20, 33 and 46 cm long, each ending in an identical dark-grey plug body. Three identical black
socket modules stand 21 cm (ahead), 34 cm (ahead-right) and 47 cm (right) from the clamp, each with one socket cavity
facing the clamp. Nothing is labelled: the robot must see that a shorter branch cannot reach a farther module, so
the only way to connect all three is short -> near, middle -> middle, long -> far module. The starting layout is a
derangement: the short branch points straight at the middle module but its plug stops about 5 cm short of the
opening, the middle branch's plug lies nearest the far module (out of its reach) and the long branch's plug lies
nearest the middle module. "Plug each plug into the socket nearest to where it lies" puts the long plug into the
middle module and scores 0; "long branch into the closest module first" also scores 0.

The task is bimanual: the near module is only reachable by the left arm and the middle/far modules and the
long/middle plugs only by the right arm. In the demo the left arm connects the short branch while the right arm
routes the long branch around to the far module at the same time, then the right arm connects the middle branch.

Instruction: `Connect the wiring harness: plug each of the three grey branches into one of the three module sockets so that all three are plugged in, then let go.`

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated cables) and `cabledemo.py` (scripted-demo runner) are in this folder next to `task.py`.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Connect the wiring harness: plug each of the three grey branches into one of the three module sockets so that all three are plugged in, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live plug poses. Each plug is pinched 12 mm from its branch end, lifted just clear of the table and
turned so the gripper (and its wrist camera) faces along the insertion direction; the plug is lined up 15 mm in front
of the socket opening, its measured offset in the grip corrected, pushed in to about 21 mm, released, and the arm
backs off. If a plug stalls at the opening the arm backs off, re-centres it and pushes again; if it stays turned in
the grip it is let go and pinched again squarely along the socket axis. Step 1 (both arms at once): left arm short branch -> near module, right arm long branch -> far module
(swung round past the middle branch). Step 2: right arm middle branch -> middle module while the left arm parks.
`demo/` holds the videos, `setup.png` and `result.json`: success holds for 1 s and ends the episode at step 714
(~36 s); task_score 1.0.

Shortcut variants (test only, `HARNESS_DEMO=<name> python cabledemo.py task.py out`), each scoring exactly 0:

| variant | what it does | score |
|---|---|---|
| `nothing` | does nothing | 0 |
| `nearest` | each plug to the socket nearest where it lies, as far as it reaches: short plug -> near module (right; its nearest socket, the middle one, is out of reach), then long plug -> middle module (wrong) | 0 (long plug 19.5 mm into the middle module) |
| `greedy` | long branch into the closest (near) module first: left arm swings the short branch aside, right arm sets the long plug down where the left arm takes it, left arm loops the spare length out and plugs it in; then middle branch -> middle module | 0 (long plug 17.5 mm into the near module) |

## Scoring (`score()`, at the end; `success()` = score 1.0 with all branches at rest, held 1 s)

Depth = how far a plug body reaches into a socket past its opening, measured along the socket axis; a plug whose
centre is more than 8 mm sideways off the socket axis, or lifted above the cavity, is not in that socket.

- **0** if at the end a gripper's TCP is within 3.5 cm of any plug centre (still holding).
- **0** if any plug reaches 5 mm or more into a socket that is not its branch's match (short -> near, middle ->
  middle, long -> far).
- Otherwise **1/3 per branch** whose plug reaches at least 15 mm into its matching socket; 1.0 when all three are in.

Real-world measurement: a tape mark on each plug body 15 mm and 5 mm from its tip, checked by eye or photo after the
grippers have moved away.

## Runtime and seeds

Run limit 1200 steps (60 s at 20 Hz). A trial takes a few minutes wall-clock on one GPU (first Isaac Sim
start adds 1–2 min). The layout is fixed (`object_jitter=0`; modules and clamp are fixed, branches always start
straight from their exits), so every seed starts identically.
