# Recover a crawler by its tether around a wall

A hazardous-zone mock-up. On the far side of the table, out of reach of both arms, an orange inspection crawler
(80 x 60 x 40 mm, 120 g, low-friction skids, black camera head on its front) sits directly behind a fixed grey wall
(20 x 2 x 6 cm) with a round end post. Its blue 6 mm tether (65 cm) leaves the crawler's rear and lies straight past
the wall's end post towards the robot's left, ending in a black handle. In the wall's lee, nearer the robot, a 15 x 15
cm recovery zone is taped in green. The robot must bring the crawler into the zone by its tether alone, as inspection
crawlers are pulled out of pipes and radioactive tunnels by their tethers when they get stuck or disabled. A direct pull towards the zone
drags the crawler into the wall's end post (the tether wraps around it), so the crawler must first be hauled out along
the clear lane past the wall end and then steered sideways around the end into the zone, without the grippers ever
touching it, without it striking the wall or post, and without it tipping. Both arms are needed: only the left arm
reaches the tether's free part and the hauling lane; only the right arm reaches the right side the final sideways
pull must come from, so the left arm lays the tether across to the right arm (a handoff through the table).

Instruction: `Recover the orange crawler into the green taped zone using only its blue tether: never touch the crawler, don't let it hit the grey wall or its post or tip over. Steer it around the wall's end, then let go of the tether.`

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated tether) is in this folder next to `task.py`; `cabledemo.py` runs the scripted demo.
  `task.py` welds the tether and the camera head to the crawler, gives the crawler low-friction skids, and turns on
  PhysX contact reporting between the robot links and the crawler.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Recover the orange crawler into the green taped zone using only its blue tether: never touch the crawler, don't let it hit the grey wall or its post or tip over. Steer it around the wall's end, then let go of the tether."
```

Run one trial per process (or one per embodiment reset): the scorer keeps a short history of the trial (tipping,
touching, wall strikes), which `task.py` clears on every embodiment reset.

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The left arm grips the tether 25 cm from the crawler and hauls it 25 cm straight in along the clear lane, regrips
10 cm from the crawler and hauls again until the crawler is past the wall's front face (beyond the end post), then
regrips and steers it to the right, laying the tether across the middle of the table. The right arm takes the tether
there and pulls the crawler sideways into the zone, closed loop on the crawler's position, then both arms let go and
move clear. Both wrist cameras look along the tether towards the crawler and the wall end while pulling.
It reaches `task_score` 1.0 in 593 steps (about 30 s) of the 1,200-step limit; videos are in `demo/`.

Shortcut variants (`CRAWLER_DEMO=...`), each scoring exactly 0: `nothing` (do nothing), `direct` (haul the tether
straight towards the zone: the crawler is dragged into the wall's end post), `straight` (haul straight in along the
tether's lane and stop: the crawler ends left of the wall, not around it), `push` (haul it in, then push the crawler
into the zone with the closed gripper).

## Scoring (`score()`, partial credit; `success()` = score 1.0 with the crawler at rest, held 1 s)

- **Disqualified (latched, score 0):** the crawler tilts more than 45 deg at any time; a gripper TCP comes within
  3 cm of the crawler body or any robot link touches it (PhysX contact, every physics substep); the crawler's
  footprint comes within 3 mm of the wall or its end post at any time.
- **Not released (score 0):** at the end a gripper TCP is within 3.5 cm of the tether or the handle.
- Otherwise **1.0** if all corners of the crawler's footprint are inside the taped square (inner tape edges);
  **0.5** if the crawler has come around the wall end (whole footprint nearer the robot than the wall's front face and
  its centre between the wall's two ends) but is not fully inside the square; else **0**.

Real-world measurement: a top photo for the zone, video for tipping, hand/gripper contact and wall strikes.

## Runtime and seeds

Run limit 1,200 steps (60 s at 20 Hz). A trial takes several minutes wall-clock on one GPU (first start of Isaac Sim
adds 1–2 min). The layout is fixed (`object_jitter=0`; the cable is never jittered), so every seed starts from the
same state.
