# Roll a cabinet shelf wider than its face-frame opening through it, level it inside, and lower it onto its four pins

A kitchen wall cabinet's shelf is being put back after the cabinet was moved. The shelf is wider than the cabinet's opening: the face frame's stiles narrow the 343 mm inside to a 305 mm opening, and the shelf is 340 mm wide. The robot rolls it about 35 degrees to pass the opening, pushes it in, levels it about its own centre with 1.5 mm to spare at each side, and lowers it onto its four pins together.

**What this task adds.** Shelving tasks on open shelves slide a board straight on. Here the opening is narrower than the board, so it goes in rolled and is levelled inside a space only 3 mm wider than itself. Then all four corners come down onto 5 mm pins together, square and back: a shelf pushed in short, or levelled with one end caught under its pins, falls past them when let go.

**How the demo holds it.** Two pinches on the shelf's front edge, 18 cm apart, fingers pointing into the cabinet. The shelf's 0.98 kg sits 13 cm behind the front edge, about 1.3 N m in all, which the two pinches share. Under that load the wrists let the back edge droop several centimetres, so the demo carries the shelf 9 cm above the pins and levels it from its pose before lowering it.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
  --instruction "Put the shelf in the cabinet on its four pins."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                         # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode short,notback,tilted --hold 1     # failure checks, below
python check_score.py                                                              # the scoring table on hand-built records, no simulator
```

The demo is a scripted reference: it reads the shelf's pose from the simulator, and a policy gets only the cameras. The shelf starts on two foam blocks 9 cm tall in front of the cabinet, as it might be set down while the cabinet is moved: the blocks leave its front edge free for the fingers, and the arms cannot reach a board lying flat on the bench that close to their bases. Both arms come in level from the front, pinch the shelf's front edge, lift it, roll it 35 degrees, push it in through the opening until its back edge is 6 mm off the back panel, level it about its centre from its pose, lower it until all four pins carry it, let go and withdraw.

**Result** (scripted, GPU physics on an RTX 5090, 20 Hz): 1.0. On all four pins (gaps 0 to 0.17 mm), level within 0.03 degrees, square, its back edge 7 mm off the back panel. 391 steps (19.6 s of simulated time), about 2 minutes of wall time. No seed changes the scene.

| Failure check | What it does | Score |
|---|---|---|
| `short` | pushed in 10 cm short, then levelled and lowered | 0: there it could not be levelled inside the stiles and stayed rolled about 30 degrees; let go, it fell past the pin row, its centre 13 cm above the table |
| `notback` | pushed in 2.5 cm short | 0: levelling caught its right end under the right pins (21 mm low, rolled 3.6 degrees); let go, it fell twisted onto the left pins and the floor |
| `tilted` | lowered with its left end 4 degrees high | 1.0: it settles level onto all four pins; a 4 degree tilt does not miss a pin in this cabinet |

## Building it for a real trial

- **Cabinet:** a stock 15 in (381 mm) wide, 12 in (305 mm) deep face-frame wall cabinet, 18 in (457 mm) tall to stay within the arms' reach, fixed to the table with its opening to the robot. Its 19 mm frame's 38 mm stiles and rails leave a 305 mm wide, 381 mm tall opening; inside it is 343 mm wide.
- **Shelf:** 16 mm melamine particleboard, 340 x 265 mm, about 0.98 kg.
- **Pins:** four 5 mm brass shelf pins, 6 mm proud, 37 mm from the inside's front and back, 20 cm above the table.
- **Start:** the shelf on two foam blocks (200 x 60 mm, 90 mm tall, fixed to the table at y +15.5 and -15.5 cm, centred at x 34.8 cm) in front of the cabinet, its front edge at x 18.5 cm.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Inside width, opening width | 343 mm, 305 mm | A tape across the inside and across the stiles |
| Shelf size and mass | 340 x 265 x 16 mm, 0.98 kg | A tape and a kitchen scale |
| Friction, shelf on pins and sides | 0.5 | A tilt test: the angle at which the shelf starts to slide on a pin |

## Safety for a real trial

The shelf is light. Keep people clear of the arms' reach while the robot runs and keep an emergency stop in reach.

## Scoring

See `scoring` in `task.json`, and `score`, `measure` and `Tracker` in `task.py`. The scorer reads the shelf's pose against the four pins' tops: each pin counts as carrying the shelf when the shelf's underside is within 1 mm above it.

## Assets

All geometry is built in code from boxes, and 12-sided cylinders (convex hulls) for the pins. No third-party assets.
