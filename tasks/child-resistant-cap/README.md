# Take the cap off a child-resistant medicine bottle

An amber pharmacy vial stands on the table with its child-resistant cap on. The instruction is: **"Take the cap off
the medicine bottle."**

The cap is the common push-and-turn kind. It only unscrews while it is pressed down: pressed and turned anticlockwise
by half a turn, it comes free. Turned without pressing, its outer shell just spins and nothing unscrews. Its top
says so, printed "PUSH DOWN & TURN" around an anticlockwise arrow. The vial stands free on the table, so it also turns
or tips if pushed carelessly; holding it with the other arm, as people do, is the reliable way. To open it, the robot
has to:
- know or read how the cap opens;
- keep the vial still, for example by holding it with one arm;
- press the cap down and keep pressing while it turns it;
- lift the cap off once it is free.

Most prescription medicines come in child-resistant packaging (in the US it is required under the Poison Prevention
Packaging Act of 1970), and many older people with stiff or painful hands struggle to open it. A robot
that helps at home has to open these bottles. Opening them has been studied in robot learning (Edmonds et al.,
"Feeling the force", IROS 2017), but not, as far as we know, set as a benchmark task. Doing nothing, or turning the
cap without pressing it, scores 0.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the vial, the monitor (which turns pressing and turning into unscrewing, and locks and frees the cap), `success()` and `score()` |
| `geometry.py` | Dimensions, the vial's position, and the mechanism's numbers |
| `build_assets.py` | Writes the textures and `assets/pill_bottle.usda` (needs `pxr`, PIL and the DejaVu fonts) |
| `assets/` | The vial and its cap (one articulation), and their textures |
| `scenekit.py` | USD bodies, articulations with per-joint drives, and reset hooks for yam_arena task files |
| `demokit.py` | Helpers for the scripted demo |
| `demo.py` | The scripted full-score demo, and a variant that turns without pressing |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. Tested with yam-isaac-lab-arena at `bd62771`
(Isaac Lab-Arena 0.3.1).

## Run a trial

From this folder, in a shell with the yam_arena environment active. New shells on the walkthrough's Brev instance
have it; elsewhere, `source <ARENA_DIR>/yam.env`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the cap off the medicine bottle." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cap-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- **Seeds:** there is one fixed layout and no randomisation (`object_jitter=0`), so the seed has no effect.
- **Runtime:** a full 2,400-step Dream-1 trial took 3 min 25 s on an RTX 5090 (mean inference latency 0.61 s),
  plus about a minute to start Isaac Sim. A successful trial ends earlier: the scripted demo stops after 274 steps.

Dream-1 scored 0: both arms went to the vial again and again. Once the right arm pushed the cap past the 2 mm that
unlocks it (2.9 mm), but it turned the cap only about 3 degrees while pressing, so nothing came loose. The vial stayed
standing. The run used one continuous Dream-1 session (50 requests, no retries).

## Scripted demo

```bash
python demo.py task.py demo_out               # full score: writes demo_*_cam.mp4, setup.png, result.json
python demo.py task.py nopress_out --no-press  # turns the cap the same way without pressing it: 0
```

1. **Hold.** The left arm comes in from the front and grips the vial 3 cm up its body.
2. **Press and turn.** The right arm grips the cap from above and pushes 10 mm down, so the cap bottoms out at its
   3 mm travel. Still pushing, it turns the cap anticlockwise until it comes free. The wrist turns up to 200 degrees
   in one go, which is enough.
3. **Lift.** The right arm lifts the cap off and holds it.

The demo succeeds (1.0) in 274 steps (14 s): the cap comes free within the first 200-degree turn. The variant
grips the cap and turns it 200 degrees without pressing: the cap spins, nothing unscrews, lifting it does nothing,
and it scores 0.

## Scoring

Judged from the video and, at the end, a photo and one hand test.

| Score | When |
|---|---|
| 1.0 (success) | The cap is off the vial (held, or put down anywhere), no pills have spilled, and the vial was never knocked over. Held for 2 s, which ends the trial; a cap taken off in the last 2 s of the run limit also scores 1.0 |
| 0.5 | The cap is still on the neck but loose: at the end, lifted straight up by hand without pressing or turning, it comes off. The vial was never knocked over |
| 0 | Anything else. This includes doing nothing, turning the cap without pressing it however far, a cap screwed back on, and knocking the vial over at any time (tilted more than 45 degrees, whether or not it is stood up again) |

- **Knocked over** means tilted more than 45 degrees at any moment; a vial lifted or held upright is not knocked over.
- **In simulation** the hand test is the cap's state: it is loose once it has turned half a turn anticlockwise while
  pushed down more than 2 mm of its 3 mm travel, counted relative to the vial, in one go or several. Turning it
  clockwise, pressed or not, screws it back on, as on a real cap. It is off once lifted 1.5 cm up its axis.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left. Heights are above
the table top.

- **Vial.** An amber pharmacy vial, 4.6 cm across, with an 8.8 cm body and 10.6 cm tall with its cap. It stands free
  (not taped) with its centre at (34, 0).
  - It holds pills: about 80 g with them, 90 g with its cap, so it rattles but doesn't move by itself.
  - A white pharmacy label runs round its front: PHARMACY and Rx 0427 in a blue band, TAKE 1 TABLET BY MOUTH DAILY,
    and KEEP OUT OF REACH OF CHILDREN in an orange band.
- **Cap.** The vial's own push-and-turn child-resistant cap: white, 5.1 cm across and 2.2 cm tall, with a ribbed side.
  Its top is printed PUSH DOWN & TURN round an anticlockwise arrow, as these caps usually are.
  - It presses down about 3 mm and comes free after about half a turn while pressed.
  - Real caps differ in how hard they must be pressed, and are likely stiffer than the simulation's spring (2.4 N at
    the bottom of the travel). Choose one the robot can press, and measure it.
- **Arms** start at the ready pose with open grippers.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control.
- **Vial:** one floating articulation, standing free on the table, so it slides, turns and tips like a loose bottle.
  The vial with its pills weighs 80 g and the cap 8 g (plus 8 g of joint links). The pills are a mass, not simulated.
  Vial and cap have a friction of 0.5 static and 0.4 dynamic.
- **Cap:** on three joints on the vial: a press (3 mm down, against an 800 N/m spring), a spin (free about its axis)
  and a lift (up its axis).
- **The lock** is task code, not a modelled mechanism. After every control step the monitor reads the press and the
  spin. While the cap sits on the thread, its anticlockwise turn relative to the vial counts toward unscrewing only
  while it is pressed past 2 mm, and its clockwise turn always counts back. Pressed, the spin drags (0.05 N m s/rad),
  which pulls a loose vial round; not pressed, it spins almost freely (0.0005 N m s/rad). The lift unlocks once half
  a turn has been counted, and locks again if the cap is turned back.
- **Off:** the freed cap lifts straight up its axis (up to 10 cm), not off in any direction. Once lifted 1.5 cm (its
  edge 6 mm above the neck) it counts as off for good: it can't be carried away or set down, and if let go it slides
  back down to 1.5 cm and stays there. A real cap let go above the neck could land back on it and score 0.5; in
  simulation it stays off.
- **Knocked over** is checked after every control step and latched, so turning the vial over to tip the cap off
  scores 0.
- **Resets:** before each reset the monitor puts the cap back down and locks it, and the reset stands the vial back
  on its mark, so trials can run back to back in one process.

## Reset between trials

Put the cap back on and turn it clockwise until it is tight (it tightens without pressing). Check that it doesn't
open without pressing, and stand the vial on its mark.
