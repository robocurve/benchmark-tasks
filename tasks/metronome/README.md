# Start a metronome at 120 beats per minute

A mechanical metronome stands on the table, its tempo scale facing the arms. Its weight is on 60 and its pendulum
is still. The instruction is: **"Start the metronome at 120 beats per minute."**

To do it, the robot has to:
- read the scale (Maelzel's marks, 40 to 208, read at the weight's top edge);
- slide the weight down the rod to the 120 mark, where it clicks into a notch;
- swing the pendulum out and let go, so that it ticks on its own.

The higher the weight, the slower the pendulum swings, and the marks are not evenly spaced: the 120 mark has the
116 mark 2.4 mm above it and the 126 mark 3.6 mm below it. A pendulum let go from too small a swing stops, because
the escapement does not catch. Starting the metronome without moving the weight makes it tick at 60, which scores 0.

Metronomes set the tempo for music practice and teaching, and the mechanical pyramid kind is still common. The task
asks for two things robots rarely have to do together:
- **Set a slider on a fine, uneven scale.** The marks are a few millimetres apart, so the weight has to be placed
  to within about a millimetre and read where its top meets the scale. On a real metronome the marks fall where its
  maker put them, so they have to be read, not remembered.
- **Set a dynamic system going and leave it alone.** The pendulum needs a release that is big enough, and then
  nothing may touch it.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the metronome, the monitor (escapement, the weight's notches, the knocked-over check), `success()` and `score()` |
| `geometry.py` | Dimensions, the metronome's position, the pendulum's physics and where each mark goes |
| `build_assets.py` | Writes the scale texture and `assets/metronome.usda` (needs `pxr`, PIL and the DejaVu fonts) |
| `assets/` | The metronome (one articulation) and its scale |
| `scenekit.py` | USD bodies, articulations with per-joint drives, and reset hooks for yam_arena task files |
| `demokit.py` | Helpers for the scripted demo |
| `demo.py` | The scripted full-score demo, and a variant that starts it without moving the weight |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. Tested with yam-isaac-lab-arena at `bd62771`
(Isaac Lab-Arena 0.3.1).

## Run a trial

From this folder, in a shell with the yam_arena environment active. New shells on the walkthrough's Brev instance
have it; elsewhere, `source <ARENA_DIR>/yam.env`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Start the metronome at 120 beats per minute." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/met-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- **Seeds:** there is one fixed layout and no randomisation (`object_jitter=0`), so the seed has no effect.
- **Runtime:** a full 2,400-step Dream-1 trial took 3 min 9 s on an RTX 5090 (mean inference latency 0.57 s),
  plus about a minute to start Isaac Sim. A successful trial ends earlier: the scripted demo stops after 409 steps.

Dream-1 scored 0. Its right arm reached toward the metronome's side several times and drew back, but neither arm
touched it: the weight stayed on 60 and the pendulum never moved. The run used one continuous Dream-1 session (50
requests, no retries).

## Scripted demo

```bash
python demo.py task.py demo_out              # full score: writes demo_*_cam.mp4, setup.png, result.json
python demo.py task.py noset_out --no-set    # starts it the same way without moving the weight: 0
```

1. **Set the tempo.** The right arm grips the weight from the front, fingers either side, and pushes it down the
   rod. It reads the weight's height from the simulator's joint state and corrects until the weight is on the 120
   mark, then lets go and backs off.
2. **Start it.** The left arm takes the rod above the weight between its fingers, swings it out to 25 degrees along
   its arc, and backs straight off the rod's front. The pendulum swings on its own and the escapement keeps it going.
3. Both arms go back to the ready pose, clear of the swing.

The demo succeeds (1.0) in 409 steps (20 s): the metronome ticks at 120.0 beats a minute, and the trial ends
once it has ticked on its own for 5 s. The variant starts it the same way without moving the weight: it ticks at 60
(59.6 measured) and scores 0.

## Scoring

Judged from the video and a photo at the end.

| Score | When |
|---|---|
| 1.0 (success) | The weight is on the 120 mark (the mark nearest its top edge), and the metronome, standing on the table, has ticked on its own for the last 5 s with nothing touching it. It was never knocked over. This ends the trial |
| 0.5 | The weight is on 120 but the metronome has not ticked on its own for the last 5 s: never started, stopped, held, or touched |
| 0.5 | The metronome has ticked on its own for the last 5 s with the weight one mark off: on 116 or 126 |
| 0 | Anything else. This includes doing nothing, starting it without moving the weight (it ticks at 60), the weight two or more marks off, and knocking the metronome over at any time (tilted more than 45 degrees, whether or not it is stood up again) |

- **On a mark** means the mark nearest the weight's top edge. The weight clicks into a notch at each mark, so it
  sits on one.
- **Reading it.** The weight's back stands 1.6 mm off the scale, and its top is dark, so from the arms' cameras, which
  look down on it, the mark is where the dark top meets the scale (its front edge looks a few marks lower).
- **In the real world**, a photo at the end, taken square to the scale with the camera level with the weight's top
  edge, shows the mark. The video shows whether it ticked on its own for the last 5 s, with both arms clear
  and the metronome standing on the table. Counting ticks (one per swing each way) checks the tempo if the photo is
  unclear: 60 in 30 s at 120, 58 at 116 and 63 at 126.
- **In simulation**, "on its own" means the escapement has been running for 5 s, the swing's energy has not been
  disturbed (by more than 3 degrees of amplitude) and the case has stood on the table (base less than 5 mm up,
  tilted less than 10 degrees). A hand holding or pushing the pendulum changes its energy, and a pendulum held still
  misses its next beat, so a metronome driven or stopped by hand does not count.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left. Heights are above
the table top.

- **Metronome.** A mechanical pyramid metronome with a Maelzel scale (40 to 208) and a sliding weight, wound up,
  for example a Wittner Maelzel series 800 (22.5 cm). The simulated one is a walnut case 22.5 cm tall with its
  cap, on an 11.8 x 11.8 cm plinth, about 600 g with its pendulum. It stands free (not taped) on a tape outline of
  its base, the centre of its base at (34, 0), its scale facing the arms.
  - The pendulum hangs on a pivot 8.5 cm up, 0.7 cm in front of the scale: a steel rod 4 mm wide, reaching 13 cm
    above the pivot. Its brass weight (2 x 1 x 1.3 cm, 25 g) slides on the rod, its back 1.6 mm off the scale, and
    clicks into a notch at each mark. Its top is dark; on a real one, put a strip of black tape on it.
  - The cream-coloured scale (5 x 11.6 cm) carries Maelzel's 39 marks, 40 at the top and 208 at the bottom, with Largo,
    Andante, Allegro and Presto beside them. The weight's top edge reads it, as on most such metronomes. On the
    simulated one the 120 mark is 7.5 cm above the pivot; on a real one the marks fall where its maker put them. If
    a real weight has no click stops, it is judged on the mark nearest its top edge.
- **Start:** the weight on 60, the pendulum upright and still (not latched).
- **Arms** start at the ready pose with open grippers.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control.
- **Metronome:** one floating articulation standing free on the table, so it slides and tips like a loose one. The
  case is the root (500 g), with the pendulum (80.5 g) and the weight (25 g) on two joints. Its friction (0.6
  static, 0.5 dynamic) is averaged with the table's.
- **Pendulum:** a compound pendulum on a revolute joint. A 72.5 g bob 5 cm below the pivot (mass only, hidden in the
  case) and the 8 g rod hang against the 25 g weight above the pivot, so the period follows the weight's height:
  T = 2π √(I / (g m d)) for the pendulum's inertia I and the mass moment m d below the pivot. Each mark is placed
  where this gives its tempo at a 25-degree swing (`geometry.mark_height`). Measured in simulation, it ticks at 120.0
  on the 120 mark and 59.3 to 59.6 on 60.
- **Escapement:** task code. Each time the pendulum swings through the middle with enough swing, it ticks and
  nudges the swing toward 25 degrees with a kick of energy (at most 5 degrees' worth per beat), so the period stays
  the pendulum's own. To start, the swing through the middle must be at least 10 degrees; a gentler push dies away.
  A beat more than 30% late (held, or the swing died) stops it. While it is stopped, the pivot brakes the swing
  (4 x 10⁻⁴ N m s/rad).
- **Notches:** the weight's prismatic joint has a drive (400 N/m) whose target is the current mark; once the weight
  is pushed past half way to the next mark (0.3 mm of hysteresis), the target moves to that mark, so it clicks. The
  clip gives at about 0.5 N, softer than most real ones.
- **Knocked over** is checked after every control step and latched.
- **Resets:** before each reset the monitor stops the pendulum and puts the weight back on 60, and the reset stands
  the metronome back on its mark, so trials can run back to back in one process.

## Reset between trials

Stop the pendulum and leave it upright and still. Slide the weight back up to 60. Give the key a few turns if it has
run down, and stand the metronome on its tape outline, scale facing the arms.
