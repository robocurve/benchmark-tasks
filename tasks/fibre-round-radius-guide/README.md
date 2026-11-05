# Route a fibre patch cord round a radius guide without over-bending it

A yellow 3.0 mm fibre patch cord (44.5 cm, black SC-style connector on its free end) leaves a grey fibre
enclosure at the left of the table and lies straight across the table towards the right arm. On its far side it
passes 3 mm from a white round radius guide (76 mm diameter, 25 mm tall) and, 10–13 cm further on, 5 mm from a
dark grey sharp-edged steel bracket (30 mm cube). A white open-top connector holder lies beyond the guide, its open end 10 cm past the guide's centre,
on the line that leaves the guide at a right angle. The robot must turn the cord 90 degrees round the guide, seat the
connector in the holder and let go, and the cord must never be bent tighter than its 25 mm minimum bend radius at
any moment of the episode, not only at the end. Fibre cords lose signal or break when bent too tightly
(TIA-568-B.3 gave 25 mm for 2–4-fibre indoor horizontal cable with no pulling load, 50 mm under a 222 N pull); radius guides exist so they turn corners
safely.

The trap is geometric: swinging the connector straight over to the holder, low over the table, catches the cord
on the bracket's square corner, and the cord is far too short for the path round the bracket, so it is pulled
taut round the sharp edge (radius under 25 mm, score 0). The cord has to be lifted over the bracket and wound onto
the guide. Because the constraint is checked at every control step, a cord that is kinked and straightened again
also scores 0.

The natural solution is bimanual: one arm pins the cord's first leg to the table just before the guide, so the
other can wind the free part onto the guide without dragging the whole cord along.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Route the yellow fibre patch cord round the white radius guide and seat its connector in the white holder beyond the guide, then let go. Never bend the cord tighter than the guide's curve (its minimum bend radius is 25 mm) at any moment, and leave it lying against the guide."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed loop (reads the live cord). The left arm presses the cord's first leg to the table 6 cm before the guide
with closed fingertips. The right arm pinches the connector, lifts it 8 cm and carries it along the involute of the
guide (the free length is measured from the live cord and corrected for the lift), high while the cord sweeps over
the bracket, then down near the table, so the cord winds onto the guide and stays straight between guide and
connector. At the end it heads away from the robot with its wrist camera facing the holder, lowers the connector
into the slot and lets go; then the left arm lifts off. Success at step ~260 (~13 s); task_score 1.0. The
smallest bend radius during the demo is ~41 mm.

Shortcuts (env var `FIBRE_CHEAT`, each scores exactly 0):

| `FIBRE_CHEAT` | what it does | result |
|---|---|---|
| `nothing` | no motion | 0.0 (connector not seated) |
| `direct` | the natural careless move: left arm pins the cord, right arm carries the connector straight towards the holder low over the table | 0.0: the cord catches on the bracket corner and is pulled taut round it (21 mm radius at the corner, then the over-stretched cord simulation fails) |
| `kinkfix` | folds the free end back on itself (tight U, 13.7 mm), unfolds it, then does the full correct routing; the end state is perfect | 0.0 (radius limit broken mid-episode) |
| `unclipped` | correct routing but leaves the connector 4 cm short of the holder, then lets go | 0.0 (not seated) |
| `hold` | correct routing, connector in the holder, but the right arm keeps holding it | 0.0 (not released) |

`FIBRE_CHEAT=probe_face` is a noise check, not a strategy: it presses the cord flat against the bracket's side
face and holds it there: the smallest radius stays above 0.6 m, so a cord lying along a flat face is not read as a kink.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- Bend radius, every control step (20 Hz): from the cord's segment centres p_i, chord directions t_i = p_(i+1) - p_i;
  over every 3 cm of cord (two 1.5 cm segment pitches) radius = 3 cm / angle(t_i, t_(i+2)). The smallest value of
  the whole episode is kept (reset with the scene). In words: no 3 cm length of cord may turn through more than
  1.2 rad (about 69 degrees). The window is the sim's resolution scale: the cord bends only at joints 1.5 cm apart, so a bend
  confined to one joint is reported as spread over 3 cm; noise is far below the limit (a resting straight cord
  reads > 0.7 m, one pressed flat against the bracket face > 0.6 m, a cord carried by its connector > 0.2 m, the demo cord on the guide >= 41 mm; the bracket corner under tension 21 mm, a fold 13.7 mm).
- Score 0 if: the smallest radius was ever < 25 mm; the connector is not in the holder slot (centre between the
  walls, at most 4 mm out of the open end, on the floor); either gripper is within 3 cm of the cord or connector;
  the cord is not round the guide (cord surface within 20 mm of the guide face at 15, 45 and 75 degrees round the
  quarter of the guide between the cord's two legs, below the guide top).
- Otherwise 1.0 if the cord is within 8 mm of the guide face at all three points (lying against it), else 0.5.
- `distance()`: connector centre to its seated position (m); 1 m once the radius limit has been broken.
- `progress()`: radius kept, round guide (0 / 0.5 / 1), connector seated, released.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes ~5–15 min wall time on a shared GPU including the
  ~1–2 min Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so every trial
  starts identically. `VARIATION` lists seeded offsets for a later runner (enclosure/cord 1 cm along the cord,
  ±2 mm across it, which keeps it at least 1 mm clear of the guide and 3 mm clear of the bracket).
