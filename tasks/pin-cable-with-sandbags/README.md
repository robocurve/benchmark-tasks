# Pin a cable with sandbags at the marks

A 55 cm black 6 mm cable leaves a fixed grey stage box on the left through a grommet (held at the exit, free to swing
sideways) and lies loose on the table, swung 22 degrees off a taped cable lane. The lane runs straight from the cable
exit (towards the robot's right): two yellow tape strips with a 40 mm clear lane between them and three red cross
marks 15, 30 and 45 cm from the exit. Three free olive-drab sandbags (50 x 40 x 25 mm, 80 g) wait off to the side (two near the left arm, one beyond the far side of the lane). The robot
must lay the cable straight inside the lane, set one sandbag on top of the cable at each red mark and let go of
everything. Temporary cable runs on event floors and outdoor sites are kept out of the walking path and weighed
down so foot traffic and wind cannot shift them; a bag next to the cable, or bags on a cable that is not in its lane, do
nothing, so both earn 0. It is naturally bimanual: one arm keeps the cable straight in the lane while the other sets
the bags on it, near mark first.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
    --instruction "Lay the black cable straight inside the yellow taped lane, then pin it with the sandbags: put one sandbag on top of the cable at each of the three red marks. Let go of the cable and the bags."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1600
```

Closed-loop (reads the live cable and bag states), both arms: the left arm takes the cable ~24 cm from the exit and
the right arm takes the plug; together they lift the cable 6 cm, carry it over the lane and lay it down, and the
right arm pulls along the lane until the measured sideways deviation is under 3 mm, then keeps holding the plug. The
left arm lets go of the cable and sets a sandbag on the cable at the near mark, then one at the middle mark (each
centred over where the cable actually is). The right arm then lets go of the plug and sets the third bag at the far
mark, carrying it from beyond the lane end. The bags rest on the cable (bottom lifted ~6 mm) and the cable stays
straight. Grips are turned so the wrist cameras look ahead: the left arm's along the cable towards the lane and its
marks, the right arm's towards the far mark while it carries the third bag (only the plug grip looks back along the
cable: the right wrist's joint limit rules out the reversed grip there). Success holds 1 s and ends the episode at
step 749 (~37 s); task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Shortcut variants (test only, `PIN_CHEAT=<name> python cabledemo.py task.py out`), all score exactly 0:
`none` (do nothing: 10 % of the cable in the lane), `bagsonly` (set the three bags on the cable where it lies,
15/30/45 cm from the exit, without straightening it: all bags rest on the cable, lifted ~6 mm, but only 10 % of the
cable is in the lane), `straight` (lay the cable straight in the lane, 100 % in, and let go without any bags),
`beside` (cable straight in the lane, bags set at the marks but 34 mm to the side of the cable: lifted 0.4 mm, not on
it), `hold` (everything done, all three marks pinned, but the left arm keeps holding the middle bag).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** unless both grippers have let go: each gripper centre at least 3.5 cm from every part of the cable, its plug
  and every sandbag.
- **0** if less than 90 % of the cable's length (exit to plug, centreline sampled every 2 mm) lies inside the lane:
  within 20 mm sideways of the lane centre line, between the box face and the lane end.
- Otherwise **1/3 per red mark** that has a sandbag resting on the cable: the cable centreline passes under the bag at
  least 5 mm in from its edges, the middle of the bag's bottom face is 2-12 mm above the table (lifted by the cable;
  a bag flat on the table beside the cable or stacked on another bag fails), and the bag centre is within 2 cm of the
  mark along the lane. Each bag counts for one mark only.
- `success()` additionally requires the bags and the cable to be at rest. `distance()`: share of the cable outside
  the lane + 0.3 per unpinned mark.

## Runtime and seeds

- Run limit 1,600 steps (80 s at 20 Hz). One trial takes ~5-15 min wall time on one GPU including the ~1-2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight at the same
  angle), so every trial starts identically.
