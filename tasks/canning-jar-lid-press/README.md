# Put away three jars of home-canned sauce: one lid did not seal

Three half-pint jars of home-canned tomato sauce stand in a row on the bench, cooled overnight, screw bands off. One
lid did not seal. Nothing shows it: all three jars and lids look the same. Only pressing the middle of each lid finds
it (the unsealed one goes down about 1.5 mm and springs back). A wooden pantry shelf stands beside the left arm and a
4 L mini fridge with its door open beside the right arm. The instruction is only: "These three jars of tomato sauce
were canned yesterday and have cooled overnight. Put them away for the winter. The wooden shelf is the pantry shelf."

The safe way, as the National Center for Home Food Preservation describes it: press each lid, put the unsealed jar in
the fridge and the two sealed jars on the pantry shelf. The unsealed jar let go on the pantry shelf scores 0 for the
whole run.

A blind guess scores 0. The lid springs back, so the end state shows nothing; the scorer therefore reads the press from
the whole trial: the unsealed jar's lid centre must go down at least 0.8 mm while that jar still stands at its start
place (the demo's press moves it 1.44 mm; a sealed lid moves about 0.2 mm). Without that, every outcome scores 0. Each
sealed jar earns its 0.3 only if a fingertip was held on its lid centre for 0.3 s while it stood at its start place.
In a real trial both are read from the video (a fingertip on the lid's middle; the unsealed lid visibly gives).

The fridge and the shelf carry visual-only cues so a camera can tell them apart: a backboard with a printed PANTRY card,
spice tins and cans on and under the riser; a door seal, handle, lit ceiling, control display and the printed word
FRIDGE on the fridge. In a real trial these are real card and tins, clear of every place a jar goes.

Files: `canning_jars.py` (scene, rules, scorer and the scripted demo's plan), `layout.py` (every size, mass,
friction and place, with sources), `jardemo.py` (the demo's planning hook and arm-sag correction for
`yam_scripted`), `bodykit.py` (multi-shape bodies and joints for `yam_arena`), `dev/run.sh` (one trial),
`dev/reach.py` (inverse-kinematics reach check of the demo plan).

## Run and score a trial

On a machine with Isaac Lab-Arena and the `yam_arena` embodiment, from this folder:

    bash dev/run.sh <name> yam_scripted <seed> 1400                          # scripted demo (FRAMES=1 stores frames)
    bash dev/run.sh <name> yam_scripted <seed> 1400 DEMO_VARIANT=fixed_jar   # part-done plans: nothing, fridge_only, pantry_all, fixed_jar
    bash dev/run.sh <name> dream 0 1400                                      # Dream-1 (bridge on 127.0.0.1:8211)

`dev/run.sh` runs `inspect-robots run --embodiment yam_arena -E task=canning_jars.py -E object_jitter=0
-E control_hz=20 --scorer task_score --max-steps 1400 --seed <seed>` with the instruction above. The score is printed
as `task_score:` in `run.out`; `debug.log` holds one line every 4 control steps (each jar's pose, tilt and lid
travel, and the first breach).

- Run limit: 1,400 control steps (70 s at 20 Hz).
- Seeds: the seed picks only which jar has the unsealed lid (the episode seed modulo 3). Seeds 0, 1 and 2 give jars
  2, 3 and 1, so they cover all three places. Everything else is at one fixed place (`object_jitter=0`).
- Runtime per trial: about 12 to 14 minutes of wall-clock time on a shared GPU, including about 1.5 minutes of
  start-up (the demo ends after 1,193 control steps).

## Sources for sizes (opened 2026-11-20)

- Jar: Ball 60000ZFP 8 oz half-pint regular-mouth glass canning jar (WebstaurantStore listing): "Height: 3 15/16
  Inches", "Top Diameter: 2 3/4 Inches", "Capacity: 8 oz.".
  https://www.webstaurantstore.com/ball-60000zfp-8-oz-half-pint-regular-mouth-glass-canning-jar-with-silver-metal-lid-and-band-case/52760000.html
- Lid test: National Center for Home Food Preservation, "Cooling Jars and Testing Jar Seals": "Press the middle of the
  lid with a finger or thumb. If the lid springs up when you release your finger, the lid is unsealed."
  https://nchfp.uga.edu/how/can/general-information/cooling-jars-and-testing-jar-seals
- Mini fridge: Cooluli Classic 4 L, inside 5.3 x 5.7 x 8.3 in (135 x 145 x 210 mm), outside 8 x 10.2 x 11 in.
  https://cooluli.com/products/classic-4-liter-compact-mini-fridge
- Pantry shelf: a wooden display riser, 4 in high, 15 in wide, 10 in deep (Wayfair "Wrentham Riser Wood Lg").
- Estimates: the jar's body width (72 mm) and filled mass (0.45 kg), the lid's flexing middle (48 mm across), its
  travel (1.5 mm) and force (1.5 N to start it, 6 N at the bottom; no source gives these), the fridge's wall
  thickness and floor height, the boards' thickness, and all frictions.

No stand-ins: real trials use real jars of sauce, one with a lid that did not seal.

## The scripted demo

Scripted demo, reads object poses and the lids' travel from the simulator to plan its motions; the jars and the lids
are moved only through the grippers, by contact. It plans in two stretches:

1. The press test. With its fingers closed, each gripper presses straight down on the middle of a lid (the fingertip
   target 20 mm below the lid's top: the arm is soft and stops on a sealed lid), holds it 0.5 s and lifts off. The
   left gripper presses jar 1 while the right gripper presses jar 3; then the right gripper presses jar 2.
2. From the lids' travel under the press (read from the simulator) it calls a lid unsealed if it went down more than
   0.5 mm (a sealed lid reads about 0.2 mm, the unsealed one about 1.4 mm). The left gripper grips each sealed jar
   from above, lifts it 30 mm above the shelf's height, carries it over, stands it on the shelf, lets go and backs
   off 100 mm up. The right gripper grips the unsealed jar from the front, carries it level into the fridge, stands
   it on the floor, lets go and backs straight out. Each arm waits parked while the other works.

`DEMO_VARIANT` picks a weaker plan for the part-done runs: `nothing` (both arms stay parked), `fridge_only` (press,
put the unsealed jar in the fridge, leave the sealed two on the counter), `pantry_all` (no press, all three jars to
the shelf), `fixed_jar` (no press; always jar 2 to the fridge, jars 1 and 3 to the shelf: a guess that is right on
one seed in three), `fridge_unpressed` (no press; the unsealed jar, read from the simulator, to the fridge and the
sealed two to the shelf: the right end state without the test). "All three to the fridge" cannot be run: the 4 L
fridge holds two of these jars at most.

## Results

All runs on one code version (`canning_jars.py` sha256 6cccceb0..., `layout.py` d1174462..., `jardemo.py`
ff737808..., `bodykit.py` 4e76ca44...), 1,400-step limit. Part-done plans: the score was written down before the run.

| Run | Seed (unsealed jar) | Predicted | Score | Steps | What happened |
|---|---|---|---|---|---|
| Scripted demo (video) | 0 (jar 2) | 1.0 | 1.0 | 1,193 | Lid travel under the press 0.22 / 1.44 / 0.22 mm; jars 1, 3 on the shelf, jar 2 in the fridge |
| Scripted demo | 1 (jar 3) | 1.0 | 1.0 | 1,193 | Press 0.22 / 0.32 / 1.45 mm; jar 3 in the fridge |
| Scripted demo | 2 (jar 1) | 1.0 | 1.0 | 1,193 | Press 1.45 / 0.32 / 0.22 mm; jar 1 in the fridge |
| `fixed_jar` (no press, jar 2 always to the fridge) | 0 (jar 2) | 0 | 0 | 1,029 | End state exactly right (jars 1, 3 on the shelf, jar 2 in the fridge), but no lid was pressed: 0 |
| `fixed_jar` | 1 (jar 3) | 0 | 0 | 1,029 | Unsealed jar 3 let go on the shelf (step 535) |
| `fixed_jar` | 2 (jar 1) | 0 | 0 | 1,029 | Unsealed jar 1 let go on the shelf (step 277) |
| `fridge_unpressed` (no press, the right jar to the fridge) | 0 (jar 2) | 0 | 0 | 1,029 | End state exactly right, no breach, but no lid pressed: 0 |
| `pantry_all` (no press, all three to the shelf) | 0 (jar 2) | 0 | 0 | 905 | Unsealed jar 2 let go on the shelf (step 535) |
| `fridge_only` (press, unsealed jar to the fridge, others left) | 1 (jar 3) | 0.4 | 0.4 | 740 | Jar 3 in the fridge, jars 1 and 2 left on the counter |
| `fridge_only` | 0 (jar 2) | 0.4 | 0 | 740 | Jar 3, left standing, was in the carry's path and tipped past 30 degrees (step 630): a fair 0 |
| `nothing` (both arms parked) | 0 (jar 2) | 0 | 0 | 81 | Untouched scene |
| Dream-1 | 0 (jar 2) | | 0 | 1,400 | Moved no jar and pressed no lid |

"All three to the fridge" cannot be run: the 4 L fridge holds two of these jars at most.

Runtime per trial: about 12 to 14 minutes of wall-clock time on a shared GPU, including about 1.5 minutes of start-up.
