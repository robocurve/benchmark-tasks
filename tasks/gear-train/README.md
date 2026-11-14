# Mesh a three-gear train, finding each gear's tooth phase by contact

A gearbox assembly step at full size. Three spur gears (62, 42 and 22 mm across) lie on the table beside a fixed plate
with three shafts. The robot slides each gear onto its shaft and all the way down, so the medium gear meshes with
both others, then lets go and moves its gripper clear. The source is a dealer-service step for a rotary cutter's spindle gearbox (a crown gear and its pinion
there, spur gears here): "align gear (1) and gear (5) so that gear teeth are a match"
([Woods DS8.50 Operator's Manual, page 31, step 11](https://www.manualslib.com/manual/1174793/Woods-Ds8-50.html?page=31)).

The hard part is physical. The bore slides over its shaft with little play, and a gear goes down only when its teeth
line up with the gaps of the gear it meshes with. Out of phase it stops on the other gear's teeth, 7 to 10 mm above
its seat, where it looks almost right from a camera above. The gears have 60, 40 and 20 teeth (module 1 mm), so the
large gear's teeth repeat every 6 degrees, and in a sweep it went down only within about 1 degree of the right angle.
The robot has to find the phase: turn the gear a few degrees and try again, or turn it slowly under light contact.
Nothing on the table marks the right angle; it depends on how the first gear came to rest. On a real bench, forcing a
gear that is out of phase chips or burrs its teeth.

## Parts and dimensions

The plate and gears are Isaac Lab's Factory gear assets. Isaac Lab's asset notes say the gears and shafts are NIST's
models "with minor modifications to clearances" and the base plate is NVIDIA's own design
([factory-acknowledgments.txt](https://github.com/isaac-sim/IsaacLab/blob/main/docs/licenses/assets/factory-acknowledgments.txt)).
NIST's Assembly Task Board 1 lists the real parts as MISUMI catalog items
([assembly instructions, parts list](https://www.nist.gov/document/assemblyinstructionsv4docx)), so the props can be
bought. Sources: N = NIST parts list, G = MISUMI page for GEABP1.0-40-10-B-10 (module 1.0 table), S = MISUMI page for
SPTR10-20-M6-SC12, C = code (file:line below), A = arithmetic from the other columns.

**Real parts (the props to build the meshing assembly)**

| Part | Catalog no. | Material | Dimensions | From |
|---|---|---|---|---|
| small gear | MISUMI GEABP1.0-20-10-B-10 | white polyacetal | 20 teeth, module 1.0, pressure angle 20 deg; reference (pitch) dia 20, tip 22, root 17.5; face width 10; hub dia 16, overall length 20; bore 10 H7 with set-screw tap; JIS B 1702 class 5; backlash provided | N, G |
| medium gear | MISUMI GEABP1.0-40-10-B-10 | white polyacetal | 40 teeth; pitch 40, tip 42, root 37.5; face 10; hub 30, overall 20; bore 10 H7 | N, G |
| large gear | MISUMI GEABP1.0-60-10-B-10 | white polyacetal | 60 teeth; pitch 60, tip 62, root 57.5; face 10; hub 35, overall 20; bore 10 H7 | N, G |
| shaft (x3) | MISUMI SPTR10-20-M6-SC12 | 52100 bearing steel, induction hardened | dia 10 g6 (-0.005 / -0.014), length 20, M6 tapped end, 12 mm wrench flats | N, S |
| shaft spacing | (not a part) | | standard centre distance m (z1 + z2) / 2: 50.0 mm large to medium, 30.0 mm medium to small | A |

All lengths in mm. The tooth form is the standard 20 deg involute of a module gear (JIS B 1702 is an involute-gear
accuracy grade); the MISUMI page states the pressure angle and grade, not the word involute. The H7 bore tolerance on a
10 mm bore is 0 / +0.015 mm (ISO 286, not read from a source here), so bore to shaft is a 0.005 to 0.029 mm diametral
slip fit. NIST's own gear plate (MISUMI JTNAS-AM 120 x 60 x 5 mm, 5052 aluminium) is not the plate in the sim.

**As spawned in the sim**

| Item | Value | From |
|---|---|---|
| assets | `gear_base`, `large_gear`, `medium_gear`, `small_gear` from Arena's library: `Factory/factory_gear_{base,large,medium,small}.usd` | C: IsaacLab-Arena `object_library.py:681,702,723,744` |
| size | each scaled uniformly to its `extent` (widest horizontal side): plate 150 mm, gears 62, 42, 21.8 mm | C: `gear_train.py:35,38`, yam_arena `station.py:229` |
| plate | 150 x 75 x 5 mm (collision), fixed (kinematic); shafts on its centre line at -30.25, +20.25, +50.75 mm from its origin (Factory offsets), so 50.5 and 30.5 mm apart: 0.5 mm more than the standard centre distance (the added backlash). Measured from the plate's centre: -30.28, +20.26, +50.73 mm; the scorer's shaft axes (-30.3, +20.3, +50.8 mm) agree within 0.07 mm | C: `gear_train.py:31,33`, Isaac Lab `factory_tasks_cfg.py:193-195`, yam_arena `scene.py:203`; M; A |
| shafts | 9.52 to 9.57 mm across (radius 4.76 to 4.78 mm; 3.76 to 3.78 mm at the chamfered tip), tips 20.0 mm above the plate top. The real SPTR10 is 10 mm g6: the Factory assets add clearance | M |
| bores and play | bores 9.99 to 10.02 mm, so 0.22 to 0.25 mm radial clearance on the shafts (about 0.23 mm). The real H7 bore on a g6 shaft has 0.0025 to 0.0145 mm radial (half the diametral slip fit above) | M; A |
| gear length | 24.97 to 25.05 mm overall in the sim; the real GEABP parts are 20 mm overall | M; G |
| seat | a seated gear's bottom 6 mm above the table (5.9 to 6.6 mm measured from the box low, before the tilt correction); shaft tips 26 mm above the table | C: `gear_train.py:1-3,41` |
| masses | 60, 30, 10 g (the real polyacetal parts are about 51, 25 and 5 g: tooth ring at the pitch diameter plus hub, less the bore, at 1.41 g/cm3, a typical acetal density not read from a source) | C: `gear_train.py:36`; A |
| friction | every object 1.2 static, 1.0 dynamic (rubber-like; dry acetal slides far more easily) | C: yam_arena `constants.py:61` |
| collision | each asset's own mesh colliders (not convex hulls); loose gears contact_offset 5 mm, rest_offset 0 | C: yam_arena `scene.py:116,206-207` |
| sim mesh | tooth count, tip / root / pitch dia, face width, hub, bore dia, shaft dia, radial clearance, in the table below | M |

M = measured from the four USDs the simulator loads, at the spawn scale (`tools/measure_assets.py`, run on the sim
machine). All in mm.

| gear | teeth | module | tip dia | root dia | pitch dia | face | overall | hub dia x len | bore dia | shaft dia | radial clearance |
|---|---|---|---|---|---|---|---|---|---|---|---|
| large_gear | 60 | 1.001 | 62.05 | 60.81 | 60.05 | 9.72 | 25.02 | 60.32 x 15.30 | 10.01 | 9.57 | 0.22 |
| medium_gear | 40 | 1.002 | 42.08 | 41.50 | 40.07 | 9.35 | 25.05 | 40.34 x 15.70 | 10.02 | 9.52 | 0.25 |
| small_gear | 20 | 0.999 | 21.97 | 21.74 | 19.98 | 9.12 | 24.97 | 21.37 x 15.84 | 9.99 | 9.53 | 0.23 |

How to read it. Teeth, tip, bore, overall and shaft are direct: the tip and bore are the largest and smallest vertex
radius about the box's centre, overall is the box's height, the shaft is the largest vertex radius of the shaft's
straight part. Module is 2 x tip radius / (teeth + 2) and pitch is module x teeth, so both follow from the tip and are
not separate measurements. The root, face and hub columns are not trustworthy:

- Root. The script takes the toothed face as the height span of the vertices within 2 % of the tip radius, and the
  root as the smallest radius of vertices inside that span (and outside tip - 2.6 module). The measured roots (60.81,
  41.50, 21.74) sit far above a module gear's root circle (57.5, 37.5, 17.5) and only 0.2 to 1.2 mm inside the tip;
  the large gear's equals 0.98 x its tip, the selection cut itself. So the column reads the vertices that fall in that
  span, not the root circle. A likely cause is that the tooth ends are chamfered or rounded, putting the tip-land
  vertices at a narrower height span than the root vertices, but this is not checked [INFERENCE, unverified]. The
  sim's root circle is not known from this measurement.
- Face and hub follow from the same span (face = its height; hub = the largest radius outside it, over the rest of
  the length), so they share the problem. The hub diameters read within 0.6 to 1.7 mm of the tips, far wider than the
  real hubs (16, 30, 35 mm) [unverified for the same reason].
- Involute. The script fits flank points to a 20 deg involute. For the large gear it gave RMS 0.00 mm over 480
  points, but its flank band runs from root + 25 % to tip - 15 % of the tip-to-root depth, so with the high root
  reading it covered only radii 30.56 to 30.93 mm, the last 0.5 mm under the tip [ARITH from the script and the
  table]: that says little about the flank. For the medium and small gears the fit was not obtained (0 flank points,
  NaN). This README claims no involute form for the sim meshes.

## Assets and licenses

| File | What | Source | License |
|---|---|---|---|
| `factory_gear_large.usd`, `factory_gear_medium.usd`, `factory_gear_small.usd` | the three gears | NIST's models of the MISUMI gears above, clearances modified by NVIDIA; shipped with Isaac Lab, loaded by Arena from NVIDIA's asset server | BSD 3-Clause, NVIDIA ([factory-license.txt](https://github.com/isaac-sim/IsaacLab/blob/main/docs/licenses/assets/factory-license.txt)); NIST credited per the acknowledgments file |
| `factory_gear_base.usd` | the plate with its three shafts (shafts from NIST, plate custom-designed by NVIDIA) | same | same |
| `gear_train.py`, `test_gear_train.py`, `task.json`, `README.md` | this task | written by our team | MIT |
| `arm_watch.py` | the hooks that let the scorer read both arms each step | adapted from robocurve/benchmark-tasks pull request #223 (`tasks/spring-plunger-panel/mechanism.py`); notice in the file | MIT |

This folder holds no mesh, USD, texture or image file. Nothing was generated or downloaded into it: the simulator fetches
the four USDs above by registry name.

## The scripted demo
`demo()` seats the large gear, then the medium and the small ones, with the right arm. It does not rely on knowing
angles or positions to the millimetre: each gear rests on its shaft's tip while the wrist spirals in from 3 mm until
the bore drops over the tip. The medium and small gears then press lightly on the teeth they mesh with and turn
through more than two tooth pitches until they drop into mesh. Each gear is then pushed home and let go. After the
small gear, the last, the fingers open fully and wait 0.8 s at the seat before the hand backs off upward over 1.5 s:
the plan's hand stands 16 mm off the nearest gear when fully open and past 50 mm once up (URDF forward kinematics of
the plan's targets, `test_gear_train.py`; the sim arm lags its targets by a few mm). `demo(delay)` lengthens only the
first move, for start-delay checks.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=gear_train.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Assemble the gear train on the grey plate: put the large gear on the nearest shaft, the medium gear on the middle shaft and the small gear on the farthest shaft, each pushed all the way down so its teeth mesh with the gear next to it, then let go and move the grippers clear." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/gear-train-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=gear_train.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Assemble the gear train on the grey plate: put the large gear on the nearest shaft, the medium gear on the middle shaft and the small gear on the farthest shaft, each pushed all the way down so its teeth mesh with the gear next to it, then let go and move the grippers clear." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/gear-train-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/gear-train-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step, which never reaches 1.0.

## Scoring
| Condition | Number | Check on a real table |
|---|---|---|
| seated | centre within 2 mm of its own shaft's axis, the centre of its bottom face within 1 mm of 6 mm above the table, axis within 3 degrees of vertical | a depth gauge on the gear's top face |
| still | under 0.01 m/s and under 0.5 rad/s (yam_arena `resting()`) | nothing turning or settling |
| clear | no part of either arm within 10 mm of any gear: a lower bound on the gap from each arm body's collision hulls to points laid 2 mm apart over each gear's bounding cylinder (sim tip diameter by 25.05 mm, centred on the gear's box and turned with it); no arm reading counts as not clear. The bound errs toward not clear: in random poses near the train it read up to about 7 mm under the true gap (so a hand 10 to 17 mm off may still count as near), never over it | hands off, a finger's width away |
| meshed pair | large and medium, or medium and small, both seated | the two turn together |

| Score | Condition |
|---|---|
| 1.0 | all three gears seated and still, and both arms clear, for 1 s (ends the trial); the only way to 1.0 |
| 0.8 | the limit ends before the 1 s hold: all three seated and still, both arms clear, at the last step |
| 0.7 | all three seated and both arms clear at the last step, but at least one gear moving |
| 0.6 | all three seated at the last step with an arm within 10 mm of a gear (still holding or touching one), moving or not |
| 0.5 | one meshed pair: 0.1 for each seated gear plus 0.3 for the pair |
| 0.2 | the large and small gears seated without the medium: no mesh |
| 0.1 | one gear seated |
| 0 | nothing seated: gears stopped on teeth, on the wrong shaft, on a shaft's tip, or lying elsewhere |

The bottom face's height is read from the gear's box with the box corners' tilt drop added back, so a gear tilted
within 3 degrees cannot pass while proud of its seat. Only the medium gear meshes with the others: the large and small shafts are 81 mm apart and those gears' tips reach
42 mm, so seating those two first matches no phase and scores 0.2. Two seated neighbours cannot overlap at seat
height, so a seated pair is meshed. `success()` and `score()` see only object states, so a hook (`arm_watch.py`) reads
every body of both arms from the simulator before each scored step. A seated train still held pays 0.6, not the pair
sum (0.9), so letting go always pays more. The partial tiers (0.5 and below) do not ask for clear hands.

## Runtime and seeds
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- Dream-1 gets 2,400 control steps (2 minutes at 20 Hz); the scripted demo's plan ends at 95.5 s. It reaches
  success at 94.45 s (step 1,889; steps 1,890 and 1,891 at start delays 0.05 and 0.10 s). The 1 s hold starts 0.85 s
  after the small gear's release begins, with that hand open and at least 11.2 mm from every gear (inspect-robots
  1.0.0).
- `python -m pytest -q test_gear_train.py` checks the scoring and the demo's reach without the simulator.

## Scores
The Dream-1 run (0) was scored by the first scorer, which paid 0.1 for any gear within 2 mm of its own shaft's axis
and below the shaft tips, seated or not. A 0 there means no gear was on its own shaft at the last step, and every gear
the final scorer pays for is on its own shaft, so the run scores 0 under the final scorer too. The scene and step
limit are unchanged. The instruction is not: that run had it without the closing "then let go and move the grippers
clear", which was added with the clear check.

## Fidelity
The plate and gears are the Isaac Lab Factory gear assets at full size, collided by their meshes. The plate is fixed.
The gears weigh 60, 30 and 10 g, near the real polyacetal parts (about 51, 25 and 5 g). Every object has rubber-like
friction (1.2 static, 1.0 dynamic), so teeth grip each other and the shafts more than dry acetal on acetal or steel.
