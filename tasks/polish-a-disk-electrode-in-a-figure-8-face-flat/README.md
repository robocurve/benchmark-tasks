# Polish a disk electrode in a figure-8, face flat

**Instruction:** "Polish the electrode's face on the alumina spot in figure-8 strokes, keeping the face flat on the pad
with light pressure, then hang it in the stand's clamp with only its tip in the rinse water and let go."

A glassy-carbon disk electrode is re-polished on alumina before most voltammetry runs, often daily. The maker's guide
(Pine Research, Electrode Polishing Guide, DRK10013) gives the routine clean: a 3 mm round spot of 0.05 um alumina on a
microcloth pad over a flat glass plate; "keeping the electrode surface parallel to the surface of the cloth. A figure-8
pattern while gently turning the electrode is a useful technique"; then rinse, suspending only the electrode surface,
"perhaps by using a ring stand and clamp". The knack is contact control through a compliant arm: the face must stay flat
(3 degrees or less) at light pressure (1-5 N on a load cell) while the hand sweeps two loops of opposite turn through the
spot. A tip held a few degrees off rides its edge and rounds the disk; the figure-8 is the maker's prescribed path. Then only the tip may go in the rinse, below the 10 mm ring, and the tip is left hanging in the clamp.

Sources: https://pineresearch.com/support-article/electrode-polishing-guide/ (DRK10013, sections 3.1-3.3);
https://pineresearch.com/products/e5-rde-gc-ptfe/ (AFE5T050GC: 5 mm GC disk, 15 mm PTFE shroud 25.4 mm long, 49 mm
overall); cheaper option with the same body: https://pineresearch.com/products/e5tq-rde-changedisk-peek/ with a
https://pineresearch.com/products/5-mm-od-x-3-mm-thick-disk-gc/ insert. Clamp: https://www.avantorsciences.com/pr/en/product/24817664/deluxe-burette-clamps-united-scientific-supplies.
Why it matters for lab automation: robot chemistry platforms report manual electrode polishing as a hurdle to automated
electrochemistry, https://doi.org/10.1039/d4dd00323c (which also found the polishing pattern's effect on recovery not
significant; this task follows the maker's figure-8 as the prescribed procedure) and https://arxiv.org/abs/2401.06949.

## Run and score a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=polish_a_disk_electrode_in_a_figure_8_face_flat.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Polish the electrode's face on the alumina spot in figure-8 strokes, keeping the face flat on the pad with light pressure, then hang it in the stand's clamp with only its tip in the rinse water and let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/electrode-dream

# The scripted one-hand demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=polish_a_disk_electrode_in_a_figure_8_face_flat.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Polish the electrode's face on the alumina spot in figure-8 strokes, keeping the face flat on the pad with light pressure, then hang it in the stand's clamp with only its tip in the rinse water and let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/electrode-demo
```

The run prints `task_score`, read at the end (latched zeros aside). Runtime per trial: 2,400 steps at most (120 s
simulated); the scripted demo reaches 1.0 at step 1,137 (57 s) in Isaac. One fixed layout (`object_jitter=0`), no seeds.
Tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q`.

## Scoring (the scorer is `grade()` in the task file)

| Score | State at the end |
|---|---|
| 1.0 | 5 or more figure-8s, then rinsed and hanging in the clamp, face 3-10 mm under, grippers 2 cm away, still 1 s |
| 0.75 | 5 or more figure-8s and rinsed (face 3-10 mm under for 2 s, no pad contact after) |
| 0.5 | 5 or more figure-8s |
| 0.25 | 2 or more lobes in any pattern |
| 0 | anything else, including straight strokes and the untouched start; a drop or a face strike at any time |

A lobe is the face centre's track from one exit of the 10 mm crossing circle round the spot to its next entry; it counts
if it reaches 20 mm, is 10 mm wide, has 90% of its frames at 1.0-5.0 N with only the face on the pad and the face
within 3 degrees, never 6 degrees or 8 N, and no finger below the 10 mm ring. Figure-8s are the longest run of lobes
alternating sides and turn, halved.

### Measuring it in a real trial

1. Tape the glass plate to the load cell's top, tare it, and log it at 80 Hz; tap the pad once at the start to sync.
2. Film two side views at pad height, one along x and one along y, with a printed 3-degree guide behind the pad and a
   ruler on the pad for scale, at 30 frames a second or more. The face centre is the midpoint of the shroud's silhouette
   5 mm above the face. Step through the frames (any video player, or a short OpenCV script): tilt is the shroud
   edge's angle against the guide in each view, the track is the face centre's position in the two views.
3. Paint thin rings on the shroud 3 mm and 10 mm above the face; read the dip against the water line through the dish.
4. Count lobes from the two side views and the log; check grippers off and the tip still in the last second.

## Layout (cm from the midpoint between the arm bases, x forward, y left; table top z 0)

| Object | Type | Material | Size | Mass | Pose |
|---|---|---|---|---|---|
| Electrode tip (AFE5T050GC) | free | PTFE, glassy carbon, stainless | 15 mm x 25.4 mm shroud, 10 mm stem, 49 mm | 25 g (est.) | face-down on the wipe at (13, -5) |
| Lint-free wipe | free | paper | 60 x 60 x 1 mm | 5 g | (13, -5) |
| Load-cell platform | free | aluminium | 160 x 160 x 40 mm, top on a 10,000 N/m spring | 1.5 kg | (25, 4) |
| Glass plate + microcloth pad | on the platform | glass, cloth | 150 x 150 x 6 mm, pad at 4.7 cm | 0.4 kg | (25, 4); 3 mm alumina spot at the centre |
| Crystallizing dish | free | borosilicate | 100 x 50 mm, 40 mm DI water (line 4.3 cm) | 0.6 kg | (22, -16) |
| Ring stand + spring clamp | free | cast iron, steel, rubber | 150 x 180 mm base, 12 mm rod at (33, -16), jaws 14 mm gap, flared on top | 2.0 kg | jaws over the dish centre, 4.7-5.3 cm up |

Parts list (real trial): Pine AFE5T050GC tip (or AFE5TQ050PK with AFED050P030GC), Pine AKPOLISH kit, 5 kg bar load cell
and HX711 board, 150 mm glass plate, 100 x 50 mm crystallizing dish, ring stand with a spring-loaded burette clamp (e.g. United Scientific deluxe, rubber-sleeved jaws), lab wipes,
nitrile glove fingertips over the gripper pads.

## Simulation and the real case

Rigid bodies. Alumina, water and wear are not simulated: the slurry spot is a painted mark and the rinse is the face's
depth under a fixed water line (the water is a non-colliding volume). The load cell is a stiff 10,000 N/m spring under
the plate (0.5 mm at 5 N); its deflection gives the force. The tip collides as a 15 mm square prism (shroud) and a 9 mm
square prism (stem), drawn round, so the pads and jaws bear on flats. The clamp's jaws slide on 1,500 N/m springs (about
0.75 N each on the shroud; the friction holds the 25 g tip about 3.7 times over). Friction 0.15 on the tip and pad, 0.6 on
the clamp's rubber; the tip's mass and the stem's size are estimates to measure on the real tip. Nothing is moved,
latched or held by code.

## The demo (scripted, one hand)

1. Open over the stem, come down so the pad tips sit 30 mm above the face, close, lift.
2. Carry over the spot and lower the face to 1 mm above the pad.
3. Press to 2.5 N (the load cell is read every step and the hand moves by the force error over 0.45 N/mm), then trace
   six Gerono figure-8s, 30 mm half-length, 5 s each, holding the force.
4. Lift 30 mm, read the hand-to-face offset, carry over the clamp, and lower the tip through its flared jaws with a
   face servo (the hand moves by 0.4 x the face's position error each step) until the face is 5.5 mm under water;
   hold there 2.5 s (the rinse).
5. Let go (the jaws keep it; it settles 1-2 mm), rise, go home and hold.

## Failure variants (MuJoCo physcheck, `KIT_DEMO=<mode>`)

| Mode | What it does | Score |
|---|---|---|
| demo | as above | 1.0 (step 1,136) |
| none | both arms stay home | 0.0 |
| strokes | back and forth through the spot (every lobe 0 wide) | 0.0 |
| circles | one-way circles beside the spot (never alternates) | 0.25 |
| held | polishes and rinses, never lets go | 0.75 |
| dunk | dips past the 10 mm ring, then back up into the clamp | 0.5 |

Robustness (demo, one factor at a time, `TASK_SWEEP`): friction x0.8 1.0, x1.2 0.0 (the tip twists in the pickup grip
and never reaches the pad); mass x0.75 1.0, x1.25 1.0; clamp spring x0.5 1.0, x2 1.0.

## Licences and safety

All geometry is built from boxes in the task file (MIT). No third-party assets. Safety: run behind the cell's guard
with the e-stop in reach; alumina slurry and DI water only.
