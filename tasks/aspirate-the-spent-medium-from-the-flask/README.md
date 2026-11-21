# Aspirate a tilted flask with the tip at its back wall

**Instruction:** "Aspirate the spent medium out of the open flask with the aspirator without touching the cell layer, then
lay the flask flat on its outline, put the aspirator back on its stand and let go."

An open T75 flask lies flat with 15.0 mL of medium (water dyed red) over its 75 cm2 floor, where the cells grow;
a vacuum aspirator rests in its stand. Aspirated where it lies, the 2 mm sheet leaves about 2 mL, and a tip put down
on the floor scrapes the cells. As labs do it, one hand tilts the flask neck-up and onto one side so the medium runs to
the low back corner, the other steers the pipette through the neck and rests its tip there until the corner is dry;
then the flask goes back flat and the aspirator back on its stand.
The work happens inside a Class II biological safety cabinet: the two-arm robot stands on the work surface of a
4-foot Thermo Scientific 1300 Series A2 cabinet (interior 1199 x 780 x 630 mm), the sash behind it at its 10 in
working height (the elbows ride 340-450 mm up, so the robot works from inside, as lab robots do). What binds, in the
layout used, are the walls: the right elbow (link 3) passes 4.4 cm from the right side wall while it carries the
aspirator (preflight: 4.5 mm clear of a wall at 56 cm, 5 mm into one at 52 cm; the wall is at 59.95 cm), and the left
elbow 4-5 cm from the back wall while it holds the tilt. In the maker's 3-foot cabinet (899 mm wide) the right elbow
hits the side wall, and with the flask and stand 4 or 6 cm further back the left elbow hits the back wall: the demo
scores 0 in both (physcheck). The ceiling, sash plane, 4 in line and rear grille are kept as the cabinet's own rules
(sticky zeros); in this layout the work does not come near them.

Sources: Galloway Lab adherent cell culture protocol
(https://gallowaylabmit.github.io/protocols/en/latest/protocols/tc/tc-basics/adherent_cell_culture.html: tilt with one
hand while aspirating with the other; do not touch the bottom until the very end); Gibco Cell Culture Basics,
"Subculturing adherent cells" (remove the spent medium first); INTEGRA VACUBOY operating instructions (residual-suction
screw); SPL flask data sheet SPL-TDS-CCF (124.10 x 84.20 x 39.60 mm, 75 cm2); Thermo Scientific 1300 Series A2
brochure (https://assets.thermofisher.com/TFS-Assets/LED/brochures/1300-Series-A2-Biological-Safety-Cabinet.pdf:
4-foot interior 47.2 x 30.7 x 24.8 in, 3-foot 35.4 in wide, front window 8 or 10 in); cabinets built for robots: NuAire custom robotic BSCs
(https://www.nuaire.com/resources/sanofi-custom-biosafety-cabinets-for-robotic-automation-white-paper), Erlab H.Box
Class II hoods for robots; BSC practice: "Perform all operations on the work surface and at least 4 inches from the
front grille" (https://www.drs.illinois.edu/SafetyLibrary/BiologicalSafetyCabinets).

## Run and score a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=aspirate_the_spent_medium_from_the_flask.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Aspirate the spent medium out of the open flask with the aspirator without touching the cell layer, then lay the flask flat on its outline, put the aspirator back on its stand and let go." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir logs/aspirate-dream

```
The scripted two-arm demo: the same with `--policy yam_scripted -E cam_width=640 -E cam_height=352 --log-dir logs/aspirate-demo`.

The run prints `task_score`, read at the end. Runtime per trial: 3,600 steps at most (3 minutes simulated); the demo
takes about 73 s (1,457 steps; 1.0 at the last step, once both grippers are clear). One fixed layout (`object_jitter=0`), no seeds. Tests: `python3 -m pytest -q`.

## Scoring (the scorer is at the top of the task file)

The ladder is task.json's `scoring`, by medium left (sim: the volume rule; real: a 0.01 g balance against the
written empty mass). Real-trial cell-layer check: the 1 mm agarose witness layer in the setup (0.75 g agarose in 75 mL, microwaved in
10 s bursts until clear, 7.5 mL poured at about 55 C, scraped off the 10 mm strip with a cell scraper). A tip touch
is judged first on the bench-level camera (a phone on the work surface 30 cm to the flask's left, lens level with its
floor, looking along it through the clear side wall: a frame of the tip on the layer beyond the marker line), then on
the light box (a groove beyond the marker line); either counts. The once-per-session
groove check and the per-trial empty mass are in reset_notes.

| Strategy (KIT_DEMO mode) | Cap | MuJoCo (physcheck) |
|---|---|---|
| demo | 1.0 | 1.0 (0.27 mL left, no touch, step 1,456) |
| none: both arms stay home | 0 | 0.0 (15.0 mL left) |
| hover: tilted, tip held 10 mm over the corner | 0.5 | 0.25 (5.1 mL left) |
| middle: tilted, tip down to the floor 30 mm from the back wall | 0.25 | 0.0 (13.3 mL left) |
| still_held: full draw, aspirator kept in the hand | 0.75 | 0.75 |
| not_flat: full draw, aspirator returned, flask left tilted | 0.75 | 0.75 |

## Simulation and the real case

- Free rigid bodies at catalogue size; nothing is moved or latched by code; the tubing is omitted (clipped overhead).
- The medium is the declared volume rule in fidelity (top of the task file); removed medium never returns; a red
  floor tint fades with the volume (cosmetic only).
- Neck: a 12-sided tube on the 21.5 mm bore; pipette square (6.18 mm); the medium's 15 g rides with the flask body.
- Stand: printed PLA cradle (4 mm side play) on 12 mm columns over a 6 mm steel plate 100 x 200 mm, 2.0 kg, free.
- Friction 0.5 (set, not measured), static = dynamic. Robustness (MuJoCo, one factor at a time): friction 0.4 and
  0.6, flask mass and aspirator mass each x0.75 and x1.25: the demo scores 1.0 in all six (0.26-0.28 mL left).
- No house vacuum: an SP Bel-Art water-jet aspirator on a lab tap (about US$42, Avantor 11725682; needs mains tap
  pressure) on a 500 mL filter-flask trap, a screw pinch clamp set to 2.0 mL/s, a PLA tube 25 x 130 mm as the hand
  piece. Cabinet for repeated trials: a full-size plywood mock of the 4-foot interior (1199 x 780 x 630 mm, acrylic
  sash at 254 mm, grilles as tape bands 100 and 50 mm wide).

## Safety of real trials

Safe to run: dyed water only (no cells or chemicals), plastic flask and pipette, low residual suction, about 1 N at
the tip and a 20 N grip; nothing sharp or hot.

## Demo

Both arms servo the held object's pose each step (`servo.py`). Left: grips the flask, pivots it 35 deg neck-up about
its low back corner and 15 deg onto its side, later lays it flat and lets go. Right: slides the pipette through the neck
about 14 deg off its axis, rests the tip at the back wall for 7.5 s, withdraws at 10 mm/s and returns it to the stand.
Third-party assets: none (boxes only). `ourkit/` is the team's shared mechanism kit, copied unchanged.
