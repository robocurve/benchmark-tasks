# Thaw a cryovial by swirling it, cap above the water

A frozen 2 mL cryovial (Corning 430659, external thread, silicone washer in the cap) stands on a mat beside a
Thermo Precision GP 02 water bath at 37 C. The suppliers' method: hold the vial in the bath and swirl it gently until
a small bit of ice is left (about 1-2 minutes), keeping the cap and its seal out of the water, then take it out. Two
marks on the vial make this measurable: 15 mm (the top of the contents, kept under) and 33 mm (just under the cap,
kept above).

Instruction: "Thaw the frozen vial in the 37 C water bath, swirling it with the contents under the water and the cap above it for about 1 to 2 minutes, then stand it back on the circle."

## Run and score a trial

    # scripted demo (the task file's demo) on the Isaac Lab arena
    python -m yam_arena.run --task tasks/thaw-a-cryovial-in-the-water-bath-cap-and-o-ring-kept-out --policy yam_scripted --steps 4800
    # tests (kit tests 1-7)
    cd tasks/thaw-a-cryovial-in-the-water-bath-cap-and-o-ring-kept-out && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q

- Runtime per trial: up to 4,800 steps (4 minutes of sim time at 20 Hz); the demo takes about 2,100 steps.
- Seeds: one fixed layout, no randomisation (seed 0, object jitter 0).
- The score (`task_score`) is read at the end of the run by `score()` in
  `thaw_a_cryovial_in_the_water_bath_cap_and_o_ring_kept_out.py` (top of the file: the props, then the dip clock,
  the swirl check and the scorer, then the demo). The dip clock and the swirl are kept step by step from the vial's
  pose and the water height.
- The scripted demo grips the cap with the hand pitched 45 deg (a straight-down hand cannot reach over the rim),
  holds the water at the 24 mm mark and moves the vial in 16 mm circles for 72 s. A policy may do it any way.

## Scoring (from task.json)

A dip runs while the 15 mm mark is under the water. 0 at any time: the water reaches the 33 mm mark anywhere round
the vial, or the vial is let go with its base under the water. Otherwise 0.25 a dip of 15 s; 0.5 a dip of 30 s;
0.75 a dip of 45-120 s with the vial moving 10 mm side to side in every 5 s of it, no dip over 120 s, the vial out of
the water; 1.0 and the vial standing within 10 deg of upright in the 50 mm circle, grippers 20 mm clear, moving under
2 mm in the last 1 s.

## Real trials

Real Corning 430659 vials with 1.0 mL of freezing medium (no cells) frozen at -80 C and carried on dry ice; a GP 02
bath at 37 C with a 10 mm grid sticker on the tank floor and a 5 mm grid strip on the far, right-hand inner wall at the water line.
Two cameras and a clock: a phone on a stand at the bath's left side, its lens 280 mm from the vial along a line
pitched 33 deg down (about 150 mm above the water and 125 mm above the rim, 235 mm out; its line of sight passes about
23 mm over the rim) (the steel wall hides the water line from a level camera; this view looks
over the rim), which reads the waterline against the 15 mm and 33 mm marks with the wall grid behind; the holding hand
comes from the bath's right side, its fingers in front of and behind the cap, so neither is between this camera and the
marks. In the simulator the idle left arm's wrist camera takes the same pose through the dip and the trial video records
it: the cap, the red band and the vial's body going down through the water surface. And a camera above the tank, which reads the swirl against the floor grid. At lift-out, photograph the vial from
the side against the wall grid: full credit needs the contents liquid with at most one ice sliver 2 mm or less across
(Gibco: "until there is just a small bit of ice left"); a sample not thawed scores at most 0.5. In the simulator the
swirled 45-120 s dip is the timed proxy for the thaw. After the trial, touch a dry tissue to the cap seam (it stays dry
when the 33 mm mark stayed out). The water is at body temperature and the vial holds no cells; trials are safe to run.
Wipe the pads dry between trials.

## Files and assets

- `thaw_a_cryovial_in_the_water_bath_cap_and_o_ring_kept_out.py`: scene, dip clock, scorer, demo.
- `ourkit/`: the shared mechanism kit, copied unchanged.
- `test_thaw_a_cryovial_in_the_water_bath_cap_and_o_ring_kept_out.py`: the kit's tests.
- No third-party meshes or scans: every part is built from boxes.
