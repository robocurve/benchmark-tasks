# Spot three samples on a TLC plate and stand it in the developing tank

A glass-backed TLC plate lies silica up on a narrow bar in front of the robot, a pencil baseline and three ticks drawn
10 mm above its bottom edge. Three empty glass capillary spotters stand in a rack and three vials of dyed water stand
in a block. A rectangular glass TLC developing tank with 5 mm of water stands on the right, its lid on the bench in
front of it. The robot does what a chemist does to check a reaction by TLC (Nichols, *Organic Chemistry Lab
Techniques*, 2.3E): fill each spotter from its vial, touch it once, quickly and lightly, to its tick, put it back; then
stand the plate in the chamber, baseline above the solvent, leaning on the wall, and put the lid on.

What makes it hard, and what is tested:

- **A light, brief touch with a fragile glass tool, placed to a millimetre.** A spot only forms where the tip
  presses the silica with 0.05 to 1 N for at least 0.1 s, its size grows with the time in contact (3 mm after 0.5 s),
  pressing harder than 1 N for 0.1 s or dragging gouges the layer, and more than 3 N on a spotter breaks it. The tick
  must end up inside the spot, so the tip has to land within about 1 mm of it.
- **Filling by capillary action.** A spotter fills only when its tip goes 2 mm under the sample's surface, through a
  vial neck 8.5 mm across, for 0.25 s; wetting it in a second sample spoils both.
- **Handling the plate.** The plate goes from lying silica up on its riser to standing in the chamber. Nichols holds
  it only by its edges and back; here a plate put down face down smears its spots and scores 0, one lying face up
  does not.
- **Placing it in the chamber.** It has to stand on the tank's floor with its bottom edge in 5 mm of water and its
  baseline at least 3 mm above it, leaning on the back wall at most 35 degrees from vertical, spotted face out, and
  the lid has to go on, all without pushing the free-standing tank: a hand or the plate that shoves it more than 5 mm
  scores 0.

> **Status: the scripted demonstration scores 1.0** (3,033 of 4,800 steps, run twice with the final code, byte-identical scorer traces; "Native runs"). **Dream-1 has been run once on the final code and scores 0.0** (voided at step 4,770 of 4,800: capillary 1 slipped from the grip and tipped over).

## Frame and placement

Tabletop frame (task.json): x forward from the line through the two arm bases, y to the robot's left, z up from the
table top; millimetres in the text, metres in code. Arm bases at (0, +305) and (0, -305). Every number lives in
`rdb_tlc_spotting/layout.py`; the scene file is generated from it by `tools/build_assets.py`.

| What | Where (mm) |
|---|---|
| Plate (25 x 75, silica up, bottom edge toward the robot) on the bar | centre (262, 20), top face 15 above the table |
| Baseline (10 above the bottom edge) and ticks 1, 2, 3 | x = 234.5; ticks at y = 26, 20, 14 |
| Bar (12.7 x 12.7 x 76.2 aluminium) | under the plate's centre line, its far end at x = 300 |
| Capillary rack (30 x 110 x 25 foam, 5 mm holes 20 deep) | holes 1, 2, 3 at (165, 190), (165, 150), (165, 110) |
| Vials (15 x 45, 1 mL each, surface 10.1 above the table) in their block | (235, 180), (235, 140), (235, 100) |
| Tank (83 front to back, 121 side to side, 108 tall; inside 77 x 115; 5 mm water) | middle at (250, -160); back wall's inside face at x = 288.5 |
| Lid (87 x 125 x 4, flat glass) | lying on the bench at (135, -250) |

The left arm spots (all three spotters, vials and ticks are in front of its base); the right arm moves the plate and
the lid. Reach was checked with the arm's own inverse kinematics (pointing down): the right arm cannot point down high
(z 190) far in front of the midline, so the hanging plate is carried low to the tank's left side, lifted there, and
moved over the tank.

## The items, and where the numbers come from

| Item | Value used | Source |
|---|---|---|
| TLC plate | MilliporeSigma 1.15327, silica gel 60 F254 on glass, 2.5 x 7.5 cm, 250 um layer | maker |
| Glass thickness | 2.0 mm | estimate: measure with a caliper |
| Plate mass | 9.7 g | estimate: weigh one |
| Spotters | 1.5 mm OD x 0.86 mm ID x 100 mm borosilicate, standard wall, no filament (Warner Instruments 30-0053) | maker |
| Vials | 4 mL glass, 15 x 45 mm, neck opening about 8.5 mm | estimate: any 4 mL screw-thread vial; caliper |
| Samples | 1 mL water with two drops of food colouring (red, blue, green): non-hazardous | task design |
| Developing tank | Sigma-Aldrich Z146226 rectangular TLC developing tank, complete (tank and lid): 12.1 x 10.8 x 8.3 cm (L x H x W) | maker |
| Tank glass | walls 3 mm, floor 4 mm (so 115 x 77 mm inside); 400 g | estimates: caliper and scale |
| Lid | flat glass, 125 x 87 x 4 mm, 105 g | estimate: caliper and scale |
| Solvent | water, 5 mm deep (44 mL) | task design (water in real trials) |
| Rack, vial block | foam blocks, 30 x 110 x 25 mm with 5 mm holes 20 mm deep and 30 x 120 x 20 mm with 16 mm holes | task design; any rack that holds them upright |
| Pad covers | nitrile finger cots over each pad | task design (keeps dye and glass off the pads) |

### Estimates, and how to measure them

| Value | How |
|---|---|
| Spot size against contact time (sqrt(1 + 16 t) mm) | A person touches a filled spotter to a plate ten times for 0.1 to 1 s (counted on video), photographs the plate beside a ruler and fits the curve; adjust `SPOT_D0` and `SPOT_GROWTH`. |
| Light touch band (0.05 to 1 N) and the gouge threshold (1 N) | Press a filled spotter on a plate standing on a kitchen scale; note the reading at which the layer first shows bare glass under a 10x loupe. |
| Break force (3 N) | Press spare spotters sideways at their tip on a kitchen scale until they break (from 50 mm of free length). |
| Plate and pad friction; the pinch | Pinch the plate's top end in the gripper as the demonstration does and check that it swings down and stays; tilt the bar with the plate on it and read the slip angle. |

## The stated rules (no liquid or silica is simulated)

All in `rdb_tlc_spotting/tlc.py` (pure Python), fed every 20 Hz control step by the adapter from the bodies' poses
and contact forces.

- **Filling.** A spotter takes up sample j when its tip has been at least 2 mm under that sample's surface, inside
  the vial's bore, for 0.25 s (5 steps). A spotter wetted in a second sample is mixed, and so is that vial.
- **Spotting.** A touch is a run of steps on which the spotter's contact force on the plate (read from PhysX, the last
  physics substep of each step) is at least 0.05 N with its tip on the silica face. A filled spotter makes a spot only
  once a touch has lasted 0.1 s within 0.05 to 1 N; the spot is centred where the touch began and its diameter grows
  as sqrt(D0^2 + 16 mm^2/s x t), D0 = 1 mm, with every step in the band (2.2 mm after 0.25 s, 3 mm after 0.5 s). A
  later touch within 1 mm of a spot of the same sample grows that spot.
- **Gouging.** A touch pressing harder than 1 N for 0.1 s (two control steps) scrapes 1.5 mm of layer off; a
  single-step spike as the tip lands is an impact of a few milliseconds and does not count. A tip sliding more than
  1 mm while it presses scrapes a track that long. Bare glass longer than 1 mm within 3 mm of a tick fails that
  lane's check (c).
- **Breaking.** More than 3 N on a spotter from anything but the grippers' pads (net contact force minus the pads' and
  the plate's) or from the plate breaks it: score 0, and the trial stops.

The scene changes nothing because of these rules: spots, bare glass and the sample in a spotter's tip are only drawn
(hidden marks switched visible by `skin.py`).

## Scene (simulation)

`rdb_tlc_spotting/tlc.usda`, generated by `tools/build_assets.py`:

- Loose rigid bodies (`/Fixture/Actors`), moved only by contact and gravity: the plate (one 75 x 25 x 2.25 mm box
  collider, 9.7 g), the three spotters (1.5 mm square colliders 100 mm long, 2 g each; a round tube would roll between
  flat pads, and the real 0.26 g makes the pinch jitter), the tank (a floor and four walls, boxes; 444 g with its
  water, centre of mass low) and the lid (one box, 105 g).
- Fixed colliders (`/Fixture/Static`): the bar, the rack and the vial block (boxes around their holes) and the vials
  (12 wall segments, a narrower neck, a floor). The samples and the tank's water are drawn only. The foam blocks are
  rigid here: real foam gives, so a spotter pushed sideways in its rack hole meets more force in simulation than it
  would on the bench (the 3 N break rule counts that force).
- Contact offsets 1 mm (0.5 mm on the spotters), rest offsets 0; friction 0.5 to 0.6 on glass, 0.5 on the bar, 0.8 on
  the foam (combined by averaging with the pads' default 0.5).
- See-through glass: the tank's panes, the lid and the vials are drawn with Isaac Sim's `OmniGlass` MDL material,
  `thin_walled` on, and the renderer runs with `/rtx/translucency/enabled = true` (`recording.py`, `RT2`, used by the
  overhead, wrist and review cameras alike). With the beaker's first look (UsdPreviewSurface, opacity 0.1 to 0.25) the
  glass rendered as polished metal and hid the plate; the setup and end frames of the tank probes show the plate
  through the walls.

## Scoring

`grading.py` (pure Python) scores every step; the final score is the last step's. Twelve checks, one twelfth each:

- Per lane k: (a) exactly one spot within 3 mm of tick k, of sample k, covering the tick's crossing with the
  baseline; (b) that spot is at most 3.0 mm across; (c) no bare glass longer than 1 mm within 3 mm of the tick.
  (b) and (c) count only with (a).
- Plate: (d) inside the tank, its top end up, its bottom edge under the water's surface and the baseline's lowest
  point at least 3 mm above it; (e) its top edge touching the back wall (within 2.5 mm), its spotted face away from
  that wall, its length at most 35 degrees from vertical; (f) the lid centred within 6 mm of the tank's middle,
  level within 4 degrees, square to the tank within 8 degrees and resting on the rim (within 3 mm). (e) and (f) count
  only with (d).
- 1.0 also needs all three spotters standing in rack holes (tip within 3 mm of a hole's axis and 4 mm of its floor,
  within 12 degrees of vertical) and both grippers' centre points at least 50 mm from the tank and the lid; otherwise
  a 12/12 state scores 11/12.
- Latched 0, and the trial stops: a spotter broken (above) or dropped (more than 60 degrees from upright while no
  gripper holds it, that is while the pads squeeze it with less than 1 N in total, or its tip under 3 mm above the
  table outside the rack and the vials); the plate dropped (lying more than 60 degrees from upright with its silica
  face down and a corner on the bench outside the tank, where the spots smear, or any corner more than 10 mm below
  the table top, off the bench; a plate lying face up is not a zero, it just scores no plate checks until it stands
  in the tank); the baseline under the water while the plate is in the tank; the tank tipped past 10 degrees or
  moved more than 5 mm.
- The simulated trial ends 1 s after 1.0 is reached, at a zero, or at 4,800 steps; a real trial ends when both arms
  are back and still for 1 s, at a zero, or at the run limit. Possible scores: 0, 1/12, ..., 11/12, 1.0.

In a real trial (task.json) a phone on a stand outside the arms' reach films the plate on its bar throughout, zoomed so
the plate fills the frame, and is not a robot input. (a) and (b) are read from its last frame before the plate is
lifted, against the 6 mm tick spacing (a spot at most 3 mm across is at most half of it), because the spots start to
move once the plate is in the water; (c) with a 10x loupe on the dried plate after the trial; (d) to (f) from a side
photo of the tank with a ruler and a protractor; the rest from the trial video. The tank's start is a tape outline of
its footprint, so a shove of more than 5 mm can be measured from it.

Why 35 degrees: the chamber and the solvent set the angle. A 75 mm plate standing on the tank's floor and resting on
its back wall leans asin(d / 75) from vertical, d being how far its bottom edge stands from the wall (the tank is 77 mm
deep inside, so any d up to 75 mm is possible); with 5 mm of solvent the baseline, 10 mm up the plate, stays at least
3 mm above the solvent up to 37 degrees (10 mm x cos 37 = 8 mm). A plate leaning on the chamber wall is how it stands
in a jar or tank chamber.

Unintended ways through, each tested in `tests/test_grading.py` or `tests/test_rules.py`:

| Way | What happens |
|---|---|
| Brush the tick, or press too briefly (under 0.1 s) | no spot |
| Press long (0.7 s) | spots over 3 mm: (b) fails in every lane |
| Press hard (over 1 N for 0.1 s) or drag the tip | bare glass: (c) fails |
| Fill from the wrong vial, or dip one spotter in two samples | wrong or mixed sample: (a) fails |
| Touch twice in different places | two spots in the lane: (a) fails |
| Stand the plate facing the wall, or upside down | (e) fails, or (d) fails |
| Shove the tank with the hand or the plate | 0 |
| Put the plate down face up on the bench, or carry a spotter on its side in the hand | no zero; nothing scored for it |
| Put the plate down face down, or let a spotter go lying on the bench | 0 |
| Leave a spotter out of the rack, or a hand over the tank | at most 11/12 |

## The scripted demonstration

`scripted.py`, closed loop on simulator state (bodies' poses, contact forces, what the rules recorded), as the Guide
allows for scripted demonstrations; it moves nothing itself.

How it stands the plate up (one way among others; a policy may do it any way it likes): it pinches the plate by its
two long edges at its very top, so that the plate can turn in the pinch. Lifting the hand lets the bottom end drag
along the bar and the plate swings down to hang from the pads, 14 to 32 degrees off vertical (the pinch's friction
holds it there; a lighter pinch lets it slide out). It then stands the plate's bottom edge on the bench just behind
the bar's far end and moves the hand toward the robot: the bar's end face stops the bottom edge and the plate turns
nearly upright in the pinch. The pads (26.4 mm wide) stand out 23 to 30 mm behind the plate's top, on its glass side,
so the plate cannot be held against the wall: the demonstration stands it on the tank's floor 35 to 37 mm in front of
the back wall, 4 degrees back from upright, and lets go, and it settles back against the back wall at 27 to 28
degrees.

1. Left hand, for k = 1, 2, 3: from 60 mm over spotter k, pads 20 mm apart, down until the pad tips are 50 mm above
   its tip, shut the pads (1.5 mm between them); lift 40 mm, forward along the rack's row; over vial k with the tip
   lined up on the neck (from the spotter's pose), into it slowly until the tip is 4 mm under the surface, 0.4 s; up;
   over tick k, down to 3 mm and then 1 mm above the silica, settling and lining the tip up on the tick each time, then a
   quick dab, the procedure's "quick up-and-down": the command goes 2.2 mm down in 0.1 s (the tip lands and the arm
   gives about 1 mm); while the tip is down the command is nudged 0.25 mm lower whenever there is no contact force
   and 0.15 mm higher above 0.7 N, and the hand lifts 12 mm as soon as the spot (read from the rules' state) is 2.3 mm
   across (spots end about 2.6 mm). Back along the rack's row over hole k, lined up 3 mm above the rack, down into it at
   10 mm/s (stopping and lining up again if the tip catches on the rack), open, up.
2. Right hand: pads 35 mm apart over the plate's top end, so that only its last 7 mm will be between them, pad tips
   6 mm under its lower face; close to 1 mm narrower than the plate (about 3 N on the pads); lift 90 mm: the plate
   swings down. Over the bench just behind the bar's far end, down until the plate's bottom edge stands on the bench
   4 mm behind the bar, then the hand moves toward the robot at 8 mm/s: the bottom edge slides into the bar's end face
   and stops, and the plate turns in the pinch; the hand stops when the plate is 4 degrees short of upright (leaning
   away from the bar: the bar's top edge stays 1 mm off the silica, the spots 0.9 mm). Lift it: the pinch keeps the
   angle.
3. Low to the tank's left side, up to 190 mm, over the tank with the plate's bottom edge 31.7 mm plus 75 mm times the
   sine of its tilt (35 to 37 mm) in front of the back wall (the pads, which stand out 30 mm behind the plate's top,
   then clear the wall by about 2 mm), down until it stands on
   the floor, press 1 mm, open the pads 2 mm and lift: the plate settles back against the back wall.
4. Over the lid, turned so the pads close along x, pads fully open (93.9 mm), down until the pad tips are 1.5 mm above
   the bench, close across its 87 mm width (2 mm squeeze), lift 130 mm, over the tank, lined up, down until it is
   1.5 mm above the rim, open: it drops onto the rim. Both hands back to the ready pose; the trial ends 1 s after 1.0.

Variants (`RDB_TLC_DEMO`): `plate_only`, `spot_only`, `one_lane` (probes), `press_hard` (lane 2 pressed past 1.5 N:
gouged), `no_dip` (touches without filling: no spots).

## Native runs

On one NVIDIA A10 (24 GB), Ubuntu 22.04, Isaac Sim window shown (`--live`), seed 0, the GPU often shared with other
engineers' Isaac Sim processes (about 0.45 s a step shared, 0.25 to 0.3 s alone). `out/` holds each run's
`RESULT.json`, scorer trace, videos and stills.

With the final code (the instruction, rules and descriptions as shipped; hashes in `validation.json`):

| Run | What | Result |
|---|---|---|
| `out/scripted-11` | full (the recorded run) | 1.0 in 3,033 steps (success, 1428 s); spots 2.57, 2.57, 2.57 mm, 0.26, 0.39, 0.9 mm off their ticks; plate leaning 28.17 degrees, baseline 4.88 mm above the water; lid 1.43 mm off centre |
| `out/scripted-12` | full, the same code again | 1.0 in 3,033 steps (success, 1421 s); spots 2.57, 2.57, 2.57 mm, 0.26, 0.39, 0.9 mm off their ticks; plate leaning 28.17 degrees, baseline 4.88 mm above the water; lid 1.43 mm off centre |
| `out/control-press_hard-2` | control `press_hard`, capped at 2,000 steps (after all three touches) | lane 1: a 2.57 mm spot 0.26 mm off its tick (3/3); lane 2: a 2.05 mm spot 0.94 mm off its tick, gouged (2/3); lane 3: a 2.57 mm spot 0.91 mm off its tick (3/3); plate checks 0/3; 8/12 at the cap (the full demonstration: 9/12 at that step) |
| `out/control-no_dip-2` | control `no_dip`, capped at 2,000 steps (after all three touches) | lane 1: no spot (0/3); lane 2: no spot (0/3); lane 3: no spot (0/3); plate checks 0/3; 0/12 at the cap (the full demonstration: 9/12 at that step) |
| `out/dream` | Dream-1 (`dreamlabs/dream-1-beta`), headless, the GPU not shared | 0.0: voided at step 4,770 of 4,800 (capillary 1 dropped), 1,333 s, 177 Dream-1 requests |

**The Dream-1 trial** (`out/dream/`; `validation.json`, `runs.dream`): `dreamlabs/dream-1-beta` through `inspect-robots-dream` (commit b5e936a2) with `dreamlabs-client` 1.0.14, on the final code (its hashes equal the recorded run's), with the same scene, instruction, scorer, cameras and 20 Hz control as the demonstration; 177 action requests (cap 300; our list-price estimate US$0.24). For about 4,700 steps it moved both arms over the bench, opening and closing both grippers, and touched nothing: the plate, the lid, the tank and all three capillaries stayed where they started. From about step 4,730 the left gripper pinched capillary 1 (up to 3.5 N on its pads) and lifted it from its rack, its tip about 20 mm up by step 4,760; the pinch then slipped (no pad force by step 4,770) and the capillary tipped more than 60 degrees from upright with no gripper holding it, which the rules count as dropped: the trial was voided at step 4,770, 0 of 12. No capillary was filled and no spot was made; no force above 0.31 N reached any capillary from anything but the pads. The combined video `out/dream/model_run.mp4` (2560 x 704, 10 fps, 238.6 s) was made with the same ffmpeg command as `demo.mp4`; the images Dream-1 received are in `dream-input-rgb.zip` and its replies in `dream-action-trace.jsonl`. An earlier Dream-1 trial on the code before the changes of 20 November (`out/dream-oldcode/`, the code of `out/scripted-10`) also scored 0.0: voided at step 2,022 when capillary 1 took 3.6 N from something other than the pads; it is kept for the record only.

Runs 11 and 12 have byte-identical scorer traces (the scorer's state at each of the 3,033 steps), the same final grade and the same force peaks; so do runs 10, 9, 8, made before the changes below. The controls ran with the same code and hashes as the recorded run; their
traces are byte-identical to those of `out/control-press_hard` and `out/control-no_dip`, made before the changes.

Before the changes of 20 November (the instruction did not yet state the left-to-right pairing, a spotter past 60 degrees counted as dropped even in a hand, a plate lying face up was a zero, and `adapter.py` and `tlc.py` had two stale descriptions), with the same scene, physics and demonstration:

| Run | What | Result |
|---|---|---|
| `out/tank-1` | plate_only; loosen the pinch from 1.0 to 0.8, 0.6, 0.4 mm while shaking the hand +-3 mm | the plate slid out at the first loosening: dropped, 0 |
| `out/tank-2` | the same without shaking | slid out at 0.8 mm too: a pinch lighter than about 1 mm does not hold the plate, so the hang cannot be made straighter that way |
| `out/tank-3` | plate_only, straightened against the bar's end face | 31.6 degrees off upright to 3.5, lifted at 3.8; then carried too close to the tank's side wall and pushed the tank: 0 |
| `out/tank-4` | the same, carried clear of the tank's side | stood 35 mm in front of the back wall at 4.4 degrees, let go: leaning 26.9 degrees on the back wall, face out, baseline 4.9 mm above the water, tank unmoved; lid on 1.3 mm off centre, level, square: 3/12 (all three plate checks) |
| `out/scripted-10` | full (the recorded run until the changes) | 1.0 in 3,033 steps (success); spots 2.57, 2.57, 2.57 mm, 0.26, 0.39, 0.9 mm off their ticks; plate leaning 28.17 degrees, baseline 4.88 mm above the water; lid 1.43 mm off centre |
| `out/scripted-9` | full, the same trial earlier (older docstring in scripted.py) | 1.0 in 3,033 steps (success); spots 2.57, 2.57, 2.57 mm, 0.26, 0.39, 0.9 mm off their ticks; plate leaning 28.17 degrees, baseline 4.88 mm above the water; lid 1.43 mm off centre |
| `out/scripted-8` | full, the same trial earlier (older docstring in scripted.py) | 1.0 in 3,033 steps (success); spots 2.57, 2.57, 2.57 mm, 0.26, 0.39, 0.9 mm off their ticks; plate leaning 28.17 degrees, baseline 4.88 mm above the water; lid 1.43 mm off centre |
| `out/control-press_hard` | control `press_hard`, capped at 2,000 steps (after all three touches) | lane 1: a 2.57 mm spot 0.26 mm off its tick (3/3); lane 2: a 2.05 mm spot 0.94 mm off its tick, gouged (2/3); lane 3: a 2.57 mm spot 0.91 mm off its tick (3/3); plate checks 0/3; 8/12 at the cap (the full demonstration: 9/12 at that step) |
| `out/control-no_dip` | control `no_dip`, capped at 2,000 steps (after all three touches) | lane 1: no spot (0/3); lane 2: no spot (0/3); lane 3: no spot (0/3); plate checks 0/3; 0/12 at the cap (the full demonstration: 9/12 at that step) |

Runs 8, 9 and 10 have byte-identical scorer traces (the scorer's state at each of the 3,033 steps), final grades
and force peaks; run 10 used the shipped `scripted.py`, runs 8 and 9 one whose module docstring was older (no code
line differs). The rendered frames are not bit-identical from run to run (the renderer's sampling noise: on average
0.5 to 0.7 of 255 per pixel, at most 29 on a few pixels of the stills), and the wall time follows the GPU's other
load. The two controls stop at 2,000 steps, after all three touches; up to there they run the same code as the full
demonstration except for the one change each makes. In `press_hard` lane 2's tip was pushed to 1.85 N, and the
scorer recorded a 1.5 mm gouge (`why: pressed`) and took that lane's no-bare-glass check. In `no_dip` the empty
spotters rested on their ticks at 0.2 to 0.5 N for about 0.8 s each, inside the touch band, and left no spot; lane
3's landing also had a single-step 1.7 N spike, which is shorter than the 0.1 s a gouge needs.

Earlier, with a 150 mL beaker and a 75 mm watch glass as the chamber (replaced by the tank: in the round beaker the
pads, standing out behind the plate, kept meeting the rim):

| Run | What | Result |
|---|---|---|
| `out/plate-1` | plate_only | crashed at step 1 (float32 values in the trace; fixed) |
| `out/plate-2`, `out/probe-3` | plate_only, pinch squeeze 0.12 mm | the pads never gripped the plate; the watch glass was set on the beaker (0.36 mm off its axis) |
| `out/probe-4` | plate_only, 7 mm overlap, squeeze 0.2 mm | 0.03 N on the pads: the plate slid out |
| `out/probe-5` | plate_only, squeeze 1 mm | the plate swung to 16 degrees and hung there; lowered tilted, the fingers hit the beaker's back rim (beaker moved 10 mm) and the plate fell forward facing the wall: 1/12 |
| `out/probe-6` | plate_only with squaring up on the bar | upright within 3 degrees, released in the beaker: leaning 22.4 degrees, face out, baseline 5.1 mm above the water, beaker moved 0.02 mm (capped at 860 steps before the watch glass was down) |
| `out/probe-7` | spotting, fast moves | the arm wandered 3 mm sideways while lowering the spotter fast into vial 1: its tip hit the neck (3.7 N), spotter broken, 0 |
| `out/probe-8` | spotting, slow press (2 mm/s, stop at 0.05 N) | filled; the slow press landed 1.2 mm off the tick with a 1.3 N one-step spike (counted as a gouge then); the returned spotter caught the rack's edge (3.0 N): 0. Since then: 5 mm holes, a slower, re-aligned entry, and a gouge needs 1 N for 0.1 s |
| `out/scripted-1-killed` | full, press to 0.05 N at 1 mm/s | no spot in lane 1; stopped by hand at step ~950 (no RESULT.json) |
| `out/probe-10` | one lane, press to 0.15 N | the tip landed with a 0.98 N one-step spike and then rested 0.06 mm off the silica (stick-slip): no spot. Replaced by the dab |
| `out/scripted-2` | full, the dab without the contact keeper | lanes 1 and 2 spotted (2.24 and 1.84 mm, 0.27 and 0.47 mm off); in lane 3 the tip bounced off after a 1.8 N landing and stayed off: the demonstration stopped itself (PolicyError) at step 1,830 |
| `out/scripted-3` | full, the dab with the contact keeper and a fixed hold | lane 1: 2.72 mm, 0.26 mm off; lane 2: 3.49 mm (0.7 s in contact: the keeper's nudges kept it down too long), over the 3 mm circle: the demonstration stopped itself at step 1,181. Since then the dab lifts once the spot reads 2.3 mm |
| `out/scripted-4` | full, the dab lifting at a 2.3 mm spot | all three lanes right (2.57 mm spots, 0.26, 0.39 and 0.90 mm off their ticks, no gouge, landing peaks 1.38, 0.73, 0.36 N), spotters back in the rack, 11/12 at step 2,725; but squaring up, the plate's bottom edge slid 47 mm along the bar instead of the plate turning, it went in 9 degrees off upright, the pads met the beaker's rim on the release and pushed it more than 5 mm: 0 (the shove rule). Since then the square-up presses 3 mm, turns in steps of at most 8 mm and re-centres and repeats if the plate is still off upright |
| `out/scripted-5` | full, square-up pressing 3 mm in small steps, retried | spotting identical to scripted-4 (the same trace to step 2,100: the glass's new look changes nothing in the physics); the square-up slid along the bar again in all three rounds (plate 13 to 16 degrees off upright); lowered into the beaker it stood upright on the floor by itself as the hand came down, but the hand was 14 mm behind the axis and the pads pushed the rim: 0 (the shove rule), 10/12 before. Since then: no square-up; the hand goes in 5 mm behind the axis and the floor stands the plate up |
| `out/scripted-6` | full, no square-up; the hand 5 mm behind the beaker's axis, the floor to stand the plate up | spotting identical (9/9); the plate hung 22 degrees off after the swing and swung to 29 degrees while carried, so with the hand 5 mm behind the axis its bottom edge came down on the beaker's front rim and pushed it: 0 at step 2,438 (the shove rule) |
| `out/scripted-7` | full, plate stood on the bar and the hand brought straight down | spotting identical (9/9); coming straight down slid the plate's bottom edge forward along the bar and laid the plate further over (56 degrees off upright), and in the beaker it pushed the rim: 0 at step 2,846. At that point the plate-up step was not yet solved for full runs (it worked in the plate-only probe 6, side moves over the bottom edge, and failed with the same code in full runs 4 and 5); the tank runs above stand the plate up against the bar's end face |

## Tests and dry runs

```sh
python3 -I -m unittest discover -s tests      # standard library only
python tools/dryrun_demo.py                   # needs numpy and yam_arena (no Isaac Sim)
python tools/build_assets.py                  # regenerate the scene file from layout.py
```

- `test_rules.py` (filling, spotting, gouging, the plate in the tank, the lid, the rack), `test_grading.py` (the
  twelve checks, the 11/12 cap, every zero condition, that a held spotter past 60 degrees and a plate lying face up are
  not zeros, the end of the trial), `test_scene.py` (the scene file is what
  the builder writes; bodies, colliders, see-through panes and marks), `test_package.py` (task.json limits and
  numbers against the code, README, validation.json and its hashes, no identifying text), `test_demo.py` (the dry run,
  skipped without yam_arena).
- `tools/dryrun_demo.py` drives the real demonstration through the arms' kinematics with a lag; spotters, the plate
  and the lid follow a closed hand; a spotter pressed on the plate stays on the silica while the arm gives (300 N/m);
  the plate hangs upright from the pads and on release falls back to the back wall. It checks reach, joint limits, the
  control flow and the step count, not contact. Final code (rerun after the changes of 20 November): full 1.0 in
  2,547 steps; `press_hard` 10/12 (lane
  2: a 4.4 mm spot and bare glass); `no_dip` 3/12 (no spots). Moving the plate's bottom edge behind the bar's far
  end runs the right arm's joint 4 to its limit (the dry run's tool point is then 4.3 mm off its target); in
  Isaac Sim the plate still came down 0.6 mm from the bar's end in every run.

## What has not been shown

- Dream-1 on more than one trial of the final code: it was run once (the scene is fixed and the seed is 0; its replies
  may still differ between calls).
- No real items were bought, so none of these checks was done: spotting dyed water from a 30-0053 capillary on a
  1.15327 plate ten times, 0.1 to 1 s each, photographed beside a ruler, to fit `SPOT_D0` and `SPOT_GROWTH`; standing a
  plate in the tank with 5 mm of water at 28 and at 35 degrees, five times each, to see whether it slides on the wet
  glass (a uniform ladder on a floor with friction f against a wall with friction w stays put while
  f >= tan(lean) / (2 + w tan(lean)): about 0.24 at 28 degrees with w = 0.4, 0.35 at 35 degrees with w = 0); pinching a
  capillary in the gripper with the finger cots on, ten times, to see whether it cracks. Every value marked as an
  estimate is therefore unmeasured: the touch, gouge and break forces, the spot growth, the glass thicknesses and
  masses, and the pinch that lets the plate swing yet holds it (in simulation a squeeze 0.2 mm lighter dropped it).
- Repeatability on other GPUs.
- That a policy can see the 2 mm ticks: at 320 x 176 the overhead camera gives about 0.5 pixel per mm there; the wrist
  cameras see them closer.

## Text and code that disagreed (fixed or noted)

- `adapter.py` still described the chamber as "a 150 mL glass TLC developing tank" (from the beaker version); it now
  says what the scene holds, the rectangular tank. Fixed before the final runs.
- `tlc.in_rack`'s docstring said 1.5 mm; the code accepts a tip within 3 mm of a hole's axis and no more than 4 mm above
  its floor. The docstring now says so; the code did not change.
- `skin.py` draws each spot centred on its tick (display only); the scored spot is where the touch began, 0.26 to
  0.9 mm off the tick in the demonstration (`RESULT.json`, scorer trace). Not changed.
- The `out/` folders are not in every copy of this package; `validation.json` summarises the recorded runs, and the
  team keeps every run for reference.

## Third-party assets and licenses

- All task geometry is generated by `tools/build_assets.py` (boxes, convex hulls and drawn surfaces of revolution); no
  meshes, textures or images from any third party. Product names identify what was modelled. The drawn glass (tank,
  lid, vials) references `OmniGlass.mdl`, a material that ships with Isaac Sim (NVIDIA Omniverse license); it is not
  bundled and only changes how the glass looks.
- Robot, station and cameras: `yam-arena` (its README states no license yet; the YAM arm model in it is MIT). Not
  bundled.
- Isaac Lab-Arena and Isaac Lab: Apache 2.0. Not bundled.
- This folder's code and scene: MIT, see `LICENSE`.

## Sources

- TLC procedure: [Nichols, Organic Chemistry Lab Techniques, 2.3E Step-by-Step Procedures for TLC](https://chem.libretexts.org/Bookshelves/Organic_Chemistry/Organic_Chemistry_Lab_Techniques_(Nichols)/02%3A_Chromatography/2.03%3A_Thin_Layer_Chromatography_(TLC)/2.3E%3A_Step-by-Step_Procedures_for_Thin_Layer_Chromatography)
  ("Obtain a TLC chamber with lid. An inexpensive chamber can be made using a beaker and watch glass"; baseline about
  1 cm up, lanes at least 5 mm from the edges, spots about 2 mm with a quick up-and-down touch, don't gouge, forceps
  into the chamber, liquid below the pencil line, cap the chamber).
- Developing tank: [Sigma-Aldrich Z146226, rectangular TLC developing tank, complete](https://www.sigmaaldrich.com/US/en/product/aldrich/z146226)
  (tank and lid; L x H x W 12.1 x 10.8 x 8.3 cm; no technical drawing available from the maker).
- Plate: [MilliporeSigma 1.15327 / 1.15341](https://sigmaaldrich.com/CA/en/product/mm/115341) (glass, 2.5 x 7.5 cm,
  250 um layer).
- Spotters: [Multi Channel Systems 30-0053 (Warner Instruments)](https://shop.multichannelsystems.com/30-0053.html),
  borosilicate standard wall without filament, 1.5 mm OD, 0.86 mm ID, 100 mm.
