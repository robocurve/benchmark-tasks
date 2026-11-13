# Find a wall stud with a magnetic stud finder

A magnetic stud finder stands in a holder in front of a painted stud-wall section. The robot slides the finder
flat over the wall until its magnets pull it onto a drywall screw hidden under the paint, then lets go so the
finder hangs on the wall by itself, marking the stud. Where the studs and screws are changes from trial to trial.

## Run and score

With the official simulator installed, run from this folder at 20 Hz:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=find_stud_with_magnetic_stud_finder.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 \
  --instruction "Find a stud behind the wall with the magnetic stud finder in its holder. Keep its back flat on the wall and slide it slowly over the wall until its magnets pull it onto a hidden screw, then let go so it hangs there." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ./demo-run
```

The model run uses the same command with `--policy dream`. Encode recorded cameras with
`inspect-robots video <run.json>`. Scores are in task.json.

**Seeds and layouts.** The trial's seed picks the wall section: seed mod 4 (sections 0 to 3; with no seed,
section 0). Only the hidden screws move; every section looks the same. Inspect Robots derives each trial's seed
from `--seed` and the epoch: `--seed 1`, `3`, `0` and `2` give sections 0, 1, 2 and 3 in their first epoch
(`--seed 0 --epochs 2` gives sections 2 then 0). With nonzero object jitter the finder's start in its holder also
shifts; the demonstration reads where it stands.

**CPU checks.** `python -m pytest -q -p no:cacheprovider` in this folder (with yam_arena installed) runs the scorer
on the start, the goal and named partial and cheat layouts; `test_red_team.py` checks the magnet law against its
integral, the hang on a vertical wall, the layouts, the 1 s windows over control steps, and the demonstration's
search on all four sections.

Runtime per trial: at most 3,600 control steps (180 s simulated). The demonstration lets go at 13 to 22 s, reaches
full score at 17 to 28 s and its plan ends at 21 to 30 s, depending on the section (a second, lower pass is needed
on sections 0 and 2).

## Scoring (task.json has the full text)

- On a screw: the back within 2 mm of the wall's face plane at four points beside the two badges, and a badge's
  centre (a magnet's centre) within 10 mm of a screw's axis in the wall's plane.
- 1.0 hanging: on a screw, lowest point 20 mm or more above the table, no gripper TCP within 30 mm of the finder's
  box for 1 s, and the finder moved under 1 mm and 2 degrees over the last second (pose samples each step).
- 0.5 on a screw but held or not yet still; 0.25 held flat on the wall but not on a screw; 0 otherwise.
- The scorer reads only poses, the grippers' TCPs and the trial's section; no contact forces.

## The magnet's pull (`magnet_law.py`)

The maker does not publish the magnets' size, grade or pull, so the force is computed from stated geometry:

- Magnets: N42 neodymium discs, 1/2 in across and 1/8 in thick (12.7 x 3.175 mm, remanence 1.30 T), one behind
  each badge, 25 mm above and below the finder's centre, their faces 1.0 mm behind the back (plastic skin).
- Field: a uniformly magnetised disc's field is that of its two faces carrying surface charge Br/mu0, summed as
  rings with closed-form ring fields (elliptic integrals); the test checks it against direct integration and the
  on-axis formula.
- Screw: #6 x 1-1/4 in coarse-thread drywall screw, bugle head 8.3 mm across, driven 1.0 mm under the face and
  covered with compound and paint, so a magnet on the wall is 2.0 mm from the head. The head is a soft-steel pole
  over its top face; the flux it carries is capped by saturation of the shank's 2.4 mm root (2.0 T), and below
  that it draws in twice the magnet's own flux through the face. The pole's sign follows the field, so the pull is
  always toward the magnet.
- Result at a 2.0 mm gap: 1.49 N onto the wall straight over the screw, 1.07 N at 4 mm off its axis, 0.62 N at
  6 mm, nothing past 9 mm; sideways toward the screw's axis 0.38 N at 2 mm off, 0.77 N at 6 mm. At 3 mm gap the
  straight pull is 1.16 N; 8 mm away, 0.21 N.
- Hang check (68 g, 0.67 N): back-on-paint friction 0.40 times the pull plus the sideways pull holds up to 1.11 N
  (x1.67 the weight; x1.52 at friction 0.30; x1.45 if the gap is 2.5 mm). The finder settles under 1.5 mm below
  the screw. Retail listings say the finder allows hands-free use; reviewers report it hangs on the stud unheld.
- In the simulation the force of each magnet on each screw is applied to the finder at the magnet before every
  physics step (`stud_runtime.py`). Nothing else is written to the simulation.

**Must be measured on the real finder:** the pull straight off a screw at this wall (spring scale), the sideways
force to slide it off, the friction of its back on the painted wall, its mass and outline, and where its magnets
sit. Set `MAGNET_D`, `MAGNET_T`, `MAGNET_BR`, `SKIN` and the friction to match.

## Physical setup

- Wall section (made for the trial): 1/2 in drywall 660 x 230 mm, face at x = 255 mm, flat white; pine 2x4 bottom
  plate laid flat; 1/2 in plywood cap painted white; two 2x4 studs 406 mm apart, one screw in each on its centre
  line. Sections (left stud y, screw height): 0: 190, 114; 1: 110, 126; 2: 150, 114; 3: 230, 126 mm; the right
  stud 406 mm to the right. The studs are hidden and never touched, so they are not simulated as bodies.
- The section is a full-thickness, true-size piece of an ordinary stud wall (real drywall, 2x4 studs at 16 in on
  centre, real screws); only its height is cut, to 230 mm, which keeps its top under the arms' ready pose. The
  finder is used on its face as on any wall, and hangs wholly on the face, its top 38 to 50 mm below the wall's top.
- Stud finder: CH Hanson 03040 (rare-earth magnets, rotating two-way level, soft rubber body): body 82 x 32 x
  20 mm, side tabs 38 mm across, pointer loop to 90 mm, 68 g. Colliders: a waist box, two 16 mm-radius lobes,
  six side tabs, two badges, the level ring and vial, the pointer (`FINDER_SHAPES`).
- Holder (made for the trial): grey ABS block 70 x 80 x 34 mm with a 25 x 44 x 28 mm slot; the finder stands in it
  on its pointer, back toward the wall.

## Demonstration and runtime

The left arm lifts the finder by its middle side tabs with the hand pitched 22 degrees about the finger axis, its
top leaning back toward the robot (the YAM's fingers taper toward their tips; at this pitch the whole hand stays
3 mm or more in front of the wall's face and the pads' flat faces cover about 90% of the tabs' sides). It presses
the finder's back lightly onto the wall and sweeps it along the wall at about 26 mm/s in two passes 13 mm apart.
The scripted demonstration reads simulator state, as the program allows: where the finder stands, where it sits in
the hand, and where this trial's screws are, which decides where it stops (a person feels the tug instead). Where
the lower magnet comes over a screw it stops and eases its press. The YAM's two fingers are driven separately
toward one target, and the finder slides about 15 mm along the finger axis as it is lifted; opening around an
off-centre grip, the far finger would push the finder that far along the wall, off the screw. So, as the fingers
open, the hand moves along the finger axis by the finder's offset in the hand (read then), and the finder stays
put. It then backs straight away from the wall and parks; the magnets hold the finder. `DEMO_PACE` scales the
demonstration's speed; nothing else depends on it. Diagnostic `STUD` lines are printed for probes; the scorer does
not read them.

## Fidelity and approximations

Rigid bodies throughout. The finder's outline is from the maker's photographs and listings (3.5 x 1.5 in, 0.75 to
0.8 in deep, 0.15 lb). The magnets' size, grade and place are assumptions (above). The screws are points the
magnets pull toward; the studs, adhesive and compound are not simulated. The magnets' pull on the gripper or other
steel and paint scuffing are not modelled.

## Files and licence

`find_stud_with_magnetic_stud_finder.py` (scene, scorer, demonstration), `magnet_law.py` (the pull),
`stud_runtime.py` (layout by seed, the pull applied in the simulation), `yam_parts.py` (compound parts kit),
`test_red_team.py`. No third-party assets: every shape is built from boxes and cylinders in the task file.
