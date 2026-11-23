# Clipper-over-comb taper

A barber's training head faces away from the robot. The back of the head carries one continuous panel of hair, every hair 35 mm long: 8 cm from the nape upward and 70 degrees across (about 10 to 10.6 cm wide). The barber has already cut the hair around it: beside the panel to the taper's length for its height, and above it to 22 mm. The robot tapers the panel as a barber would, with a guardless clipper run over a comb: 8 mm at the bottom, getting longer evenly to 22 mm at the top, everywhere, blended into the hair around it with no lines. Then it puts the comb back in its rack and the clipper back on its holder.

## What the task tests

- **Two hands, two roles.** One hand holds the comb in the hair at the distance from the scalp that sets the length. The other runs the clipper along the comb's back. A clipper on its own either misses the hair, which lies flat, or shaves it to the scalp.
- **Covering a curved surface.** The head curves both across and up, so a comb held flat across the whole back leaves the sides several millimetres long. The work has to go section by section, and band by band up each section, with every pass overlapping the last. Any patch that is skipped stays 35 mm long, and every point of the panel is scored.
- **A real taper.** The length grows with height. The comb has to lean out from the scalp as fast as the taper grows. A comb held parallel to the scalp leaves a 2.8 mm step at every band, which scores as a line.
- **Irreversible cuts.** Any point under 5 mm is a bald patch, and the trial scores 0.
- **Safety.** A tool touching an ear pad scores 0.

## Running a trial

Put `clipper_comb.py` and `hingekit.py` in one folder, then:

```bash
inspect-robots run --embodiment yam_arena --policy <policy> \
  -E task=$PWD/clipper_comb.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Taper the long hair on the back of the training head with the clipper over the comb: 8 mm at its bottom edge, getting longer evenly to 22 mm at its top edge, blended into the hair around it with no lines. Then put the comb and the clipper back where they were." \
  --scorer task_score --max-steps 3600
```

- **Policy:** `--policy yam_scripted` runs the scripted demo (`demo()` in the task file).
- **Offline check:** `python clipper_comb.py` checks the demo plan's IK and replays its cut without physics, with the Isaac corrections below turned off.

## Scoring

The scoring is `score()` in `clipper_comb.py`, evaluated at the end of the trial. Cuts are tracked every control step by `monitor()`.

**Where the panel is measured.** The panel is scored on a grid of 896 points: every 2.5 degrees across and every 2.5 mm up. A point's length is how far its hair reaches from the scalp when combed straight out. In a real trial, comb each of the 28 vertical lines straight out, photograph it side-on against a millimetre grid card, and read the length every 2.5 mm up.

**The target.** The taper's length is 8 mm at the panel's bottom edge and rises evenly to 22 mm at its top.

**Lines.** A point is on a line when it differs from a neighbour (up, down or across) by more than 2 mm beyond the taper's own change between them. The taper itself changes 0.44 mm between vertical neighbours. The panel's side and top edges are checked the same way against the already-cut hair beside and above it, which is taken as exactly on the taper, so the panel has to blend into it.

| Point | Credit |
|---|---|
| Right (2 mm short to 3 mm long of the taper at its height) and on no line | 1 |
| Cut (at least 1 mm under 35 mm) but too long, or on a line | 0.25 |
| Uncut, or too short | 0 |

| Score | Condition |
|---|---|
| 0.0 | Any point under 5 mm (a bald patch), a tool within 1 cm of an ear pad, or the clipper cutting hair with no comb between the blade and the scalp (in a real trial, judged from the video; in simulation this shaves the hair to 1 mm, a bald patch) |
| 1.0 | All 896 points right and on no line, and both tools put back (the comb resting in or beside its rack, the clipper lying flat on or beside its holder) |
| 0.9 × average point credit | Everything else (0 to 0.9, rounded to 0.001) |

Example scores from scripted strategies (offline replay of the cut, no physics):

| Strategy | Score |
|---|---|
| Nothing done | 0 |
| The scripted demo | 1.0 |
| The demo, tools not put back | 0.9 |
| Comb held parallel to the scalp in each band (steps, not blended) | 0.725 |
| Every other band (gaps between passes) | 0.588 |
| Only the bottom three of five bands | 0.541 |
| Only the middle two of four sections | 0.438 |
| One side only | 0.426 |
| No taper: 15 mm everywhere | 0.399 |
| One comb placement across the whole back per band | 0.338 |
| Everything 3 mm too short | 0.329 |
| Reversed taper (22 mm at the bottom) | 0.263 |
| Everything 5 mm too long | 0.221 |
| Clipper without the comb, at the taper's distance | 0 (nothing cut) |
| Clipper without the comb, close to the scalp | 0 (bald) |

## The scripted demo

Both arms pick up their tool. The back is worked in four vertical sections, each 17.5 degrees wide, from right to left. Within a section, the work starts at the nape and moves up in five bands, 1.6 cm apart. The clipper's blade cuts 2 cm, so each band overlaps the one before. For each band:

1. The left arm sets the comb in the hair. The comb's back sits at the taper's length, and the comb leans out at the taper's rate.
2. The right arm runs the clipper along the comb's back across the section, alternating direction, its blade pressed on the comb's back (`FACE` = -0.5 mm).
3. The clipper backs off while the comb moves up to the next band.

Then both arms put their tools back.

**Isaac corrections.** In Isaac the arms hold the comb farther out from the head than planned when they reach low (about 4-5 mm at the nape, about 1 mm at the panel's top, the same in all four sections). The demo therefore aims the comb closer than the taper by `TRIM_LO` = 7.5 mm at the panel's bottom edge (plus `TRIM_SEC` = 0.8, 0, -0.6, -0.4 mm by section, right to left), fading evenly to `TRIM_HI` = 0 at its top, and `FACE_TILT` corrects how the hand holds the clipper. These only change where the scripted demo aims its tools; the scoring is unchanged. Each can be set from the environment (`CC_TRIM_LO`, `CC_TRIM_HI`, `CC_FACE`, `CC_FACE_TILT`).

## Runtime and seeds

- **Runtime:** the scripted demo takes about 110 s (about 2,200 steps), within the 3,600-step limit.
- **Seeds:** the layout is fixed, and `object_jitter=0` was used for all runs. The scoring is deterministic.

## Simulation notes

- **Hair.** Hair is not simulated. The panel is 896 point lengths. Each point is cut by rule where the clipper's blade face crosses its straight-out line, and never shorter than the comb's back if the comb crosses nearer the scalp, because the teeth shield the hair inside them. With no comb there, the hair lies flat, and the clipper shaves it to 1 mm within 6 mm of the scalp and otherwise passes over it.
- **How the hair is drawn.** The hair is drawn as single strands, about 25,000 on the head (about 10,000 in the panel), as USD `BasisCurves`, which the RTX renderer draws as fine tubes. Each strand grows from its own point of the scalp, leaves it more upright than it lies, bends down toward its tip and thins from 0.32 to 0.12 mm; strands are drawn thicker and sparser than real hair (about 100 per cm² in the panel) so they show on camera. Combed, the hair lies down the head. Each panel strand is drawn cut to the length of the scored point nearest its root, so the video shows exactly the cut that is scored; its points are rewritten in place when that length changes (`kit.on_curves` in `hingekit.py`). The panel's strands are grouped in 7 x 16 locks that stand straight out while the comb is in them (as hair stands up through a comb) and lie down again when it leaves (`kit.on_kinematic`). The strands don't move on their own and nothing collides with them; this is display only and does not change the scoring. `CC_HAIR=boxes` draws flat locks instead, if a renderer can't draw curves.
- **Head.** The head is a smooth outline built from 3 mm stacked slices. The already-cut hair around the panel is drawn as strands at its length (beside the panel the taper's length, above it 22 mm, at the nape 6-7 mm); for the line check it counts as exactly on the taper.
- **Tools.** The comb has a 7 mm spine and 40 teeth (1.2 mm wide, 1.8 mm gaps); the clipper's steel blade has 22 teeth. Both are collision shapes; the cut itself is computed from the comb's and the blade's faces.
- **Preview-only settings.** The `PB_*` names are used only by an offline pybullet preview and do nothing in Isaac.
- **Assets.** There are no external assets: every object is built from boxes and cylinders in the task file.
