# Paint all five fingernails of a manicure practice hand with red polish, keeping it off the skin

A nail station on the table. A soft practice hand of the kind nail technicians train on, held by its table clamp, lies
palm down on a folded white towel with its fingers spread and pointing at the robot. Its five bare nails are pale pink.
A bottle of red polish stands closed in a rubber holder beside it; as on every polish bottle, the brush is part of the
cap. The robot must paint all five nails, keep the polish off the skin, and put the brush back in the bottle.

Painting nails is an everyday grooming act that many people do themselves and almost everyone has watched up close, at
home or at a salon. What makes it hard:

- **Precision on a small, raised target.** Each nail is 8.5 to 12.5 mm wide. A 6 mm brush must be laid on at an angle
  and drawn from just short of the cuticle out past the free edge, in two or three side-by-side strokes, so the coat
  meets both side edges without running onto the skin. One stroke started on the skin behind the cuticle scores 0.
- **Light, even contact.** Only the bristle tuft's tip should touch: too high and nothing is coated, too low and the
  stiff brush is pressed down; the stroke must follow the nail plate along its length.
- **One loaded brush at a time.** A dip carries about one nail's worth of polish, so the brush goes back into the
  bottle between nails: straight down through a 1.4 cm neck and out again without knocking the bottle.
- **The thumb lies at an angle** (about 35 degrees off the fingers), so its strokes run in a different direction.
- **Put it back.** The cap must end seated on the neck and both hands must be clear and still.

## Scene

Positions in cm from the midpoint between the arm bases (x forward, y left), heights above the table.

| object | type | size | material (real part) | position |
|---|---|---|---|---|
| towel | fixed | 26 x 20 x 0.6 | white cotton hand towel, folded | centred (35.5, -15) |
| practice hand | fixed (held by its clamp) | 28.4 cm overall; palm 9 x 8.6 x 2.2 thick; fingers 1.36-1.9 cm thick | soft nail-practice hand with table clamp (see Sources), skin colour; a right hand, palm down, fingers spread, thumb flat and nail up | knuckle line at x 33.5, palm y -18.8 to -10.2, fingertips at x 25.7-27.7; thumb tip at (35, -22), pointing 35 degrees to the robot's right of the fingers |
| nails | fixed, part of the hand | thumb 11.5 x 12.5 mm, index 11 x 10.5, middle 11.5 x 11, ring 11 x 10.5, little 9 x 8.5 (length x width); 1.2 mm proud of the finger | bare, pale pink | on each fingertip, free edge about 5 mm in from the fingertip's end |
| polish bottle | fixed (in a holder taped down) | glass body 2.9 x 2.9 x 5 cm, neck 1.4 cm wide, 9 cm tall with its cap | essie original 13.5 mL, red | centred (21.5, -25.5) |
| bottle holder | fixed | ring 4.5 x 4.5 x 1.2 tall | rubber polish-bottle holder | round the bottle's base |
| brush (cap) | free, 8 g | cap 1.6 cm dia x 3.6 cm; stem 3 mm x 4.2 cm; bristle tuft about 6 x 2 x 10 mm | the bottle's own cap-brush | seated on the neck (not screwed), tuft in the polish |

## Scoring (real trial)

From the trial video and an end photo of the hand taken from above through a clear sheet ruled in a 0.5 mm grid. Cameras: a scene camera, plus a close-up phone fixed about 25 cm above the hand, framing all five fingertips, recording the whole trial so any polish more than 1 mm past a nail edge is visible when it happens. The bottle is filled to 3.6 cm of polish (about 70% full, as in the sim); top it up to that line before each trial.

Score 0 if:
- polish touches skin more than 1 mm outside any nail's edge at any time (a smear on a finger, the cuticle area or the
  palm); polish within 1 mm of the edge counts as on the nail, as a salon would clean it up.

Otherwise:

| part | points |
|---|---|
| each nail with at least 90% of its area coated | 0.18 |
| each nail with 50-90% coated | 0.09 |
| at least one nail scored, the cap seated back on the bottle's neck (upright within 10 degrees, centred within 5 mm, its underside within 3 mm of the neck's top), and both grippers 10 cm or more from the brush, the bottle and the hand, and still (under 2 cm/s) for 0.5 s | 0.10 |

1.0 = all five nails coated over 90%, brush home, both hands parked.

## Run a trial

Requirements: yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \
    -E cam_height=176 --scorer task_score --max-steps 4000 \
    --instruction "Paint all five fingernails of the practice hand with the red polish, keeping it off the skin, then put the brush back in the bottle."
```

The layout is fixed (object_jitter=0). About 200 s per trial at 20 Hz.

`score()` measures, every control step: the contact force between the brush and each nail plate, reported by the
physics engine (a nail is coated only while the tuft presses on it, over 0.02 N); the bristle tuft's lowest edge
(sampled at 7 points across its width) against each nail's surface and outline; coated cells on a 0.5 mm grid per nail; the brush's polish load (loaded by dipping
the tuft below the polish level inside the bottle, used up by newly coated area); any contact of the tuft's edge with
skin more than 1 mm outside the nails once the brush has been in the polish; at the end, the
cap's pose on the neck and the grippers' distance and speed.

## Demo video

A scripted demo (right arm only; the left arm stays at rest) scores 1.0 in about 2,400 steps. It pinches the cap
straight down, draws the brush up out of the bottle, tilts it 45 degrees and paints each nail in two or three lanes,
each drawn from just short of the cuticle out past the free edge; between nails it goes straight back down the neck
to reload. After the little finger it seats the cap on the neck, opens and parks the arm clear of the station, wrist
camera looking down at the painted hand and the bottle.

## Shortcut checks

| behaviour | score |
|---|---|
| do nothing | 0.0 |
| paint all five nails, but one stroke starts 3 mm behind the index cuticle (on the skin) | 0.0 |
| strokes 2 mm above the nails (bristles never reach the plate) | 0.0 |
| paint the thumb, then lay the brush on the towel and leave | 0.18 |
| paint all five, seat the cap, but never let go of it | 0.9 |

## Real-trial notes

The robot's finger pads wear disposable nitrile finger cots for the trial, and the folded towel plus a sheet of paper
under the bottle holder protect the table; acetone and cotton pads for the reset stay off the table during the run.

## Fidelity

Rigid bodies, 20 Hz control. The hand is rigid and fixed in place, as its clamp holds the real one; its fingers are
cylinders with rounded tips and its nails are flat plates (real nails curve slightly across). The brush is one rigid
body (cap, stem and tuft all collide); real bristles flex and splay. A nail is coated only while the physics engine reports the
tuft pressing on it (contact force over 0.02 N); the coat is then laid over 1.2 mm round each point of the tuft's
lowest edge that lies on the plate (within 0.4 mm of it, standing in for bristles splaying flat under the press). Polish is not simulated as a liquid: dipping the tuft
below the polish level inside the bottle loads it with about one nail's worth (160 mm2), and newly coated area uses that
up; drips, streaks and drying are not modelled. The coat is drawn as red squares on the nails. The cap sits on the neck
and is not screwed down. The brush and bottle sizes follow the named bottle; the tuft size is approximate.

## Sources

- essie nail polish, original formula, 0.46 fl oz (13.5 mL) glass bottle, 1.14 x 1.14 x 3.54 in with its cap:
  https://www.essie.com/nail-polish/enamel/nudes/gilded-galaxy
- Soft practice hand with table clamp, 11.2 in (284 mm) overall, for nail technicians:
  https://transdesign.com/products/practice-hand-soft-hand-with-clamp.html
