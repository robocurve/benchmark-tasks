# Braid a practice mannequin head's long hair: three sections, six crossings, outer over middle each time, that stay put

Braid the mannequin head's hair: lay an outer section over the middle one, then the other outer section over the new middle one, and keep alternating sides until there are six crossings. Then let go of the hair.

A three-strand braid is the plait people put in their own hair, in a child's before school, or in the hair of someone
who cannot do it themselves: carers braid long hair for people in bed or in a wheelchair so it does not tangle, and
hairdressing students learn it on a training head like this one. It is done the same way every time: an outer section
over the middle one, then the other outer section over the new middle, and repeat. It is hard for a robot because hair
is limp and it has two hands for three sections: each section must be pinched near its end, carried over the middle one
without dragging the others along, and laid down straight and under light tension on the far side so the crossing forms
snug against the last one, and each crossing must stay put while the next is laid. Laid slack, a section bows near the
braid, the crossings drift outward and the hair runs out before the sixth.

## Scoring

Judged from a top photo, which may be taken whenever both grippers are at least 3 cm from the hair and no part of the
hair moves faster than 1 cm/s (on a real trial, a top-video frame with both hands 3 cm clear where the frame 1 s later
shows no section moved 1 cm); the last such photo of the run counts (none: 0). Read the crossings from the gathering
elastic outward, tracing each section back from its coloured tip elastic (zoom into the photo or step through the
trial video where a crossing is tight): at a crossing the upper section lies over (hides) the lower one. A crossing
counts when an outer section lies over the middle one and the sides alternate (right over middle, left over middle,
and so on, starting on either side). Counting stops at the first
crossing that breaks the pattern: the same side twice in a row, a section passing under the middle one, or a section
laid across both others (only its first crossing counts). A section laid over and straight back is no crossing.
Score = counted crossings / 6, at most 1.0 (six or more). If the head or the gathering elastic has moved more than
1 cm from its tape marks, the score is 0. The run ends early with success once photos have shown six crossings for
5 seconds in a row.

| Last photo with the hands clear | Score |
|---|---|
| Six or more alternating outer-over-middle crossings from the elastic | 1.0 |
| Five / four / three / two / one, then the pattern breaks or the hair ends | 0.83 / 0.67 / 0.5 / 0.33 / 0.17 |
| First crossing already wrong (a section under the middle), or no crossing | 0 |
| No photo all run (a hand never 3 cm clear, or the hair never still); head or elastic moved over 1 cm | 0 |

All three sections are the same dark hair; a judge tells them apart by the red, white and blue tip elastics. The
towel is taped down, so the head in it stays on its mark under the light pulls.

How the simulation takes the photo (`score()` and `success()` in `plait_three_cotton_rope_strands.py`): a crossing is
where two sections' centre-lines cross seen from above with the upper one at least 4 mm higher (a 10 mm section over
another sits ~10 mm higher). Crossings are read outward from the elastic; the sections' places (right, middle,
left) swap at each crossing, so "outer over middle" and "same side twice" follow from the places, as on a photo. The
hands are clear when no point of the hair is within 3 cm of either gripper's envelope (a 50 mm-radius cylinder from the
pad tips 120 mm up the hand). While a hand is on the hair the last photo's score is kept, so holding a crossing in place
earns nothing until the hand lets go and it stays.

Why the shortcuts fail: one hand laying its outer section over the middle again and again makes a twist, not a braid;
its second crossing is on the same side, so it scores 1/6 (the scripted twist, six crossings by the right hand,
scored 0.17 in the simulation). Passing sections under the middle (an inside-out braid) scores 0 from the first
crossing. Holding the sections in a braid shape never yields a photo.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=plait_three_cotton_rope_strands.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Braid the mannequin head's hair: lay an outer section over the middle one, then the other outer section over the new middle one, and keep alternating sides until there are six crossings. Then let go of the hair." \
  --scorer task_score --max-steps 3600 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial, replace `--policy yam_scripted` with `--policy dream`. Each trial allows 3,600 control steps
(180 seconds at 20 Hz). The layout is fixed (the sections are held in the elastic, so object jitter does not move
them). The scripted demonstration scores 1.0 on this layout: each crossing shows in the next photo (1/6 at 7.4 s, then
one more every 10.2 s), six from 58.4 s, and the run ends with success at step 1,267 (63.4 s) of the 3,600 allowed.

`pytest` in this folder runs the offline checks (no GPU) on synthetic centre-lines: the start, full and partial
braids, the shortcuts above, a hand still on the hair and moving hair.

## The scene

The layout is in task.json's setup: the head on its back on a folded towel, crown toward the robot at x 590; the hair
gathered by one elastic on the table against the crown (x 541-549); three 450 mm sections fanned toward the robot with
red, white and blue tip elastics. The head lies on its back with the hair spread above it, as when braiding the hair
of someone lying in bed. A person braiding holds all three sections at once; the robot has two hands, so it braids on
the surface: the hand moving a section keeps it under light tension while the other two sections rest on the table,
which holds each finished crossing the way a braider's fingers do. The elastic is 550 mm out, beyond where the hands
work: the crossings must form near it by how each section is laid from its end, not by reaching in. Each section is
combed smooth and given a light twist so it lies as one round bundle (synthetic hair left loose spreads flat and
frays); the tip elastics keep each tip together and let a judge tell the sections apart, and take no part in the
braid.

Real objects (under $50 in all): a hairdressing training head with long straight synthetic fibre hair in a dark
colour, for example the Hairingrid 26"-28" mannequin head, synthetic fibre hair, colour black #2, sold with a table
clamp (https://www.amazon.com/dp/B07D3NHCVG; listing: total length 26-28 in from the forehead to the back hair end,
individual hair strand 20 in, braiding among its uses); one plain hair elastic; three small coloured hair elastics; a
hand towel; masking tape. Figures not published, each an estimate with a one-minute check: the head's size
(crown to chin about 225 mm, 145 mm ear to ear, 185 mm back to brow: tape measure); each section's thickness (about
10 mm: wrap a strip of paper round the gathered hair at the elastic; about 55 mm round, a third of it is a 10 mm
bundle); its mass (about 40 g/m: weigh a 10 cm cut sample of the same fibre on a 0.1 g kitchen scale, about 4 g); its
stiffness (held level over a table edge with 10 cm overhanging, a section's end hangs about 7.6 cm below the edge and
5.7 cm out); and its length beyond the elastic (about 450 mm, the 20 in hair less its run over the scalp: tape measure).

## Scripted demonstration

Two hands, one crossing per 10.2 s cycle, right and left alternately, six times. The hand pinches the outer section
at its tip elastic, lifts it so the chord from the section's root (its furthest crossing, or the elastic) to the hand is
5 mm short of the section's length there, swings it on that arc over the middle section, lowers it along the arc so it
lands straight, pulls it 1.5 mm past its length (a light tension of a few newtons) and lets go; then it slides the old
middle section, now outer, out to 30 degrees about its previous root, which draws the new crossing toward the elastic.
The demo reads the sections' link poses (simulator state); a model under test sees only the cameras.

In the scripted run the six crossings form right, left, right, left, right, left from the elastic and stay in place
after both hands park, as the top video's last frame shows. Laying each section taut matters: lowered with 8 mm of
slack, an earlier rope model of this plan ran out of length before the sixth crossing in two runs of three.

## Simulation and the real objects

Each section is an articulation of 30 capsule links (15 mm long, 10 mm across, 0.6 g each), stretch and twist locked,
two bending axes free to +-60 degrees on springs toward straight (EI 3e-5 N m^2: about 7,000 fibres of about 75 um
bending side by side; 10% damping; joint armature 5e-6 kg m^2 for solver stability). Sections collide with each other,
the table and the grippers (friction 0.8, an estimate for a fibre bundle on fibre and on bare wood). A section is one
bundle: single hairs, flyaways, flattening and splitting are not modelled, which is why the real set-up twists each
section lightly and binds its tip.
Each section's first link is fixed inside the gathering elastic. The head (ellipsoids and a cylinder at true size,
three invisible colliders) and the towel are fixed. Physics at 240 Hz, control at 20 Hz.

Files: `plait_three_cotton_rope_strands.py` (scene, head, scorer, demo), `rope_strand.py` (the sections as capsule
chains, and the fixed head), `plait_word.py` (reads the crossings; builds the tests' synthetic braids),
`test_plait_three_cotton_rope_strands.py` (offline checks).
