# Tighten an inline skate's ratchet cuff buckle: pump the lever until the cuff is snug, then fold it flat

A whole adult inline skate (a left Rollerblade Zetrablade, men's US 10, black and silver, 80 mm wheels) stands upright
on the table with its wheels in a low wooden wheel chock, its toe toward the robot's left and its outer side toward
the robot. A foam boot shaper fills the boot. The skate's cuff closes with its cuff buckle: a black toothed ladder
strap, riveted to the cuff's inner side, wraps round the front of the cuff and enters a ratchet buckle riveted to the
cuff's outer side. The strap's tip has been pushed into the buckle's mouth, past the catch, as people leave it when
they slip the skate on; the cuff is still loose. The buckle's red lever stands 90 degrees open, pointing at the robot.
The instruction asks for the cuff buckle to be tightened until the cuff is snug, and the buckle left closed; it does
not say how.

A ratchet buckle is closed by pumping its lever. Each swing of the lever toward the cuff makes a drive pawl on the
lever pull the ladder strap a tooth or two further into the buckle; a catch in the buckle takes each tooth, so the
strap stays in while the lever is lifted for the next swing; when the cuff is snug the lever is folded flat against
the buckle, where it stays. Swung right open (past 120 degrees on this buckle) the lever lifts the catch and the strap
slides back out: that is how the buckle is undone. The Zetrablade's listing gives its closure as "Cuff buckle, 45°
strap, laces" and says the ratchet can be adjusted individually ([warehouse-one.de][zb]). K2's instructions for its
skates' ladder-strap buckles ([K2 Skates, Tips for Product Use][k2]): "When buckling up, push the ladder in as far as
possible to insure proper centering." and "\"Ratchet\" the ladder strap tight with the silver lever".

One arm: the skate stands in its chock and needs no holding, so this task does not claim two arms. The hand skill is
the pumping itself: take a small lever (55 mm long, 18 mm tall) by its grip end, swing it through repeated arcs about
its hinge pin, turning with it, as the strap's pull grows with each tooth; lift it back for the next stroke without
swinging it so far open that it lets the strap go; and finish by folding it flat. The strap really slides through the
buckle (by up to 45 mm), tooth by tooth, and its tip comes out behind the buckle as it goes in.

The task is the task file [`tighten_an_inline_skates_ratchet_cuff_buckle.py`](tighten_an_inline_skates_ratchet_cuff_buckle.py)
for the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) and
[`strapkit.py`](strapkit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. No third-party models or assets are used: every object is built in
the task file from boxes and cylinders.

`strapkit.py` builds the skate, its ladder strap and its lever as one PhysX articulation (rigid parts made of
`bodykit.py`'s shapes on prismatic and revolute joints, with joint springs, limits and Coulomb friction that the task
file changes during the run). Importing it hooks `yam_arena.scene` when the simulator starts, without changing
yam_arena, and gives `score()` every joint's position.

## What is modelled and what is not

At true size: the skate (boot 32 cm long and 9.6 cm wide, cuff 12.4 x 10.8 cm from 15 to 28.5 cm high, four 80 mm
wheels on a 30 cm frame), the buckle base (55 mm long, 22 mm tall, 23.5 cm above the table), the ladder strap (15 mm
tall, 2.5 mm thick, 5 mm tooth pitch) and the lever (55 mm long, 18 mm tall, 6 mm thick, 9 mm over its 14 mm grip
end, on a vertical hinge pin at the buckle's front end). The skate is held fixed: in the real setup the chock keeps it
upright and from rolling, and lifting it out of the chock is not modelled. Shell, liner, laces and boot shaper are
rigid.

The mechanism, as rules on real moving parts:
- The ladder strap's part at the buckle is a slide joint along the cuff's side (its bend round the front of the cuff
  is drawn fixed; only the straight part at the buckle moves). The cuff and the boot shaper pulling it back out are a
  spring on that slide: 3 N at the start, plus 0.8 N per mm drawn in (about 27 N at 30 mm, snug).
- The catch is the slide's outward limit: every 5 mm tooth that passes it is taken, and the strap cannot slide back
  out past it.
- The drive pawl is a rule: while the lever swings toward the cuff, the strap is driven in by 9.55 mm per radian of
  lever (one tooth per 30 degrees of swing), counted from where that stroke began, and the lever's hinge friction is
  set to the strap's pull times 9.55 mm, so the hand pushing the lever feels the strap (about 0.26 N m at 6 teeth). When
  the lever is lifted the pawl rides back over the teeth and the strap settles on the last tooth the catch took.
- Swung 120 degrees or more open, the lever lifts the catch and the strap slides out of the buckle (its tip leaves the
  mouth); feeding it back in is not modelled. On the real buckle, release is checked once on the bought skate (some
  cuff ratchets release by a separate tab instead); the task never needs the release.
- The lever stays where it is left (0.02 N m of hinge friction); folded flat it rests on the buckle base.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. The
score is the buckle's state at that time (teeth can be lost again by swinging the lever open):
- teeth: 5 mm teeth of the strap drawn in past the catch since the start, none once the strap has been let go;
- 0.1 per tooth, up to 0.6 at 6 teeth (30 mm drawn in: the cuff snug);
- 0.8: 6 or more teeth and the lever folded flat (within 5 degrees of flat, its tip within 5 mm of the buckle);
- 1.0: that, with both grippers' centres 4 cm or more from the lever (hinge, middle and tip), the buckle and the
  strap's tip, and the lever and the strap still (the lever turning less than 1 degree and the strap moving less than
  0.5 mm over the last 1 s).
Doing nothing scores 0. `-E object_jitter=0` keeps every object exactly where the task file puts it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tighten_an_inline_skates_ratchet_cuff_buckle.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Tighten the skate's cuff buckle until the cuff is snug, and leave the buckle closed." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/skate-dream
```

The scripted demo, which reaches full score: the right gripper, level and pointing along the open lever toward its
hinge, takes the lever's grip end between its fingers (top and bottom edges) and pumps it, turning with it about the
hinge pin: toward the cuff to 20 degrees, back up to 85 degrees, three times (2, 2 and 2 teeth); it lets go, comes
round to the lever's outer face and pushes it flat with its closed fingertips (one more tooth, 7 in all), and moves
away. The left arm stays put.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tighten_an_inline_skates_ratchet_cuff_buckle.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Tighten the skate's cuff buckle until the cuff is snug, and leave the buckle closed." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/skate-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/skate-dream/*.json; do inspect-robots video "$log"; done
```

`SKATE_DEBUG=1` prints, every 10 steps (`SKATE_DEBUG_EVERY=5` for every 5), the lever's angle, the strap's slide,
the teeth held, whether a stroke is drawing the strap, the strap's pull, the lever's friction, the stroke count,
whether the catch is lifted or the strap is out, the nearest gripper's distance, whether everything is still, the
score, and each gripper's centre.

`SKATE_WRONG=1` runs the demo the wrong way: before pumping, the lever is swung right open (to about 120 degrees),
which lifts the catch; the strap slides out of the buckle, the following strokes and the fold have nothing to draw,
and the score is 0.

## Runtime per trial

On one RTX PRO 6000 Blackwell (shared machine): the scripted demo reaches full score at about step 440 (22 s of
simulated time) and takes a few minutes with 1280 x 720 rendering; a Dream-1 trial of 2,400 steps at 320 x 176 takes
about 25 to 40 minutes. Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

One left Rollerblade Zetrablade inline skate (men's US 10; any recreational skate with a ratchet cuff buckle on a
ladder strap works the same way), a foam boot shaper or a rolled towel filling the boot to the cuff, and a wheel
chock: two pine strips (35 x 2 x 1.5 cm) screwed to a board 2.8 cm apart, clamped to the table, holding the wheels so
the skate stands upright and cannot roll, toe to the robot's left, outer side toward the robot. Paint a thin mark
across the strap at the buckle's mouth with the strap's tip pushed in to its start position. Before each trial: swing
the lever right open so the strap slides out, push the tip back in by hand until the mark is at the mouth, and set
the lever 90 degrees open, pointing at the robot. Score with a ruler (teeth past the mark, 5 mm each), a photo (the
lever lying flat on the buckle) and the trial's video (grippers clear and nothing moving at the end).

[zb]: https://warehouse-one.de/en/rollerblade/zetrablade-inline-skate-2022-black-silver.html
[k2]: https://k2skates.com/support/tips-for-product-use
