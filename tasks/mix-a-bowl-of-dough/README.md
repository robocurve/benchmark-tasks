# Mix a bowl of dough with a wooden spoon, steadying the bowl

A stainless mixing bowl stands loose on the worktop with the ingredients of a small bread dough in it: flour, with
egg and water poured into a well in the middle. A wooden spoon with a built-up grip stands in a spoon clip on the
bowl's near rim, its head down in the flour. The robot has to mix the flour and the liquid into a dough.

**Instruction:** "Mix the flour and the liquid in the bowl into a dough with the wooden spoon."

Mixing is judged the way a baker judges it: by how much dry flour is left. Flour only takes up liquid where the spoon
works it into the wet part of the batch, so the spoon has to be driven through the whole batch, round the bowl and
across it, not just dabbed in the middle. The bowl is a light steel bowl on a laminate worktop that stirring pushes
about, so the natural way, the way people do it, is to hold the bowl by its tab with one hand while the other stirs.

## Where this comes up

- **Home cooking and baking.** Mixing a dough, batter or stuffing in a bowl with a spoon is one of the most common
  steps in a recipe. It is a core task for any home or assistive kitchen robot.
- **Assistive cooking.** People with reduced grip strength or tremor often find stirring stiff mixtures, and holding
  the bowl still at the same time, among the hardest kitchen jobs. Built-up spoon handles and bowls with grips (as in
  this scene) are standard adaptive equipment for them; a robot that can use the same equipment can help or take over.
- **Food service.** Cooks, food preparation workers and bakers mix small batches by hand every shift (stuffings,
  batters, doughs too small for a mixer).

## Potential impact

- **Two hands with different jobs.** One hand steadies; the other works a tool against a load that pushes back. The
  same pattern runs through a great deal of household work: holding a pan while scraping it, a cutting board while
  slicing, a jar while scooping.
- **Tool use with process feedback.** Progress is not a pose. The robot has to keep working until the batch is
  mixed throughout, which needs coverage of the whole bowl, not a single scripted motion.
- **Granular and mixing materials.** Most manipulation benchmarks use rigid objects. Here the state that matters is
  the condition of a few hundred particles, and the score rewards how much of the batch has been worked.

## What it measures

- **Coverage.** Flour away from the spoon's path stays dry, so the spoon has to be worked round the middle, out
  toward the wall and across the bowl, not just in one place.
- **Holding the tool firmly.** A spoon pinched loosely swings in the fingers and rides over the top of the batch.
- **Keeping the batch in the bowl and the bowl upright.** Flinging more than 5% of the batch out, or tipping the
  bowl, scores 0.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: scene, the mixing rule, scoring, the instruction |
| `doughkit.py` | Builds the bowl (with its grip tab and spoon clip), the spoon (with its grip) and the batch's beads; reads and resets the beads through one PhysX view |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Offline tests of the mixing and scoring rules (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mix the flour and the liquid in the bowl into a dough with the wooden spoon." \
  --scorer task_score --max-steps 2400
```

Run limit: 2,400 steps (2 minutes at 20 Hz).

## Scripted demo

```bash
python demo.py task.py out_dir
```

1. The left hand comes straight down on the bowl's grip tab and takes it between its fingers.
2. The right hand comes straight down on the spoon's grip, takes it, and lifts the spoon 3 cm up out of its clip.
3. It moves the spoon in until the middle of its head is over the middle of the bowl, down in the batch, and stirs:
   five circles each way 2.5 cm round the middle, five each way 4.5 cm round it, then strokes across the bowl.
4. It puts the spoon back in its clip and lets go, and both hands rise clear.

The script is open loop: every pose comes from the layout constants in `task.py`, and object states are read only for
the result file.

## Scoring

Tools: the trial video, a kitchen scale, a sieve with 2 mm holes and a dry spatula.

- **0 for the whole trial** if at any time the bowl tips more than 30 degrees, or more than 5% of the batch by weight
  (19 g) is out of the bowl.
- Otherwise the score is the share of the flour mixed in (taken up into the dough), divided by 0.95, at most 1.0:
  1.0 when 95% or more of the flour is mixed in, 0.5 when 47.5% is, 0 when none is. Mixed-in flour that ended up
  outside the bowl does not count.
- Read at the end of the run.

**In the real world:** the score weighs the dough itself, so only flour actually worked into the dough can earn
credit. Flour that was spilled, spooned out, left loose in the bowl or hidden dry inside a lump never reaches the
scale.

1. With the bowl level, lift every lump of dough out of the bowl and off the spoon with a dry spatula onto a sieve with
   2 mm holes. Leave loose flour and any free liquid in the bowl. Nothing from outside the bowl is used.
2. Break every piece bigger than 1 cm into pieces under 1 cm over the sieve, then shake it for 10 seconds. Dry flour
   that was trapped inside a lump falls through.
3. Weigh what stays on the sieve: the dough, D grams.
4. At most 140 g of the dough can be the egg and water, so at least D - 140 g of it is flour worked in.
   Mixed-in share = (D - 140 g) / 240 g, and 0 if D is under 140 g.

So a fully mixed batch weighs about 380 g of dough (share 1.0), a half-mixed one about 260 g (0.5), and a bowl from
which some dry flour was spooned out without mixing has no dough to weigh (0). In code: `bench_score()` in `task.py`,
tested in `test_scoring.py` (dry flour removed, liquid alone, flour hidden in lumps, half mixed, spill and tip).

**In the simulator:** each bead of flour is dry until the spoon has worked it into the liquid: while the spoon's head
is moving through the batch (faster than 3 cm/s) and the bead is within 4 cm of the head's middle, in contact with liquid (or flour already mixed in) and sliding against it at more
than 2 cm/s, it adds up the distance it slides, and at 1 cm in total it is mixed in (and turns from white to tan).
Flour lying against the liquid without being worked stays dry, and so does flour that is shaken, carried round
with the bowl or stirred by anything but the moving spoon. In code: `wet_step()`, `_watch()` and `score()` in
`task.py`.

## Verification

| Run | Score | Notes |
|---|---|---|
| Scripted demo | 1.0 | 96% of the flour mixed in and 1.2% of the batch out of the bowl at step 1,218 (61 s) |
| Dream-1, 2,400 steps | 0.0 | The right gripper reached into the bowl and knocked the spoon out of its clip; the left hand did not touch the bowl and nothing was mixed |
| Arms still | 0.0 | Nothing mixed, nothing spilled |
| `pytest test_scoring.py` | 18 passed | Scoring rule, the spill and tip gates, the real-world weighing (dry flour spooned out scores 0, liquid alone is not flour, dry flour hidden in lumps is not counted), and the mixing rule (worked flour is mixed in; resting contact, a resting spoon, shaking far from the spoon and beads not touching are not) |

## Runtime and seeds

About 4 minutes per demo run and 8 minutes per Dream-1 run on one L40S, including about a minute of start-up.
`object_jitter=0`; the layout is fixed, and there is no randomness in the task.

## Fidelity

- The batch is 200 beads of 12 mm: 160 of flour (1.5 g each, 240 g) and 40 of egg and water (3.5 g each, 140 g),
  rigid spheres with friction. Dough's stickiness and its growing stiffness as it comes together are not modelled;
  "mixed in" is decided by the rule above, and the bead changes colour.
- The bowl, its grip tab and spoon clip, and the spoon are rigid. The bowl's wall is 32 flat segments.

## Real-world build

A 2.5 qt (2.4 L) stainless mixing bowl, 21 cm across the rim and 8.5 cm deep, with a flat grip tab riveted to its
left side (4 x 2.5 cm, 5 mm thick) and a clip-on spoon rest on its near rim (two posts 14 mm apart, 2 cm tall). A
26 cm beech spoon with a built-up silicone grip, 50 mm long, 24 x 30 mm, its middle 19.9 cm from the spoon's tip. 240 g of
bread flour with 100 g of water and one egg (about 40 g) poured into a well in the middle. Put the bowl on a laminate
worktop and stand the spoon in its clip, its head in the flour.

## Third-party assets

None: every object is built from primitive shapes in `doughkit.py` and `task.py`.
