# Strip three patches of old paint off a window sash: one hand keeps the heat gun on the paint while the other scrapes it

This task builds on an earlier submission, whose 48-hour revision hold lapsed without a revision. This version answers
that submission's review (TK-ZN85NF):

| Review point | What changed |
|---|---|
| Construct validity: a scraper edge up to 7 mm above the wood counted as stripping | Paint is no longer taken off by a rule. Each patch is a row of six physical flakes stuck to the rail by a joint PhysX breaks; a flake comes off only when something pushes it with more than 3 N while its patch is hot (200 N while it is cold). A blade held above the paint touches nothing, so nothing comes off: the hover variant of the demo scores 0 |
| Scoring: stripped status came from blade travel, not paint removed | A patch is stripped when at least 80 percent of its paint is off the wood (5 of its 6 flakes moved from where they were stuck). In real trials the same share is read from before and after photographs on a 5 mm grid (see *Real trials*) |
| Clarity: no reproducible paint, preparation or curing | Named lead-free alkyd gloss enamel, brushed on replaceable pine slips in two coats and cured before use; three slips are swapped in for each trial (see *Real trials*) |
| Feasibility: heat gun running near a cotton sheet, no fire or fume procedure | The cotton dust sheet is replaced by a fibreglass welding blanket on a cement backer board; the gun's stand is steel, screwed to that board, its nozzle pointing across the board away from the robot and the glass; ventilation, a mains kill switch and a shutdown and cool-down procedure are given (see *Real trials*) |

A wooden window sash lies flat on a fibreglass welding blanket, its glass pane in the middle. It has been stripped to bare wood except
for three patches of old, cracked cream paint along its near rail. To the left an orange heat gun stands on its wire
stand, already running; to the right lies a paint scraper, its handle on a little wooden rest. One hand takes the heat
gun and aims it, nose down, at the first patch from a few centimetres away; once the paint blisters, the other hand sets
the scraper's blade down on the near side of the patch and pushes it through the hot paint toward the gun while the
gun keeps heating,
and the softened paint is pushed off the wood ahead of the blade. Then the next patch, and the last. The heat gun goes back on its
stand, the scraper back on its rest, and both arms go back to where they started. The task is one task file,
[`strip_flaking_paint_off_a_sash_with_a_heat_gun_and_a_scraper.py`](strip_flaking_paint_off_a_sash_with_a_heat_gun_and_a_scraper.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py) and
[`paint_score.py`](paint_score.py) (the scoring rules, tested on their own in [`tests/`](tests/)), which it imports from
the same folder.

What it tests: using two tools on one spot at the same time (one heating, one scraping), keeping a hot tool aimed where
it should be and away from what it would damage, and putting both tools back.

Why two hands: old paint softens only while it is hot. In the task it blisters after about two seconds of heat and
hardens again about two seconds after the heat leaves it, so the scraper has to go through the patch while the gun is
still on it; a single hand cannot hold both tools. Heat is a rule standing in for the paint's temperature; taking the
paint off is contact physics (see below).

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The blanket, the sash with its glass, the stand and
the scraper's rest are kinematic, fixed where they are put; the heat gun and the scraper are loose rigid bodies made of
shapes from `bodykit.py`.

- Heat: after every control step the task file follows the line along the gun's nozzle from its tip. Where it first
  meets the rail's top (or, over the opening, the glass) within 10 cm of the tip is what the gun heats. Each patch
  keeps a heat value: it rises by 0.5 a second while the gun points at the patch (within 2 cm of it) and falls by 0.5 a
  second otherwise, between 0 and 1.6. From 1.0 the patch is blistered (the flat paint is swapped for blistered paint)
  until its heat falls below 0.6.
- Glass: pointed at the glass for a second in all, it cracks (crack lines appear on it) and the trial scores 0.
- Paint: each patch is a row of six flakes of old gloss paint, 2 mm thick, each a 1 cm strip across the rail, loose
  rigid bodies that collide with the rail, the blade, the gun and the fingers. Each flake is stuck to the rail by one
  joint that PhysX breaks for good once more than its break force pulls on it: 200 N while the paint is cold, 3 N while
  its patch is blistered (the task file sets it as the patch heats and cools). So a flake comes off only when something
  pushes it that hard while its patch is hot: the blade's carbide edge drawn along the wood into the flake's end does.
  A blade held above the paint touches nothing and nothing comes off; a blade pushed into cold paint is stopped by it.
- Paint removed: a flake is off once it has moved more than 1.5 mm from where it was stuck, or tilted more than 8
  degrees (a stuck flake moves well under that). A patch is stripped once at least 80 percent of its area is off the
  wood: 5 of its 6 flakes. Paint pushed off a patch stays where it ends up, as real scrapings do.
- The 3 N and 200 N bonds and the 2 mm paint are modelling choices, not measurements: enough that the blade has to press along the
  wood and push, and little enough for the arm to do it, as heat-softened paint is.
- The blisters and the glass's cracks are drawn shapes that nothing collides with; they wait, hidden, under the glass
  until they are needed.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Real trials

**Paint.** Lead-free alkyd gloss enamel, brushed: for example Rust-Oleum Stops Rust Protective Enamel (brush-on,
oil-based gloss, almond or another cream), https://www.rustoleum.com/product-catalog/consumer-brands/stops-rust/protective-enamel-brush-on-paint.
Paint sold to consumers in the US may not contain more than 0.009 percent lead (16 CFR 1303,
https://www.ecfr.gov/current/title-16/chapter-II/subchapter-B/part-1303), so no lead-safe procedure is needed; never use
the task on paint of unknown age.

**Patches that reset in a minute.** The paint goes on replaceable slips, not on the sash itself. Rout three recesses
6.2 by 3.6 cm, 3 mm deep, into the near rail where the patches go, and cut a batch of pine slips 6 by 3.4 cm by 3 mm
that sit flush in them. Sand each slip (120 grit), wipe it and brush on two coats of the enamel, waiting the can's
recoat time between coats, then let the slips cure for the can's full cure time (a week is a safe default) on a rack
indoors. Score each cured slip's paint across with a utility knife every 1 cm, as old paint cracks, so it lifts in
strips as in the simulation. For a trial, press three painted slips into the recesses with a strip of double-sided
carpet tape under each; after the trial, prise them out and set them aside. A batch of 30 slips covers ten trials.

**Paint removed, from photos.** Before and after each trial, photograph each slip from directly above through a clear
acetate sheet printed with a 5 mm grid (84 squares over the slip), after brushing loose scrapings off with a soft
brush. Count the squares whose centre still shows paint. The share removed is 1 minus the after count over the before
count; a patch is stripped at 80 percent or more. This is the same 80 percent as the simulation's 5 of 6 flakes.

**Fire and fumes.** Work under a range hood, a fume extractor or a window fan blowing out, with the extractor's inlet
within 50 cm of the sash; no other open flames or solvents on the table. The table is covered with a fibreglass
welding blanket over a 12 mm cement backer board covering the table under the sash, the stand and the 30 cm in front of
the nozzle. The gun's stand is bent steel, screwed to the cement board. On its stand the gun points toward the near
right across the cement board, away from the glass and with nothing within 30 cm of the nozzle. Keep a CO2 or dry-powder
extinguisher within reach.

**Shutdown.** The gun's mains lead runs through a switched power strip in the operator's hand: the operator switches it
off at once if the gun is dropped, points at the glass or at anything but the slips, or the robot stops holding it. At
the end of each trial the operator switches the gun off at the strip, leaves it on its stand to cool for 10 minutes
before anyone touches its nozzle, and waits for the slips and the sash to cool before the reset.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the heat gun and the scraper still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or
1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=strip_flaking_paint_off_a_sash_with_a_heat_gun_and_a_scraper.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Strip the three patches of old paint off the window sash's near rail with the heat gun and the scraper: with one hand hold the heat gun aimed at a patch, a few centimetres from it, until the paint blisters, and keep it there while the other hand pushes the scraper's blade along the rail through the patch to take the soft paint off; then do the next patch, and the last. Never point the heat gun at the glass. Then stand the heat gun back on its stand, lay the scraper back with its handle on its rest, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/strip-dream
```

The scripted demo, which reaches full score: the left hand comes down on the heat gun's housing, fingers closing across
it, lifts it off its stand and aims it nose down at the first patch from the far left, its nozzle 8 cm from the paint.
Meanwhile the right takes the scraper by its handle, 9 cm behind the blade. Once the paint has blistered, the right sets
the blade down on the bare wood just short of the patch, pressed onto the rail, and pushes it along the rail into the
paint toward the gun while the gun keeps heating, breaking the flakes off one after another; then the scraper moves on to the next patch, the gun is lifted and follows. After the last patch the gun goes back on its stand and the scraper
back with its handle on its rest, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=strip_flaking_paint_off_a_sash_with_a_heat_gun_and_a_scraper.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Strip the three patches of old paint off the window sash's near rail with the heat gun and the scraper: with one hand hold the heat gun aimed at a patch, a few centimetres from it, until the paint blisters, and keep it there while the other hand pushes the scraper's blade along the rail through the patch to take the soft paint off; then do the next patch, and the last. Never point the heat gun at the glass. Then stand the heat gun back on its stand, lay the scraper back with its handle on its rest, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/strip-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/strip-dream/*.json; do inspect-robots video "$log"; done
```

Hover variant: the same demo with `STRIP_VARIANT=hover` set holds the blade's edge 5 mm above the wood all along the
rail, every other move the same. The blade passes over the paint without touching it and nothing comes off: it scores
0 (see *Results*). The scoring rules alone: `python -m pytest tests/` in this folder (4 tests, no simulator needed).

Set `STRIP_DEBUG=1` to print what the score is built from four times a second: what the gun points at and where, how
steeply and which way it points, where it sits in the hand, each patch's heat, whether it is blistered, which of its
flakes are off and whether it is stripped, how long the glass has been heated, where the scraper's blade edge is,
whether each tool is back, and where each TCP is.

## Results

| Run | Score | What happened |
|---|---|---|
| Scripted demo | 1.0 | All 18 flakes broken off by the blade, each while the blade's edge was on the wood in its patch (it pushed most of them 1.5 to 7.5 cm along the rail); full score held for 1 s at step 816 of 1,200 |
| Hover variant (`STRIP_VARIANT=hover`) | 0.0 | The blade's edge passed along all three patches 7 to 9 mm above the wood, above the 2 mm paint: no flake moved, nothing came off |
| Dream-1 (seed 0) | 0.0 | Ran the full 1,200 steps without stripping a patch |
| `python -m pytest tests/` | 4 passed | Flake-off threshold, hover takes nothing, 80 percent area rule, score tiers |

## Runtime per trial

On one L40S shared with other simulations: the scripted demo takes 816 steps (40.8 s of simulated time) and Dream-1
the full 1,200 steps; wall-clock time depended on how many other simulations shared the GPU (6 to 18 minutes here).
Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
