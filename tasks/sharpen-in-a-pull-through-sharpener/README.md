# Sharpen a knife in a pull-through sharpener: hold it, coarse then fine, pull only

A chef's knife stands edge-down in a low wooden dock on the right of the table, and a small pull-through sharpener
stands on a wet plastic cutting board in front of the robot. The sharpener has a handle on its left end and two
V-slots: a coarse carbide slot (dark) and, further right, a fine ceramic slot (white). The instruction is only:
"Sharpen the chef's knife with the pull-through sharpener, then put the knife back in its dock." Instructions for
such sharpeners give a fixed procedure: hold the sharpener's handle firmly with one hand so it cannot slip, then pull
the knife toward you from heel to tip, in the coarse slot first and then the fine slot, never back and forth
([Etch Couteaux's guide to manual sharpeners](https://etch-couteaux.com/en/blogs/infos/how-to-sharpen-a-knife-with-a-manual-sharpener)).
The [manual of a pull-through sharpener (item 191H)](https://m.media-amazon.com/images/I/91r6ZQR8e9L.pdf) says to
insert the blade vertically and always pull from heel to tip, never back and forth, with the No. 1 coarse carbide slot
setting the edge and the No. 2 fine ceramic slot honing it. The
[directions for Smith's Knife & Scissors Sharpener](http://pdf.lowes.com/useandcareguides/027925501016_use.pdf)
add moderate downward pressure, and the [KitchenKnives.com guide](https://kitchenknives.com/how-to-sharpen-kitchen-knives/)
gives 3 to 5 passes in the coarse slot, then the same number in the fine slot. The instruction mentions none of this,
so the task checks whether the robot follows the procedure unprompted:

- **Hold, or it slides.** The sharpener weighs 100 g and stands on slippery feet on a wet board while its slot grips
  the blade, so one arm has to hold its handle while the other pulls the knife. In the recorded `one_arm` variant,
  with nobody holding it, the sharpener slid 15.32 cm along with the knife during its first pressed contact with the
  coarse slot, and no pass counted.
- **Pull, coarse then fine.** A pass counts only if the edge is pressed into the slot (1.5 N) and drawn at least
  12 cm from heel toward tip. Three passes are needed in the coarse slot and then three in the fine slot. Pushing the
  knife back more than 2 cm in one contact is sawing and scores 0.
- **Hand clear of the slots, knife back in its dock.** The holding hand must stay 1.5 cm from the slot housings. The
  knife starts in a dock and must be returned to it; until it is, the score is capped at 0.75.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files, and the per-step readings the scorer uses: part poses, finger-pad points and the knife-on-sharpener contact force |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure and cheating variants (`democore.py` holds the demo runner and planning helpers that `demo.py` calls) |
| `demo_results.json` | Each variant's score and end-state measurements, including every pass's travel, pressed share, backward travel and sharpener movement, from the runs reported below |
| `variant_scores.csv` | One row per variant: score, success, steps and the end-state report |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |

`task.py` registers the knife and the sharpener with `kitchenkit.watch_contact`, so the scorer reads the
knife-on-sharpener contact force every step. The task runs with the standard `yam_arena` embodiment and `task_score`
scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sharpen the chef's knife with the pull-through sharpener, then put the knife back in its dock." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/sharpen-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Runtime:** the recorded full demo ran 933 control steps (46.65 s of simulated time at 20 Hz). Its wall-clock time
was not recorded with these results. A model trial runs up to 2,400 steps, plus the policy's own response time. Measured on Nov 15: the Dream-1 run (`dreamlabs/dream-1-beta`, 2,400 steps, `rows_per_request=12`) took 8.5 min of wall-clock time on one NVIDIA RTX PRO 6000 Blackwell GPU, simulator start-up included.

## Scripted demo

```bash
python demo.py task.py demo_out                         # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_saw --variant saw            # any variant below; --no-record skips the video
python demo.py task.py --plan-only                      # inverse-kinematics check of every variant's plan
DEMO_DEBUG=10 python demo.py task.py demo_out           # also print the sharpening state every 10 steps
```

The left arm pinches the sharpener's handle from above and holds it. The right arm pinches the knife's bolster and
handle front from above, lifts the knife out of its dock and sets the heel in the coarse slot. It presses the edge in
(commanded 18 mm below the slot's floor; `demo.py` notes that 10 mm gave about 1.6 N) and pulls the knife 16 cm toward
the robot, from heel to tip, then lifts it out and starts again. After three passes in the coarse slot it makes three
in the fine slot and puts the knife back in its dock, and the left arm lets go of the sharpener.

| Variant | What it does | Score |
|---|---|---|
| `full` | as described above | **1.0** (933 steps; 3 valid coarse and 3 valid fine passes; per valid pass: travel 14.08 to 14.53 cm, pressed share 0.85 to 0.89, backward travel 0.09 to 0.33 cm, sharpener moved 1.09 to 1.25 cm; knife in dock; sharpener ended 0.40 cm from its start) |
| `one_arm` | the same passes with nobody holding the sharpener | 0.0 (1,026 steps; 2 coarse-slot contacts recorded, none valid; in the pressed one (pressed share 0.95) the sharpener slid 15.32 cm with the knife while the edge advanced 0.02 cm through the slot; no later slot contact was recorded, since the plan aims at the sharpener's start position; sharpener ended 19.81 cm from its start; knife in dock) |
| `saw` | as `full`, but in the coarse slot the knife is pushed back to the heel while still pressed | 0.0 (1,172 steps; sawing flagged: 12.53 cm of tip-to-heel travel in the first pressed coarse contact, against a 2 cm limit, and the held sharpener moved 8.19 cm in that contact; none of 9 recorded contacts valid; knife in dock) |
| `fine_first` | three passes in the fine slot, then three in the coarse slot | 0.5 (1,052 steps; the 3 valid fine passes came before any coarse pass, so they are recorded as `fine_before_coarse` and do not count; 3 valid coarse passes count; knife in dock) |
| `light` | as `full`, but the blade only skims the slots without pressing | 0.0 (1,052 steps; 13 slot contacts, 7 coarse and 6 fine, none reaching 1.5 N: pressed share 0.0 in each; knife in dock) |
| `no_return` | as `full`, but the knife is kept in the hand at the end | 0.75 (972 steps; the same 3 + 3 valid passes as `full`; knife not in dock, recorded as `knife_in_dock` 0.0; capped at 0.75) |

The numbers are from `demo_results.json` (the same scores, steps and reports are in `variant_scores.csv`). In `full`
the pass list also holds three entries with zero travel: the first contact in the coarse slot, one between the first
and second valid coarse passes, and the first contact in the fine slot. These are contacts of at least 0.5 N that
never reached 1.5 N; they do not count and are not penalised.

## Scoring

Every step, from poses and the knife-on-sharpener contact force; the score at the end counts.

- **In a slot:** the blade lies along the slot and upright (each within about 20 degrees, cosine 0.94), the middle of
  the slot meets the edge between 1 cm behind the heel and the point, and there the edge is inside the slot (within
  its 1.4 cm wide top opening and no higher than its top).
- **Pass:** one contact. It starts when the blade is in a slot with a contact force of at least 0.5 N and ends after
  more than 3 steps off the abrasive or when the blade moves to the other slot.
- **Valid pass:** the point where the slot's middle meets the edge travels at least 12 cm from heel toward tip over
  the steps pressed at 1.5 N or more; at least 80% of the pass's touching steps are pressed; no more than 1 cm of
  backward (tip-to-heel) travel; and the sharpener moves no more than 2 cm horizontally during the pass.
- **Order:** valid coarse passes count. Valid fine passes count only after three valid coarse passes; earlier ones
  are recorded as `fine_before_coarse` and do not count.
- **Score:** 1/6 per counted coarse pass and per counted fine pass, up to three of each, capped at 0.75 while the
  knife is not back in its dock. Passes beyond three per slot neither add nor subtract.
- **In its dock:** the knife's heel is within the first 8 cm of the groove from the dock's near end, within 0.8 cm of
  the groove's centre line and no more than 6 mm above its floor; the blade is within 30 degrees of upright; its point
  is within 15 degrees of straight away from the robot; and no finger pad is within 1 cm of its handle.
- **1.0 (success):** three counted passes in each slot and the knife in its dock.
- **0:** from the moment of sawing (more than 2 cm of tip-to-heel travel in one contact), any finger pad of the left
  (holding) hand within 1.5 cm of a slot housing, the sharpener tipped more than 20 degrees, or the knife dropped (its
  heel falls 3 cm within 10 steps while no finger pad is within 2 cm of its handle).

`report()` records the current force, slot and contact point, the counted coarse and fine passes, the fine passes made
too early, every closed pass (slot, travel, pressed share, backward travel, sharpener movement, valid), the four
zero-score flags, whether the knife is in its dock, and the sharpener's horizontal shift from its start.

## Real-world trials

Use a pull-through sharpener of about 100 g with a handle on its left end and a coarse and a fine slot, on a plastic
cutting board wetted with water, and a chef's knife standing in a wooden dock on the right. Mark the sharpener's start
position on the board with tape and measure how far it moves during each pass (a pass counts only if it moves no more
than 2 cm). Mark the blade every 2 cm from the heel so an overhead camera can read how far the edge is drawn through
the slot (at least 12 cm) and any backward travel (sawing above 2 cm). The 1.5 N pressing threshold needs a force
reading, for example from a force-torque sensor at the right wrist if one is fitted. A sharp knife on a robot arm is a
hazard: keep people outside the arms' reach during trials, or use a blunt practice blade of the same size.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Context.** Instructions for pull-through knife sharpeners describe a fixed procedure. Etch Couteaux's guide to sharpening with a manual sharpener: hold the sharpener's handle firmly with one hand so it cannot slip, pull the knife toward you from heel to tip in the coarse slot, then in the fine slot, never back and forth. The manual of a pull-through sharpener (item 191H): insert the blade vertically, at 90 degrees to the sharpener; the No. 1 coarse carbide slot sets the edge and the No. 2 fine ceramic slot hones it; always pull from heel to tip, never back and forth. The directions for Smith's Knife & Scissors Sharpener add moderate downward pressure, always pulling. The KitchenKnives.com sharpening guide: 3 to 5 passes in the coarse slot, then the same number in the fine slot. The instruction says only to sharpen the knife and put it back, so the task checks whether the robot follows this procedure unprompted. Two arms are needed because of friction: the sharpener weighs 100 g and stands on slippery feet on a wet board while its slot grips the blade, so one arm has to hold the sharpener's handle while the other pulls the knife. In the recorded one-arm variant, with nobody holding it, the sharpener slid 15.32 cm along with the knife during its first pressed contact with the coarse slot while the edge advanced 0.02 cm through the slot; it ended 19.81 cm from its start and no pass counted (score 0). Held by the other arm in the full demo, it moved 1.09 to 1.25 cm per pass. The knife starts in a dock and must be returned to it. Done by: cooks (O*NET 35-2014.00, 35-2012.00) and anyone who sharpens kitchen knives at home.

**Simulation fidelity.** Rigid bodies: boxes (the knife's blade, bolster and hollow handle; the sharpener's feet, body, handle and slot walls; the board and the dock). The board and the dock are fixed; the knife and the sharpener are free. Contact forces and friction are simulated, and the scorer reads the knife-on-sharpener contact force each step. Abrasion is not simulated: the carbide and ceramic strips are visual, no steel is removed and the edge does not change, so the scorer stands in for sharpening by counting passes made as the instructions describe (pressed, heel to tip over at least 12 cm, never back, coarse before fine, with the sharpener held still). The water on the wet board is not simulated as a liquid: it is the board's friction coefficient of 0.05 under sharpener feet of 0.10, while the slots' grip on the blade is friction between the blade (0.50 static, 0.45 dynamic, dry steel) and the slot walls (0.6, 0.5); together these make an unheld sharpener slide with the knife. The edge cannot cut, so the rule keeping the holding hand's finger pads 1.5 cm from the slot housings stands in for keeping fingers out of the blade's path.
