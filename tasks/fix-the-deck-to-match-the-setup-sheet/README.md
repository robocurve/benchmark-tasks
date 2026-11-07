# Fix the deck to match the setup sheet

A liquid handler deck with six numbered positions, five coloured plates and a setup sheet taped down beside it. Four
plates are where the sheet says. One is not. The robot has to compare the deck with the sheet, find that plate and move
it to its own position, leaving the other four alone.

The hard part is the looking. There's only one move to make, and the sheet is the only way to know which one. The hole in
the deck shows where a plate belongs, but not which plate. The left arm does the work and the right arm stays at rest.

## Files

| File | What |
|---|---|
| `fix_the_deck_to_match_the_setup_sheet.py` | The yam_arena task file: objects, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `benchkit.py` | The shared kit, unchanged. This task uses only plain boxes, but the folder keeps the same layout as the others |
| `test_fix_the_deck_to_match_the_setup_sheet.py` | Offline tests: no GPU needed |

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fix_the_deck_to_match_the_setup_sheet.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "One plate on the deck is in the wrong position. Move it so the deck matches the setup sheet." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/deck-dream

# The scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fix_the_deck_to_match_the_setup_sheet.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "One plate on the deck is in the wrong position. Move it so the deck matches the setup sheet." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/deck-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/deck-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial ends at 1.0 once the deck matches the sheet and the plates have stood
still for 1 s; otherwise it scores the last step.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_fix_the_deck_to_match_the_setup_sheet.py
```

## Scoring

A plate is **on** a position when its centre is inside the pad and its bottom is under 3 cm above it. It is **down** when
it also leans under 10 degrees with its bottom under 0.5 cm above the pad. It is **in place** when it is down, with its
centre within 2 cm of the pad's centre along x and along y and turned under 15 degrees. It is **beside 6** when it leans
under 10 degrees,
its bottom is under 1 cm above the table, it is on no position and its centre is under 10 cm from position 6's centre. A
photo and a ruler settle all of these, once the gripper has opened and moved away.

| Outcome | Score |
|---|---|
| Yellow plate in place on position 6, the other four in place | 1.0 |
| Yellow plate down on position 6 but not in place, the other four in place | 0.7 |
| Yellow plate beside position 6, the other four in place | 0.3 |
| Any of the above, with one or more of the other four nudged but still on its own position | half that: 0.5, 0.35 or 0.15 |
| Any plate on a position that isn't its own, including the yellow plate left on position 1 | 0 |
| Blue, green, orange or pink plate off its own position | 0 |
| Any plate off the table | 0 |
| Yellow plate anywhere else on the table or deck, including shoved off position 1 | 0 |
| Yellow plate leaning, on its edge, still held in the air, or on top of another plate | 0 |
| Nothing done | 0 |

## Design notes

- **The sheet is the only way in.** The hole shows where a plate is missing, and five plates could fill it. Four of the
  five score 0, so a robot that doesn't read the sheet has one chance in five.
- **Colour does what text can't.** Real setup instructions name the labware, but text won't read at 320 x 176. Each block
  on the sheet is about 9 x 10 px in that view, with about 2 px between blocks, and the tests check this from the top
  camera's calibration. The empty position's block is pad grey, so the sheet shows a bare pad rather than a gap. The white
  squares that number the positions are for people reading the scene.
- **Colours are set for the station's lamp.** Its tone-mapping pales everything, and the first palette would have faded
  yellow towards orange, green and the white sheet. The plates are painted darker and purer to allow for it. The tests
  check the gaps with an estimate fitted to pixels from earlier station renders, not with a render of this scene.
- **One way to read it.** The sheet is drawn the same way round as the deck. Turned or mirrored, it would call for at least
  two plates to move, so it can't be misread as a different one-plate fix. The tests check every such reading.
- **Making a new error isn't a fix.** Taking one of the other plates off its position leaves the deck as wrong as it
  started, so it scores 0. A nudge is easier to put right, so it halves the score. This is fair to a careful robot: the demo's
  route touches nothing, open fingers clear the next plate by 1.7 cm, and the first 2 cm of any nudge is free, so a finger
  that only brushes a plate costs nothing.
- **Checking, not building.** Five plates are already down and one needs moving. The job is finding the difference against
  a flat map drawn the same way round as the deck, not assembling a copy from loose parts or working in a turned frame.
- **Only a near miss earns the low partial.** A plate set down within 10 cm of position 6 shows the right plate reached the
  right place. Moving it anywhere else earns nothing, because a shove can take a plate off position 1 without any reading,
  and position 1 is too far from position 6 for a shove to get there.
- **Unintended strategies tried.** Doing nothing: 0. Filling the hole with any other plate: 0. Moving the yellow plate
  to any position but 6: 0 (the only other free one is its start). Shoving it off position 1, or knocking it aside onto the
  table: 0. Shuffling several plates: 0. Stacking: 0. Knocking a plate off the table: 0. Holding the plate above its
  position at the end: 0, unless it is under 0.5 cm up, which a real trial rules out by reading the deck after the gripper
  opens. Standing a plate on edge: 0. Moving the
  sheet to suit the deck isn't possible: it is taped down.
- **The real failure.** Agilent's [Bravo guide](https://automation.help.agilent.com/AutomationSolutionsKB14/Bravo%20User%20Guide/Maintain.06.10.html)
  gives "The tip box on the deck does not match the labware definition specified in the protocol" as a cause of askew
  tips, and says to confirm the correct box is in the correct location. Tecan's project manager calls user error "the
  cause of a substantial proportion of workflow crashes" in [CSEM's account](https://csem.ch/en/news/digital-support-for-lab-techs-helps-them-avoid-mistakes)
  of its deck check, which flags discrepancies "such as missing plates or incorrectly loaded tip boxes". Beckman sells a
  [final check](https://beckman.com/liquid-handlers/software/biomek-method-launcher) that confirms "labware is placed
  correctly". O*NET lists equipment setup and troubleshooting among
  [biological technicians'](https://www.onetonline.org/link/summary/19-4021.00) tasks.
- **What isn't modelled.** Plates are solid three-quarter-size blocks, because a full plate is too wide for the gripper
  to take with room to spare. There are no wells, notched corners or locating corners. No instrument runs. Some labs check
  a screen rather than a card; the sheet stands for whichever reference they use.

## Runtime and seeds

- The scripted demo takes about 15 s of simulated time (about 300 steps at 20 Hz).
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
