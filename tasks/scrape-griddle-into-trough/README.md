# Scrape charred bits off a griddle over the lip into its grease tray with a scraper

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). A range-top add-on griddle
stands on the bench with its grease tray toward the robot, the near end of its plate covered with charred flakes and
crusts stuck to the steel. The robot takes a 4 in. griddle scraper from the bench, breaks the debris free and pushes it
over the lip into the tray, and sets the scraper down. The setup, scoring and fidelity notes are in `task.json`.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/scrape-griddle-into-trough/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Scrape the griddle clean into the grease tray with the scraper." \
      --scorer task_score --max-steps 2050 --store-frames

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`. Offline checks (no simulator): `python3 test_task.py`.

## Scoring

`--scorer task_score` gives 1.0 when the task succeeds (the full condition held for 1 s) and otherwise `score()` at
the end of the trial. Every share is by mass, of the 22.6 g of debris, read from where the 88 pieces really are (their
poses in the simulator; in a real trial, a 0.01 g scale).

| Where a piece is | Counts as | Why |
|---|---|---|
| Inside the tray's walls (x 7.4 to 15.0, across the plate's width), below the plate's top | in the tray | delivered |
| Outside the griddle's footprint (plate, tray and splash guards) | off the griddle | on the bench or floor, or carried off on the scraper |
| Anywhere else: on the plate, against a guard, on the blade over the griddle, falling into the tray | left on the griddle | not yet delivered |

- **In the tray, by tiers.** 0.25 at 25 % in the tray (a first strip cleared), 0.5 at 60 % (most of the area done),
  0.75 at 90 % with at most 5 % left on the griddle, and 1.0 (with the set-down below) at 98 % with at most 1 % left:
  the plate is clean. A flake is 0.5 to 1.1 % of the mass and a crust 4.0 to 4.9 %, so 1 % allows one small flake to
  stay behind and 5 % about one crust. Debris pushed loose but left on the plate is not clean: it is the same carbon in
  a pile.
- **Set down.** 1.0 needs the clean plate and the scraper set down on the bench off the griddle (its origin within
  3.0 cm of the bench top and outside the griddle's footprint: a scraper lying on the plate or dropped in the grease
  tray does not count), released (no gripper's palm or fingers touching it: `touching()` in `guardkit.py`, a contact
  check, not a distance zone) and still (under 1 cm/s and 0.5 degree per step, 10 degrees/s) for 1 s. The job is not
  finished while the scraper is in hand or lying on the plate. The arm's pose is not scored.
- **Off the griddle: capped at 0.25.** More than 2 % of the debris (about 0.45 g, a few flakes) off the griddle caps
  the score at 0.25: grease and carbon thrown onto the line or the floor is a slip and fire hazard, which is why cooks
  push the pile over the lip and lift the blade slowly. Judged from where the pieces are, like the tiers.
- **Gouge: a proxy, capped at 0.5.** While the scraper is held and sliding with its edge on the plate (within 0.6 mm
  of the plate's top, about the simulator's resting gap; 1.0 mm once a stroke has begun), with the blade steeper than
  70 degrees to the plate, or its edge tilted more than 10 degrees out of the plate's plane so only one corner touches,
  sliding 1 cm or more in a stroke, the score is capped at 0.5 for the rest of the trial (`scrapekit.py`, with no soil
  in it: only its blade-angle rule is used). The simulation has no seasoning or scratches; this stands in for the
  chatter marks and scored lines a steep or cornered blade leaves in a griddle's seasoning. In a real trial the cap
  is judged on the plate itself: a new scratch 1 cm or longer, against a photo taken before the trial under a
  raking light.
- Doing nothing scores 0. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

### Real-trial measurement

The debris is real charred food residue, cooked onto the plate to the simulator's layout before each trial. Season the
plate, grease it lightly (about 1 g of vegetable oil wiped over it with a paper towel), weigh the empty tray on a 0.01 g
scale and set it in, and weigh the whole griddle on a 0.1 g bench scale. With the plate cold, lay a template cut from a
plastic folder (the 88 outlines from `task.py`) over it and put thin pancake batter (pancake mix and water, 1 to 1 by
weight) through it with a syringe: a 0.25 g dot on each flake's square and a 1.5 g spoonful on each crust's (32 g in
all). Lift the template off. On a range burner away from the robot's bench, heat the plate to 230 degrees C, hold it
there for 10 minutes so the batter chars black and sticks, and turn the burner off. Let the griddle cool on the range
until the plate is under 40 degrees C, read with a contact surface thermometer (or an IR thermometer on a strip of
masking tape: bare steel reads low). Weigh the griddle again: the gain is the residue, dry because its water has
cooked off. Clamp the griddle on the bench; nothing there is hot or plugged in. Photograph the plate under a raking
light.

After the trial, weigh the tray: its gain is in the tray. Wipe what is left on the plate (still stuck or loose),
against the guards and on the blade over the griddle with dry paper towels weighed beforehand, and weigh them: their
gain, less the session's blank, is left on the griddle (the blank: once per session, wipe the freshly greased plate
the same way, weigh the towels and grease the plate again, so the grease film is not counted as residue). Gather what
is on the bench, the floor and the scraper the same way: that is off the griddle. Shares are of the three together; if
they fall short of the residue weighed onto the plate by more than 0.2 g (the bench scale's error), the shortfall counts
as off the griddle. The thresholds are the simulator's shares. Compare the plate with the photo: a new scratch 1 cm or
longer is a gouge (cap 0.5). Nothing else about how the job was done is scored.

## The debris (`debriskit.py`)

- **Real bodies, bonded.** Each of the 88 pieces is its own rigid body: a box collider with the same visible box and
  its real mass, held 0.2 mm above the plate by a breakable joint (a PhysX fixed joint from the plate to the middle of
  the piece's underside). The physics solver breaks the joint, for good, the first time holding the piece takes more
  than its break force; from then on the piece is free. Nothing is drawn or moved by a script: the blade's collider has
  to meet a piece and push it hard enough to free it, and a piece falls into the tray only when it is pushed past the
  lip. The score reads only the pieces' poses.
- **Bond strength.** Break force = 0.15 N per square cm of the piece's footprint: 0.08 to 0.13 N for a flake, 0.41 to
  0.50 N for a crust, 46 to 68 times the piece's weight, so nothing comes loose by itself. Break torque = break force
  x 2 mm (an edge that gets under a piece peels it off with less push than a flat shove). Real-world basis: a blade
  clearing a deposit pushes with the deposit's removal energy per unit area times the width it meets (N per m of edge
  = J per square m). Here that is 11 to 14 N per m for a flake and 24 to 28 for a crust. Millimanipulation
  measurements of baked food soils on stainless steel span about 0.3 to 80 J per square m (baked tomato paste 1 to
  20); heavily baked, polymerized lard is about 420 (Ali et al., Food and Bioproducts Processing, 2015). So the pieces
  are freshly charred bits from the last order: this bond is how the simulator represents the real trials' charred batter.
  Aged, polymerized carbon (years of it, which would need several times the arm's push) is out of scope. The value also sits below what the arm delivers: through the held scraper it frees a full row of
  about eight flakes at once only up to roughly 1.2 N in all; at 0.25 N per square cm the edge rode up over the first
  full row it met, pass after pass (measured in the simulator).
- **Calibration.** PhysX tests a joint's break force against its own measure of the joint's load, not the push on the
  piece. Pushed in the simulator with a slowly rising force, pieces came free at about 0.031 of the value written
  (a thirty-second; flakes 0.028, crusts 0.034 to 0.038), so the task writes 32 times the bond (`BREAK_CAL`): flakes come free about 11 %
  under the values above and crusts 10 to 23 % over. A broken bond is not restored by the scene's reset (one trial per
  run).
- **Real-trial bond.** The real debris is charred batter (see Real-trial measurement); the forces above are how
  the simulator represents it, not a measurement of it. To record the real bond, cook five extra dots and one extra crust at
  the far end of the plate (x 60 to 70) with each batch, push each one's side through a blade edge with a 0-10 N force
  gauge, write the forces down with the trial, and wipe that end clean before the griddle is weighed again.
- **Contact settings.** Pieces are at least 2 mm thick (the blade is 1.5 mm, moving 4 cm/s: 0.2 mm per 240 Hz
  physics step, so nothing passes through); contacts start 2 mm out and rest at zero gap; 16 position iterations; a
  piece pinched into a collider is pushed out no faster than 0.3 m/s.
- **Friction.** Carbon on steel, 0.5 static and 0.4 sliding, combined by "min" with the other surface: on the greased
  plate (0.20 and 0.15) a flake slides as easily as on a working griddle, and on the blade (0.5) it slides back off a
  40 degree blade (tan 40 degrees is 0.84) instead of riding up with it.
- **The layout is fixed.** The pieces are not in the task's object list, so the simulator's object jitter (which moves
  each free object up to 3 cm on its own) can't pile them into each other; the scene's reset puts them back where they
  started. The scraper is still jittered.
- **Why the strokes overlap.** Pushed flakes spread sideways and spill round the blade's ends; a lane's leftovers lie
  in the next lane's path, so the demo's half-blade overlaps and repeated passes pick them up.
- **The plate's surface.** A working griddle is seasoned and greased, so a steel blade slides on it easily. The plate
  has the friction of greased steel on steel (0.20 static, 0.15 sliding) instead of the simulator's default 1.2, at
  which a blade edge sticks and chatters.
- **Speed.** 88 extra bodies and their bonds slow the simulator to about 0.28 s per control step (about twice the
  scene without them).

## Runtime and seeds

- Demo: a 92 s plan (1,838 control steps), run with `--max-steps 2050`. The recorded run scored 1.0 at step 1,786
  (the trial ends once the set-down has held for 1 s, before the plan's final wait): at least 98 % in the tray and at
  most 1 % left on the griddle; the last frame shows the plate clean.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- The demo: the right hand takes the scraper by the handle from above, fingers closing across the handle (yaw 0: along
  world y), lifts it level and carries it over the right splash guard (tilted, its edge would hang below the guard's
  top), then tilts it over the plate so the blade meets the plate at 40 degrees with the edge toward the robot. The
  tilted hand is turned 6 degrees about its approach, undoing the turn the handle takes in the fingers when they
  close, so the edge lies flat on the plate. Each stroke sets the edge on the plate at x 32.5 (4.5 cm beyond the
  debris; the hand lags about 1.5 cm, so the edge lands about 3 cm clear of every piece: an edge that lands on a
  bonded piece presses it down and slides over it), aimed 10 mm into the plate (the plate stops it; a hand holding the
  scraper sits 3.5 to 6.5 mm above where it is sent, so a shallower aim lets the edge lift off mid-stroke and the
  flakes slip under it), draws it toward the robot at 4 cm/s to x 13.5 (1.5 cm past the lip, over the tray), and lifts
  it 3 cm in 0.6 s, slowly, so what rides on the blade slides off into the tray. Three lanes 5 cm apart (y -7.0, -2.0
  and 3.0; the blade covers y -12.1 to 8.1), three times over: the later passes pick up the flakes that spill round
  the blade's ends. Then the scraper is leveled over the tray, carried back over the guard, laid back where it was,
  and let go.
- Seeds: none. The layout is fixed (`object_jitter=0`); the pieces' sizes and places come from a fixed seed in
  `task.py` and are the same every trial.

## Real-world build

Everything is off the shelf. The griddle is a range-top add-on griddle top of the size above (69 x 31 cm overall,
7-gauge stainless, end grease tray), standing on its own frame with the plate's top 5.0 cm above the bench (shim the
frame's feet if needed), clamped to the bench with two C-clamps on its frame. It is heated only to make the debris, on a
range burner away from the bench, and is under 40 degrees C and unplugged when it is clamped down: nothing at the
bench is hot, so the trial is safe for the robot and the operator. The scraper is a 4 in. stiff griddle scraper with a
stainless blade and a polypropylene handle in line with it (150 g, handle about 3.0 x 2.4 cm, within the gripper's
opening). The debris is thin pancake batter cooked onto the plate and cooled (see
Real-trial measurement). Also needed: a template cut from a plastic folder, a syringe, a contact surface thermometer,
a 0-10 N force gauge, paper towels, a kitchen scale (0.01 g) and a bench scale (0.1 g) that takes the griddle.

## Files

- `task.py`: the scene, the debris layout, `score()`, `success()` and the scripted `demo()`.
- `debriskit.py`: the task's helper module that spawns the debris pieces as rigid bodies, bonds them to the plate with
  breakable joints, and reads their poses.
- `scrapekit.py`: the task's helper module for the blade-angle rule (the gouge cap); its soil model is not used.
- `yamkit.py`: the task's helper module that spawns the modeled props (griddle plate, scraper).
- `guardkit.py`: the task's helper module for contact between the grippers' own shapes and the scraper.
- `test_task.py`: offline checks of the scorer, the debris layout and the demo plan.
