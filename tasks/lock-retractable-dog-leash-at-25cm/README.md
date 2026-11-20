# Pull a retractable dog leash out to 25 cm against recoil tension, press the brake and slide its lock home

Dog owners lock a retractable leash at a short length before a road crossing, a doorway or a crowd: they draw the
tape out against the reel's recoil spring, press the thumb brake in and push it forward into its lock, and the tape
then stays at that length. Here a full-size retractable leash lies on a bench mat with its snap hook clipped to a
dog collar on a post. The robot must draw the housing out until the snout is in the green 23 to 27 cm band, hold it
there against the spring (2.5 N rising to 5 N), press the orange thumb button in and slide it forward behind its
catch, and let go. If the button is only pressed, it springs back out on release and the tape reels back to 0 cm; if
it is pushed forward without pressing, its catch blocks it after 2 mm. The hard part is physical: one hand holds a
part under live spring load while the other does a two-axis press-then-slide on an 11 mm button, and the result must
survive both hands letting go.

## The parts (real sizes and materials)

Positions are in cm from the midpoint between the arm bases, x forward, y left; the table top is z 74.75 cm.

| Part | Real product | Size and material | Where |
|---|---|---|---|
| Retractable leash | TUG 360 Tangle-Free Retractable Dog Leash, Small (16 ft / 4.9 m of 9.5 mm flat nylon tape, dogs under 15 kg) | ABS housing with rubberised (TPE) handle loop, 173 x 109 x 33 mm, 330 g: white round reel drum 96 mm across, grey tape snout 26 x 16 x 24 mm, blue handle loop (grip rail 20 x 72 mm, rear bar 84 x 22 mm, both 33 mm deep), orange thumb button 14 x 24 x 18 mm with a 5 mm thumb ridge | Lies flat on its side on the mat, snout at x 26.5, y +33.0 (the ruler's 0 cm), housing x 21.6 to 32.4, y +15.7 to +33.0; thumb button on the near (-x) edge of the handle at x 20.0 to 21.4, y +20.2 to +22.6 |
| Tape | the leash's own orange flat nylon webbing | 9.5 mm wide, 2 mm thick, runs level 1.65 cm above the mat from the snout to the snap hook | appears between snout and collar as it pays out |
| Snap hook | the leash's chrome swivel snap hook | 20 mm long, 7.6 mm across, zinc alloy | y +33.0 to +35.0, clipped to the D-ring |
| Dog collar | any 25 mm (1 in.) flat nylon dog collar with a steel D-ring, e.g. PetSafe Nylon Dog Collar, Medium, red | woven nylon strap 25.4 mm wide, 4 mm thick; chrome D-ring 20 mm, 2.5 mm wire | buckled round the post 1.0 to 3.5 cm above the table; D-ring toward the leash at y +34.8 to +36.0 |
| Anchor post | STZ 1 in. x 2-1/2 in. galvanized Schedule 40 pipe nipple (Home Depot model PDG N1X212) in a Southland 1 in. galvanized malleable iron floor flange (model 511-605HN) | 33.4 mm OD steel pipe on a 95 mm flange, 70 mm tall | centred x 26.5, y +38.3, flange screwed to a 10 x 10 cm plywood plate held by a 2 in. C-clamp |
| Bench mat | non-slip silicone work mat, dark slate, cut to size | 220 x 545 x 4 mm silicone rubber | x 16 to 38, y -21 to +33.5 |
| Ruler and band | self-adhesive metric tape-measure strip; 40 mm green vinyl tape | 24 x 370 mm white strip, 0 cm at y +33.0 to 35 cm at y -2.0; green band 28 x 40 mm over 23 to 27 cm | along x 35, the band at y +6.0 to +10.0 |

The leash's mechanism as built in `lock_retractable_dog_leash_at_25cm.py` (one fixed `mechkit` Mech, three chained
slide joints):

| Body | Rides on | Motion | Spring, friction | What it does |
|---|---|---|---|---|
| `housing` | bench | slides along the tape line (-y), 0 to 34 cm | recoil 10 N/m from a 25 cm preload: 2.5 N at 0 cm, 5.0 N at 25 cm; 0.15 N slide friction | the leash body, pulled out against the reel spring |
| `brake_carrier` | housing | pressed in (+x), 0 to 11 mm | 220 N/m return spring (2.4 N at 11 mm) | the brake plunger under the thumb button |
| `thumb_button` | brake carrier | slides forward (+y), 0 to 11 mm | detent spring with stops at 0 and 11 mm | the orange button and its catch lug |

A catch lip inside the housing (8 x 12 x 26 mm) stands 2.0 mm in front of the button's catch lug at rest, so the
button cannot slide forward unless it is first pressed in at least 7 mm; pressed fully, the lug clears the lip by
4.0 mm. Slid forward and let go, the return spring pushes the lug back onto the lip, which holds the brake pressed at
about 7 mm: that is the lock. Clearances are checked with `mechkit.self_gap` and `mechkit.gap` at every stop.

## What the scripted demo does and reads

The demo reads the live extension of the leash (its housing slide joint) from the simulator to place the handoff
grasp, trim the pull to 25.0 cm and aim the button push; everything else comes from the fixed layout
(object_jitter=0), its joint encoders and gripper opening. It never moves an object or writes a joint.

1. **Draw (left arm):** grips the blue grip rail across its 20 mm width and draws the housing to 15.5 cm at up
   to 3.5 cm/s.
2. **Handoff (right arm):** opens to 48 mm, reaches into the handle loop, grips the 22 mm rear bar and checks the
   grip by its finger gap; the left arm lets go and lifts away.
3. **Draw to 25 cm (right arm):** draws to 25.0 cm and holds the housing there against the 5 N spring.
4. **Lock (left arm):** with closed fingers, tilted 60 degrees forward, it pushes the orange button 11 mm in (the
   brake engages), keeps pushing while it sweeps forward so the button slides 11 mm behind the catch, then backs
   off; the button's lug settles on the catch lip. Once the brake is on, the right hand eases its pull to where it is.
5. **Let go:** the right arm opens and lifts, both arms return to the ready pose with open grippers and wait while
   the locked leash stays at 25 cm.

The demo takes about 61 s (1,221 steps at 20 Hz) and keeps every tool point under 0.14 m/s.

## Fidelity

- The housing slides on a level sprung joint along the taut tape line instead of lying free on the mat with a
  clock-spring reel inside, so it cannot yaw or lift; in a real trial the mat's friction and the holding hand do that
  job. The recoil (10 N/m, 2.5 to 5.0 N) is our estimate for a small tape leash; measure it on the unit before trials.
- The reel brake is modelled: while the brake plunger is pressed 5.5 mm or more, an `on_step` hook stops the reel
  (the spring's rest moves to and follows the housing, so a locked reel pulls neither way, and its slide friction rises to 60 N); below 4.5 mm the recoil
  spring is back. The catch lip and lug that hold the brake pressed are real colliding geometry.
- The button's return to its 0 or 11 mm position is a detent spring; the brake plunger's return is a plain spring.
- The tape is shown as 25 mm look-only pieces that appear as the snout passes them (no rope is simulated).
- The simulator scores the last step without opening the grippers, so a leash still held, or a gripper open less
  than 70 mm, scores 0.
- Hold (slide friction) units are not yet confirmed in Isaac; no score depends on its exact value.

## Files

- `lock_retractable_dog_leash_at_25cm.py`: the task file (scene, brake hook, `score()`, `success()`, `demo_script`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit, unchanged (procedural geometry and joints, the
  closed-loop demo controller, geometry helpers).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Pull the dog leash out to the green 25 cm band, press and slide its thumb lock to lock the reel there, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lock_retractable_dog_leash_at_25cm.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/leash-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lock_retractable_dog_leash_at_25cm.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/leash-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the ruler's 0 cm mark is the leash's fixed start.
- **Runtime per trial:** the demo takes about 61 s of simulated time (1,221 steps); a Dream-1 trial runs up to 2,400
  steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 230 s for the demo and about 360 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` (a full score) has held for `SUCCESS_HOLD_S` = 8 s,
  otherwise `score()` at the last step.

## Scoring

Measured at the end of the trial with the bench ruler, an end photo and the trial video (final 3 s); in simulation
every condition is checked at every step of the final 3.0 s (60 steps).

1. **Score 0** if, at any time in the final 3 s: either gripper is open less than 70 mm; any finger pad is within
   30 mm of the leash housing or its button; the lock is not engaged (brake pressed at least 5.0 mm and the button
   slid at least 8.5 mm forward; in a real trial, the orange button sits forward in its lock position in the end
   photo); or the snout moves 2.0 mm or more along the ruler. A value that is not a number at any step also scores 0.
2. **Otherwise, by where the snout's front face ends on the ruler:** 1.00 for 23.0 to 27.0 cm (the green band); 0.50
   for 20.0 to 30.0 cm; 0.20 for 10.0 cm or more outside that; 0.00 under 10.0 cm.

Possible scores: 0.00, 0.20, 0.50, 1.00. Doing nothing scores 0 (the leash stays at 0 cm, unlocked). Pressing the
brake without sliding the button lets the tape reel back in when the hand lets go, so it scores 0; so does holding the
housing at 25 cm to the end.

## Real-world build

Parts (about 40 US dollars): the TUG 360 Small leash (about 16 dollars, Amazon, Chewy, Walmart); a 1 in. nylon dog
collar; the STZ pipe nipple PDG N1X212 and Southland floor flange 511-605HN (Home Depot); four wood screws, a 10 x 10 cm
plywood plate and a 2 in. C-clamp; a silicone work mat; a metric tape-measure strip and green vinyl tape. No custom
or printed parts.

1. Screw the flange to the plywood plate, thread in the pipe nipple, and clamp the plate so the post stands at
   x 26.5, y +38.3.
2. Tape the mat down at x 16 to 38, y -21 to +33.5. Buckle the collar snugly round the post with the D-ring facing
   the mat and clip the leash's snap hook to it.
3. Lay the leash flat, button side toward the robot, tape fully wound in; stick the ruler along x 35 with 0 cm level
   with the snout's front face, and the green tape over 23 to 27 cm.
4. Check the recoil pull with a luggage scale at 0 and 25 cm and the button's travel (press, then slide) by hand.

Forces stay well under the YAM's roughly 40 N per finger: the draw needs 2.5 to 5 N, the press about 2.4 N and the
slide under 1 N.

## Real-world reset and cleaning

Between trials (about 10 s, nothing consumed): pull the orange button back so it pops up and let the tape reel in
gently by hand, then lay the housing back flat with its snout at the 0 cm mark. Everything is dry plastic, nylon,
silicone and steel; wipe the mat with a damp cloth now and then so its grip stays the same.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
