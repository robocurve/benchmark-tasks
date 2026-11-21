# Play a slow 4/4 rock-ballad groove on an electronic drum kit: technical notes

Two YAM arms each hold a Vic Firth 5A drumstick by friction alone and play a four-bar groove on three Alesis Nitro Max
pads at real size: hi-hat eighths with accents on the numbers, the snare on 2 and 4 together with the hi-hat, a tom
fill on 4 and 4 & of bar 4, and a closing hi-hat and snare together on 1 of bar 5; 41 notes, 8 two-stick pairs, at
75 bpm. The trial is scored from MIDI: in a real trial the drum module's own USB MIDI output, in simulation the
240 Hz hit log written as a MIDI file, read by the same scorer.

> **Status:** the scripted demonstration has been run in Isaac Sim, headless, and scores 1.0 (305 of 1,200 steps; repeated with an identical trace); 11 controls have been run natively, each scoring as expected. **Dream-1 has been run once on the final code and scores 0.0** (1,200 steps; no hit was counted).

## Frames and placement

Tabletop frame (task.json): x forward from the line through the two arm bases, y to the robot's left, z up from the
table top; millimetres in the text, metres in code. Arm bases at (0, +305) and (0, -305).

| What | Where (mm, tabletop frame) |
|---|---|
| Hi-hat pad (10 in), centre of its edge circle | (500, +180), edge 120 above the table, level |
| Snare pad (10 in mesh, dual zone), head centre | (500, -140), head 100 above the table, tilted 5 degrees toward the robot |
| Tom pad (8 in mesh, single zone), head centre | (500, -420), head 90 above the table, tilted 5 degrees toward the robot |
| Gaps in plan | hi-hat edge to snare rim 54; snare rim to tom rim 27 |
| Farthest reach of the kit | tom rim at y = -534; snare rim at x = 639; rack tube along x = 670 |
| Left stick on the hi-hat (strike) | heading +16.0 degrees (from +x toward +y), pitch 18.0 degrees down; tip on the bow 60 mm in from the centre |
| Right stick on the snare / tom (strike) | heading -11.7 / -73.0 degrees, pitch 12.0 degrees down; tip on the head's centre |

The layout puts each pad in front of the arm that plays it (open-handed: the left stick
plays the hi-hat, the right stick the snare and the tom; which hand plays which pad is not scored). The prototype's
reach scan (yam_arena's inverse kinematics on the YAM's URDF, `tools/reach_scan.py`) found every pad reachable, with
the tip lifted 15 to 100 mm over it, every joint at least 12 degrees from its limits.

The arms start at a playing pose, not the station's ready pose (an up-and-down grip cannot be set at the ready pose,
where the finger pads face sideways). Joint angles (rad, joints 1 to 6; `fixed_scene.START_Q`): left
(-0.32717, 0.93901, 0.21786, 0.72071, 0.96495, 1.56246), right (0.25913, 1.56991, 0.27403, 1.29533, -1.10679,
-1.66646). Each gripper points sideways toward the robot's centre line, its finger pads on the stick's upper and lower
sides, the stick at right angles to the gripper, its tip 80 mm over its pad (the hi-hat's bow, the snare's head).

## The parts, and where the numbers come from

| Value used | Source |
|---|---|
| Alesis Nitro Max: 10 in dual-zone mesh snare, 8 in single-zone mesh toms, 10 in hi-hat pad with foot controller; USB MIDI | retail listings (Thomann) |
| Module trigger settings: sensitivity, threshold (THR), crosstalk (CRO), curve (CUR), rim and splash sensitivity | Alesis support page |
| Vic Firth American Classic 5A, wood tip: 406 mm long, 14.4 mm across | retail listings |
| Stick 50 g (a pair is listed at 99 g), balance 175 mm from the butt | estimate, weigh and balance it |
| MIDI notes: 42 closed hi-hat (46 open and 23 half-open count as hi-hat, 44 pedal ignored), 38 snare, 40 snare rim, 43 floor tom | General MIDI; set on the module and recorded with the trial |
| Hoop 12 mm wide, its top 6 mm above the head; shells 65 mm deep | estimate, measure the pads |
| Hi-hat pad 12 mm thick at the edge, the bow rising 6 mm to a bell 90 mm across and 15 mm above the edge | estimate, measure the pad |
| Restitution 0.7 (tip on mesh or rubber); stick friction on the finger pads 0.8 | estimates |
| Velocity curve: 20 + 107 ln(v / 0.5 m/s) / ln 6, clipped to 1 to 127 | estimate |

### Estimates, and how a bench builder measures them

| Value | How to measure it |
|---|---|
| Hit threshold 0.5 m/s | Set THR so that a stick pivoting loosely at its tape mark, its tip let fall from 10 mm above the pad's centre, does not trigger, and from 20 mm does (a free fall from 10 and 20 mm reaches 0.44 and 0.63 m/s; a pivoting stick's tip somewhat less). |
| Velocity curve | Drop the stick that way from 10 to 300 mm; fit the module's velocities against the computed tip speeds; record CUR. |
| Restitution | Stick tip down, dropped 300 mm through a loose tube onto each pad's centre; 240 fps video of the rebound height h: e = sqrt(h / 300 mm). |
| Grip friction and slip | Close the YAM on the stick at its tape mark; hang weights from the tip until it turns (torque = weight x 0.271 m); play 40 loud strokes and photograph the tape mark before and after. |
| Pad positions | Tape marks at each pad's clamp and a protractor on its head; the scorer gives 0 if a pad was knocked 10 mm off its mark. |

## Scene (simulation)

`rdb_drum_groove/kit.usda`, generated by `tools/build_assets.py` from `layout.py` and `fixed_scene.py`:

- `/Fixture/Kit`: one kinematic rigid body per trigger zone, fixed: `hihat` (the rubber pad: its bow and its raised
  bell, two convex pieces), `snare_head` and `tom_head` (each the pad as one solid convex piece under the head, so a
  fast tip always meets a thick collider; a grey mesh head drawn on top), `snare_rim` and `tom_rim` (the black hoop:
  16 convex pieces round the head, 6 mm above it). Each zone is a contact filter of its own.
- `/Fixture/Sticks`: two free rigid bodies, 50 g, frame at the grip point: a 7.2 mm shaft capsule, a 5 mm capsule
  for the taper and a 5 mm tip sphere; solver 32 position and 8 velocity iterations, contact offset 15 mm.
- `/Fixture/Drawn`: the kit's hardware, drawn only (no colliders): a 38 mm rack tube along the table's far side held
  by two C-clamps, and for each pad a clamp, an L-arm and a holder.
- Materials: the pads restitution 0.7 with combine mode "max"; the sticks restitution 0 with combine mode "min" (a
  stick never bounces off the finger pads) and friction 0.8 with combine mode "max" (the YAM's finger colliders have no
  physics material of their own, so the grip's friction is the stick's 0.8).
- Physics as the runtime sets it (read from the stage at every run, `RESULT.json`, `physics_settings`): GPU dynamics,
  GPU broad phase, TGS solver, 240 steps a second, bounce threshold 0.5 m/s (equal to the hit threshold, not above it).
- **CCD is off.** The sticks request it, but the scene runs GPU dynamics with `physxScene:enableCCD = false`, so the
  flag has no effect. Fast tips are stopped instead by speculative contacts: the sticks' 15 mm contact offset is more
  than a tip at 3.6 m/s moves in one physics step (the demonstration's fastest stroke is about 1.8 m/s), and the pads'
  colliders are solid (65 mm shells, a 12 to 21 mm hi-hat pad). In the prototype's runs the tip went at most 0.05 mm
  into a snare or tom head and about 1.4 mm into the hi-hat (approximate) at up to 2.5 m/s. A contact pair that the
  large offset creates before the surfaces touch carries no impulse, and only impulse counts.
- The sticks start between the finger pads: at reset the arms settle at the start pose with the grippers open and
  the sticks held in place by code, the grippers close (0.6 s), the code lets go, and 0.5 s of settling follows;
  from then on only the fingers' friction holds them (`adapter.reset`, which checks each stick within 2 degrees and
  3 mm of its grip). Each finger presses with about 35 N (the gripper's position drive, 5000 N/m, closing on a
  14.4 mm stick; 50 N cap).

## The hit rule (simulation), read every physics step

`adapter.py` wraps `sim.step` (Isaac Lab's ManagerBasedEnv calls it once per physics substep for PhysX), so after
every physics step it reads, through PhysX tensor views: each stick's pose and velocity, its contact force on each of
the five zones, both finger pads' force on it, and every other robot part's force on the pads. `hits.py` turns that
into hits:

- a hit is a held stick's contact with a zone whose impulse over its first two physics steps is at least
  0.003 N s, whose tip moved toward the zone at 0.5 m/s or more on the step before, after the stick had left that
  zone (2 mm, 2 steps), and more than 40 ms after the last hit on that pad;
- a stick is held while both finger pads press on it with 1 N or more, its tape mark within 100 mm of the grip
  point and turned less than 45 degrees; a stick that has left its gripper never plays;
- zones and notes: the hi-hat pad and the tom (rim included) are single zones; the snare's rim is a zone of its own
  (note 40, a wrong hit in this groove);
- a gripper or arm striking a pad (link 4, link 5, the gripper or a finger over 2 N) is one event per contact;
- a stick on one pad (touching it, or its tip within 2 mm of it, so a stick chattering on a head still counts) without
  a break for 0.5 s or more is a rest;
- velocity from the tip's approach speed on the estimated curve.

The hits are written as `media/hits.mid` (Standard MIDI File, format 0, channel 10, 960 ticks a beat at a tempo of
500,000 us a beat, so 0.52 ms a tick; a note-off 50 ms after each note-on) and the other facts as `media/events.json`
(each hit's stick, the gripper or arm strikes, the rests, any dropped stick, and when the end became clean).

In stroke tests run before this package was built (their numbers are under "Physics checked in the prototype"), a
loud stroke's tip came back down onto the pad 8 to 21 ms after the hit at up to 1.3 m/s, because the hand pushes on
toward its target until the next 20 Hz step; the re-arm rule and the 40 ms mask merge these into the hit.

## Scoring (`judge_midi.py`, pure Python, the same code for real and simulated trials)

1. Note-ons become pads by their note (`layout.PAD_OF_NOTE`); note-ons on the same pad less than 40 ms apart are one
   hit. In a real trial, notes sent by a gripper, an arm or a dropped stick (from the video) are removed.
2. One tempo (60 to 90 bpm, every 0.25 bpm, then every 0.01 bpm round the best) and one start: the most written notes
   with a hit on their pad within 50 ms of their place, then the least squared error; then a least squares fit of
   start and beat length to the matched notes. Each note takes the nearest unused hit on its pad within 50 ms.
3. Each hit is a played note (on time within 25 ms, off the grid 25 to 50 ms), a count-in (up to 4 hi-hat hits in the
   bar before the first note), a wrong-pad hit (a rim, or an unmatched hit within 50 ms of another pad's note) or an
   extra hit.
4. H notes played; G pairs together (both notes played, by the two different sticks, within 40 ms); E extra or wrong
   hits plus gripper or arm strikes. Accent rule: hi-hat numbers average at least 25 velocity above the ands, each
   number louder than the ands either side, and the snare averages at least 25 above the ands. Clean end: no hit after
   the last pair, and both sticks held with tips at least 50 mm from every pad for 1 s, starting within 3 s of it (in
   simulation measured from the tip sphere to each pad's collider; the hi-hat counted as a disc as high as its bell).
5. 1.0: H 41, G 8, RMS at most 25 ms, fitted tempo 71 to 79 bpm, the accent rule, E at most 1, a clean end. 0.9: all
   but the accent rule. 0.75: H 37, G 7, RMS 35 ms, E 3. 0.5: H 31, G 5, E 3. 0.25: H 16, E 3. Else 0. A gripper or
   arm striking a pad caps the score at 0.5; a stick resting on a pad for 0.5 s or more caps it at 0.75; in a real
   trial a pad knocked 10 mm off its mark scores 0.

In simulation the adapter judges the hits so far once the end has been clean for 1 s (and ends the trial if the score
is then 1.0) and at the last step; Inspect Robots' scorer `drum_groove_score` takes that last judgement.
`run.py` then writes the MIDI file and the events, and judges them again from the files: `RESULT.json` records both
(`score`, `score_from_files`), and they agree in every run below. The score-vs-hits plot, `scored-hits.csv` and
`score-vs-hits.json` are written from that second judgement.

## The scripted demonstration

`scripted.py`, closed loop on one piece of simulator state (when each stick last touched a pad), as a scripted
demonstration may; it moves nothing itself.

- Strokes move joint 6 alone, the wrist roll, whose axis passes through the grip point: the stick pivots up and down
  about its tape mark. The YAM's wrist behaves as a first-order lag (stiffness 10 N m/rad, damping 1.5 N m s/rad:
  0.15 s), so the tip meets the pad at about lever x overshoot / 0.15 s: 1.8 (left) and 1.65 (right) m/s per rad of
  joint 6 past the contact angle (prototype). Loud strokes: 0.90 and 0.97 rad past contact (about 1.6 m/s, MIDI 90);
  soft: 0.45 rad (about 0.84 m/s, MIDI 52); the tom 0.80 rad.
- The stick waits lifted (joint 6 0.39 rad above contact for a loud stroke, the tip about 90 to 97 mm over the pad;
  0.225 rad, about 48 mm, for a soft one). One control step before its count a soft stroke is thrown: joint 6
  commanded to its overshoot. A loud stroke needs more than the runner's delta limit allows in one step (no joint
  target more than 1 rad from the last), so it is thrown in two: 0.08 (left) or 0.01 (right) rad above contact one step
  earlier, then the overshoot. Both land about 40 ms into the throw's step, so the hand presses on the pad for at most
  about 10 ms; at the first control step after the stick touches the pad, joint 6 is commanded back up.
- The right arm glides between the snare's and the tom's poses in joint space: over about 1.3 s after its snare note
  on 2 of bar 4, and back to the snare in the 0.4 s between the tom's 4 & and the closing note.
- After the closing pair both sticks are lifted 100 mm and held still; the trial ends once the end has been clean
  for 1 s with the score at 1.0.

Controls (`RDB_DRUM_DEMO`), each a variant of the demonstration; expected scores from the scoring rules:

| Variant | What it does | Expected |
|---|---|---|
| flat | every hi-hat stroke the same (0.66 rad, about MIDI 70) | 0.9 (accent rule fails) |
| tempo82 | everything at 82 bpm (quantized to the 20 Hz steps) | 0.75 (tempo outside 71 to 79) |
| no_fill | stops after 3 & of bar 4: no tom, no closing pair | 0.75 (H 37, G 7) |
| flam_bar3 | bar 3's two snare notes thrown one step late (about 50 ms after the hi-hat) | 0.5 (G 6) |
| one_arm | the left stick alone plays the 31 hi-hat notes | 0.25 (G 0) |
| air | every stroke stops 10 mm over the pad | 0 (no hits) |
| press | at each count the tip is brought onto the right pad at about 0.2 m/s and held 0.3 s | 0 (no hits) |
| rest | after each snare note the stick stays on the head 0.6 s | 0.75 (rests) |
| flail | random strokes, about 6 a second for 13 s | 0 (E over 3) |
| no_stop | one more bar after the closing pair | 0 (E 10) |
| gripper | after the closing pair the right gripper, its stick turned up, presses on the snare's near rim | 0.5 (a gripper strike) |

Native results (same machine and code as the recorded run; `validation.json`, `controls`):

| Run | Variant | Expected | Native score | Steps | What the scorer saw |
|---|---|---|---|---|---|
| `control-flat` | flat (must run) | 0.9 | 0.9 | 400 | 41/41 notes, 8/8 pairs, E 0, 75.0 bpm, RMS 2 ms, accent rule fails |
| `control-tempo82` | tempo82 | 0.75 | 0.75 | 400 | 41/41 notes, 8/8 pairs, E 0, 82.0 bpm, RMS 15 ms |
| `control-no_fill` | no_fill | 0.75 | 0.75 | 400 | 37/41 notes, 7/8 pairs, E 0, 75.0 bpm, RMS 3 ms |
| `control-flam_bar3` | flam_bar3 | 0.5 | 0.5 | 400 | 41/41 notes, 6/8 pairs, E 0, 74.8 bpm, RMS 16 ms |
| `control-gripper` | gripper | 0.5 | 0.5 | 420 | 41/41 notes, 8/8 pairs, E 1, 1 gripper strike, 75.0 bpm, RMS 3 ms |
| `control-one_arm` | one_arm (must run) | 0.25 | 0.25 | 400 | 31/41 notes, 0/8 pairs, E 0, 75.0 bpm, RMS 2 ms, accent rule fails |
| `control-air` | air (must run) | 0.0 | 0.0 | 400 | 0/41 notes, 0/8 pairs, E 0, no tempo |
| `control-press` | press (must run) | 0.0 | 0.0 | 400 | 0/41 notes, 0/8 pairs, E 0, 2 rests, no tempo |
| `control-rest` | rest | 0.75 | 0.75 | 400 | 41/41 notes, 8/8 pairs, E 0, 8 rests, 75.0 bpm, RMS 3 ms |
| `control-flail` | flail (must run) | 0.0 | 0.0 | 400 | 29/41 notes, 1/8 pairs, E 90, 83.5 bpm, RMS 28 ms, accent rule fails |
| `control-no_stop` | no_stop | 0.0 | 0.0 | 480 | 41/41 notes, 8/8 pairs, E 10, 75.0 bpm, RMS 3 ms |

## Native runs

All on one NVIDIA A10 (24 GB), Ubuntu 22.04, headless, seed 0, with up to three runs sharing the GPU. `out/` holds each run's `RESULT.json`, scorer trace, MIDI file, events, plot, videos and stills.

| Run | Code | Result |
|---|---|---|
| `scripted-1` (recorded) | final code; hashes in `validation.json` | 1.0 in 305 steps, 272 s |
| `scripted-2` | final code, run again | 1.0 in the same steps; scorer trace, MIDI and events byte-identical |
| `control-flat` | final code, `RDB_DRUM_DEMO=flat`, cap 400 | 0.9 (expected 0.9) |
| `control-tempo82` | final code, `RDB_DRUM_DEMO=tempo82`, cap 400 | 0.75 (expected 0.75) |
| `control-no_fill` | final code, `RDB_DRUM_DEMO=no_fill`, cap 400 | 0.75 (expected 0.75) |
| `control-flam_bar3` | final code, `RDB_DRUM_DEMO=flam_bar3`, cap 400 | 0.5 (expected 0.5) |
| `control-gripper` | final code, `RDB_DRUM_DEMO=gripper`, cap 420 | 0.5 (expected 0.5) |
| `control-one_arm` | final code, `RDB_DRUM_DEMO=one_arm`, cap 400 | 0.25 (expected 0.25) |
| `control-air` | final code, `RDB_DRUM_DEMO=air`, cap 400 | 0.0 (expected 0.0) |
| `control-press` | final code, `RDB_DRUM_DEMO=press`, cap 400 | 0.0 (expected 0.0) |
| `control-rest` | final code, `RDB_DRUM_DEMO=rest`, cap 400 | 0.75 (expected 0.75) |
| `control-flail` | final code, `RDB_DRUM_DEMO=flail`, cap 400 | 0.0 (expected 0.0) |
| `control-no_stop` | final code, `RDB_DRUM_DEMO=no_stop`, cap 480 | 0.0 (expected 0.0) |
| `dream` | final code (hashes equal to the recorded run's), `dreamlabs/dream-1-beta` | 0.0 at the 1,200-step limit, 714 s, 49 Dream-1 requests; no hit |

**The Dream-1 trial** (`out/dream/`; `validation.json`, `runs.dream`): `dreamlabs/dream-1-beta` through
`inspect-robots-dream` (commit b5e936a2) with `dreamlabs-client` 1.0.14, on the final code, with the same scene,
instruction, scorer, cameras and 20 Hz control as the demonstration; 49 action requests (cap 120; our list-price
estimate US$0.07), on an A10 shared with one other Isaac Sim run. Until its first reply arrived (step 33, 1.65 s) the
runtime held every joint, the grippers included, at its measured position, so the sticks sat in the fingers at under
1 N (not held by the hit rule); nothing touched a pad in that time. Its first reply closed both grippers (27 to 33 N;
both sticks held from step 34); from about step 49 it opened both grippers wide, and both sticks fell onto their pads at
2.7 s (left on the hi-hat, right on the snare). The right gripper took the right stick again twice (steps 134 to 214
and 582 to 744), the left gripper the left stick only for a moment (near 13.8 s), and touched the snare with it 21 times, none faster than 0.19 m/s. The four contacts at 0.5 m/s or
more were all by sticks that were not held (2.7 s on the snare and the hi-hat, 53.7 and 53.8 s on the snare's rim), so
none was a hit. The left stick ended on the table beside the left arm's base, the right one loose on the snare. No
gripper or arm touched a pad; 0 of 41 notes and no tempo, so the score was 0 throughout. Elapsed 713.85 s for 1,200
steps. `out/dream/model_run.mp4` (2560 x 704, 20 fps, 60.05 s) is the combined video with the drum track
(`tools/soundtrack.py`; no hits, so silent; `media/sync.json`: both streams start at 0); the images Dream-1 received
are in `dream-input-rgb.zip` and its replies in `dream-action-trace.jsonl`.

## Physics checked in the prototype

Before this package was built, a prototype on the same runtime measured (about 90 to 170 strokes per arm and
condition; the numbers are the prototype's, kept here because they set the design):

- **Grip.** With the finger pads on the stick's upper and lower sides (this package), the stick turned in the pads'
  plane 0.03 to 0.15 degrees a stroke and slid 2 to 13 mm along its axis over 80 to 90 strokes; it tilted less than
  1 degree out of the plane and never dropped. With the finger pads on the stick's left and right sides (the gripper
  pointing down), a stroke's torque lies in the pads' plane, where only friction resists it: the stick turned 7 to 41
  degrees in the first 8 strokes and then missed the pads. With pads on the left and right, joint 4 also sits within 12
  degrees of its -97 degree limit at the strike, too little to throw with the wrist.
- **Pressing.** Strokes that landed after a control step's boundary (the hand pressing 30 to 60 ms) turned the stick
  about 0.28 degrees a stroke; landing 4 to 12 ms before it, about 0.06.
- **Contact.** Rigid pads with restitution 0.7: each contact one physics step, the held stick rebounding at 0.63 to
  0.80 of its approach speed, peak force at most 19 N. Compliant contact (2x10^4 and 3x10^3 N/m) made contacts last 5
  to 7 physics steps with rebound about 0.5; it is not used.
- **Speed.** 0.29 to 2.47 m/s from 0.2 to 1.3 rad of joint 6 past contact; joint 6 reached about 10 rad/s, half its
  20 rad/s limit. Loud and soft strokes 40 MIDI velocity apart (the accent rule needs 25).
- **Soft strokes.** 80 soft strokes per arm on this layout: 80 hits each, no misses, no double hits (left 0.82 m/s,
  right 0.79 m/s).

## Sound in the videos

No sound is simulated. `tools/soundtrack.py` synthesizes a drum track from `media/hits.mid` (every logged hit, at its
exact time, its loudness and brightness from its velocity; procedural voices written for this package: a closed hi-hat
from six detuned square waves and noise with a fast decay, a snare from two decaying sines and a noise burst, a tom
from a falling sine) and lays it under the combined overhead and side video (20 fps), captioned "Sound added offline
from the logged hits; not simulated; the robot heard nothing". At 20 fps the picture can trail the sound by up to
50 ms. The tool checks the result (`media/sync.json`): both streams start at 0, their lengths agree within a frame,
the decoded soundtrack cross-correlates with the rendered one at 0 ms, and every hit's onset is found within a few
milliseconds of its time.

## Tests and dry runs

```sh
python -I -m unittest discover -s tests      # or: python -m pytest -q tests
python tools/build_assets.py                 # regenerate the scene file from layout.py
python tools/reach_scan.py layout --name P   # the reach scan (numpy and yam_arena)
```

- Standard library only (Python 3.9 and later): `test_hits.py` (the trigger: a clean hit, a bounce inside the mask,
  two strokes 80 ms apart, a slow press and a rest, re-arming, a split impulse, a dropped stick, a light touch; the
  MIDI writer and reader; the velocity curve; the written groove), `test_judge.py` (synthetic trials written as MIDI
  and read back, one for every control's level and every rule's edge: tempo limits, pair spread, RMS, count-in, merged
  note-ons, the clean end, a real trial's video annotations; the plot's text is ASCII and English), `test_scene.py`
  (the scene file is what the builder writes; pads where the layout says; real sizes and the kit's relative layout;
  sticks, materials, drawn hardware; with yam_arena, the frozen stroke poses still put each tip on its pad),
  `test_package.py` (task.json limits and numbers against the code, README, validation.json against the recorded runs
  and its hashes against the files here).

## What has not been shown

- Dream-1 on more than one trial: it was run once (the scene is fixed and the seed is 0; its replies may still differ
  between calls).
- Every estimate above on real parts: above all the restitution, the velocity curve, the threshold calibration, the
  grip's friction and how far a real stick turns in a real gripper.
- A real gripper or arm strike on a real pad (the gripper control presses the simulated right gripper on the snare's
  rim at 25 N).
- Runs on other GPUs (only the A10).

## Third-party assets and licenses

- The pads, sticks and kit hardware are convex hulls, capsules and boxes generated by `tools/build_assets.py`; no
  meshes, textures or images from any third party. Product names identify what was modelled.
- Plot and caption font: DejaVu Sans, bundled with matplotlib (DejaVu Fonts licence, a free licence derived from the
  Bitstream Vera Fonts licence). Not bundled here.
- Drum sounds: synthesized by `tools/soundtrack.py` (this package's code, MIT); no samples or recordings.
- Robot, station and cameras: `yam-arena` ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena);
  its README states no license yet; the YAM arm model in it is MIT). Not bundled.
- Isaac Lab-Arena (Apache 2.0) and Isaac Lab (BSD-3-Clause). Not bundled. The full list, with each robot, station and
  camera asset and its license, is in README.md, "Third-party assets and licenses".
- This folder's code and scene: MIT, see `LICENSE`.

## Sources

- Alesis Nitro Max kit: [Thomann](https://www.thomannmusic.com/alesis_nitro_max_kit.htm); module trigger settings:
  [Alesis support](https://support.alesis.com/en/support/solutions/articles/69000832380-alesis-surge-nitro-modules-adjusting-understanding-trigger-settings).
- Vic Firth American Classic 5A: [zZounds](https://www.zzounds.com/item--VIC5A).
- General MIDI percussion notes: [General MIDI, percussion key map](https://en.wikipedia.org/wiki/General_MIDI#Percussion).
- Open-handed playing: [Open-handed drumming](https://en.wikipedia.org/wiki/Open-handed_drumming).
- The basic rock beat (hi-hat eighths, snare on 2 and 4): [Soundbrenner, "Rock beat"](https://www.soundbrenner.com/blogs/articles/rock-beat).
