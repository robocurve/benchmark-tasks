# Tune a ukulele's G string to pitch with a clip-on tuner

A soprano ukulele stands on its side in front of the robot, as it is held to play: soundboard facing the robot, neck
along the table with the headstock on the robot's right, held in two foam blocks and a strap, with a clip-on tuner on
its headstock. Its G string, the top string, has gone flat by a semitone (it sounds F#4). The robot has to tune it
back to G4 the way people do: one hand plucks the string over the soundhole, again and again, while the other turns
that string's tuning button a little at a time until the tuner shows G, centred; then it lets go of the button,
plucks once more to check, and moves both hands away.

What makes it hard, and what is tested:

- **Finding the right button.** Four buttons stick out of the headstock's two edges: the G and C strings' straight up
  from the upper edge, the E and A strings' straight down from the lower edge. The G string's is the one on the upper
  edge nearer the nut, 37 mm from the C string's (as on the real KA-15S: the G and A strings wind onto the posts
  nearer the nut). Turning the C string's instead moves the C string and scores 0.
- **A small, stiff geared button, turned a long way.** The button is 18 x 12 x 6 mm and turns against a drag; at the
  estimated 0.22 cents per degree a semitone takes about 455 degrees of it, more than a wrist can roll in one grip, so
  the turning hand has to let go, roll back and pinch again.
- **Closed-loop, fine rotation.** Full marks need the pitch within 5 cents, about +-23 degrees of the button at that
  estimate; the gear's free play (about 4 degrees) means a reversal first turns the button without changing the pitch.
- **Two hands in turn.** The tuner reads only for 3 s after a real pluck (the string pushed at least 2 mm across the
  strings and let spring back), so plucking and turning have to be interleaved and the reading taken while it lasts.
  Full marks also need a pluck after the button's last turn (the final check the instruction asks for), so tuning by
  counting turns, blind, cannot reach 1.0.

Why on its side: lying face up, the buttons would point sideways at table height and the gripper would have to point
horizontally forward to pinch one, which the YAM cannot do there. Standing on its side, the G button points straight
up; the right hand pinches it from above, in front of and to the left of its own base, and turns it with its wrist
roll, and the left hand plucks the G string from above, in front of the soundboard.

> **Status: the scripted demonstration has been run in Isaac Sim and scores 1.0** (675 of 2,400 steps;
> "Native runs" below). **Dream-1 has been run once and scores 0.0** (2,400 steps; it never touched the ukulele).
> What else has and has not been checked is in "Tests and dry runs" and "What has not been shown".

## Frame and placement

Tabletop frame (task.json): x forward from the line through the two arm bases, y to the robot's left, z up from the
table top; millimetres in the text, metres in code. Arm bases at (0, +305) and (0, -305). The code describes the
ukulele in an *instrument frame* (`layout.py`): origin on the plane of the body's back under the middle of the nut, x
across the strings toward the G string, y along the strings toward the bridge, z out of the back toward the strings.
`fixed_scene.py` turns it a quarter turn about y (the instrument's x points up, its z toward the robot) and puts its
origin at (322, -128, 100.5). Key places on the table:

| What | Where (mm) |
|---|---|
| Soundboard (vertical, facing the robot), back | x = 262, x = 322 |
| The neck's centre line; nut, headstock tip, tail | z = 100.5; y = -128, -249, 286 |
| Body (241 long, lower bout 178) | y 45 to 286; upper edge 166 above the table at the soundhole, 189.5 at the lower bout |
| G string's button (middle; its top at z 148.5), nearer the nut | (282, -163, 142.5) |
| C string's button, farther from the nut | (282, -200, 142.5) |
| E and A strings' buttons (pointing down) | (282, -200, 58.5), (282, -163, 58.5) |
| Soundhole (51 mm) | centre (262, 97, 100.5) |
| G string over the soundhole (the pluckable stretch's middle) | (253, 97, 118); the C string 11.7 below it |
| Tuner screen (middle), facing the robot and up, its normal 50 degrees above level | (223.5, -239.7, 113.9) |
| Foam block under the lower bout's widest point (60 long, 12 mm cheeks rising 15); the body's edge rests 11.5 above the table | y = 232 |
| Foam block under the neck's lower edge at the 3rd fret (one cheek, behind); the neck's edge rests 81.8 above the table | y = -73 |
| Strap round the lower bout, behind the bridge | y 237 to 262 |

In simulation the blocks' tops stop 0.5 mm short of the edges they hold (11.0 and 81.3 above the table): the
ukulele is welded in place and the gap keeps its colliders from touching the blocks'.

The placement was chosen for reach (the runtime's notes: an arm pointing down reaches the table 0.10 to 0.50 m in
front of its own base, less higher up, and not in front of the other arm). The right hand pinches the G button with
its tool point at (282, -163, 150), 282 mm in front of its own base and 142 mm to its left; the left hand plucks about
330 mm (in plan) from its base. Reach, joint limits and the hands' clearances are checked kinematically by
`tools/dryrun_demo.py` and were confirmed in the native runs. If the layout changes, change `fixed_scene.py` (and the
numbers in task.json, which `tests/test_package.py` checks against the code).

## The ukulele, and where the numbers come from

Kala publishes the KA-15S's measurements and strings ([K] in `layout.py`), and its product photos show how the
strings reach the posts and what the tuners are ([P]). The setup (from the task's design) fixes the strings' tuning,
the G string 100 cents flat and the Snark ST-2 on the headstock tip. **Everything else is an estimate: measure it on
a real Kala KA-15S** and change `layout.py`, which is the one place every number lives (the scene file is generated
from it by `tools/build_assets.py`).

| Value used | Source |
|---|---|
| Scale 346, overall length 535, body length 241, 12 frets | Kala's specification: 13.625, 21.062 and 9.5 in |
| Upper bout 132, waist 113, lower bout 178 (the body's widths) | Kala's specification: 5.187, 4.467 and 7.0 in |
| Body depth 60 (constant here; a real body tapers a little toward the neck) | Kala's specification: 2.367 in |
| Fingerboard 35 wide at the nut, 42.5 at the neck/body joint | Kala's specification: 1.375 and 1.672 in |
| Aquila Super Nylgut strings | Kala's specification |
| G and A strings to the posts nearer the nut, straight from it; C and E strings angling outward, just inside those posts, to the posts farther from it, which are only a little farther out | Kala's front photo of the KA-15S |
| Open-gear tuners (worm and gear exposed on a plate) with pearloid buttons | Kala's back photo of the KA-15S |
| Where along the body the bouts and the waist fall, and the outline's curves | estimate, measure on a real Kala KA-15S |
| Neck joins the body at the 12th fret (173 from the nut), so the headstock is 121 from nut to tip (535 less 241 less 173); fretboard 5 thick, ends 185 from the nut | estimate, measure on a real Kala KA-15S |
| Neck 19 to 22 deep (fretboard top to back) | estimate, measure on a real Kala KA-15S |
| Soundhole 51 across, centre 225 from the nut | estimate, measure on a real Kala KA-15S |
| Bridge 60 x 17 x 8, 336 to 353 from the nut | estimate, measure on a real Kala KA-15S |
| String spacing 26 at the nut, 40 at the saddle; string middles 1.5 above the fretboard plane at the nut, 5.5 at the saddle | estimate, measure on a real Kala KA-15S |
| Headstock 52 wide, 14 thick, flat (in the neck's plane) | estimate, measure on a real Kala KA-15S |
| Posts 35 and 72 from the nut; near posts 16, far posts 18.5 from the centre line; strings wound on the posts' inner sides | estimate, measure on a real Kala KA-15S |
| Each open gear drawn as one chrome box behind the headstock (14 along the neck, 12 thick, 10 to 30 from the centre line) | stand-in: the worm and gear are not modelled; the box carries the button's shaft and is in the fingers' way as the gear would be |
| Buttons 18 x 12 x 6, shafts 20 behind the headstock's face, buttons 36 to 48 from the centre line | estimate, measure on a real Kala KA-15S |
| 0.22 cents per degree of the button (about 80 cents a turn) | estimate (a 14:1 worm on a 6 mm post with a nylon string); measure as below |
| Gear free play about 4 degrees | estimate, measure on a real Kala KA-15S |
| Button drag 0.03 N m | estimate, measure on a real Kala KA-15S |
| Clockwise seen from the button's end raises the pitch | estimate (assumed for all four); measure as below |
| G string tension 36 N at pitch (gives the spring across the strings, 458 N/m at the soundhole) | estimate, measure on a real Kala KA-15S |
| Snark ST-2 head 46 x 38 x 14, screen 30 x 24, 34 in front of the headstock's face | estimate, measure on a real Snark ST-2 |
| Foam blocks 60 long, cheeks 12 thick rising 15 | estimate; any blocks that hold the ukulele on its side at these heights |

### Estimates, and how a bench builder measures them

| Value | How to measure it on the real KA-15S |
|---|---|
| Cents per degree | Tune G to pitch with a phone tuner that shows cents; put a paper protractor behind the G button and a tape flag on it; tighten exactly one turn while plucking; read the change (about 80 cents expected). Repeat three times and loosen-then-tighten before each. |
| Free play | With the string sounding, tighten a little, then loosen slowly and note on the protractor how far the button turns before the reading starts to fall. |
| Which way is up | Note which way, seen from the button's end, the reading rises. If it is anticlockwise, set `TIGHTEN = 1` in `layout.py`. |
| Drag | Wind a thread round the button's edge and pull with a 0 to 5 N spring scale while it turns slowly: torque = force x radius (9 mm). |
| String spring and return | Push the G string across the strings at the soundhole's middle with the spring scale: 1 N should move it about 2 mm. |
| Sizes and heights | Ruler and caliper with the ukulele in its blocks: button size and height above the table, post and button positions, soundhole, string height over it, where along the body the bouts and the waist are. |
| Tuner | Caliper on the ST-2; protractor for the screen's tilt once clipped on. |

## The stated rules (no sound is simulated)

All three are in `tuning.py` (pure Python), fed once per 20 Hz control step by the adapter.

**Pitch.** Each string's pitch, in cents from its own note, follows its button's angle. The button's joint angle is
turned into tightening degrees (clockwise seen from the button's end is tightening). The string's post follows the
button through a dead band 4 degrees wide, the gear's free play: on a reversal the button turns 4 degrees before the
post moves. The pitch changes 0.22 cents per degree of the post. At reset the free play is taken up in the tightening
direction, as the reset notes require (the G string is brought up to F#4 from below), so tightening acts at once and
a first loosening turn does nothing for 4 degrees. The C, E and A strings start in tune (0 cents), the G string at
-100. Strings do not stretch, drift with temperature or break; the button has no end stop (a geared tuner turns on and
on).

**Pluck.** A pluck counts when the G string's 36 mm stretch over the soundhole is pushed at least 2 mm across the
strings (either way: on its side, up or down) and is back within 0.5 mm no later than 0.3 s after the last step on
which it was 2 mm out, with no part of either arm touching it (contact force over 0.05 N) at any step in between.
Plucks closer together than 0.3 s count once. So a string pushed and eased back by the finger, held, or only brushed
is not a pluck, and a pluck does not depend on how the fingertip let go (slid off, lifted) or which part of the hand
pushed it. The stretch slides on a spring set from the string's tension (458 N/m: about 0.9 N for 2 mm) with a damped
return that is back within 0.5 mm 0.07 to 0.09 s after release from 2 to 5 mm out (on the spring's own equation,
`tests/test_scene.py`). In the recorded run the fingertip pressed the string until it was 5.2 to 5.3 mm out (with up
to 2.6 N), and all three plucks counted on the first try.

**Tuner.** From the step a pluck counts the tuner reads for 3 s (a new pluck restarts the 3 s); the rest of the time it
is blank: needle at its left stop (-45 degrees), every lamp off. While it reads it shows the G string's computed pitch
live (turning the button while it reads moves the needle), as a chromatic tuner does: the nearest note's lamp (F#, G
or G#; halfway between two notes counts as the upper one), the needle 0.9 degrees per cent from that note (45 degrees
at 50 cents, right for sharp), and the green lamp when within 3 cents of that note. So at the start a pluck shows
**F#, centred and green**: the policy has to read the note, not only the needle. The simulated display is a stand-in
for the ST-2's: it has lamps for F#, G and G# only, and beyond 50 cents outside them it shows no note and its needle
sits at that end (a real ST-2 would show the note's name). It shows only the G string (only the G string's stretch is
pluckable in simulation).

How it is drawn: the screen is part of the tuner's body; the note names are printed on it; each lamp is a coloured
patch drawn hidden and shown (made visible) while lit; the needle is a separate body on a hinge about the screen's
normal whose position target is set every step from the display (it follows one control step behind). The side
review camera shows the lamps and needle clearly (`out/scripted-1/media/side_cam.mp4`); how well the wrist cameras
see the screen while the hands work has not been checked.

## Scene (simulation)

`rdb_ukulele_tuning/ukulele.usda`, generated by `tools/build_assets.py` from `layout.py` and `fixed_scene.py`:

- One fixed-base articulation: its root link (body, neck, fretboard, headstock, the four open gears' boxes, posts,
  nut, bridge, saddle, strings and the clip-on tuner, all one rigid body) is welded to the world (as if strapped into
  its blocks). Parts of the articulation do not collide with each other.
- Four pearloid buttons, each a body on a revolute joint about its shaft, no end stops; a velocity drive toward rest
  with its torque capped at 0.03 N m stands in for the gear's drag (it resists turning and holds nothing at rest). The
  buttons' colliders are flat 18 x 12 x 6 mm boxes, their broad faces toward +y and -y at rest.
- The G string's 36 mm stretch over the soundhole: a capsule on a prismatic joint across the strings (vertical on the
  table), +-8 mm (the C string is 11.7 mm below), with a spring-damper drive (458 N/m, 21.4 N s/m, 0.51 kg of joint
  inertia, so that it returns in 0.07 to 0.09 s). The rest of each string between the nut and the saddle is a static
  capsule (1.2 mm across, a little thicker than the real 0.7 to 0.9 mm, so cameras see it); from the nut to its post
  each string is drawn only.
- The tuner's needle: a body on a revolute joint about the screen's normal, with a stiff position drive.
- Rigid colliders only (boxes, capsules, cylinders and convex hulls). The body's figure-eight is drawn as one mesh over
  17 hidden convex slices; the soundhole is drawn (its collider is closed: a fingertip cannot go into the body).
- Static: the two foam blocks (bases and cheeks). Drawn only: the strap.

Nothing in the scene depends on a pitch: the pitch, the pluck and the display are the stated rules above.

## Scoring

`grading.py` (pure Python) scores every step; the final score is the last step's. It says what task.json's scoring
says:

- **Within 5 cents of G4: 1.0** if the final check stands and both grippers' centre points (from the joint readings)
  are at least 50 mm from the ukulele and the tuner (measured to boxes around body, neck, headstock, buttons, tuner
  head and clip); **otherwise 0.75**. The final check: the G string was plucked after the G button's last turn, so the
  tuner showed the final pitch. In simulation (`tuning.FinalCheck`) the last counted pluck stands until the G button
  is more than the gear's free play (4 degrees) from its angle at that pluck; then the button has turned since, and
  turning it back does not restore the check. A turn within the free play can still move the pitch by up to 0.9
  cents. On the bench the judge decides it from the trial video: a pluck visible after the last turn of the button.
- **Over 5 and up to 15 cents off: 0.75; up to 30: 0.5; up to 50: 0.25; over 50 either way: 0.** The bands depend
  only on the G string's final pitch (which way it is off does not matter). The final check and the grippers'
  clearance affect nothing but the 1.0.
- **0** whatever the G string reads if the C, E or A string is more than 15 cents from its start. (On the bench also:
  the ukulele left its blocks or the strap came undone, the tuner knocked off, a string broken; none of these can
  happen in simulation, where the ukulele and tuner are fixed and strings cannot break.)
- The trial ends at 2,400 steps, or once 1.0 has held for 1 s (21 steps in a row: the trial ends 20 steps after 1.0
  is first reached, unless it is lost meanwhile); since 1.0 needs the final check, that can only happen after the
  final pluck. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

Unintended ways through, each tested in `tests/test_grading.py` or `tests/test_tuning.py`:

| Way | What happens |
|---|---|
| Turn the C string's button (beside the G string's, farther from the nut) | C moves: 15 cents is 68 degrees of its button; more is 0 |
| Turn the G button the wrong way, or a whole semitone too far (G#4) | 0 |
| Tune it but keep hold of the button, or leave a gripper within 50 mm | at most 0.75 |
| Brush or press the string without pushing it 2 mm out, or push it and ease it back | no pluck: the tuner stays blank |
| Read only the needle (F# at the start shows centred and green) | the note lamp shows F#, not G |
| Turn without plucking, counting turns (blind) | at most 0.75: no pluck after the last turn |
| Pluck, then turn the button again and stop without plucking | at most 0.75: the pluck came before the last turn |

## The scripted demonstration

`scripted.py`, closed loop on simulator state (button angles, computed pitches, the free play's state, the string's
displacement, the pluck count); only the scripted demonstration reads it, no policy observation contains it, and it
moves nothing itself. Step numbers are from the recorded run.

1. Both arms at once (steps 1 to 80). The right hand goes 30 mm above the G button, pointing straight down along its
   shaft, pads 20 mm apart; its wrist roll (joint 6) is chosen among the equivalent ones (a flat button looks the same
   every half turn) to leave the most room for tightening. The left hand, closed, fingers pointing down and tipped 18
   degrees toward the soundboard, goes to a waypoint 40 mm in front of the G string and 40 mm above the body's upper
   edge, then in a straight line down to 30 mm above the string. Below that edge it only moves in straight lines, in
   front of the soundboard; it leaves the same way at the end (a joint-space move from the ready pose would pass over
   the body and bring the fingertips down on its upper edge).
2. The right hand corrects for sag from its joint readings, comes down along the shaft and closes on the button (pads
   5.9 mm apart on the 6 mm button, up to 29 N of contact); the pinch is checked from the pads' measured separation
   and retried up to three times.
3. Pluck: the left hand comes down until its fingertip is 2 mm above the G string (1 mm past it toward the
   soundboard), corrects its sag, presses the string down until it reads 4.5 mm out (the arm gives under the string's
   spring, so the command goes further), slides 14 mm toward the robot in 0.15 s so the string slips off the
   fingertip and springs back, lifts 25 mm and waits for the pluck to count, then goes back to 30 mm above the string.
   Up to three tries.
4. The reading (the G string's computed pitch, what the tuner shows) gives the turn: -error / 0.22 degrees, plus the
   free play still to take up if the turn reverses. Joint 6 alone turns the button (up to 120 degrees a second); it
   uses about 224 degrees of roll (its limits, +-120 degrees, less 8 degrees each way), and when it runs out the hand
   opens, rises 20 mm, rolls back half a turn or more (lined up with the button as measured) and pinches again. After
   each turn the button's measured angle is checked and corrected.
5. Repeat 3 and 4 until the reading is within 2 cents (recorded: two plucks, one turn of 455 degrees with three
   regrips and one correction).
6. Final check: the right hand opens and rises 30 mm along the shaft, and the left hand plucks once more, so the last
   pluck comes after the button's last turn (letting go could turn it). If that reading is more than 2 cents off,
   the right hand takes the button again (lined up with it as measured), corrects, and the check is repeated.
7. Both hands return to the ready pose; the trial ends once 1.0 has held for 1 s.

Controls: `RDB_UKULELE_DEMO=wrong_button` turns the C string's button (the one farther from the nut) by the G
string's error once and must score 0; `RDB_UKULELE_DEMO=hold_on` tunes but never lets go of the button and must score
0.75 at the step limit; `RDB_UKULELE_DEMO=blind` tunes from the simulator's pitch without ever plucking, lets go and
withdraws, and must score 0.75 (in tune, no pluck after the last turn).

## Native runs

All on one NVIDIA A10 (24 GB), Ubuntu 22.04, headless, seed 0, with the GPU shared with two other Isaac Sim
processes. `out/` holds each run's `RESULT.json`, scorer trace, videos and stills.

| Run | Code | Result |
|---|---|---|
| `out/scripted-1` (recorded) | final code; hashes in `validation.json` | 1.0 in 675 steps, 489.5 s |
| `out/scripted-2` | the same code | 1.0 in 675 steps, 448.3 s; scorer trace byte-identical to scripted-1 |
| `out/control-wrong_button` | the same code, `RDB_UKULELE_DEMO=wrong_button` | 0 at the 2,400-step limit, as required |
| `out/dream` | the same code (hashes equal to the recorded run's), `dreamlabs/dream-1-beta` | 0.0 at the 2,400-step limit, 725.9 s, 73 Dream-1 requests |

**The recorded run, step by step** (`out/scripted-1/RESULT.json`, diagnostics): both arms in place at step 80; pinch
at 128 (joint 6 at 26.7 degrees); first pluck counted at 175 (string 5.2 mm out), tuner F#, centred, green; turn of
455.0 degrees from 189 with regrips at 210, 300 and 390 (joint 6 back to -72.2, -70.8 and -70.8 degrees), 444.9
degrees after the first pass (-2.2 cents), corrected to 454.5 (-0.10 cents) by 458; second pluck counted at 506: G,
+0.04 cents; let go at 520 to 546 (the button moved 0.26 degrees); third pluck, the final check, counted at 593: G,
+0.05 cents; both hands back from 607; 1.0 from step 655 (both grippers clear), end at 675. C, E and A unmoved; arm
contact only on the G button (29 N peak) and the G string (2.6 N peak); none on the C button beside it or the rest of
the ukulele, and none between the arms. Elapsed 489.5 s, with the GPU running two other Isaac Sim trials at the same
time.

**Repeatability.** The same code run twice gave byte-identical scorer traces and identical diagnostics: physics and
scoring are deterministic on this machine. The videos differ byte for byte (path-traced rendering), and the elapsed
times differ (489.5 and 448.3 s) with the load on the shared GPU.

**Controls run natively.** `wrong_button` (`out/control-wrong_button`): after one pluck (F#, centred) the right hand
pinched the C string's button, beside the G string's and farther from the nut, and turned it by the G string's error
(454.5 degrees, three regrips), then let go: C +100.1 cents, G still -100; the score was 0 from step 208 (C past 15
cents) to the 2,400-step limit, and the G button was never touched (1,298.7 s on the shared GPU). `blind` and
`hold_on` were run in the dry run only.

**The Dream-1 trial** (`out/dream/`; `validation.json`, `runs.dream`): `dreamlabs/dream-1-beta` through
`inspect-robots-dream` (commit b5e936a2) with `dreamlabs-client` 1.0.14, with the same scene, instruction, scorer,
cameras and 20 Hz control as the demonstration; 73 action requests (cap 128; our list-price estimate US$0.10). It
closed both grippers (the right one from about step 215) and moved both arms above the table. No arm contact with the
ukulele, tuner or buttons was recorded, no pluck was counted and all four buttons stayed at 0 degrees, so G stayed at
-100 cents and the score was 0 throughout. The right gripper came closest, 29.5 mm from the ukulele at step 910; the
left never came closer than 73.2 mm. Elapsed 725.9 s, with the GPU shared with one other Isaac Sim process for part of
the trial. The combined video `out/dream/model_run.mp4` (2560 x 704, 10 fps, 120.1 s) was made with the same ffmpeg
command as `demo.mp4`; the images Dream-1 received are in `dream-input-rgb.zip` and its replies in
`dream-action-trace.jsonl`.

## Tests and dry runs

```sh
python -m unittest discover -s tests -v       # or: python -m pytest -q tests
python tools/dryrun_demo.py                   # needs numpy and yam_arena (no Isaac Sim)
python tools/build_assets.py                  # regenerate the scene file from layout.py
```

- Standard library only (run where this was written, Python 3.9, no NumPy): `test_tuning.py` (pitch, free play,
  unwrapping, pluck rule, display timing and readings, the final-check rule, the demonstration's arithmetic),
  `test_grading.py` (every band and its edges, the final check: in tune with a final pluck 1.0, without one 0.75, a
  pluck before the last turn not counting; zero conditions, clearance, end of trial), `test_scene.py` (the scene file
  is what the builder writes, to the last written digit; Kala's published sizes; which post each string reaches;
  bodies, joints, drives and limits; colliders; lamps; orientation and placement; the string's spring returns in
  about 0.1 s), `test_package.py` (task.json limits and numbers against the code, the scoring text against the
  scorer, README, validation.json against the recorded run and its hashes against the files here).
- `test_demo.py` runs `tools/dryrun_demo.py`, which drives the real demonstration through the arms' kinematics with a
  lag, a button that turns with a gripper closed on it and a string pushed by the closed fingers, scored by the real
  rules, and checks by position how close the YAM's own collision hulls (link 5, gripper and fingers, sampled over
  their surfaces about 2 mm apart) come to the ukulele. Skipped without yam_arena; run on the GPU machine:

| Case | Score, steps | Closest approach (mm) |
|---|---|---|
| nominal | 1.0 in 636 | left hand: body 4.9, C string over the soundhole 3.0; right hand: C button 18.2, headstock, neck, posts and gear boxes 6.7, strings between the nut and the posts 27.4, tuner 30 or more |
| slower arm, sagging 2 to 4 mm | 1.0 in 703 | left: body 2.8, C string 0.8; right: C button 15.6, headstock parts 2.7, strings 26.0 |
| C button turned instead | 0 (C +100 cents, G still -100) | right: G button (now the neighbour) 18.2, headstock parts 6.7 |
| blind | 0.75 (in tune, no pluck after the last turn) | |
| never letting go | 0.75 (a gripper on the button) | |

  Wrist roll per grip (nominal): the first grip turns joint 6 from 26.7 to 111.5 degrees (the button 84.5 degrees),
  the next two from -68.5 to 111.7 (180 degrees each), the last from -68.4 to -58.5 (the corrections, 10 degrees);
  joint 6's limits are +-120 degrees, so every grip stays at least 8 degrees inside them.

## What has not been shown

- Dream-1 on more than one trial: it was run once (the scene is fixed and the seed is 0; its replies may still
  differ between calls).
- Every estimate in the tables above, on a real ukulele and tuner, including the turning direction and the cents per
  degree.
- The open gears' worm and gear (a box stands in for each).
- That the wrist cameras can read the tuner while the hands work (the side review camera can; it is not a policy
  input). In the side view the left hand hides the string while it plucks; each counted pluck shows on the tuner's
  lamps and needle.
- Runs on other GPUs (only the A10 above); the trial is deterministic there.

## Third-party assets and licenses

- The ukulele, tuner, foam blocks and strap are boxes, capsules, cylinders, convex hulls and simple meshes generated
  by `tools/build_assets.py`; no meshes, textures or images from any third party. Product names identify what was
  modelled.
- Robot, station and cameras: `yam-arena` ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena);
  its README states no license yet; the YAM arm model in it is MIT). Not bundled. The dry run reads the arm's
  collision meshes from it at run time.
- Isaac Lab-Arena and Isaac Lab: Apache 2.0. Not bundled.
- This folder's code and scene: MIT, see `LICENSE`.

## Sources

- Kala KA-15S: [Kala's product page](https://kalabrand.com/products/ka-15s) ("Specs" and "Measurements": scale,
  overall and body length, frets, bouts, waist, body depth, fingerboard widths, Aquila Super Nylgut strings; no tuner
  type is listed); Kala's product photos, [front](https://kalabrand.com/cdn/shop/products/KA-15S_20_F_2048x.jpg)
  (which post each string reaches) and [back](https://kalabrand.com/cdn/shop/products/KA-15S_20_B_2048x.jpg)
  (open-gear tuners with pearloid buttons).
- Snark ST-2: [Sweetwater](https://www.sweetwater.com/store/detail/SnarkST-2--snark-st-2-clip-on-super-tight-chromatic-allinstrument-tuner),
  [B&H](https://www.bhphotovideo.com/c/product/1276197-REG/snark_st_2_super_tight_clip_on.html): chromatic clip-on tuner,
  vibration and microphone modes, calibration 415 to 466 Hz.
