# Reconnect a car battery's positive cable

A car battery stands on the bench in a mock engine bay: an inner fender to the robot's right, standing to the hood
line, and a hold-down bar across the back of the lid. Its positive cable runs from a P-clip on a steel stand behind the
battery, forward over the hold-down and across the lid, to its clamp. The clamp has been set back over the positive
post, but the cable, held in its clip, holds it 6 mm above its seat, riding high on the post's taper, and its
pinch-bolt nut is run down by hand only. A 13 mm combination wrench stands in a holder in front of the battery. The
robot is told only: "Reconnect the positive battery cable to the battery."

The job, as a technician does it: push the clamp fully down on the post by its cable, keep holding it down, and tighten
the nut with the wrench in the other hand until a torque wrench reads 5 to 7 N m; then take the wrench off. The
instruction says none of this. The scene enforces it:

- The cable springs the clamp back up. Pushed down and let go, the clamp rides back up off its seat (the cable pushes
  with 4,000 N/m: 12 N for 3 mm at 6 cm behind the post, 24 N at the 6 mm it rides high). It has to be held down until
  the nut has closed the bore on the post.
- Once the bore closes, the clamp is pinched where it is: tightened while it rides high (or crooked), it stays high and
  loose (score 0).
- The nut pushes back. It turns only against its torque, and that torque comes back through the wrench into the hand:
  past the bore closing it rises 1 N m per 30 degrees, so 6 N m is 44 N on the handle where the demo holds it. 5 to
  7 N m is tight; taking it past 7 N m at any time over-tightens the clamp (score 0.25, even if it is backed off
  after). The wrench has to come off the nut and away from the battery for full credit: a steel wrench left on a
  battery can bridge its posts.
- The battery stands in a mock bay. The right hand starts outside the fender, a solid board between its arm and the
  battery, and comes in over its top; the wrench swings in the open front of the bay, below the hood line. The bay is
  surroundings, not a hard limit: measured, a hand sent straight at the cable also clears the fender (Results, probes).


Each trial starts differently: the seed moves the battery and its bay (up to 12 mm front to back and 15 mm side to
side), tips the clamp a different way as it sits 6 mm up on the post (up to 2.5 degrees, and turns it up to 4
degrees), and moves the wrench stand (up to 15 mm and 10 mm). So a memorised motion cannot score.

## Revision for review TK-3JJ2Q3
| Review point | What changed |
|---|---|
| Real-trial peak torque was inferred from a modelled 1 N m per 30 degrees, never measured on the clamp | The angle estimate is gone. The wrench the robot uses is now instrumented: the same 13 mm combination wrench with two strain-gauge bridges on its shank, 3 cm and 7 cm from the box end's centre, logged at 100 Hz. The two bending readings, extended in a straight line to the box end's centre, give the torque at the nut wherever the handle is held. The 7 N m limit is checked from that log: over-tightened if the logged torque stayed above 7 N m for 0.15 s at any time in the trial. The setup lists the wrench, the reset gives its check against known weights and the dial torque wrench, and the scoring text uses the log. |
| Update the simulation to match | The sim's wrench now records the torque passing through its box end into the nut every control step: 0 with the box end off, near 0 while it only rests on the nut, rising as the hand pushes, and the nut's full torque while the nut slips forward. The scorer takes the peak from that record with the same 0.15 s rule, and no longer from the nut's angle. `CLAMP_WRENCH_LOG=trace.csv` writes the record (seconds, N m; one trial per file). The modelled 1 N m per 30 degrees is still how the simulated nut resists; no real trial is scored from it. |
| (Our own check) tightening the gauges cannot see | The nut turned clockwise more than 5 degrees with the box end off it (the open end, a finger) is not on the wrench's log, so it counts as not verified and is scored as over-tightened (the final torque goes on with the box end, which holds all 12 points where an open end can round the nut), on the bench (from the video) and in the sim. A trial with no log is void and is run again. |
| Flag: the fender does not force the hand to detour | Considered and measured: a fender tall enough to block the straight path (over 36 cm) is beyond the arm's 37.5 cm reach, so it would also block the demonstrated path. The bay is kept as a reproducible obstacle the hand must clear by 2 to 3 cm, and that limit is stated under the previous revision below. |

## Revision for review TK-BM66SK
| Review point | What changed |
|---|---|
| The simulated wrench turned the nut without reacting the 5 to 7 N m torque | The override is gone. The nut's torque (1 N m running, then 1 N m more per 30 degrees once the bore closes) is now a friction that resists whatever turns it, wrench on or off. The box end holds the nut's flats through a stiff spring (300 N m/rad), so the arm has to push the full torque through the wrench and its grip: 6 N m is 44 N on the handle at the demo's grip, 13.5 cm out. Nothing drives the nut to the wrench's angle any more. |
| The full-score demo did not show that the robot can apply the torque | The demo was re-planned for the reacted torque and the new bay: six 30-degree strokes with the handle between level and 30 degrees down, where the arm's 7 N m wrist joints can push it, the grip moved out along the handle before the first stroke, and each stroke aimed past 30 degrees by the arm's measured lag. A feasibility check (`dev/feasibility_gate.py`) runs two strokes against the reacted torque and prints the peak torque the arm reached. New demo and Dream-1 recordings are below. |
| Engine-bay surroundings that constrain access, reproducibly | The battery now stands in a mock bay, fixed to the bench and set out from the battery: an inner fender (18 mm plywood, 24.5 x 30.5 cm, 4.5 cm from the battery's right-hand end, its top at the hood line, 8 cm above the lid) and a hold-down bar across the back of the lid (25 x 6 mm steel flat on a rubber strip, on two 8 mm J-bolts). Sizes, materials and how to build them are in the setup and the Objects table. The right hand starts outside the fender, which stands between its arm and the battery, and comes in over its top. Measured limit of this bay: the front and the top are open, and a hand sent straight at the cable without the over-the-fender waypoints still gets in (it crosses 2 to 3 cm above the 30.5 cm top, and it glanced off and over a 35 cm top we tried). The arm's reach there ends 37.5 cm above the bench, so a taller fender would shut the right hand out; the bay is present and reproducible, but it does not force a detour. |
| Flag: use an inert battery or insulate the negative post | The setup now uses an inert dummy battery (an empty BCI group 24 case with its lead posts, sand-ballasted to 18 kg, no acid, no charge). The reset says to fit a rubber boot on the negative post if a live battery is used instead. |
| (Our own check) a nut taken past 7 N m and backed off could score 1.0 | The scorer now keeps the highest torque the nut reached in the trial (held for 0.15 s, so one solver step's jitter does not count). Past 7 N m at any time scores 0.25. (The real-bench check is now the instrumented wrench's log: see the revision above.) |
| (Our own check) the box end's ring did not collide | The ring now collides. Its 17 mm bore clears the nut's 15 mm corners, so it goes on only when it is centred on the nut within about 1 mm (the catch tolerance is 2 mm, down from 4 mm). The pinch bolt sits 13 mm out from the collar (was 4.8 mm), on 18 mm ears, so the 22.4 mm ring clears the collar and the battery's front edge. |

## Revision for review TK-GABW96
| Review point | What changed |
|---|---|
| The cable had no anchor for its upward force | The cable now runs from the clamp straight back over the lid to a rubber-lined steel P-clip on a stand fixed to the bench, 1.5 cm behind the battery. It holds the clamp 6 mm high; pushed down 3 mm at 6 cm behind the post it pushes back 12 N (4,000 N/m). The setup gives the stand's size and place, and the reset gives the load check that reproduces it. |
| Nut angle from an unspecified loose start could give full credit to an under- or over-tightened joint | Tightness is now the nut's torque, read with a dial torque wrench: 5 to 7 N m for full credit, the usual torque for a top-post clamp's pinch nut. Under 5 N m (loose) or over 7 N m (over-tightened) scores 0.25. The start is defined: the nut run down by hand until it stops against the ear. |
| No reset condition for repeatable trials | Reset below: loosen, re-set the clamp and let the cable hold it, run the nut down by hand, load check, replace a clamp taken past 7 N m. |
| Check the demo's gripper clearance at the end | The scorer now measures every finger pad to every part of the clamp (collar, ears, nut, lug, cable to its clip) and the wrench. The demo ends 172.9 to 183.7 mm clear on seeds 0 to 4 (40 mm needed), with the wrench back in its holder. |

## Reset
Use the inert dummy battery (no acid, no charge). With a live battery instead, fit a rubber boot over the negative post
before every trial, so a steel wrench across the posts cannot short them. Loosen the nut a full turn, lift the clamp off
the post, set it back on and let go: the cable holds it where it rests, collar 6 mm above the boss (check with a 6 mm
drill shank). Run the nut down clockwise by hand until it stops against the ear (finger-tight, no tool): that is the
start. Put the wrench back in its holder; set the battery, the bay (fender board and hold-down J-bolts), the cable stand
and the tool stand on the trial's marks, and run the hold-down nuts down finger-tight until the bar just touches the
lid. Load check: pushing the cable down 3 mm from where it rests, 6 cm behind the post, takes 12 N (a push gauge); if
less, move the cable stand closer to the battery, if more, farther. A clamp taken past 7 N m may be deformed: replace it.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable), from this folder:

```bash
COMMON=(--embodiment yam_arena -E task=car_battery_clamp.py -E object_jitter=0 -E control_hz=20
        -E cam_width=320 -E cam_height=176 --instruction "Reconnect the positive battery cable to the battery."
        --scorer task_score --max-steps 1800 --store-frames)

# Dream-1 (needs a Dream API key in ~/.dreamlabs/credentials)
inspect-robots run "${COMMON[@]}" --policy dream --log-dir ~/yam-eval/car-battery-clamp-dream

# The scripted demo, which reaches full score
inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/car-battery-clamp-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/car-battery-clamp-demo/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial ends at 1.0 once the full-score state has held for 1 s; otherwise it
scores the last step. `--seed N` and `--epochs K` give other starts.

**Keep `-E object_jitter=0`.** The task file lays out each trial's start itself, from the trial's seed.

To check that the arm can push the nut's torque before recording, `python dev/feasibility_gate.py` runs the scripted
policy from a start with the clamp already seated and the nut at 4.5 N m, makes two strokes and prints the peak torque.

## Scoring
Every check is a measurement a person can make on the real bench: a feeler gauge under the clamp, a dial torque wrench
on the nut, a protractor or a photo for the lug, the instrumented wrench's logged torque for the peak, and a tape for the battery.

| Outcome | Score |
|---|---|
| Clamp seated, nut 5 to 7 N m and never past 7 N m, lug in line, wrench off and away, every finger pad at least 4 cm clear, all still for 1 s | 1.0 |
| The same, but the wrench still on the nut or on or against the battery, a hand on the clamp or the wrench, or something moving | 0.75 |
| Seated, tight and never over-tightened, lug 10 to 30 degrees out of line | 0.5 |
| Seated but the nut under 5 N m at the end, or over 7 N m at the end or at any time before; or seated and tight with the lug over 30 degrees out | 0.25 |
| Clamp high or crooked (tightened there, it is a loose joint), off the positive post, on the negative post, or the battery moved over 5 mm | 0 |

The numbers:

- **Seated:** at four points under the collar (front, back, left, right, at its outer edge), the gap to the post's
  boss is at most 1.0 mm; the clamp is square to the post within 2 degrees and centred on it within 3 mm.
- **Tight:** the nut holds 5 to 7 N m, the usual torque for a top-post clamp's pinch nut. A dial torque wrench with a
  13 mm socket, turned clockwise on the nut, reads 5 to 7 N m as the nut starts to move. Turn 0 is the finger-tight
  start: run down by hand until it stops against the ear, the split still open. With the collar seated, the bore closes
  on the post after 30 degrees more; every millimetre the collar sits higher takes another 100 degrees. Past that point
  the torque rises 1 N m per 30 degrees.
- **Never over-tightened:** the nut was never taken past 7 N m during the trial. This is measured with the wrench the
  robot uses, which is instrumented (Objects): its logged torque at the nut, in the tightening direction, never stayed
  above 7 N m for 0.15 s (15 samples in a row at 100 Hz), and the dial wrench's end reading is at most 7 N m. Tightened
  off the log counts as not verified, scored as over-tightened: the nut turned clockwise more than 5 degrees with the
  box end off it (the open end or a finger on it; from the trial video, by the white line on the nut's face). The final
  torque goes on with the box end, which holds all 12 points where an open end can round the nut, and it is the end
  the gauges measure. A trial
  with no log is void and is run again. The sim's wrench records the torque passing through its box end into the nut
  every control step (20 Hz); the scorer takes the highest value held for 3 steps (0.15 s) and counts the nut's turn
  with the box end off.
- **Lug in line:** the cable lug points within 10 degrees of the cable's run (straight back to its clip).
- **Still:** the clamp, the nut and the wrench each moved less than 1 cm and 0.1 rad (6 degrees) over the last second.
- **Wrench away:** its box end at least 2 cm from the nut, and the wrench not on or against the battery: no part of it
  more than 3 cm above the bench within 1 cm of the battery's outline (lying on the bench beside the battery is fine).

On a real bench: slide a 1 mm feeler gauge under the collar at the four points, turn a dial torque wrench clockwise
on the nut until it starts to move and read 5 to 7 N m, check in the wrench's log that the torque at the nut never stayed above 7 N m for 0.15 s (and in the video that the nut was
not tightened with the box end off it), check the lug's angle against the battery's edge, and tape the battery's position. The finger-pad clearance comes
from the arms' logged joint angles at the end (forward kinematics), or a photo with a scale beside the clamp; stillness
and the 1 s hold from the video's last seconds.

## Objects
Positions are in cm on the bench, from the midpoint between the two arm bases: x forward (away from the arms), y to
the robot's left. Heights are above the bench top. Each seed moves the battery, the bay and the cable stand together,
and the wrench stand, within the ranges above; the setup screenshot and the demo video show seed 0.

| Object | Type, size, material | Where |
|---|---|---|
| Battery | inert dummy lead-acid car battery, BCI group 24 size: 26.0 x 17.3 x 22.5 cm, black polypropylene case with real lead posts, no acid and no charge, ballasted with dry sand to 18 kg | centre x 30, y 0 nominal (seed 0: x 30.5, y -1.4); long side along y; posts along the front edge |
| Positive post | SAE lead post on a 4 mm lead boss (28 mm across): cone 19.5 mm across at its base, 17.5 mm at its top, 18 mm tall; red mark on top, red + on the lid | right-hand end: 2.2 cm in from the front face, 8.5 cm right of the battery's centre |
| Negative post | SAE lead post, 17.9 mm at its base, 15.9 mm at its top, 18 mm tall, on the same boss; black mark, white - on the lid | 2.2 cm in, 8.5 cm left of centre |
| Inner fender | 18 mm plywood board, 24.5 cm long, 30.5 cm tall (its top is the hood line, 8 cm above the lid), painted primer grey, standing on edge on two steel angle brackets clamped to the bench | to the robot's right, its inner face 4.5 cm from the battery's right-hand end, from 2 cm in front of the battery's front face to 5.2 cm behind its back face |
| Hold-down | black steel flat bar 25 x 6 mm, 30 cm long, on a 2 mm rubber strip, on two 8 mm steel J-bolts (M8 threaded rod) standing in 5 x 5 cm steel foot plates clamped to the bench; nuts finger-tight, the bar just touching the lid | across the lid along its length, its centre line 1.65 cm in from the back face; the J-bolts 1.2 cm beyond each end of the battery; the cable passes over it |
| Clamp | lead top-post clamp, 0.14 kg with its lug: 12-sided collar 31.1 mm across its flats, 16 mm tall, tapered bore like the post; two ears reaching 18 mm out from the collar's front face, with an M8 pinch bolt across them 13 mm out from the collar (carriage head on the right ear, nut on the left); crimp lug and red 4 AWG cable out the back; white paint mark on the left ear | on the positive post, 6 mm above its seat, held up by the cable |
| Cable clip | black rubber-lined steel P-clip, 15 mm across, on a 25 x 6 mm steel flat-bar upright and a 10 x 6 cm steel base plate clamped to the bench | 1.5 cm behind the battery's back face, in line with the positive post; clip axis 24.3 cm above the bench |
| Nut | M8 steel hex nut, 13 mm across flats, 6.5 mm thick, white paint line across its face (pointing up at the start; it shows in the video whether the nut was turned with the box end off) | on the bolt, left side |
| Wrench | 13 mm combination wrench, chrome vanadium steel, 18.1 cm long, 0.105 kg, 12-point box end and open end. Instrumented: two strain-gauge bridges (each a pair of 350-ohm foil gauges, one on each narrow edge of the shank) 3 cm and 7 cm from the box end's centre, each a half bridge of 3 mm gauges bonded with cyanoacrylate under a thin protective coat (about 2 g in all); a slack 2 mm lead (6 g/m) leaves the shank 8 cm from the box end and hangs from a hook 60 cm above the tool stand, to a two-channel strain-gauge amplifier and data logger sampling both bridges at 100 Hz. Torque at the nut = (7 x M3 - 3 x M7) / 4, M3 and M7 the bending moments read at 3 cm and 7 cm; the two gains are set from the two weight checks in the reset notes. Any 13 mm combination wrench of this size that passes that check may be used | standing open end down in the holder |
| Wrench stand | steel pedestal 6 x 7 x 12 cm with an orange-painted wooden holder on top, 5 x 4.4 x 3 cm, slot 9 mm wide along x; 2.5 kg | centre x 10.5, y 10 nominal (seed 0: x 11.45, y 9.4), in front of the battery's left half |

## Design notes
- **The instruction is the goal only, on purpose.** A technician is asked to reconnect a battery, not told to push the
  clamp down first or to hold it while tightening. Knowing that a clamp tightened high is a loose joint (it heats,
  cranks the engine slowly and eventually fails to start it), and that the cable springs the clamp back up unless it
  is held down while the nut is tightened, is what the task measures.
- **Two hands are needed.** One hand pushes the clamp down by its cable and holds it there; the other turns the nut
  with the wrench. Seated and let go before the bore closes, the clamp rides back up about 6 mm, and a nut tightened
  there pinches it high (the seat-and-let-go failure path scores 0).
- **The wrench has to be fitted and pushed.** The nut's torque comes back through the wrench, so the hand has to push
  the handle against it: 44 N at 13.5 cm for 6 N m. The robot takes the wrench from its holder, slides the box end on
  square to the flats with the handle level, pushes it 30 degrees down, takes it off, swings it back up, puts it on
  again and pushes it round again: six strokes in all (the 12-point box end fits every 30 degrees). The swing is kept
  between level and 30 degrees down because there the push falls near the line of the arm's wrist joints, which have
  7 N m each; with the handle higher the wrist cannot push 6 N m, and lower down the open hand meets the battery's
  front face and the tool stand. Against the nut the arm lags its target, by more as the torque rises, so each
  stroke aims past 30 degrees by the lag the feasibility check measured (32 degrees down for the first, at the 1 N m
  running torque, up to 41 for the last), and the wrench gets to about 30. Before the first stroke the hand lets go,
  while the box end holds the wrench on the nut, and takes the handle again further out (13.5 cm), where the push
  needed is smaller. After every stroke it keeps hold, brings its target back to where the wrench is (30.5 degrees
  down) and lets the push it has built up run down before it slides the box end off; pulled off still pushing, the
  arm would spring on and fling the wrench. At the end of each push, while the nut holds the ring centred, it reads its
  own distance from the bolt from its joint angles and moves the rest of its plan along the handle by the difference,
  so the next fit goes on centred. After the last stroke it moves back in along the handle, takes the wrench off and
  puts it back in its holder.
- **The bay is in the way, not a barrier.** The right hand starts outside the fender and comes in over its top (the hood line) to
  hold the cable between the post and the hold-down. The wrench works in the open front of the bay.
- **Starts vary per trial**, so the demo cannot replay a fixed motion. The scripted demo looks at the top camera's
  first frame, finds the battery and the wrench stand there (the camera's published calibration, the sizes in this
  setup, colour and edge detection), and plans from those estimates. It also corrects its arms' sag from its own joint
  readings. While it runs it reads no simulator state. Two constant offsets in its edge finder (about 1 to 2 mm, the
  render's soft edges) were calibrated once against known start layouts, as a camera calibration would be.
- **How it differs from the accepted M12 sensor-cable task (TK-3SHVWE).** There, the loose part would spin with the
  coupling, so one hand stops it turning while the other hand turns a knurled ring by hand. Here, a spring (the stiff
  cable) pushes the part off its seat, so one hand holds it down against that push; the other hand fits a tool on a
  nut's flats, pushes it against the nut's rising torque in strokes, re-fits it every stroke and takes it away. The
  failure is also different: a clamp tightened high looks connected but is pinched loose, so the end state, not the
  motion, shows whether the job was understood.
- **Cheats score 0 or 0.25:** pushing the battery (over 5 mm), tightening the clamp where it rides, seating it and
  letting go before tightening, moving it to the negative post, or doing nothing score 0. Turning the nut by hand
  cannot pass 1 N m (the fingers cannot grip a 13 mm nut hard enough), so a seated clamp without the wrench scores
  0.25; so does a nut short of 5 N m, or one taken past 7 N m and backed off.

## Fidelity
- Rigid bodies with real sizes and masses, except the nut: 30 g in the sim (7 g real), so the solver holds it steady.
- Solver aids on the nut, none of which changes the torque it takes to turn it. A real nut's inertia (about 2e-7
  kg m^2), held between the box end's stiff spring and the bolt's friction spring, made the solver spin it at over
  1,000 rad/s for half a second at each catch in our gate runs, and the turn it logged gave false torque peaks. So its
  inertia is padded to 2e-4 kg m^2 about the bolt and 1e-4 across it, its turning speed is capped at 20 rad/s (the
  strokes turn it at under 1 rad/s), and the nut and the wrench get 64 solver iterations. Turning the padded nut takes
  at most 0.04 N m in the gate runs (its inertia times its largest angular acceleration), under 1 % of 5 N m. Across
  the bolt the nut is locked to the clamp, so that padding only adds to the clamp's inertia, which is moved slowly.
  The battery is free on the bench (18 kg, friction 1.1); the hold-down is drawn 1 mm above the lid, so in the sim it
  stops the battery tipping or lifting but does not clamp it.
- The post's taper is four stacked cylinders. The collar's grip on the taper is a friction force of 4 N along the post
  (it stays where it is left, and goes down when pushed harder), with 0.3 mm of side play, a sprung rocking of up to 5
  degrees front to back (15 N m/rad) and 0.8 degrees side to side. Down on the boss (within 0.3 mm of its seat) the
  rocking spring is 200 N m/rad: the collar's bottom face rests on the boss there and cannot rock, and the collar and
  the battery do not collide, so the spring stands in for that support (without it the nut's 1 N m running torque
  rocked a seated, held clamp 5 degrees and the bore closed crooked). A clamp opened up for fitting behaves this way; a
  corroded one needs more force.
- Only the first 7 cm of cable behind the lug is drawn colliding (where a hand can hold it); the rest, to the clip, is
  drawn only. Its stiffness is a spring at the lug, 6 cm behind the post: bent down from where it rests at the start,
  it pushes up with 4,000 N/m (12 N for 3 mm), until the collar is pinched.
- Once the bore closes on the post, the collar is pinched: square (within 2 degrees) it locks at its height and heading
  then; crooked, it binds loose and crooked (its centring spring is released and the cable's weight tips it).
- The nut turns on the bolt against a friction torque: 1 N m until the bore closes (30 degrees with the collar seated,
  100 degrees more per millimetre it sits higher), then 1 N m more per 30 degrees. The 5 to 7 N m band is the usual
  torque for a top-post clamp's pinch nut (one vehicle maker's manual gives 7 N m); the ramp from the bore closing to
  it is modelled, not measured on a clamp. The friction is a stiff spring (200 N m/rad) capped at that torque, whose
  anchor follows the nut as it slips, so whatever turns the nut feels the whole torque, and the nut stays where it is
  left (it gives back at most 2 degrees, under 0.1 N m, when the push stops). The scorer reads the nut's torque at its
  final turn, as a dial torque wrench would on the real bench. The instrumented wrench is the same rigid wrench: its
  gauges and its lead (about 5 g hanging on it) are not modelled. Its reading in the sim is the torque the nut's
  friction spring is wound to in the tightening direction while the box end is on the nut: near 0 while the wrench
  only rests there, rising as the hand pushes, the nut's full torque while the nut slips forward, and 0 with the
  box end off. The scorer keeps the highest reading held for 3 control steps (0.15 s), the rule the real log is
  scored with. Why 0.15 s: the wrench's own weight adds at most 0.09 N m (0.105 kg, 9 cm out) and is zeroed out; a
  real over-tightening lasts as long as the push that makes it, and the arm's commands change every 0.05 s, so
  three commands in a row is the shortest push scored; a shorter spike that still left the nut over 7 N m is
  caught by the dial wrench's end reading.
- The wrench's box end goes on the nut when it is slid on square (within 2 mm of the bolt's axis, 4 degrees of its
  direction and 7 degrees of a flat) and pushed at least up against the nut's face (its mouth's chamfer takes the
  corners from up to 2 mm off centre; 7.5 mm out from the nut's centre, the ring's face touches the nut's at 6.75 mm).
  Its ring collides with the nut, the clamp, the battery, the bay and the hands;
  its 17 mm bore clears the nut's 15 mm corners. Its grip on the flats is a joint, not contact (a 12-point profile's
  contacts at a 30 g nut are not stable in the solver): while it is on, it is locked square and centred on the nut, as
  its chamfer pulls it, and the wrench and the nut turn together through a 300 N m/rad spring (1.3 degrees of give at
  7 N m), so the nut's torque reaches the hand. Along the bolt only 0.8 N holds it on (its fit's friction), and it
  comes off when pulled 9 mm back. Gate runs with a loose fit instead (1 to 2.5 mm of play, 4 to 15 degrees of tilt)
  let the hand twist the wrench crooked until it jammed or fell off.
- The bay's fender board, hold-down bar and J-bolts are fixed boxes and cylinders; the fender's angle brackets and the
  J-bolts' foot plates are not drawn.

## Runtime and seeds
- The scripted demo's plan is 82 s; the run ends at full score after about 1,567 control steps (20 Hz) of the 1,800 allowed. One trial takes a few
  minutes on an L40S, simulator start-up included.
- `--seed 0` (the default) is the start the setup text and screenshot show. `--seed N` and `--epochs K` give other
  starts.

## Results (scripted policy)
One trial per start, scored by this folder's scorer. The demo plans from the first top-camera frame only.

Measured 2026-11-18 on this folder's code (an L40S). Every run ended at full score after 1,564 to 1,567 control steps.

| Start (`--seed`) | Score | Nut turned | Torque at the end | Peak in the wrench's log | Pads clear | What happened |
|---|---|---|---|---|---|---|
| 0 (the setup text and screenshot) | 1.0 | 188.2 degrees | 6.21 N m | 6.25 N m | 172.9 mm | Seated, held, six strokes, wrench back in its holder |
| 1 | 1.0 | 187.4 degrees | 6.17 N m | 6.21 N m | 183.7 mm | Seated, held, six strokes, wrench back in its holder |
| 2 | 1.0 | 188.0 degrees | 6.19 N m | 6.23 N m | 174.1 mm | Seated, held, six strokes, wrench back in its holder |
| 3 | 1.0 | 187.7 degrees | 6.16 N m | 6.20 N m | 179.9 mm | Seated, held, six strokes, wrench back in its holder |
| 4 | 1.0 | 188.4 degrees | 6.19 N m | 6.24 N m | 180.1 mm | Seated, held, six strokes, wrench back in its holder |

The clearance column is the scorer's smallest distance from any finger pad to any part of the clamp, the nut, the cable
to its clip, or the wrench at the end; the full-score check needs 40 mm.

Failure paths, start 0: the same plans with one hand changed (set `CLAMP_FAIL=release`, `nohold` or `idle`, or
`CLAMP_STROKES=4` or `8`, in the environment of the demo run; unset for the demo itself):

| Path | Score | Why |
|---|---|---|
| The right hand seats the clamp, then lets go before the nut is turned | 0 | The cable springs the clamp back up 6 mm (gap 5.95 mm at the end); the box end does not go on the nut there, and the nut is never turned |
| Eight strokes instead of six | 0.25 | The nut is taken to 7.95 N m (the wrench's log peaks at 7.99 N m): over-tightened, with the clamp seated and the wrench put away |
| Four strokes instead of six | 0.25 | The nut stops at 4.11 N m (125.0 degrees): seated but loose |
| Nothing holds the clamp; the left hand tightens alone | 0 | The clamp stays 5.9 mm high; the box end does not go on the nut there, and the nut is never turned |
| Nothing moves | 0 | The clamp stays 5.9 mm above its seat |

The feasibility check on this code (`python dev/feasibility_gate.py --seed 0`, the nut starting at 4.5 N m): both fits
went on (0.81 and 0.55 mm off centre), the nut turned 29.8 and 29.7 degrees for 33.1 and 33.5 degrees of wrench swing,
the box end's spring gave at most 1.9 degrees, the nut turned at 0.61 rad/s at most, the peak was 6.55 N m, the left
arm's wrist joints used at most 4.3 of their 7 N m, and the padded inertia took at most 0.033 N m.

The wrench's log (`CLAMP_WRENCH_LOG`) in these runs reads 0 between strokes and rises through each push: the demo's
peaks at 6.20 to 6.25 N m on starts 0 to 4, the eight-stroke run's at 7.99 N m. In every demo run the nut turned at most
0.3 degrees with the box end off it (5 allowed). The feasibility check below was not re-run for this revision: it measures
the physics, which this revision does not change (only what the scorer reads).

Scorer probes, start 0 (throwaway copies of this folder with a few lines added to the task file; the scorer unchanged):

| Probe | Score | Why |
|---|---|---|
| Eight strokes, then the nut turned back 60 degrees once the wrench's log had passed 7.2 N m | 0.25 | End torque 5.26 N m, inside the band, but the wrench's log peak (7.20 N m) is kept: over-tightened |
| The box end put on the nut and the wrench left there, the nut never turned | 0 | The right hand lets go, the cable springs the clamp up 5.8 to 6.0 mm |
| The right hand's up-and-over waypoints removed | 1.0 | The straight path from the ready pose also crosses above the fender's top (fingertips 2 to 3 cm over it), and the task is then done as in the demo: the fender does not force the detour. With the fender raised to 35 cm and its front end moved 4 cm forward the same probe again scored 1.0 (the hand glanced off the top edge and went over); the arm cannot cross a fender taller than about 36 cm at all |

Dream-1, start 0: score 0.0 after 1,800 steps. It did not seat the clamp (still 5.9 mm high at the end) and did not turn the nut.

Cases checked with the scorer alone (the folder's tests): seated at 5, 6 and 7 N m, 1.0; tightened 3 mm high or 5
degrees crooked, 0; 5 mm off the positive post's axis (on the negative post is 17 cm off), 0; battery pushed 1 cm, 0;
pushed down with the nut only run down by hand, 0.25; seated at 2 or 4.9 N m (loose), 0.25; seated at 7.1 or 12 N m
(over-tightened), 0.25; lug 20 degrees out, 0.5; wrench left on the nut, a finger pad 3 cm from the cable, or something
still moving, 0.75.

## Files
- `car_battery_clamp.py`: the task file (objects, the bay, joints, seeded start, success, score, demo plans).
- `clampdemo.py`: the scripted demo's camera step (finds the battery and the stand in the first frame) and its sag
  correction.
- `bodykit.py`: multi-shape rigid bodies and joints for yam_arena task files.
- `test_car_battery_clamp.py`: scoring and demo-reach checks without the simulator
  (`python -m pytest -q test_car_battery_clamp.py`, needs `yam_arena`).
- `dev/feasibility_gate.py`: the torque check above (needs the simulator).
