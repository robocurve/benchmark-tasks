# Reconnect a car battery's positive cable

A car battery stands on the bench. Its positive cable runs from a P-clip on a steel stand behind the battery, forward
across the lid, to its clamp. The clamp has been set back over the positive post, but the cable, held in its clip,
holds it 6 mm above its seat, riding high on the post's taper, and its pinch-bolt nut is run down by hand only. A 13 mm
combination wrench stands in a holder in front of the battery. The robot is told only: "Reconnect the positive battery
cable to the battery."

The job, as a technician does it: push the clamp fully down on the post by its cable, keep holding it down, and tighten
the nut with the wrench in the other hand until a torque wrench reads 5 to 7 N m; then take the wrench off. The
instruction says none of this. The scene enforces it:

- The cable springs the clamp back up. Pushed down and let go, the clamp rides back up off its seat (the cable pushes
  with 4,000 N/m: 12 N for 3 mm at 6 cm behind the post, 24 N at the 6 mm it rides high). It has to be held down until
  the nut has closed the bore on the post.
- Once the bore closes, the clamp is pinched where it is: tightened while it rides high (or crooked), it stays high and
  loose (score 0).
- The nut turns only with the wrench on it. Past the bore closing, its torque rises 1 N m per 30 degrees: 5 to 7 N m is
  tight, and past 7 N m the clamp is over-tightened (score 0.25). The wrench has to come off the nut and away from the
  battery for full credit: a steel wrench left on a battery can bridge its posts.


Each trial starts differently: the seed moves the battery (up to 12 mm front to back and 15 mm side to side), tips the
clamp a different way as it sits 6 mm up on the post (up to 2.5 degrees, and turns it up to 4 degrees), and moves the
wrench stand (up to 15 mm and 10 mm). So a memorised motion cannot score.

## Revision for review TK-GABW96
| Review point | What changed |
|---|---|
| The cable had no anchor for its upward force | The cable now runs from the clamp straight back over the lid to a rubber-lined steel P-clip on a stand fixed to the bench, 1.5 cm behind the battery. It holds the clamp 6 mm high; pushed down 3 mm at 6 cm behind the post it pushes back 12 N (4,000 N/m). The setup gives the stand's size and place, and the reset gives the load check that reproduces it. |
| Nut angle from an unspecified loose start could give full credit to an under- or over-tightened joint | Tightness is now the nut's torque, read with a dial torque wrench: 5 to 7 N m for full credit, the usual torque for a top-post clamp's pinch nut. Under 5 N m (loose) or over 7 N m (over-tightened) scores 0.25. The start is defined: the nut run down by hand until it stops against the ear. |
| No reset condition for repeatable trials | Reset below: loosen, re-set the clamp and let the cable hold it, run the nut down by hand, load check, replace a clamp taken past 7 N m. |
| Check the demo's gripper clearance at the end | The scorer now measures every finger pad to every part of the clamp (collar, ears, nut, lug, cable to its clip) and the wrench. The demo ends 121.7 to 123.6 mm clear on seeds 0 to 4 (40 mm needed), with the wrench back in its holder. |

## Reset
Loosen the nut a full turn, lift the clamp off the post, set it back on and let go: the cable holds it where it rests,
collar 6 mm above the boss (check with a 6 mm drill shank). Run the nut down clockwise by hand until it stops against the
ear (finger-tight, no tool): that is the start. Put the wrench back in its holder; set the battery, cable stand and tool
stand on the trial's marks. Load check: pushing the cable down 3 mm from where it rests, 6 cm behind the post, takes
12 N (a push gauge); if less, move the cable stand closer to the battery, if more, farther. A clamp taken past 7 N m may
be deformed: replace it.

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

## Scoring
Every check is a measurement a person can make on the real bench: a feeler gauge under the clamp, a dial torque wrench
on the nut, a protractor or a photo for the lug, and a tape for the battery.

| Outcome | Score |
|---|---|
| Clamp seated, nut 5 to 7 N m, lug in line, wrench off and away, every finger pad at least 4 cm clear, all still for 1 s | 1.0 |
| The same, but the wrench still on the nut or on or against the battery, a hand on the clamp or the wrench, or something moving | 0.75 |
| Seated and tight, lug 10 to 30 degrees out of line | 0.5 |
| Seated but the nut under 5 N m or over 7 N m; or seated and tight with the lug over 30 degrees out | 0.25 |
| Clamp high or crooked (tightened there, it is a loose joint), off the positive post, on the negative post, or the battery moved over 5 mm | 0 |

The numbers:

- **Seated:** at four points under the collar (front, back, left, right, at its outer edge), the gap to the post's
  boss is at most 1.0 mm; the clamp is square to the post within 2 degrees and centred on it within 3 mm.
- **Tight:** the nut holds 5 to 7 N m, the usual torque for a top-post clamp's pinch nut. A dial torque wrench with a
  13 mm socket, turned clockwise on the nut, reads 5 to 7 N m as the nut starts to move. Turn 0 is the finger-tight
  start: run down by hand until it stops against the ear, the split still open. With the collar seated, the bore closes
  on the post after 30 degrees more; every millimetre the collar sits higher takes another 100 degrees. Past that point
  the torque rises 1 N m per 30 degrees.
- **Lug in line:** the cable lug points within 10 degrees of the cable's run (straight back to its clip).
- **Still:** the clamp, the nut and the wrench each moved less than 1 cm and 0.1 rad (6 degrees) over the last second.
- **Wrench away:** its box end at least 2 cm from the nut, and the wrench not on or against the battery: no part of it
  more than 3 cm above the bench within 1 cm of the battery's outline (lying on the bench beside the battery is fine).

On a real bench: slide a 1 mm feeler gauge under the collar at the four points, turn a dial torque wrench clockwise
on the nut until it starts to move and read 5 to 7 N m, check the lug's angle against the battery's edge, and tape the
battery's position. The finger-pad clearance comes from the arms' logged joint angles at the end (forward kinematics),
or a photo with a scale beside the clamp; stillness and the 1 s hold from the video's last seconds.

## Objects
Positions are in cm on the bench, from the midpoint between the two arm bases: x forward (away from the arms), y to
the robot's left. Heights are above the bench top. Each seed moves the battery and the stand within the ranges above;
the setup screenshot and the demo video show seed 0.

| Object | Type, size, material | Where |
|---|---|---|
| Battery | lead-acid car battery, BCI group 24 size: 26.0 x 17.3 x 22.5 cm, 18 kg, black polypropylene case | centre x 30, y 0 nominal (seed 0: x 30.5, y -1.4); long side along y; posts along the front edge |
| Positive post | SAE lead post on a 4 mm lead boss (28 mm across): cone 19.5 mm across at its base, 17.5 mm at its top, 18 mm tall; red mark on top, red + on the lid | right-hand end: 2.2 cm in from the front face, 8.5 cm right of the battery's centre |
| Negative post | SAE lead post, 17.9 mm at its base, 15.9 mm at its top, 18 mm tall, on the same boss; black mark, white - on the lid | 2.2 cm in, 8.5 cm left of centre |
| Clamp | lead top-post clamp, 0.14 kg with its lug: 12-sided collar 31.1 mm across its flats, 16 mm tall, tapered bore like the post; two ears with an M8 pinch bolt across its front (carriage head on the right ear, nut on the left); crimp lug and red 4 AWG cable out the back; white paint mark on the left ear | on the positive post, 6 mm above its seat, held up by the cable |
| Cable clip | black rubber-lined steel P-clip, 15 mm across, on a 25 x 6 mm steel flat-bar upright and a 10 x 6 cm steel base plate clamped to the bench | 1.5 cm behind the battery's back face, in line with the positive post; clip axis 24.3 cm above the bench |
| Nut | M8 steel hex nut, 13 mm across flats, 6.5 mm thick, white paint line across its face (pointing up at the start) | on the bolt, left side |
| Wrench | 13 mm combination wrench, chrome vanadium steel, 18.1 cm long, 0.105 kg, 12-point box end and open end | standing open end down in the holder |
| Wrench stand | steel pedestal 6 x 7 x 12 cm with an orange-painted wooden holder on top, 5 x 4.4 x 3 cm, slot 9 mm wide along x; 2.5 kg | centre x 10.5, y 7 nominal (seed 0: x 11.45, y 6.4), in front of the battery's left half |

## Design notes
- **The instruction is the goal only, on purpose.** A technician is asked to reconnect a battery, not told to push the
  clamp down first or to hold it while tightening. Knowing that a clamp tightened high is a loose joint (it heats,
  cranks the engine slowly and eventually fails to start it), and that the cable springs the clamp back up unless it
  is held down while the nut is tightened, is what the task measures.
- **Two hands are needed.** One hand pushes the clamp down by its cable and holds it there; the other turns the nut
  with the wrench. Seated and let go before the bore closes, the clamp rides back up about 6 mm, and a nut tightened
  there pinches it high (the seat-and-let-go failure path scores 0).
- **The wrench has to be fitted and used.** The robot takes the wrench from its holder, slides the box end on square to
  the flats, pushes it round 60 degrees, takes it off, swings it back 60 degrees, puts it on again and pushes it round
  again, three strokes in all (the 12-point box end fits every 30 degrees; the wrist turns only so far). Between strokes
  it lets go and takes the handle again while the box end still holds the wrench square on the nut: under the nut's
  torque the handle creeps round in a soft grip, and a fresh grip is what lets the next fit go on. Then it takes the
  wrench off and puts it back in its holder.
- **Starts vary per trial**, so the demo cannot replay a fixed motion. The scripted demo looks at the top camera's
  first frame, finds the battery and the wrench stand there (the camera's published calibration, the sizes in this
  setup, colour and edge detection), and plans from those estimates. It also corrects its arms' sag from its own joint
  readings. While it runs it reads no simulator state. Two constant offsets in its edge finder (about 1 to 2 mm, the
  render's soft edges) were calibrated once against known start layouts, as a camera calibration would be.
- **How it differs from the accepted M12 sensor-cable task (TK-3SHVWE).** There, the loose part would spin with the
  coupling, so one hand stops it turning while the other hand turns a knurled ring by hand. Here, a spring (the stiff
  cable) pushes the part off its seat, so one hand holds it down against that push; the other hand fits a tool on a
  nut's flats, turns it in strokes, re-fits it every stroke and takes it away. The failure is also different: a clamp
  tightened high looks connected but is pinched loose, so the end state, not the motion, shows whether the job was
  understood.
- **Cheats score 0 or 0.25:** pushing the battery (over 5 mm), tightening the clamp where it rides, seating it and
  letting go before tightening, moving it to the negative post, or doing nothing score 0. Turning the nut by hand
  cannot pass 1 N m, so a seated clamp without the wrench scores 0.25; so does a nut short of 5 N m or forced past 7.

## Fidelity
- Rigid bodies with real sizes and masses, except the nut: 30 g in the sim (7 g real), so the solver holds it steady.
  The battery is free on the bench (18 kg, friction 1.1), not bolted.
- The post's taper is four stacked cylinders. The collar's grip on the taper is a friction force of 4 N along the post
  (it stays where it is left, and goes down when pushed harder), with 0.3 mm of side play, a sprung rocking of up to 5
  degrees front to back (15 N m/rad) and 0.8 degrees side to side. A clamp opened up for fitting behaves this way; a
  corroded one needs more force.
- Only the first 7 cm of cable behind the lug is drawn colliding (where a hand can hold it); the rest, to the clip, is
  drawn only. Its stiffness is a spring at the lug, 6 cm behind the post: bent down from where it rests at the start,
  it pushes up with 4,000 N/m (12 N for 3 mm), until the collar is pinched.
- Once the bore closes on the post, the collar is pinched: square (within 2 degrees) it locks at its height and heading
  then; crooked, it binds loose and crooked (its centring spring is released and the cable's weight tips it).
- The nut turns on the bolt against a friction torque: 1 N m until the bore closes (30 degrees with the collar seated,
  100 degrees more per millimetre it sits higher), then 1 N m more per 30 degrees. The 5 to 7 N m band is the usual
  torque for a top-post clamp's pinch nut (one vehicle maker's manual gives 7 N m); the ramp from the bore closing to
  it is modelled, not measured on a clamp. The scorer reads the nut's torque at its final turn, as a dial torque
  wrench would on the real bench.
- With the box end on, the nut is held to the wrench's angle: the wrench turns it, but the hand does not feel the nut's
  torque (the arm does not have to push against 5 to 7 N m in the sim; on a real bench it does).
- The wrench's box end catches the nut when it is slid on square (within 4 mm of the bolt's axis, 10 degrees of its
  direction and 7 degrees of a flat, as a box end's chamfer finds the nut); it then turns with the nut and comes off
  when pulled 9 mm back along the bolt. The box end's ring is drawn but does not collide. The fingers are position-
  controlled, so the handle can creep a few degrees in the grip under the nut's torque; the demo lets go and re-grips it
  between strokes while the box end holds the wrench square on the nut.

## Runtime and seeds
- The scripted demo finishes in about 1,090 to 1,100 control steps (54 to 55 s at 20 Hz) of the 1,800 allowed. One trial
  takes a few minutes on an L40S, simulator start-up included.
- `--seed 0` (the default) is the start the setup text and screenshot show. `--seed N` and `--epochs K` give other
  starts.

## Results (scripted policy)
One trial per start, scored by this folder's scorer. The demo plans from the first top-camera frame only.

| Start (`--seed`) | Score | Nut turned | Torque | Pads clear | What happened |
|---|---|---|---|---|---|
| 0 (the setup text and screenshot) | 1.0 | 184.0 degrees | 6.09 N m | 122.0 mm | seated, tight, wrench back in its holder |
| 1 | 1.0 | 179.2 degrees | 5.96 N m | 121.7 mm | the same |
| 2 | 1.0 | 182.3 degrees | 6.07 N m | 122.5 mm | the same |
| 3 | 1.0 | 180.3 degrees | 5.98 N m | 123.6 mm | the same |
| 4 | 1.0 | 180.7 degrees | 6.00 N m | 122.3 mm | the same |

The clearance column is the scorer's smallest distance from any finger pad to any part of the clamp, the nut, the cable
to its clip, or the wrench at the end; the full-score check needs 40 mm, so the demo clears it by three times.

Failure paths, start 0: the same plans with one hand changed (set `CLAMP_FAIL=release`, `nohold` or `idle`, or
`CLAMP_STROKES=2` or `4`, in the environment of the demo run; unset for the demo itself):

| Path | Score | Why |
|---|---|---|
| The right hand seats the clamp, then lets go before the nut is turned | 0 | the cable springs the clamp back up 5.6 mm; the box end, aimed at a seated nut, never catches it |
| Nothing holds the clamp; the left hand tightens alone | 0 | turning the nut lifts the clamp to 10.4 mm; the nut turns 140 degrees but the bore never closes that high |
| Nothing moves | 0 | the clamp rides 5.9 mm high |
| Two strokes instead of three | 0.25 | seated, but the nut stops at 4.1 N m (under-tightened) |
| Four strokes instead of three | 0.25 | seated, but the nut reaches 8.1 N m (over-tightened) |

Cases checked with the scorer alone (the folder's tests): seated at 5, 6 and 7 N m, 1.0; tightened 3 mm high or 5
degrees crooked, 0; 5 mm off the positive post's axis (on the negative post is 17 cm off), 0; battery pushed 1 cm, 0;
pushed down with the nut only run down by hand, 0.25; seated at 2 or 4.9 N m (loose), 0.25; seated at 7.1 or 12 N m
(over-tightened), 0.25; lug 20 degrees out, 0.5; wrench left on the nut, a finger pad 3 cm from the cable, or something
still moving, 0.75.

## Files
- `car_battery_clamp.py`: the task file (objects, joints, seeded start, success, score, demo plans).
- `clampdemo.py`: the scripted demo's camera step (finds the battery and the stand in the first frame) and its sag
  correction.
- `bodykit.py`: multi-shape rigid bodies and joints for yam_arena task files.
- `test_car_battery_clamp.py`: scoring and demo-reach checks without the simulator
  (`python -m pytest -q test_car_battery_clamp.py`, needs `yam_arena`).
