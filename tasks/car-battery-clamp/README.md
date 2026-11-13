# Reconnect a car battery's positive cable

A car battery stands on the bench. Its positive cable clamp has been set back over the positive post, but the stiff
cable holds it up, riding high on the post's taper, and its pinch-bolt nut is loose. A 13 mm combination wrench stands
in a holder in front of the battery. The robot is told only: "Reconnect the positive battery cable to the battery."

The job, as a technician does it: push the clamp fully down on the post by its cable, keep holding it down, and tighten
the nut with the wrench in the other hand; then take the wrench off. The instruction says none of this. The scene
enforces it:

- The cable springs the clamp back up. Pushed down and let go, the clamp rides back up off its seat (the cable pushes
  up to 12 N against the collar's 4 N grip on the taper). It has to be held down until the nut has bitten.
- Once the nut has turned 20 degrees the clamp is pinched where it is: tightened while it rides high (or crooked), it
  stays high and loose (score 0).
- The nut turns only with the wrench on it. The wrench has to come off the nut and away from the battery for full
  credit.

Each trial starts differently: the seed moves the battery (up to 12 mm front to back and 15 mm side to side), tips the
clamp a different way as it sits 6 mm up on the post (up to 2.5 degrees, and turns it up to 4 degrees), and moves the wrench stand (up to
15 mm and 20 mm). So a memorised motion cannot score.

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
Every check is a measurement a person can make on the real bench: a feeler gauge under the clamp, a paint mark across
the nut and the clamp's ear, a protractor or a photo for the lug, a tape for the battery.

| Outcome | Score |
|---|---|
| Clamp seated, nut tight, lug in line, wrench off and away, both hands at least 4 cm clear, all still for 1 s | 1.0 |
| The same, but the wrench still on the nut or on or against the battery, a hand on the clamp or the wrench, or something moving | 0.75 |
| Seated and tight, lug 10 to 30 degrees out of line | 0.5 |
| Seated and tight, lug over 30 degrees out; or seated but the nut turned less than 75 degrees; or turned more than a full turn (crushed) | 0.25 |
| Clamp high or crooked (tightened there, it is a loose joint), off the positive post, on the negative post, or the battery moved over 5 mm | 0 |

The numbers:

- **Seated:** at four points under the collar (front, back, left, right, at its outer edge), the gap to the post's
  boss is at most 1.0 mm; the clamp is square to the post within 2 degrees and centred on it within 3 mm.
- **Tight:** the nut has turned 75 to 360 degrees clockwise (seen from the nut's end) past its paint mark.
- **Lug in line:** the cable lug points within 10 degrees of the cable's run (straight back across the battery).
- **Still:** the clamp, the nut and the wrench each moved less than 1 cm and 0.1 rad (6 degrees) over the last second.
- **Wrench away:** its box end at least 2 cm from the nut, and the wrench not on or against the battery: no part of it
  more than 3 cm above the bench within 1 cm of the battery's outline (lying on the bench beside the battery is fine).

On a real bench: watch the video, then slide a 1 mm feeler gauge under the collar at the four points, read the nut's
paint mark against the ear's, check the lug's angle against the battery's edge, and tape the battery's position.

## Objects
Positions are in cm on the bench, from the midpoint between the two arm bases: x forward (away from the arms), y to
the robot's left. Heights are above the bench top. Each seed moves the battery and the stand within the ranges above;
the setup screenshot and the demo video show seed 0.

| Object | Type, size, material | Where |
|---|---|---|
| Battery | lead-acid car battery, BCI group 24 size: 26.0 x 17.3 x 22.5 cm, 18 kg, black polypropylene case | centre x 30, y 0 nominal (seed 0: x 30.5, y -1.4); long side along y; posts along the front edge |
| Positive post | SAE lead post on a 4 mm lead boss (28 mm across): cone 19.5 mm across at its base, 17.5 mm at its top, 18 mm tall; red mark on top, red + on the lid | right-hand end: 2.2 cm in from the front face, 8.5 cm right of the battery's centre |
| Negative post | SAE lead post, 17.9 mm at its base, 15.9 mm at its top, 18 mm tall, on the same boss; black mark, white - on the lid | 2.2 cm in, 8.5 cm left of centre |
| Clamp | lead top-post clamp, 0.14 kg with its lug: 12-sided collar 31.1 mm across its flats, 16 mm tall, 20.1 mm bore; two ears with an M8 pinch bolt across its front (carriage head on the right ear, nut on the left); crimp lug and 7 cm of red 4 AWG cable out the back; white paint mark on the left ear | on the positive post, 6 mm above its seat, held up by the cable |
| Nut | M8 steel hex nut, 13 mm across flats, 6.5 mm thick, white paint line across its face (pointing up at the start) | on the bolt, left side |
| Wrench | 13 mm combination wrench, chrome vanadium steel, 18.1 cm long, 0.105 kg, 12-point box end and open end | standing open end down in the holder |
| Wrench stand | steel pedestal 6 x 7 x 12 cm with an orange-painted wooden holder on top, 5 x 4.4 x 3 cm, slot 9 mm wide along x; 2.5 kg | centre x 10.5, y 4 nominal (seed 0: x 11.45, y 2.8), in front of the battery's left half |

## Design notes
- **The instruction is the goal only, on purpose.** A technician is asked to reconnect a battery, not told to push the
  clamp down first or to hold it while tightening. Knowing that a clamp tightened high is a loose joint (it heats,
  cranks the engine slowly and eventually fails to start it), and that the cable springs the clamp back up unless it
  is held down while the nut is tightened, is what the task measures.
- **Two hands are needed.** One hand pushes the clamp down by its cable and holds it there; the other turns the nut
  with the wrench. Seated and let go before the nut bites, the clamp rides back up about 6 mm, and a nut
  tightened there pinches it high (the seat-and-let-go failure path scores 0).
- **The wrench has to be fitted and used.** The robot takes the wrench from its holder, slides the box end on square to
  the flats, pushes it round, takes it off, swings it back 60 degrees, puts it on again and pushes it round again (the
  12-point box end fits every 30 degrees; the wrist turns only so far), then takes it off and lets it drop onto the
  bench in front of the battery (anywhere on the bench, or back in its holder, counts).
- **Starts vary per trial**, so the demo cannot replay a fixed motion. The scripted demo looks at the top camera's
  first frame, finds the battery and the wrench stand there (the camera's published calibration, the sizes in this
  setup, colour and edge detection), and plans from those estimates. It also corrects its arms' sag from its own joint
  readings. While it runs it reads no simulator state. Two constant offsets in its edge finder (about 1 to 2 mm, the
  render's soft edges) were calibrated once against known start layouts, as a camera calibration would be.
- **Cheats score 0:** pushing the battery (over 5 mm), tightening the clamp where it rides, seating it and letting go
  before tightening, moving it to the negative post, doing nothing.

## Fidelity
- Rigid bodies with real sizes and masses, except the nut: 30 g in the sim (7 g real), so the solver holds it steady.
  The battery is free on the bench (18 kg, friction 1.1), not bolted.
- The post's taper is four stacked cylinders. The collar's grip on the taper is a friction force of 4 N along the post
  (it stays where it is left, and goes down when pushed harder), with 0.3 mm of side play, a sprung rocking of up to 5
  degrees front to back (15 N m/rad) and 0.8 degrees side to side. A clamp opened up for fitting behaves this way; a
  corroded one needs more force.
- Only 7 cm of the cable is drawn. Its stiffness is a spring at the lug, 6 cm behind the post: bent down from where it
  rests at the start, it pushes up with 4,000 N/m, at most 12 N, until the collar is pinched.
- Once the nut has turned 20 degrees, the collar is pinched: square (within 2 degrees) it locks at its height and
  heading then; crooked, it binds loose and crooked (its centring spring is released and the cable's weight tips it).
- The nut turns on the bolt only past a thread friction of 1 N m, at any speed (fingers do not reach it; the wrench
  does). This is a constant friction, not the rising torque of a real nut being tightened (real terminal nuts are
  tightened to several N m), so tightness is scored by the nut's turn from its paint mark. The 20, 75 and 360 degree
  thresholds are this task's choices, not a published specification.
- The wrench's box end catches the nut when it is slid on square (within 4 mm of the bolt's axis, 10 degrees of its
  direction and 7 degrees of a flat, as a box end's chamfer finds the nut); it then turns with the nut and comes off
  when pulled 9 mm back along the bolt. The box end's ring is drawn but does not collide.

## Runtime and seeds
- The scripted demo finishes in about 700 to 850 control steps (35 to 43 s at 20 Hz) of the 1,800 allowed. One trial takes
  about 2 minutes on an L40S, simulator start-up included.
- `--seed 0` (the default) is the start the setup text and screenshot show. `--seed N` and `--epochs K` give other
  starts.

## Results (scripted policy)
One trial per start, scored by this folder's scorer. The demo plans from the first top-camera frame only.

| Start (`--seed`) | Score | Nut turned | What happened |
|---|---|---|---|
| 0 (the setup text and screenshot) | 1.0 | 136 degrees | seated, tight, wrench on the bench |
| 1 | 1.0 | 86 degrees | the same |
| 2 | 1.0 | 136 degrees | the same |
| 3 | 0.75 | 128 degrees | seated and tight, but the dropped wrench came to rest propped against the battery's front |
| 4 | 1.0 | 135 degrees | seated, tight, wrench on the bench |

Failure paths, start 0 (the same plans with one hand changed; run with a wrapper that is not part of this folder):

| Path | Score | Why |
|---|---|---|
| The right hand seats the clamp, then lets go before the nut is turned | 0 | the cable springs the clamp back up 6 mm before the nut bites |
| Nothing holds the clamp; the left hand tightens alone | 0 | the clamp stays 6.5 mm high |
| Nothing moves | 0 | the clamp rides 6 mm high |

Cases checked with the scorer alone (the folder's tests): tightened 3 mm high or 5 degrees crooked, 0; 5 mm off the
positive post's axis (on the negative post is 17 cm off), 0; battery pushed 1 cm, 0; pushed down but never
tightened, or turned only 40 degrees, 0.25; turned past a full turn, 0.25; lug 20 degrees out, 0.5; wrench left on
the nut, or something still moving, 0.75.

## Files
- `car_battery_clamp.py`: the task file (objects, joints, seeded start, success, score, demo plans).
- `clampdemo.py`: the scripted demo's camera step (finds the battery and the stand in the first frame) and its sag
  correction.
- `bodykit.py`: multi-shape rigid bodies and joints for yam_arena task files.
- `test_car_battery_clamp.py`: scoring and demo-reach checks without the simulator
  (`python -m pytest -q test_car_battery_clamp.py`, needs `yam_arena`).
