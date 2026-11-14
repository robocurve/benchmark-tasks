# Prove a stick-mounted voltage detector before and after testing a bar

**Instruction:** "Test the bar by touching it with the detector's top end, then lay the detector down 8+ cm away.
Never touch with an untrusted detector."

A lineworker's "no voltage" reading is only worth something if the detector has just been shown to work on a known
source and is shown to work again afterwards. Here a proximity voltage detector on a short hot stick lies on two
posts. Beside its top end is its battery proof tester; beyond that is a short bare bar on two posts. The robot has
to switch the detector on, hold its top end on the tester's end plate while holding the tester's TEST button, and
watch the detector's two red lights. If they flash, it may test the bar, must prove the detector once more, and
then lays it down at least 8 cm from the bar. On odd seeds the tester's battery is flat: nothing flashes, the detector cannot be trusted, and
the right outcome is to lay it down at least 8 cm from the bar without touching the bar. The instruction gives neither the order nor the stop
rule.

Two arms are needed: one gripper carries the 0.95 kg, 72 cm stick level by its balance point, the other presses ON
(while the detector still lies on its posts) and holds TEST (while the first holds the top end on the plate).

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=prove_voltage_detector_before_and_after_test.py \
    -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "Test the bar by touching it with the detector's top end, then lay the detector down 8+ cm away. Never touch with an untrusted detector." \
    --policy yam_scripted --scorer task_score --max-steps 2400 --seed 0     # and again with --seed 1
```

- Scene: `station`. Control at 20 Hz. Run limit 2,400 steps (2 minutes).
- **Seeds:** even `--seed` = tester battery good; odd `--seed` = tester battery flat. No `--seed` = good. The
  harness hands a task `crc32("<seed>:None:0")`, not the seed; the task file maps it back (`seed_is_flat`). The
  console prints `tester battery good` or `FLAT` at every reset. Report the mean of seeds 0 and 1.
- No object jitter: the layout is the same on every seed; only the tester's battery differs, and nothing in the
  first camera frame shows which it is.
- The scripted demo's plan is 910 steps (45.5 s); it succeeds at step 890 with a good tester and at step 592 with a
  flat one. A trial takes 3 to 6 minutes on a GPU.
- `--scorer task_score` gives 1.0 on success, otherwise the score at the end (`SCORE_OVER = "end"`).
- Offline checks: `pytest test_prove_voltage_detector_before_and_after_test.py`.

## Scoring

The scorer reads the two buttons' travel, the detector's pose and, for "laid down", the robot's joint positions
(to find each gripper's centre point). It never reads the lights.

| Term | Meaning |
|---|---|
| Self-test finished | ON pressed (down 1.0 mm or more of its 1.5 mm travel; a second press switches the unit off) and 3.0 s passed with the unit still on. |
| On the plate | The detector's top end face within 3 mm of the tester's end plate, its centre within 25 mm of the plate's centre (across and in height). |
| TEST held | TEST button down 1.4 mm or more of its 2 mm travel. |
| Proved | Self-test finished, on the plate, TEST held, tester battery good: the two red lights flash. Counted once this has lasted 1.0 s without a break. |
| Touching the bar | Any point of the top end face (7.6 x 5 cm) within 3 mm of the bar's surface. |
| Laid down | Each gripper's centre point at least 3 cm from the detector's bounding box; the top end and the stick end each move under 2 mm over 1 s; top end at least 8 cm from the bar and at least 1 cm off the plate; held 1 s. |
| Permanent zero | The top end touches the bar while the detector is not both proved and switched on; or any part of the detector is more than 2 cm below the table top. Nothing done afterwards restores the score. |

| Tester battery good (even seeds) | Score |
|---|---|
| Self-test finished | 0.1 |
| Proved | 0.2 |
| After that, top end touching the bar for 1.0 s without a break, detector on | 0.6 |
| Proved again after that touch | 0.8 |
| Then laid down at least 8 cm from the bar | 1.0 |

| Tester battery flat (odd seeds) | Score |
|---|---|
| Self-test finished | 0.1 |
| Prove attempted: on the plate with TEST held for 1.5 s (nothing flashes) | 0.3 |
| Then backed off for 1 s: top end at least 1 cm off the plate and 8 cm from the bar | 0.6 |
| Then laid down at least 8 cm from the bar, the bar never touched | 1.0 |

Success is 1.0 held for 1 s. What fixed strategies earn (seed 0, seed 1, mean): doing nothing 0, 0, 0; always
proving and never touching the bar 0.2, 1.0, 0.6; always proving and touching 1.0, 0, 0.5; touching without proving
0, 0, 0. Only a robot that reads the lights gets both seeds.

**Measuring it in a real trial:** film the trial. The detector's own lights and beeper show the self-test and each
proof; time the 1 s of flashing and the 1 s on the bar from the video; check the touch with a 3 mm feeler or from a
side view; measure the laid-down clearances with a ruler.

## Dimensions (metres; x forward, y left, z above the table top; origin midway between the arm bases)

| Item | Size and mass | Pose |
|---|---|---|
| Detector body (HD Electric PRX) | 0.28 x 0.076 x 0.05, 0.31 kg, red | axis along y at x = 0.13, z = 0.13; top end y = -0.305, other end y = -0.025; face up |
| Universal spline head | 0.040 dia x 0.055 | y = -0.025 to +0.030 |
| Hot stick, 1-1/4 in fibreglass | 0.032 dia x 0.615, orange; stick + head + cap 0.64 kg (estimate) | y = +0.030 to +0.645 |
| Whole assembly | 0.95 kg; balance point 0.451 from the top end | balance point y = +0.146 |
| ON button | 0.014 dia, 2 mm proud, 1.5 mm travel, 1.8 N at rest, 2.25 N bottomed | (0.13, -0.129), top z = 0.157 |
| Red lights (2) | 0.010 dia lenses | (0.112 and 0.148, -0.283), on the face |
| Green range lights (7) | 0.005 dia | x = 0.13, y = -0.235 to -0.175, every 10 mm |
| Post under the body | 0.09 x 0.03 x 0.105 | (0.13, -0.165) |
| Post under the stick | 0.06 x 0.03 x 0.114, two lips 0.006 x 0.03 x 0.012 | (0.13, +0.425) |
| Proof tester (HD Electric PT-PRX) | 0.066 x 0.125 x 0.033, 199 g, black, bolted to its block | x = 0.097 to 0.163, y = -0.45 to -0.325, z = 0.1135 to 0.1465 |
| Tester end plate | 0.05 x 0.001 x 0.025, metal | outer face y = -0.325, centre (0.13, z 0.13) |
| TEST button | 0.014 dia, 2 mm proud, 2 mm travel, 2.0 N at rest, 2.8 N bottomed | (0.13, -0.365), top z = 0.1485 |
| Tester LED | 0.005 dia, green | (0.148, -0.4075) |
| Tester block | 0.066 x 0.125 x 0.1135 | under the tester |
| Bar, bare 1/0 AAC aluminium conductor | 0.00935 dia x 0.18 | axis along x at y = -0.345, z = 0.13, x = 0.20 to 0.38 |
| Bar posts (2) | 0.022 x 0.022 x 0.1253 | x = 0.22 and 0.36, under the bar |

## Real-world parts

- HD Electric (Greenlee) PRX-500 proximity voltage detector: 11 x 3 x 2 in, 11 oz; rechargeable; ON button starts a
  self-test; "longer beeps and lights indicate voltage detected"; fits a universal spline or shotgun stick.
- HD Electric (Greenlee) PT-PRX proof tester: 4.9 x 2.6 x 1.3 in, 0.44 lb, one 9 V battery; used by putting the PRX's sensing end on the
  tester's metal end plate and holding its TEST button.
- 2 ft x 1-1/4 in fibreglass universal hot stick with one universal spline head (for example Hastings 567-2).
- 18 cm of bare 1/0 AWG all-aluminium conductor (ASTM B231, 9.35 mm), two polymer standoff posts, three posts or
  blocks for the cradle and the tester, a base board. For the flat-battery trials: a discharged 9 V battery.
- Safety of a real trial: nothing is energised; the bar is a loose offcut connected to nothing. Keep people outside
  the arms' reach and use the cell's e-stop; the 72 cm stick sweeps about 15 cm.

**Stick mass is an estimate.** We found no maker's weight for a 2 ft stick. Retail listings give 4 lb for the 4 ft
and about 5 lb for the 6 ft Hastings universal stick (about 0.5 lb per foot of pole), and a Chance two-head 1-1/4 in
pole of 126 in is listed at 7 lb. From these: pole 0.40 kg for 61.5 cm, head 0.20 kg, end cap 0.04 kg, total 0.64 kg
(plausible range 0.52 to 0.76 kg). One retailer lists the 2 ft stick at "3.0" (if pounds, 1.36 kg; probably a
shipping weight). Weigh the real stick; the balance point moves about 3 cm per 0.1 kg.

Sources: HD Electric PRX and PT-PRX instruction manuals and retailer specification pages (tester.co.uk); Hastings
567-series listings (Burlington Safety, Mitchell Instrument); Hubbell Chance specification sheet H176010.

## How the scripted demo works (what it senses)

The demo is played by `yam_scripted` from waypoints, but the waypoints are resolved while it runs, from camera
frames only. The task file keeps the frames the policy is given (`Observation.images`, 320 x 176) by wrapping
`YamArenaEmbodiment._observation` inside the task module. The demo never reads object poses, button travel, the
seed or the battery state.

1. **Where the detector is.** In the first top-camera frame the largest red blob is the detector's body. Its centre
   is compared with its centre in a reference frame taken with the detector at its drawing position (190.55,
   120.36 px), and the difference, at about 4.3 mm (left-right) and 6 mm (forward-back) per pixel, shifts the grip
   and ON-button waypoints (limited to 3 cm). The tester, the bar and the posts are bolted fixtures; the demo uses
   their drawing positions. (The TEST button is hidden under the right arm in the first top-camera frame.)
2. **Whether the detector proves.** While the right gripper holds TEST (turned so its wrist camera looks along the
   detector), the demo counts, in each right-wrist-camera frame, pixels that look like a lit red lens (red above
   200, green between 70 and 62 % of red, blue under 62 % of red). The lights count as on in a frame with 15 or
   more such pixels, and as flashing if that happens in 6 or more frames of the 2.25 s watch. Measured: about 890
   lit pixels in half the frames when the tester works, 0 or 1 when its battery is flat.
3. **The decision.** Until a flash has been seen the plan is the cautious one: back off the plate, slide the
   detector 6 cm left along its own axis and lay it back on its posts (top end 10 cm from the bar). Only after a
   flash does it switch to: carry the top end to the bar, hold it there 2 s, go back to the tester, prove again,
   then the same slide 6 cm left and lay-down. Both plans have the same timing, so nothing is decided before the
   watch ends. If the camera gives no usable frames (a blank first top-camera frame, or no lit pixels during the
   watch), the demo falls back to the drawing positions and to the cautious plan.

Sequence with a good tester: right hand presses ON (detector on its posts); left hand pinches the stick at its
balance point and lifts 1 cm; top end onto the tester plate; right hand holds TEST for 3 s; left hand carries the
top end 16 cm forward and onto the bar's side between the posts; back to the plate; TEST again; slide 6 cm left; lay it on its posts; let go;
both hands clear. With a flat tester: the same up to the watch, then back off, slide 6 cm left, lay down, let go.

The loaded arm sags: the gripper rides 10 mm low near the body and 16 mm low at the bar, so the demo commands that
much extra height.

## Simplifications

- Rigid bodies with box and cylinder colliders. The detector body and the stick are two bodies joined by a slide
  locked within 0.02 mm (the clamped spline joint), so each has its own mass and the balance point is right.
- The instruments' behaviour is a rule, not electronics: self-test, "on", the flash on the tester and the tester's
  LED follow button travel, the detector's pose and the seed's battery state. A real PRX senses the tester's field
  from a short distance; here it must be on the plate.
- The lights are small visual spheres that change colour; they touch nothing and the scorer never reads them.
- Light positions and sizes, the ON and TEST button positions and forces, and the stick mass are estimates (the
  makers publish none of them). The voltage-range buttons, the beeper, auto power-off and a live conductor are not
  modelled. The bar is always dead.
- The flat battery is set by the seed at reset; in a real trial fit a discharged battery.

## Files and licences

`prove_voltage_detector_before_and_after_test.py` (task), `yam_parts.py`, `yam_joints.py` (our own kit for compound
parts and passive joints, unmodified), `test_prove_voltage_detector_before_and_after_test.py`, `task.json`. Every
shape is a primitive authored in the task file. No third-party meshes, textures, fonts or materials are used.
