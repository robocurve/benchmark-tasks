# Stand a picture frame on a narrow ledge

A small picture frame stands upright in a desk slot. The robot must move it to a narrow picture ledge (a rail with
a front lip, and a back wall 4.5 cm behind the rail) and let it go so it leans back against the wall. The shelf
behind the lip is only 1 cm deep, half the frame's thickness, so the frame cannot stand upright on it: it stays only
if its foot is set on the shelf and it is released so it falls back across the gap onto the wall. Set too far
forward, it lands on the lip. Set too far back, it drops into the gap behind the shelf.

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `score`, `distance`, scripted `demo` |
| `task.json` | the submission fields (`media` is added by the upload script) |
| `test_task.py` | offline checks (NumPy and pytest, no Isaac Sim) |

## Run a trial

From a checkout of the arena (yam-isaac-lab-arena, tested at `bd62771`) on the GPU machine, with this folder at
`tasks/frame-on-ledge/`. The layout is fixed (`-E object_jitter=0`), so no seed changes the scene, and the scripted
demo is open loop and needs the fixed layout.

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/frame-on-ledge/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the picture frame on the narrow ledge, leaning back against the wall." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/frame-on-ledge-dream
```

Scripted demo (left arm only, 15.8 s of waypoints ending in a 2 s still hold). The release starts at 9.7 s. The
frame should tip onto the wall within about 0.5 s, so success should end the trial at about 11.5-12 s, roughly
step 230-300:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/frame-on-ledge/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the picture frame on the narrow ledge, leaning back against the wall." \
  --scorer task_score --max-steps 450 --store-frames --log-dir ~/yam-eval/frame-on-ledge-demo
```

`--max-steps 450` is the plan (316 steps) plus about 130. The 1,200-step run limit is also fine for both.

Videos, one MP4 per camera, in each log's `frames/` folder:

```bash
for log in ~/yam-eval/frame-on-ledge-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score`: 1.0 when the trial ended in success, otherwise `score()` at the last step. Put the
Dream-1 number in `task.json` `scores`.

Runtime per trial: 1,200 steps (1 minute simulated) for Dream-1 and about 16 s simulated for the demo, plus 50-85 s
of Isaac start-up once shaders are cached. Wall clock on an RTX 5090, Isaac start-up included: 33 s for the demo, 118 s for the Dream-1 run. Assets: none, only Box
primitives, so there is nothing to attribute.

Offline tests (about 12 s):

```bash
YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest tasks/frame-on-ledge/test_task.py -q
```

## Scoring at a glance

| Result | Score |
|---|---|
| Foot on the shelf (lowest point -8/+5 mm of the shelf top), leaning back 3-20 deg, top back edge within 5 mm of the wall, facing out (within 15 deg), sideways tilt under 10 deg, still | 1.0 (success after 1 s) |
| Still at ledge level (lowest point 1.0-4.0 cm over the table, centre over the ledge) in any other pose: upright, tipped forward onto the lip, leaning over 20 deg, lying flat, perched on the lip, turned | 0.4 |
| Left in the slot, on the table, fallen into the gap behind the shelf, over the lip onto the table, on top of the wall, off the end, held or moving | 0 |

`distance()` is the metres from the frame's centre to its centre in the demo's settled lean.

## How the release makes the frame lean (the derivation)

Loose objects start upright, so the lean has to come from where the frame is let go. Side view, x away from the
arms, in mm from the shelf's back edge (`SEAT_EDGE`, x = 0.355 m):

```
              wall face at +45
 lip  shelf   |
 [==][----]   |        frame: T = 20 thick, H = 120 tall
 -20 -10  0  +45       shelf top 20 mm over the table, lip top 35 mm, gap open down to the table
```

1. **The shelf is too narrow to stand on.** The shelf runs from the lip's back face (-10) to the edge (0), so it is
   10 mm deep, equal to T/2. An upright frame on it has its centre of mass at or behind the edge. It cannot stay
   upright, so a frame that ends up on the shelf has to lean.
2. **Which way it tips.** If the frame is let go with its bottom resting `b` mm onto the shelf (its front face `b` mm
   in front of the edge), its support is the strip from -b to 0. Its centre of mass is at T/2 - b behind the
   edge. For 0 < b < T/2 = 10 mm the centre of mass is behind the whole support, so gravity turns it about the
   shelf edge **toward the wall**. Tipping forward would need the centre of mass in front of the foot's front edge
   (-b), which never happens. The demo aims for b = 5 mm: the front face 5 mm behind the lip and the centre of mass
   5 mm behind the edge, 60 mm above it. That leaves ±5 mm of error in x before the outcome changes. At more than
   5 mm too far back, the frame drops into the gap (score 0). At more than 5 mm too far forward, its foot lands on
   the lip top, where it tips back too but stays perched (score 0.4).
3. **Where it stops.** It rotates about the edge until its top back edge meets the wall:
   (T - b) cos(a) + H sin(a) = 45 mm. This gives a = 12.3 deg at b = 0, 14.7 deg at b = 5 and 17.2 deg at b = 10, all
   inside the 3-20 deg band. The top back edge then touches the wall 112 mm over the shelf, below the wall's 150 mm
   top.
4. **Why it stays.** In the lean the centre of mass is 20 mm behind the edge horizontally
   ((T/2 - b) cos a + (H/2) sin a), so its weight presses the top onto the wall. The edge pushes along the frame's
   own up axis, toward the wall. With a frictionless wall, the friction needed at the edge is 0.08 of the normal
   force, well under wood on wood (about 0.3-0.5) and the simulator's 1.2. The lip stops the foot if it slides
   forward.
5. **Telling the outcomes apart.** On the shelf edge the frame's lowest corner dips at most (T - b) sin a < 5 mm
   below the shelf top. A frame perched on the lip's top edge has its lowest corner at least 35 - 20 sin(20 deg) =
   28 mm over the table, 3 mm above the 1.0 band's 25 mm limit (and more at the shallower lean it really takes
   there). A frame in the gap is down at the table.

The grasp sets b: the fingers close along y, across the frame's 8 cm width, so the 12-20 mm grip offset seen in
the GPU rounds acts along the ledge, where 22 cm of rail leaves plenty of room. It does not act across the gap.
Across the frame's width the offset can be at most (94 - 80) / 2 = 7 mm, because one pad stays at its open stop.
In x the frame sits where the TCP puts it.

## Design numbers

- **Gripper.** At `FINGER_OPEN` (-0.04695 m per finger) the pads are 94 mm apart. The widest allowed grip is
  84 mm. The frame is gripped across its 80 mm width: 7 mm of clearance per side on the way down, and its 120 mm
  height cannot be gripped. A 120 mm-wide frame was considered and narrowed for this reason.
- **Where the pads hold it.** The TCP is 30 mm below the frame's top, so the pads (tips 13 mm below the TCP) hold
  its top 43 mm by the side faces. The frame's top stays 31 mm below the finger carriages (61 mm over the TCP), and
  the centre of mass hangs 30 mm below the TCP, so the held frame hangs stably. At the release the pad tips are
  100 mm over the table, 6.5 cm above the lip top. The fingers never come near the rail or the lip, and the
  pads stay outside the frame's width.
- **Finger clearance (hull test).** The fingers come no closer than 12.7 mm to the wall (while lowering, with the
  fingers taken as wide open). The wall face is 40 mm behind the TCP, and the finger is at most 34 mm wide either
  side of it. They clear the frame in the slot by 6.9 mm and the settled lean by at least 7.3 mm from 0.6 s after
  the release. Every other fixture is more than 3 cm away. The carried frame, taken 15 mm off centre either way,
  clears every fixture by at least 3 mm until it is let go 3 mm over the shelf. Its front face is 5 mm behind
  the lip and its back face 30 mm in front of the wall.
- **Height budget (joint4).** Carry height is a TCP 13.5 cm over the table (frame bottom 1 cm over the 3.5 cm
  lip top and 2.5 cm over the slot strips). After lift-off it is 14.3 cm at x = 0.36. The IK test passes at 20 Hz
  for the whole plan with joints inside `ik.LOWER`/`ik.UPPER`. Straight down at x = 0.40, 15 cm was already out
  of reach, so the shelf cannot go higher than about 2 cm unless the frame gets shorter.
- **Layout.** The ledge is in front of the left arm (x 0.335-0.415, y 0.02-0.24). The slot is at (0.24, 0.37). The
  ledge was kept 6 cm or more (in plan) from the left arm's ready-pose wrist (x 0.32, y 0.305) so the top camera
  sees it in the first frame. The plan parks both arms at least 6 cm from the frame and the ledge. The episode ends on success
  before that park, so in the recorded demo's last frame the left wrist still covers the frame's top edge.
- **Demo timeline (left arm).** Over the frame 1.5 s, descend 1.5 s, close 0.8 s, lift out of the slot 1.5 s, carry
  2.0 s, settle 0.5 s, lower 2.2 cm in 1.6 s (1.4 cm/s), steady 0.3 s, release 0.6 s, lift off 3 cm in 1.0 s, back
  away 6 cm in 1.0 s, park 1.5 s, hold still 2.0 s. The right arm parks and holds.
- **Human time.** About 5 s, our estimate, not measured.

## Novelty

No task on the 2026-11-02 board puts an object on a ledge, stand or wall to lean. In the named sets, the
nearest are RLBench `hang_frame_on_hanger` and RoboTwin 2.0 `place_phone_stand`.
`hang_frame_on_hanger` hangs a frame on a hook. The hook carries the frame, and any placement that engages it
works. `place_phone_stand` sets a phone into a stand whose cradle holds it at a fixed angle. Here no hook or fitted
cradle holds the frame; it leans against the wall from the ledge. The ledge is too shallow to stand on, and the frame's pose after the release depends only on where its
foot is set relative to the shelf edge. That makes it a release-placement problem with three physically distinct
outcomes (leans back onto the wall, perches on the lip, or drops into the gap). The score measures it with a
tape and a protractor. LIBERO's shelf tasks (`put ... on the cabinet shelf`) put objects flat on a deep shelf.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 33 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 1,200-step run limit; 118 s wall clock.
- Offline tests: 21 passed.
