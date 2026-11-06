# Lift the filament spool with both hands onto the holder's rod

Hanging a spool of 3D-printer filament on a printer's spool holder, the way a fab lab or print farm technician does many
times a day. The spool weighs 1 kg and is 20 cm across, so both hands take it by the rim. It lies flat and has to be
stood up (turned a quarter turn about the line between the hands), lifted to the rod 25 cm above the table and threaded
on: the 52 mm bore clears the 45 mm rod by 3.5 mm all round, so the spool must be level and centred on the rod before
it slides on, then pushed home against the stop and let go.

| File | What it is |
|---|---|
| `spool_onto_holder.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `spool.py` | `Spool`: a task-file box made into one rigid body of box colliders around an open bore. `rod()`: cylinder colliders along x (the rod, with a chamfered nose, and the stop collar) on a fixed box |
| `grip.py` | `HandWaypoint`: demo waypoints with the gripper pointing any way (level side grips, a wrist roll), and joint-space moves that keep the arm's posture |
| `fingers.py` | Points on the four fingers' collision meshes, for the let-go check |
| `bonds.py` | Hooks into `yam_arena` (on spawn, on reset, on every step), used by the modules above |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`*.usdc`) and their textures, and `make_looks.py`, the Blender script that models and exports them |
| `test_spool_onto_holder.py` | Offline checks: the file loads, the bore is open round the rod, every scoring outcome, the let-go check, the demo's grips, clearances and reach, the mesh fits its box (no GPU) |
| `task.json` | The task's details for the board |

## The spool and the holder

| Part | Physics | Look |
|---|---|---|
| spool | one free rigid body, 1 kg: 72 box colliders in 24 segments round the axis. The core (hub and filament) runs from the bore (flats 26 mm from the axis) to the filament's surface (6.5 cm), the two 6 mm flanges from there to the 10 cm rim, and the gap between the flanges is open above the filament | black flanges with six windows, grey hub, orange filament wound 13 cm across, a blank white label |
| holder | a fixed 4 x 4 x 29 cm post with cylinder colliders: the rod (45 mm, 12 cm long, from x = 0.33 to 0.45, axis 25 cm above the table, its last 2 cm chamfered to a 28 mm nose, friction 0.3) and the stop collar (70 mm, 1 cm thick, face at x = 0.44) | grooved aluminium post on a grey foot plate, steel rod, black collar with a set screw |
| holder_foot | a fixed 14 x 16 x 1.2 cm plate | drawn by the holder's mesh |
| carton | a fixed 22 x 11 x 10 cm box under the spool, narrower than it | brown cardboard, packing tape, a blank shipping label |

No assets are downloaded. SimReady has no spool, reel or bobbin that fits (searched: spool, reel, filament, bobbin,
tape, ring, roll), so the spool and the holder are modelled in Blender: `looks/make_looks.py`, run with
`blender -b -P looks/make_looks.py`. The spool's mesh fits inside its box (a test checks it), so the centre the scorer
reads is the spool's.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=spool_onto_holder.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the orange filament spool with both hands, stand it up and slide it onto the grey holder's rod until it touches the stop." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/spool-onto-holder-dream

# Scripted demo (both arms), 700 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=spool_onto_holder.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the orange filament spool with both hands, stand it up and slide it onto the grey holder's rod until it touches the stop." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/spool-onto-holder-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The layout keeps the hands apart: they hold the spool from the left and the right of its rim, pointing at each other
  along y, 17.6 cm apart TCP to TCP, each arm in its own half of the table.
- The demo: both hands come in level at the spool's middle height, where its rim overhangs the box, and take it across
  its whole width (a finger on top of the top flange, one under the bottom flange). They lift it 9 cm and roll both
  wrists a quarter turn together, so the spool turns about the line between the grips (through its centre) and stands
  up with its bore along x. They raise it to the rod's height (1.6 cm higher than the rod, as the arms give under the
  weight), slide it on until it meets the stop, let go and draw back sideways. It reaches 1.0 and ends on success at
  step 364 (18.2 s at 20 Hz). A trial takes about 85 s on an L40S, including startup.
- The same demo sliding the spool only part of the way (`SEAT_X = 0.37`), so it is let go about 2 cm short of the
  stop, scores 0.5: on the rod, not home.
- Videos: `inspect-robots video ~/yam-eval/spool-onto-holder-demo/<log>.json` writes one MP4 per camera next to the
  frames.
- Offline checks: `python -m pytest -q test_spool_onto_holder.py`.

## Scoring

All positions are centres, in metres, in the station frame (x forward, y to the robot's left, from the midpoint of
the arm bases), and heights are above the table top. The rod's axis runs along x at y = 0, 25 cm up, from its tip at
x = 0.33 to the stop at x = 0.44.

- **On the rod:** the spool's centre within 2.6 cm (the bore's radius) of the rod's axis, its axis within 10 degrees
  of the rod's, and both its faces between the rod's tip and the stop: the rod runs through the whole bore.
- **Let go:** no point of any gripper finger within 5 mm of the spool's outside, read from the simulation every step.
- **0** if the spool has fallen off the table (centre more than 2 cm below the top) or the state is not finite.
- **1.0 (success, held 1 s):** on the rod, its inner flange within 1 cm of the stop collar, let go, and still (under
  1 cm/s and 0.5 rad/s).
- **0.5:** on the rod but 1 cm or more from the stop, or still held, or still moving.
- **0.25:** the spool's centre within 5 cm of the centre of the rod's tip: lifted to the rod but not on it.
- **0** otherwise, including the spool lying on its box, standing anywhere, or held anywhere else.

In a real trial, a tape measure gives the gap between the spool's flange and the collar, and a look at the hands shows
that they have let go.
