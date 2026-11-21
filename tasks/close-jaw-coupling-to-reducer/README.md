# Close a jaw coupling on a gear reducer drive

A worm gear reducer and a driven shaft stand in line on a steel base plate, joined by a Lovejoy L-100 jaw coupling
that is not yet closed: the spider sits on the reducer hub, and the driven hub's jaws stand square on the spider's legs.
The arm turns the driven hub until its jaws line up with the free pockets (one phase in every 120 degrees), slides it
in to Lovejoy's installation gap 'G' between the hub faces and lets go.

The scene, scorer and scripted demo are in [`task.py`](task.py) (`yam_arena` embodiment, `station` scene). Helpers:
`yamkit.py` (parts, bearing and slide joints), `yamgeom.py` (arm collision hulls for the hands-clear checks) and
`gripkit.py` (finger pads). The robot model and its collision hulls load at run time and are not redistributed here.

## Requirements

The yam_arena environment (Isaac Lab-Arena on Isaac Sim), Linux, an NVIDIA RTX GPU. No models from Arena's library.

## Run and score a trial

Scorer `task_score`: 1.0 once `success()` holds for 1 s, otherwise `score()` at the end of the episode:

| Score | End state |
|---|---|
| 1.0 | In phase; hub-face gap 0.750 in (19.05 mm) within 0.040 in; spider seated; all arm links 5 mm clear; still 1 s |
| 0.5 | In phase, jaws at least 5 mm into the pockets, hands clear, still 1 s, gap off the 1.0 band |
| 0.25 | In phase, jaws 1-5 mm into the pockets, hands clear, still 1 s |
| 0 | Anything else; a spider knocked off at any time is latched |

All checks read part poses and per-hull distances to every arm link (never flags). Thresholds: `task.json`.

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/close-jaw-coupling-to-reducer/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Close the jaw coupling between the gear reducer and the driven shaft." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir jaw-dream
```

The scripted demo (`--policy yam_scripted --max-steps 700`) is open loop from the fixed layout:

1. The right hand comes down over the driven hub, fingers either side across the axis, and grips its body.
2. It slides the hub along its shaft until the jaws stop end-on against the spider's legs.
3. It tilts the wrist 31.5 degrees about the axis, turning the hub and shaft in the bearings, and pauses 1 s.
4. It slides the hub on until the jaws drop into the pockets and its face stops on the legs (18.29 mm from the other
   face), lets go, backs out and goes home.

Videos: `inspect-robots video jaw-dream/<log>.json`.

## Failure modes

Demo variants (`JAW_VARIANT=...`):

| Route | Score |
|---|---|
| Do nothing (`nothing`) | 0 |
| Push straight in without turning (`noturn`): jaws stop on the legs | 0 |
| Turned, pushed 9 mm in, let go (`short`) | 0.5 |
| Closed but still held (`hold`), or a hand resting on or 3 mm over a part | 0 |
| Closed, then pulled back out (`undone`) | 0 |
| Closed without the spider; spider pulled off, turned or cocked | 0 |

## Sources

- Timken/Lovejoy L100-HUB-1"-1/4X1/8KW (1 in bore, 0.25 x 0.125 in keyway, sintered iron): OD 2.54 in, overall
  length 3.480 in, length through bore 1.38 in, hub length 2.09 in, 1.40 lb, bore 11-35 mm. The page's "Dimension G"
  (0.71 in) is the jaw length, hub length less length through bore, not the installation gap.
  https://cad.timken.com/item/standard-jaw-couplings-l-type/l-type-coupling-hubs-w-keyway-imperial/68514411516-2
- Lovejoy jaw coupling (L and SW style) installation guide, Table 5, L / SW 100 row: gap 'G' 0.750 in, allowed change
  0.040 in; sections 6.3 (slip fit) and 6.5 (setting the gap).
  https://www.lovejoy-inc.com/wp-content/uploads/2017/11/JawSWStyleInstallGuideAugust2012.pdf
- L/AL099/100-SPIDER-SOX (NBR, 80A) solid-center spider, 0.06 lb: A (OD) 2.540 in, H (center) 1.030 in, W 0.61 in,
  B (leg width) 0.43 in.
  https://cad.timken.com/item/standard-jaw-couplings-l-type/solid-center-spiders-metric/68514411494-2
- The rigid sim spider is sized differently, as a stand-in for the snug rubber legs: 61 mm across, 18-degree legs,
  legs 18.29 mm long (so the closed coupling has the published overall length) and a 34.4 mm solid center that rides
  on the jaws to keep it centered. The jaw width (36 degrees) is estimated from maker photos.

## Simplifications

The NBR spider is rigid, with 3 degrees of play a side in each pocket for the rubber's squeeze. The reducer, motor
and pillow blocks are fixed boxes and cylinders; the reducer side is locked out. No set screws or keys. The reducer hub
is clocked with one jaw straight down, a fixed start orientation for repeatable trials on which the rigid spider rests
centered. The driven shaft's bearings stop at 180 degrees either way (a meshing phase is never more than 120 degrees
away). The slip fit and bearings are light drag, so the hub and shaft stay where they are left.

## Real-world build

A fixed 1 in shaft in a block stands in for the reducer side; the coupling, driven shaft and pillow blocks are real.
The gap is checked with a 0.750 in gauge block or calipers after the hands are off.

## Runtime and seeds

Scripted demo: 606 steps (30.3 s simulated). Run limit: 2,400 steps (2 minutes at 20 Hz). With
`-E object_jitter=0` the layout is fixed and the demo deterministic. A real trial resets in under a minute.
