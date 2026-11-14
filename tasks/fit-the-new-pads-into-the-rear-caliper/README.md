# Fit new brake pads into a rear caliper with an integral parking-brake piston

A rear brake caliper, off the car on its carrier, is bolted piston-up to a bench stand. Its piston stands 5 mm proud
of the inner pad's ledges, where the worn pads left it. On calipers with an integral parking brake the piston rides
a self-locking screw adjuster: pushing it straight in does nothing. It goes back only when it is turned clockwise
with a two-pin tool pressed hard into its face slots, so the pins stay in as the face recedes over several turns.
The two new pads are started in the carrier's channels at the window in the caliper's bridge; the inner pad cannot
pass the proud piston until it is wound back.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. Two helper files
must sit next to `task.py`, which imports them:
- `parts.py`: the parts kit (multi-colour parts with joints), in the version with chained joints, screw threads
  (PhysX mimic joints), per-part contact offsets and compliant contact (see below).
- `tilt.py`: lets the scripted demo lean a gripper; it affects only the scripted demo, never Dream-1 or scoring.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit the two new pads into this caliper."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 3000 --store-frames --log-dir logs/dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 3000 --store-frames --log-dir logs/demo
```

- **Runtime per trial:** at most 150 s of simulated time (3,000 steps at 20 Hz); about 5 to 6 minutes of wall time
  on an L40S once Isaac Sim has started. The demo plan lasts about 135 s.
- **Seeds:** submitted runs use `-E object_jitter=0`; the layout is fixed. The pads and the tool are the loose
  objects.

## The mechanism

- **Adjuster thread:** under the piston an adjuster nut rides a helical joint: a slider (`feed`) on the caliper body
  and the nut's hinge (`nut`) on it, tied by a PhysX mimic joint so the nut travels 2.0 mm per turn (down for a
  clockwise turn seen from above).
- **Self-locking:** the nut's hinge drive holds it with up to 0.12 N m. An `on_step` callback moves the drive's rest
  point only once the nut has slipped past it (Coulomb friction: it sticks below the breakaway torque and is dragged
  at it above), so a steady push, which makes only F x lead / 2 pi of torque on the thread (0.012 N m at 40 N), never
  creeps it. PhysX joint friction is not used (it does not hold a loaded joint on the GPU).
- **Cone clutch:** the piston is a separate link (a slider `lift` and a hinge `piston`) that rests on a ring at the
  top of the nut. Only contact friction couples them: pressed hard, turning the piston turns the nut; unpressed (its
  own 120 g), the piston spins on the nut and nothing advances.
- **Checked on the GPU** (trace with a probe phase): with the pins in the slots, a straight push commanded 40 mm
  below the face for 4.5 s moved the nut 3 degrees (0.02 mm); two unpressed strokes turned the piston 341 degrees
  and the nut 0; pressed strokes then drew the face back about 0.6 to 1 mm per 190-degree stroke.
- **Tool:** the adapter and extension are a link on a hinge in the handle. Its upper joint limit is the ratchet
  pawl: every control step an `on_step` callback lowers it to where the shaft has been (plus 0.02 rad play), so the
  adapter drives clockwise and freewheels on the back swing.
- **Pads and carrier:** each pad is one rigid body (backing plate, ears, lining). The ears sit on ledges, under lips
  1.5 mm above them, between side plates, and stop against the abutment stops when seated. Contact offsets are
  0.5 mm (caliper, pads) and 0.3 mm (tool).

## Scoring as implemented

`score()` and `success()` in `task.py`:

- **Released** (`_measure_grip`, every control step): the distance from both arms' finger and gripper hulls
  (`yam_arena.analysis`) to every shape of the piston and of both pads must be over 5 mm. Until it has measured, or
  if it cannot, the state is unknown and nothing scores (fail closed). The tool's shapes must also be at least 5 mm
  from them (`_tool_clear`); the tool may be held or put down.
- **Piston back** (`piston_face`): the face (start 68 mm above the table plus the `lift` joint) at or below the inner
  ledges' top, 63 mm.
- **Pad seated** (`pad_seated`): the pad's backing plate face on its own ledge level within 1 mm, its centre within
  1.5 mm of the seated place in x and y, tilt and turn within 3 degrees, and at rest (under 1 cm/s and 0.5 rad/s).
- **Score:** 0 unless released and the piston is back; then 0.5, plus 0.25 per seated pad. Success (1.0) must hold
  1 s.
- `near_misses.py` checks every tier and boundary: the piston 0.1 mm proud and exactly at the ledges, one pad of
  either kind, a pad 1.5 and 1.6 mm short, half in, 1.2 mm high, tilted or turned 4 degrees, still sliding, both pads
  forced in with the piston proud, the tool left on the piston or 4 mm above a pad. Offline there is no robot, so it
  sets the arms as measured clear.

## Scripted demo

`demo()`, fixed waypoints from the published start layout only (no object state is read):

- Right arm: picks the tool from its holder with the palm on the handle's cap, carries it round the caliper's far
  end, turns the pins along the slots, and brings the adapter in low through the gap between the fingers (the arm
  cannot carry it over the caliper's walls). It aims 0.8 mm past the piston's axis and 6 degrees past the slot line,
  as the held adapter was measured to settle short at this pose. It presses (commanded 50 mm below seated, more each
  stroke) and makes twenty 190-degree clockwise strokes (wrist +145 to -45 degrees) with ratchet back swings (the face ends
  about 5.8 mm down, 0.8 mm past the line; later strokes gain less as the pins slip in the slots), then
  eases off, lifts the pins out and slides the tool out through the gap. The strokes leave the tool leaning about
  20 degrees in the grip with its adapter back under a finger, so the hand leans 30 degrees (`tilt.py`) to swing the
  adapter past the fingers' ends before lifting, then carries the tool leaning 23 degrees the other way so it hangs
  upright, aims 15 mm and 13 mm off the holder's centre (where it was measured to hang) and stands it back in its
  holder. These leans and offsets are fixed, measured once on a trace; nothing is read from the simulator.
- Left arm, once the right arm has moved away: with its closed fingers leaning finger tips forward (`tilt.py`, so only
  the tips meet a pad's edge), slides the outer pad home along its channel at 20 degrees, then the inner pad at
  45 degrees (at a smaller lean the hand meets the seated outer pad's edge and stops the inner pad about 6 mm short).
  It is sent 20 to 30 mm past seated; the arm stops short against the stops.

## Fidelity

True size. Fixed layout, as the board asks. The caliper and carrier are one fixed body built from boxes, bolted
piston-up to a bench stand so a straight-down tool can turn the piston (the arm turns tools only about the vertical).
There is no disc; the ledges at two levels keep its 10 mm gap between the pads. The adjuster is a 2 mm/turn helical
joint that self-locks: a straight push of up to about 40 N cannot move it measurably (0.12 N m breakaway torque, about
380 N of push), and an unpressed turn spins the piston on the nut; only turning while pressed into the face slots
winds it back, driven through contact friction as a cone clutch is. No hydraulics or dust boot. The parking-brake
lever and cable, slide-pin bolts and boots, bleed nipple and round housing casting are visual only (no colliders).
A 0.3 N m breakaway was tried and the pressed adapter could no longer turn the nut (the pins rode out of the slots),
so 0.12 N m is used.

## Licences

All geometry is built from primitives in `task.py`; no third-party assets.
