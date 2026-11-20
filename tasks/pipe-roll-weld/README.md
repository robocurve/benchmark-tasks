# pipe-roll-weld

**Instruction:** see `task.json` (also `INSTRUCTION` in `task.py`).

Two 1 1/2 in aluminium pipes lie end to end in steel V-blocks (two under each pipe), 3 cm apart. One hand holds the
fixed pipe while the other slides the loose pipe home. A pipe-welding spring clamp stands clamped on its stand: its
spring holds its jaws shut, and squeezing its handles together opens them. It is squeezed open, carried over the joint,
lowered straight down and let go: its jaws end in two slotted half-ring bands that close round both pipe ends and hold
them in line. With the clamp on, the joint is tacked through the bands' slots, the pipes (and clamp) turned a little
each way in their V-blocks so each side comes up while the clamp's handles tip away from the gun. The clamp is then
turned back upright, squeezed open, lifted off and let close on its stand again, and the rest of the joint is welded
all round, the pipes turned in their V-blocks to bring each part on top. The gun goes back on its rest and both hands
stand clear.

## Why it matters

Category: Societal relatability. Welding two pipe ends together is a scene most people know from workshops, building
sites and film, and a short clip of this task makes it plain at a glance: sparks at the joint, a clamp holding the
ends in line, and an orange bead growing as the pipe is turned. Its difficulty is just as visible: both hands work at
once and in turn, one turning and steadying the pipe while the other holds the gun steady at the joint; the clamp's
spring must be worked (squeeze hard to open it, hold it open while placing it, let go to clamp), and the clamp, heavy
and standing over the pipe on a friction grip, must be turned with it without slipping round, rolling the pipe over or
letting its handles block the gun.

## Real parts

| object | real part | size / mass | pose (cm; x forward, y left, heights above the table) |
|---|---|---|---|
| pipes (2) | 1 1/2 in NPS Schedule 40, 6061-T6 aluminium, one end bevelled 37.5 deg with a 1.6 mm land, wire-brushed | 48.3 mm OD, 3.7 mm wall, 30 cm, 0.42 kg each | axes at x 19, 3.45 up; fixed y 0..+30, loose y -33..-3 |
| V-blocks (4) | steel, 90 deg V with a relief slot, two under each pipe | 4 cm long | centred y -26, -10, +10 and +26 |
| end stops (2) | 6 mm steel plates on feet | 4 x 6 cm | y -33.6 and +30.6 |
| welding clamp | JTLD pipe-welding spring clamp for 1 1/2 in pipe (sold on Alibaba): two stainless halves on a pivot, each a jaw with a slotted half-ring band and a knurled handle on the same side; normally shut, squeezed to open | 0.5 kg, 183 mm; bands 48.3 mm inside, 5 mm thick, 44 mm wide, each wrapping 5-135 deg, slotted 28 mm from the tips to 40 deg from the top; ~22 N on each band shut; ~35 N at the grips holds the jaws 20 deg open | clamped on its stand at x 19, y +42 |
| clamp stand | 10 cm stub of the same pipe on a steel foot, a 3 mm ridge along its top | | x 19, y +42 |
| MIG gun | MB15AK MIG torch with a Teflon liner (needed to feed 1.0 mm aluminium wire), in the Euro torch socket of a MIG welder rated for aluminium, 1.0 mm ER4043 wire, cable on an overhead balancer | 0.6 kg; 15 cm handle, 16 mm copper nozzle, wire 10 mm out | on its steel rest at x 10, y -30, handle along y |

The fixed pipe carries painted roll marks 1-3 at 120 deg near its outer end and a scribed ring 12 mm from the joint,
ticked every 30 deg: the 12 segments the score counts.

## Scoring

A segment is one of the joint's 12 30-degree stretches.

- **0** if a pipe leaves its outer V-block (its axis more than 15 mm from where the block holds it, or its end less than
  1 cm onto the block) or the clamp or the gun falls off the table.
- Otherwise **0.9 x the fraction of the 12 segments welded** (rounded to 0.001),
- **capped at 0.5** if fewer than 3 tacks were laid (below), or if
  the pipes are out of line (axes over 0.8 mm or 1 degree apart).
- **1.0** when all 12 are welded and the clamp is back on its stand (within 2 cm of where it started), the gun is on
  its rest (handle centre within 2 cm, turned under 20 degrees), both hands (gripper centre points) are 6 cm or more
  from the pipes, clamp and gun, and all of them are still (under 0.01 m/s and
  0.2 rad/s), held for 1 s (20 control steps). Success is the score reaching 1.0.

**Simulation weld rule:** the held gun's wire tip heats the joint where it is within 4 mm of it, both along the pipe
and from its surface, with the pipe ends butted (root gap under 3 mm, the usual open-root gap for this bevel: a bead can't bridge a
wider one; pushed home, the ends sit ~1 mm apart in the sim); each control step
there adds 0.05 s to the 10-degree third of the joint under it. A segment is welded (drawn orange) once each of its
three thirds has had at least 1 s and the segment 3 s in all, so the gun has to travel along it at a welding pace
(a segment is 12.6 mm of joint: about 2.5-4 mm/s), not just touch it. Past 5 s it is burned through (a hole in the
real pipe): it never counts again. With the clamp gripping the joint, each 10-degree third (4.2 mm) that gets its
1 s is a tack; the three tacks may be side by side (one 12.6 mm bead) or apart. Gripping is measured, never enforced:
the jaws within -2.5 to +1.5 deg of shut (pressed onto a 48.3 mm pipe; with nothing in them they close to -8 deg), the
bands' centre within 8 mm of the joint along the pipes, the pivot within 3 mm of its seated height over the axis and
the clamp square to the pipes within 5 deg. Three tacks join the pipes rigidly where they are.

**Real trial:** after the pipe has cooled under 50 C, a segment counts when its bead covers the joint over its whole
length (both edges at least 1.5 mm past the bevels' edges), with no hole through the wall (a lamp inside the bore
shows none), no visible crack and no pore over 1 mm, checked with a weld gauge, a 10x loupe and photographs; tacks are
counted from the video: the arc on for at least 1 s over each of at least three different 10-degree stretches of the
joint (located by the ring's ticks) while the clamp is on; in line is checked with a 300 mm straightedge laid along the
pipe across the joint at four places 90 degrees apart (no gap over 0.8 mm at its ends); the clamp's and gun's places
with a tape measure.

## Real-trial safety

- **Arc control:** the supervisor, outside the curtains, holds a hold-to-run switch wired into the gun's trigger
  circuit, closed only while the wire tip is within 1 cm of the joint.
- **Emergency stop:** one button beside the supervisor cuts the robots' power and the welder's output.
- **Arc enclosure:** welding curtains (ISO 25980) close the cell; nobody is inside while the arc can strike.
- **Fume extraction:** an extractor drawing from within 30 cm of the joint for the whole trial, and general
  ventilation of the cell for ozone; no draught across the joint.
- **Robot protection:** fire-resistant covers over the arms and each gripper finger.
- **Heat and fire:** a welding blanket over the table; a fire extinguisher beside the supervisor.
- **Cameras:** each robot camera has a fixed shade 5 glass filter, with a clear cover glass in front.

## Simulation notes

- **Clamp:** two rigid halves (0.25 kg each) on a hinge at the pivot, 52 mm over the pipes' axis; each half is a jaw
  ending in its band and a handle on the same side, so squeezing the handles together swings the bands apart. A hinge
  spring (3.7 N*m/rad, aimed 25 deg past shut) holds the jaws shut: on the pipe it presses each band on with ~22 N;
  with nothing in the jaws they close to a stop at -8 deg. Holding them 20 deg open takes ~34 N at the grips (85 mm up
  the handles), 28 deg (the stop) ~40 N; the YAM gripper squeezes up to ~41 N (measured). At 20 deg each band's tip is
  ~5 mm clear of the pipe's widest point.
- **Grip by contact only:** nothing snaps or is glued: the bands' inner faces sit on the pipe with the jaws shut, and
  the spring's squeeze holds and centres the pipes through friction (stainless on aluminium, 0.8 static) and the bands'
  135 deg wrap. Placed by the demo, the clamp settled within 0.3 mm of centred and the pipes within 0.15 mm and 0.15 deg
  of in line. The 28 mm slots leave a 16 mm nozzle 6 mm to spare with the clamp a little off centre; with 20 mm slots
  the nozzle caught the prongs.
- **V-blocks:** two under each pipe (y +-10 and +-26 cm). With one per pipe (near its outer end) each pipe tipped
  nose-down onto the table and the ends met 5 deg out of line.
- **Pipes** are rigid, with real contact in the V-blocks (aluminium on steel, friction 0.6); round bodies collide as
  24-sided hulls. The welded pipes creep up to ~2.5 cm along their axis over a run as they are turned; the end stops
  bound that.
- **Not simulated:** arc, heat, wire feed, bead shape, spatter, distortion, cable drag and the arm covers; the finger
  sleeves are thicker, grippier pads (friction 0.7).

## Run a trial

From this directory, with the yam_arena environment (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1):

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    --scorer task_score --max-steps 7200 \
    --instruction "$(python -c "import json; print(json.load(open('task.json'))['instruction'])")"
```

## Dream-1 run

```bash
inspect-robots run --embodiment yam_arena --policy dream \
    -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --scorer task_score --max-steps 7200 --store-frames --log-dir dream-run \
    --instruction "$(python -c "import json; print(json.load(open('task.json'))['instruction'])")"
```

Dream-1 scored 0: it ran the full 7,200 steps without welding any segment. Its arms moved about the pipes and the
gun, but the pipes' 3 cm gap was never closed and the clamp stayed on its stand.

## Scripted demo

```bash
python rundemo.py task.py demo_out --max-steps 7200
```

Closed-loop (it reads the live object states). Left hand holds the fixed pipe while the right slides the loose
pipe home (~1 mm gap). Left: reaches down to the clamp's handles on its stand and squeezes them (~34 N): the jaws open
~22 deg; lifts it off, carries it over the joint and lowers it straight down, waiting for it to stop swinging and
correcting its aim 4.5 cm and 8 mm above the seat; opens its fingers slowly: the spring closes the bands on both pipes
(clamp 0.3 mm off centre, pipes 0.14 mm / 0.12 deg out of line). Right takes the gun. Left grips the fixed pipe and
turns pipes and clamp 25 deg (seen from the left: clockwise, handles tipping toward the robot; the pipes overshoot
and the clamp reached 30.6 deg) and holds them,
re-centring its grip each step so the pipe stays on its V-blocks; right welds along the side that came up, through its
slot, correcting the wire tip after every move from its measured position (the soft arm otherwise misses by up to
14 mm on the far side): 4 tacks, which join the pipes (0.04 mm / 0.02 deg). Left turns them back and 25 deg the other
way (no more tacks are needed), then upright. Left squeezes the clamp open, lifts it straight off, sets it on its
stand and lets go. Then it turns the pipes four times (bringing the unwelded stretches on top) while right welds the
rest at the same pace, lifting the wire tip straight off before moving over stretches already done, so none passes
5 s (segments got 3.5-4.7 s). Right puts the gun back on its rest; both arms clear. Score 1.0, success at step 5,067
(253 s). `demo.mp4` is the three cameras side by side.

Test variants: `PRW_DEMO=noclamp python rundemo.py task.py out --max-steps 7200` welds the whole joint the same way
but never puts the clamp on: with nothing holding the pipes together, the first turn opens the joint to 3.2 mm (past
the 3 mm limit), so it welds nothing and scores 0.0 (under the 0.5 cap either way). `PRW_DEMO=tacks` stops after the
tacks; `PRW_DEBUG=FILE` logs the wire tip after every weld move and dumps the parts' poses to FILE once.

## Runtime and seeds

- Run limit 7,200 control steps (6 minutes at 20 Hz); physics 240 Hz.
- Fixed starting poses (`object_jitter=0`), no randomisation; seed 0.
- One trial of the demo takes about 10 minutes of wall time on one RTX 5080 including Isaac Sim start-up.

## Files

`task.py` (scene, clamp mechanism and grip measurement, weld rule, scorer, scripted demo), `mechkit.py` (compound rigid
parts, joints, latches and the yam_arena hooks), `rundemo.py` (the closed-loop demo runner: writes the three camera
videos, `setup.png` and `result.json`). No third-party assets. No hardware validation has been performed.
