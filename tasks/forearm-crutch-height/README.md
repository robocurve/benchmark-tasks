# Set a pair of forearm crutches to the same marked hole

Two adult aluminium forearm crutches lie side by side on a fixed wooden stand, tips to the right, handgrips hanging
straight down, so the back of each crutch faces up. A forearm crutch's length is set at its leg. A thinner tube slides
inside the main tube and is held by a spring button on it, which stands out through one of a row of nine holes in the
main tube's back. Pressed flush, the button lets the leg slide. Let go over a hole, it springs out through it and locks
the leg.

The instruction is: "Set both crutches to the hole ringed in red: each leg's spring button must stand fully out through
that hole. Then let go of the crutches."

The near crutch starts three holes too long (hole 2) and the far one three holes too short (hole 8). The red hole is
hole 5 on both. Makers' and loan services' instructions say the same thing: press the spring button, lengthen or
shorten the leg, and make sure both crutches are in the same hole with every button fully engaged. A button caught on
the edge of its hole can let the leg slip when the user leans on it.

The hard part is physical. The button only frees the leg once it is pressed nearly flush. On its way to the red hole it
springs out into every hole it passes, and locks again, unless it is kept pressed while the leg slides. Pulled or pushed
without pressing it, the leg doesn't move.

## Scene

Positions in cm from the midpoint between the arm bases (x forward, y left, z up from the table). The layout is fixed.

| Object | Description |
|---|---|
| Stand (fixed) | Pine. For each crutch, two cradles: a 4 x 2 cm block with a 1 cm post either side of the tube, at y 1 (under the collar) and y 47 (under the tube between the handgrip and the cuff). An open pocket 3.4 x 3.4 cm and 7.5 tall at y 36 that the handgrip hangs in, 2 mm clear all round, so the crutch can't turn or slide along the stand |
| Crutches (2, 0.54 kg each) | Adult aluminium forearm crutches without locking collars, lying along y with their axes 13.5 above the table at x 23 (near) and x 35 (far). Main tube 22.2 mm (7/8 in) across, 1.2 mm wall, from a black plastic collar at y 0 up to an open grey plastic cuff (y 55.5 to 61.5, for a 9 cm forearm). A black foam handgrip, 3 cm across and 11.5 long, at y 36, hanging down. Nine 7 mm holes 1 in apart in the tube's back, from hole 1 (the lowest, the longest setting) at y 4 to hole 9 at y 24.3. Two red tape bands round the tube either side of hole 5 (y 14.2) |
| Legs | A 19 mm (3/4 in) tube, 58 long, inside each main tube, with a black rubber tip 4 across. A chromed steel spring button 6.4 mm across sits 3 cm below its top end: a flat top 2 mm across, its edge bevelled at 45 degrees. Let out, it stands 4 mm out of its hole (3 N); pressed into the tube, it pushes back with 5 N |
| Settings | Handle height (tip to handgrip) is 70 cm at hole 9 to 90.5 at hole 1, 2.54 cm a hole. The near crutch starts in hole 2 (88 cm) and the far one in hole 8 (72.7 cm). Hole 5 is 80.3 cm |

## Scoring

The task file follows the trial after every control step:

- **Dropped:** a crutch's axis went 4 cm below where it lay, or it turned over (rolled more than 60 degrees about its
  axis). This scores 0.
- **A crutch is set** when its leg's button stands fully out through the red-banded hole: the button's middle is within
  1.5 mm of the hole's middle along the tube, and its top stands at least 3 mm out of the tube (4 mm when fully out). A
  button caught on the hole's edge stands lower and doesn't count, and neither does one pressed in under the wall beside
  the hole. On a real rig, measure the button's height with a caliper's depth rod and photograph both red holes.
- **Let go:** every gripper's TCP (fingertip centre) is at least 4 cm from both crutches' axes, anywhere from the
  rubber tip to the cuff. A hand still pinching a leg or pressing a button fails this.
- **On the stand:** each crutch's axis is within 1 cm of the height it lay at, with its handgrip within 20 degrees of
  straight down, and the crutch has slid no more than 2 cm along the stand.

`score()` gives 0.4 for each crutch that is set. It gives 1.0 when both are set, let go, on the stand and still (every
crutch and leg moving under 1 cm/s and turning under 0.5 rad/s). The possible scores are 0, 0.4, 0.8 and 1.0. A leg freed and moved but not locked
in the red hole scores nothing: until the button is out through a hole the crutch has no setting at all.
`success()` ends the episode once 1.0 has held for 1 s.

## Run a trial

Following the task board's simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Set both crutches to the hole ringed in red: each leg's spring button must stand fully out through that hole. Then let go of the crutches." \
  --scorer task_score --no-prompt --max-steps 2400 --store-frames --log-dir ~/yam-eval/forearm-crutch-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`), so no seed is needed. A trial takes about 5 to 8 minutes of wall time on an L40S at 2,400 steps (sharing the GPU with two other simulations; 8 with frames stored), less when it ends early on success. The Dream-1 run scored 0.0: it never touched the crutches.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`. For each crutch in turn (near, then far):

1. the right hand pinches the leg below the main tube;
2. the left presses the button in with its closed fingertips, turned so the pads' width lies mostly along the tube. The arm
   is soft (about 500 N/m down at the fingertips), so it aims 1.3 cm below the tube to press with about 6 N, and 3 mm
   forward of the button, since the loaded arm sags that much toward its base;
3. the right slides the leg 1 cm, the left's fingertips going along with the button, until the button is under the
   wall;
4. for each hole the button still has to pass, the left lifts and presses down on that hole, and the right slides the
   leg on past it. The button can't spring out under the fingertips, and the press holds the crutch steady while the
   leg slides. The right only moves while the left holds still;
5. for the last stretch the left presses on the tube just beyond the red hole, where the button never gets to. The
   right slides the leg to the red hole and 2 mm past (its pinch slips a little), and the button springs out through
   the hole and centres itself. Then the right lets go and the left lifts away.

Set `CRUTCH_DEBUG=1` to print, every 5 steps, each button's position and its leg's (as fractional hole numbers), how
far the button stands out, how far each crutch has moved and rolled, whether the leg is held or free, whether the
wall is holding the button in, and whether the crutch is set. `CRUTCH_SAG_L` and `CRUTCH_SAG_R` ("dx,dy,dz", m) shift
each arm's poses, `CRUTCH_PRESS_GAP` sets where the pressing fingertips aim relative to the tube (default -1.3 cm), and
`CRUTCH_PRESS_OFF` ("dx,dy") shifts the press off the button's middle.

Shortcuts, with `DEMO_VARIANT` set in the environment:

| `DEMO_VARIANT` | What the arms do | Score |
|---|---|---|
| `full` | as above | 1.0 (success at step 770, 38.5 s) |
| `nothing` | wait | 0.0 |
| `near_only` | set the near crutch only | 0.4 |
| `no_press` | the right pulls and pushes both legs, nobody presses the buttons | 0.0 (the legs never move) |
| `half_way` | the far leg stops 6 mm short of the red hole, its button under the wall | 0.4 |
| `keep_holding` | set both, but the right keeps hold of the far leg | 0.8 |

## Offline checks

`test_task.py` checks the geometry and the lock arithmetic, the scoring rules on hand-built states, and the demo's
reach with the robot's inverse kinematics, without Isaac Sim. The lock checks cover the shank filling the hole when the
button is let out, the button freeing only past half its travel, the clearances, the spring forces, and the handle
heights against a real crutch's range.

```bash
PYTHONPATH=<path to yam-isaac-lab-arena> python -m pytest test_task.py -q
```

## How it is built

The crutches, legs, buttons and stand are rigid bodies made of shapes from `bodykit.py`. The main tube is hollow below
the handgrip: an octagon of eight 1.2 mm wall panels, 22.2 mm across its flats, its top panel broken by the nine 7 mm
holes. The leg is a cylinder 0.4 mm smaller than the tube's inside. It slides on a prismatic joint along the crutch (a
`Slider`), with its travel limited to the row of holes, and the two don't collide; the leg can't turn. The button
moves in and out of the leg on a second, sprung `Slider` (500 N/m, 3 N let out). The robot presses it physically:
the gripper's fingertips collide with it.

The button's catch is played by the task file, because a pressed button meeting a hole's edge jams in PhysX however
the edge and bevel are shaped (tried: a domed head, a bevelled head, square and rounded hole edges). The button
doesn't collide with the main tube; instead, after every control step, the task file reads where the button is along
the tube and how far it stands out, and sets the two sliders' drives:

- **Held in a hole:** the leg's drive holds it at that hole (20,000 N/m) until the button is pressed to 1 mm out or
  less, 3 mm of its 4, the depth at which a real button's shank clears the wall. A pull alone never frees it.
- **Free, between holes:** with the button's flat top under the wall (more than 2.5 mm from a hole's middle), its
  drive holds it level with the tube's inside, as the wall does, and the leg slides freely.
- **Free, over a hole:** the button springs out. Once it stands more than 1 mm out, its shank is in the hole and the
  leg is held there, centred. A button kept pressed by fingertips just passes over the hole.
- **A hole skipped between steps:** if the leg moved so fast (over about 10 cm/s) that a hole's middle passed between
  two control steps without ever being inside its window, the button is taken to have sprung out there: the leg is
  held at that hole and the button let out. Fingertips over the hole stop it rising, and the next step frees the leg.

The stand is a fixed (kinematic) body. `test_task.py` checks these rules on their own.

## Real-world build

Use two adult aluminium forearm crutches whose legs adjust with a single spring button and no locking collar, with
their adjustment holes in the tube's back (opposite the handgrip) and a handle height range of about 28 to 36 in. Wrap
red tape round each main tube either side of the fifth hole from the bottom. Make the stand from pine: four cradles (a
block with a post either side of the tube), screwed to a board, holding the tubes 13.5 cm above the table so the
handgrips hang free. Setting crutches is safe: the arms press a spring button and slide a 140 g tube.

## Licence

This task is released under the MIT licence. `bodykit.py` is shared with another task in this repository, unchanged
and under the same licence. No third-party models or assets are used. Every object is built from primitive shapes.
