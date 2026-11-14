# Lift each gnocchi out of simmering water with a slotted spoon soon after it floats up, then turn off the burner

Six fresh potato gnocchi lie on the floor of a sauté pan of simmering water on an induction hob. They went in two at a
time before the trial, and each floats up when it is cooked through, 1 1/2 to 2 minutes after it went in (America's
Test Kitchen, *Potato Gnocchi*: "cook until they float, 1 1/2 to 2 minutes", then lift them out with a slotted spoon).
The robot picks up the slotted spoon from the counter, lifts each gnocchi onto the plate soon after it floats up
without pinching it, puts the spoon down and turns the hob's knob to OFF.

Turning the burner off is how a cook finishes the cooking step: once the last gnocchi is out of the water, the heat
goes off rather than the pan being left simmering unattended, and the station is left safe for the next order. It is
part of the same job, not a separate appliance task (O*NET also lists, for restaurant cooks, "Ensure food is stored
and cooked at correct temperature by regulating temperature of ovens, broilers, grills, and roasters"). The score
says so plainly: the burner left on caps the trial at 0.5 however well the gnocchi came out, and turning it off
earns nothing by itself. The scooping carries the score.

`task.py` holds the scene, the scorer and the scripted demo. The task's helper modules: `floatkit.py` (the water),
`yamkit.py` (the modeled props and the knob's joint) and `guardkit.py` (the hand's shape for the contact checks).

## Run a trial

The scripted demo:

```bash
GNOCCHI_SEED=0 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/lift-gnocchi-as-they-float/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The gnocchi are cooking. Take them out onto the plate when they're done, then turn off the burner." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir <log-dir>
```

It scores 1.0 and ends on success at step 2,053 (about 103 s).

Model run (Dream-1): the same command without `GNOCCHI_SEED=` and with `--policy dream --max-steps 2400`.

## Scoring

`--scorer task_score` gives 1.0 when the task succeeds and otherwise `score()` at the end of the trial.

| Rule | Value | Why this much |
|---|---|---|
| Floated up | top within 3 mm of the surface on its own, not in the spoon | a cooked gnocchi (950 kg/m3) floats with about 1 mm showing; 3 mm allows for it bobbing as it comes up |
| Out | the last time its lowest point rose above the surface | a raw piece lifted and put back does not count against it; the lift that matters is the last |
| Good | out after it floated and at most 45 s later, on the plate and still for 1 s, never touched by a gripper | cooks lift each one as it comes up (America's Test Kitchen); left floating it takes on water and goes soft. 45 s is long enough for a cook busy with the previous piece |
| On the plate | center over the 14 cm well, lowest point at most 2 cm above it, not in the spoon | one piece leaning on another still counts; a spoon left on the plate is not plating |
| Touched | contact of the hand's own palm, fingers and pads with the piece (2 mm), not a distance zone | fingers dent a soft cooked gnocchi, which is why cooks use the spoon |
| Burner off | the knob's pointer within 8 degrees of OFF | 8 degrees is the knob's click: inside it the switch is open |

The trial scores 1.0 when all six pieces are good, no hand is on the spoon and the burner is off. It scores 0.75
with five good pieces, 0.5 with three or four and 0.25 with one or two: each lost piece is a portion short, and the
steps are wider at the bottom so that one or two right is not worth as much as the job. A hand still on the spoon at
the end caps the score at 0.5 (the spoon was not put down), and so does the burner left on (the cooking step is not finished while
the pan is left simmering unattended, which is how kitchen fires start). The whole trial scores 0 if any piece ends outside the pan without
having floated up before it left the water: a raw gnocchi is dense and floury in the middle and can't be served,
wherever it was put. Otherwise it scores 0, including when nothing is done. Scooping everything out early scores 0;
leaving them all and scooping them in the last 10 s of the run scores 0.25 at most. Turning the burner off with no piece plated scores 0. The burner may be turned off at any time, the spoon may end
anywhere, and no arm position is scored. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

Real trial: an overhead video at 30 fps gives each piece's floating time (its top breaks the surface and stays) and
out time (it leaves the water in the spoon), read frame by frame; the final photo shows the plate and the knob's
pointer; a piece touched by a finger shows on the wrist video and is visibly dented; a piece suspected raw is cut open.

## Runtime and seeds

2,400 steps (2 minutes at 20 Hz). The pieces go in two at a time, 85, 57 and 30 s before the trial starts, and each
floats up 1 1/2 to 2 minutes after it went in, so pieces come up in overlapping pairs. `GNOCCHI_SEED=0` is the
canonical run (the screenshot's): G1 and G2 first, then G3 and G4, then G5 and G6, floating up at about 10.5, 21.5,
38.5, 49.5, 65.5 and 78.5 s. Any other seed shares the pieces among the three drops at random, draws each floating time
anew within 1 1/2 to 2 minutes and changes the simmer drift's pattern, so the order, the times and the place of each
rise are not known in advance. With `GNOCCHI_SEED` unset a run draws its own seed and prints
`lift-gnocchi: seed N` to stderr; `GNOCCHI_SEED=N` repeats it.

Offline checks (no simulator, CPU only), from this folder with `yam_arena` installed:
`python -m pytest -q test_floatkit.py test_task.py test_demo.py` (the demo tests play the full two minutes in a small
kinematic world, `world.py`, a few minutes each).

## How the water works

There is no fluid in the simulator. `floatkit.py` applies the water's forces to each gnocchi before every physics step
(240 Hz) as an external force and torque on its rigid body; PhysX does every contact. Only what is inside the pan's
wall and below the surface is in the water.

- **Surface:** flat, at the pan's floor plus (water + displaced volume) / the pan's area.
- **Buoyancy:** water density x g x the submerged volume, from a 1.25 mm grid of points through each piece's rounded
  shape, straight up at the submerged points' centroid.
- **Cooking:** at its time each piece's density drops from 1,080 to 950 kg/m3 (ramped in over 2 s), so it leaves the
  floor and reaches the surface about 2.5 s later.
- **Drag:** quadratic drag per submerged point (drag coefficient 0.9), plus damping of a floating piece's bobbing in
  place of the waves it would make.
- **Water movement:** a gentle simmer drift (3 mm/s rms at the surface, never through the wall) and the push of the
  spoon, from how its bowl actually moves between physics steps: water ahead of the bowl moves with it, falling off as
  (2.2 cm / distance)^3. A spoon swept at 20 cm/s pushes a floating piece about 8 mm; at 3 cm/s, under 2 mm.
- **Not modeled:** waves and slosh, added mass, surface tension, heat.

## The scripted demo (open loop for GNOCCHI_SEED=0)

The demo is a fixed list of hand waypoints built from constants in `task.py` (where each piece starts and the seed-0
floating times); it reads no object state during the run. A model run gets times drawn for that run.

The right hand comes down over the spoon's soft grip where it lies on the counter, closes across the grip's 2.8 cm
width, lifts the spoon 5 cm as it lay, turns the bowl level and holds it over the pan's near-right side. For each piece,
2.5 s after it turns buoyant (it surfaces about 2.5 s later), it lowers the bowl 2.3 cm under the water, tip first and
13 degrees down, its tip 2.2 cm short of where the piece rose, slides it under at up to 3 cm/s, levels the bowl as it
rises to the surface in 0.5 s, lifts it out at 2.5 cm/s for the first 2 cm so the water drains, carries it over the rim
to the plate and tips it 45 degrees about the bowl's flush tip, 3 mm over the well, so the piece steps off with no drop.
The second piece of each pair leaves the floor just after the first is out of the water and floats 5 to 6 s while
the first is carried to the plate. The held spoon sits about
1.1 cm to one side and 0.5 cm toward the handle from where the hand is sent (measured in the simulator), so the scoops
of G2-G6 send the hand that much the other way. After the sixth it lays the spoon back on the counter as it lay, lets go
and backs off. Meanwhile the left hand comes down over the knob with its fingers across the bar, grips it, turns it
90 degrees counterclockwise to OFF in three steps, lets go and goes home. Offline (`test_demo.py`) it scores 1.0 at
seed 0, also with a laggy arm.

## Real-world build

All off the shelf: a slim portable induction hob with a rotary control knob (about 4 cm tall; the knob 4.2 cm across
with a grip bar); a 24 cm stainless sauté pan 7 cm deep; a black nylon slotted spoon about 32 cm long with a soft grip
about 2.8 cm wide and its bowl about 30 degrees from the handle; an 18 cm white plate on a damp cloth; fresh potato
gnocchi (America's Test Kitchen's Potato Gnocchi recipe) rolled to 2.3 x 1.5 x 1.1 cm (about 3 g). Tape marks on the counter fix the hob, the spoon's place and the
plate. Safety: the water is kept at a gentle simmer (small bubbles, not a rolling boil), the hand's fingertips stay at
least 2 cm above the water and only the spoon's nylon goes in, and the operator stays outside the arms' reach while
the arms move, with the emergency stop in hand, and wears oven gloves to drop the gnocchi in. The pan is not clamped:
with its water it weighs about 3.5 kg and only the spoon touches it (the simulator holds it fixed).

## Files

`task.py` (scene, scorer, demo), `floatkit.py`, `yamkit.py`, `guardkit.py` (the task's helper modules), `world.py`
(offline test world, tests only), `test_floatkit.py`, `test_task.py`, `test_demo.py`, `task.json`.
