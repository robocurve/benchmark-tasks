# Counter-twist a loose twist-lock pole's tubes to free its cam, draw it to the red mark, and lock it for a 50 N push

A twist-lock extension pole lies loose on the table, collapsed and locked. The robot sets it to the red mark on its inner tube and locks it, so it carries a 50 N push along its length.

**What this task adds.** The lock is a torque couple on two coaxial tubes lying free. A quarter turn of the inner tube against the outer frees the eccentric cam inside, and a quarter turn back wedges it. Other locking tasks use a latch or pin that shows its state; this lock has no detent, click or hole, and only the push shows whether it holds.

**The pole rolls.** A round tube lying on a table hardly resists rolling: the simulator's 32-sided tubes tip over an edge at about m g r sin(pi/32) = 0.45 kg x 9.81 x 0.0125 m x 0.098, roughly 0.006 N m, against the cam's 0.3 N m. So a twist on one tube rolls the whole pole unless the other tube is held still while it turns.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 800 \
  --instruction "Twist the pole's tubes against each other to free its lock, set it to the red mark, and lock it again."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                                     # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode one_arm,nolock,short --hold 1                # failure checks, below
python demokit.py task.py OUT --no-record --no-cameras --mode g_roll,g_pull --hold 0           # code forces: a twist on the free pole, and a pull on the locked pole
python check_score.py                                                                          # the scoring table on hand-built records, no simulator
```

The demo is a scripted reference: it reads the pole's pose from the simulator, and a policy gets only the cameras. Both arms come straight down round the pole, the left on the outer tube near its middle and the right on the inner tube just outside the collar, gripping with about 45 N, and lift it 12 mm off the table. The right hand turns the inner tube two eighths of a turn about the pole's axis, letting go and gripping again between them, which frees the cam; the left hand holds the outer tube still. The right hand draws the inner tube out 24 cm and turns it back an eighth. Then it holds the inner tube while the left hand turns the outer tube the last eighth the other way, which locks the cam. Both set the pole down, let go and rise, and the scorer pushes it.

**Result** (scripted, GPU physics on an RTX 5090, 20 Hz): 1.0. The pole set to 242 mm out, 2 mm past the mark, the cam at 0.4 degrees, and the 50 N push shortened it 1.0 mm. 407 steps (20.4 s of simulated time), about 75 s of wall time. No seed changes the scene.

| Failure check | What it does | Score |
|---|---|---|
| `one_arm` | the right hand alone twists the inner tube and draws | 0: the pole rolls 140 degrees with the hand, the cam stays locked (0.9 degrees), and the draw drags the whole pole |
| `nolock` | set to the mark, let go without locking | 0.5: 1 mm past the mark, the cam free at 86 degrees, no push test |
| `short` | drawn 40 mm short of the mark, then locked | 0: locked 38 mm short, past the 30 mm of the 0.25 tier ("anything else" scores 0) |

## Building it for a real trial

- **Pole:** a 2 to 4 ft (61 to 122 cm) twist-lock aluminium extension pole for paint rollers (Linzer Twist 2 Extend class; record the exact model used), with a 25 mm outer tube and a 22 mm inner tube. Paint a 10 mm red band round the inner tube at the trial's target length: 24 cm out from the collar for 85 cm.
- **Table:** a plain table top, no stops near the pole.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Torque to turn the cam | 0.3 N m | **Not measured; our guess.** It is set below what the demo's 45 N pinch can turn on the 22 mm tube (about 0.5 N m at pad friction 0.5), and well above the pole's 0.006 N m of rolling resistance, so the task's difficulty holds anywhere between those. Clamp a 10 cm lever to the inner tube, hold the outer tube, and pull the lever's end with a luggage scale: the reading in kg x 9.81 x 0.1 m |
| Turn at which the lock frees | Free from 70 degrees off where it was found; fully locked within 20 degrees of it; in between, the hold falls linearly | Mark both tubes and read the angle at which the inner tube starts to slide |
| Slide force when free | 2 N | A luggage scale on the inner tube's tip, the outer tube held |
| Hold when locked | up to 200 N along the pole | Stand the locked pole on its outer tube's capped end on a bathroom scale, press the inner tube's tip down to 5 kg (about 50 N) for 2 s, and check with a tape that it shortens under 2 mm |

## Scoring

See `scoring` in `task.json`, and `score` and `Cam` in `task.py`. Once the pole is at its mark, locked, and both grippers have been 3 cm clear for 0.5 s, the scorer pushes the inner tube 50 N along the pole for 2 s while holding the outer tube's end, as a floor would, and records how much it shortens.

## Safety for a real trial

The pole is light and blunt. Keep people clear of the arms' reach while it runs and keep an emergency stop in reach; the push test is done by hand afterwards.

## Assets

All geometry is built in code from cylinders and boxes. No third-party assets.
