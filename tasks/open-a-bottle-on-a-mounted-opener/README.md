# Open a crown-capped bottle on a mounted bottle opener

A full 275 ml long-neck glass bottle with a red crown cap stands in front of the right arm. Further forward a cast steel
wall-mount bottle opener is screwed to a dark post, as on a bar counter. Its jaw sticks out toward the robot. A cork
coaster lies near the middle of the table.

The instruction is: "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go."

Bartenders and servers open crown-capped bottles on fixed openers many times a shift. The opener can't move, so the
bottle has to be brought to it. The robot lifts the bottle and slides the back of the cap into the jaw's slot, so that
the catch lip at the bottom of the slot is under the lower edge of the cap's skirt. Then it pulls the bottle down. The
lip holds the back of the skirt while the bottle drops, so the skirt is bent up off the glass bead and the cap pops
off. Then the robot stands the open bottle on the coaster. A crown cap can't be pulled off by hand. Away from the catch
lip, the cap stays crimped on however it is pulled.

## The bottle, the cap and the opener

All sizes are in the model (`task.py`) and in `task.json`.

- **Bottle:** a 275 ml long-neck glass bottle with a standard 26 mm crown finish, 450 g full. It is 18.5 cm tall to the
  top of the finish. The body is 5.8 cm across and 11 cm tall. The shoulder narrows to a lower neck 3.2 cm across
  (13 to 16.2 cm up), then to 2.35 cm under the finish. The finish's bead is 2.64 cm across and 4.3 mm tall.
- **Crown cap:** crimped on, 2.88 cm across the skirt and 6 mm tall. Its skirt reaches 0.7 mm below the bead, and its
  lower edge stands 1.2 mm out from the bead all round. That edge is what a catch lip gets under.
- **Opener:** a back plate 6 cm wide, 7 cm tall and 5 mm thick, on a post 4.5 cm wide and 2.5 cm deep that stands on
  the table and runs up to the plate's top, 23.5 cm. A jaw 1.8 cm wide sticks out of the plate's front face toward the robot. From the bottom up it has:
  - **The catch lip:** 4 mm thick, its top 21.4 cm above the table, reaching 2.45 cm out from the plate.
  - **The slot:** 8.5 mm tall, its back wall 2.35 cm out from the plate. The lip sticks 1 mm out of the back wall. When
    the cap's skirt touches the back wall, the lip is 1 mm under the skirt's edge and 0.2 mm clear of the bead.
  - **The upper prong:** 3 mm thick, reaching 3.15 cm out from the plate, 8 mm over the cap's top. It is the fulcrum
    the cap tips against.
  - **Placement:** the plate's front face is at x 43.8 cm, the jaw is centred on y -20 cm, and the plate spans 16.5 to
    23.5 cm above the table.

## How it differs from nearby tasks

- **Paint-tin lids pried off with a hand tool** (a lid lifter's claw hooked under tabs, or a screwdriver levered under
  the lid). There the container stays put and a hand-held tool does the work. Here the tool is fixed and the robot
  moves the workpiece, a full bottle carried in mid-air, into it. The cap comes off from the bottle's own downward
  stroke, and nothing is levered by hand.
- **Uncorking a wine bottle with a corkscrew:** a hand-held tool is screwed in and pulled while the bottle is held still.
- **Twist-off, flip-top, child-resistant and latched bottle caps:** these are opened by hand, with no tool.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, the crimp rule, `score()`, `success()` and the scripted `demo()` |
| `partkit.py` | Rigid parts built from boxes and cylinders, with pose writes and step/reset hooks, for yam_arena task files |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go." \
  --scorer task_score --no-prompt --max-steps 1200 --store-frames --log-dir ~/yam-eval/bottle-opener-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`), so no seed is needed. A trial takes about 3.5 minutes of wall time on an L40S at 1,200 steps.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`, and `--max-steps 450`. It reaches full score: the cap
pops at step 263, and the episode ends on success at step 427 (21 s of simulated time). Set `BOTTLE_DEBUG=1` to print
the bottle, the cap and the trial record every 5 steps.

1. The right arm grips the bottle by its lower neck and lifts it 3.5 cm.
2. It carries the bottle in front of the opener at slot height. Then it moves it straight back until the back of the
   cap's skirt reaches the back of the slot, so the catch lip is under the skirt's edge.
3. It pulls the bottle down 6 mm while pressing it 4 mm further back into the jaw, as a person does. The lip holds the
   back of the skirt, which is bent up off the bead, and the cap springs off toward the robot and falls on the table.
4. It lowers the bottle 12 mm clear of the jaw, backs it out, stands it on the coaster, lets go and moves the arm clear of the top camera.

Loaded with the full bottle, the arm misses its commanded pose by a pose-dependent offset: in front of the opener it
lands 16.5 mm further forward and 11.6 mm low. The demo aims that much the other way (`SAG` at the opener, `SAG_DOWN`
over the coaster).

Shortcuts, with `DEMO_VARIANT` set in the environment:

| `DEMO_VARIANT` | What the arm does | Score |
|---|---|---|
| `nothing` | waits | 0.0 |
| `skip_opener` | carries the closed bottle straight to the coaster and lets go | 0.0 |
| `pull_cap` | pinches the cap and pulls it straight up | 0.0 |
| `under_prong` | stops the cap 6 mm short of the slot, under the prong's tip, and pulls down | 0.0 |
| `no_place` | opens the bottle and keeps holding it | 0.5 |
| `no_release` | opens it and stands it on the coaster without letting go | 0.75 |

## Scoring

The task file follows the trial after every control step and keeps a record that is cleared on reset:

- **Caught:** the back of the cap's skirt is on the catch lip. The lip's tip is 12.8 to 14.2 mm in from the cap's axis
  (past the bead and inside the skirt), the cap's axis is within 6 mm of the jaw's centre line sideways, and the
  skirt's lower edge is 0 to 2.5 mm above the lip (-0.5 mm allows for contact).
- **Popped:** while caught, the skirt's edge on the opener side has been bent at least 1.5 mm up off the bead. The cap
  then springs 10 mm forward and 5.5 mm up within one control step, clear of the bead and the prong, flips and falls.
  Until then, the cap is put back on the finish after every step unless it is caught. Pulling, pushing or knocking it
  anywhere else does nothing.
- **Tipped:** the bottle tilted more than 30 degrees at any time, or fell off the table.

`score()` gives 0 if the bottle tipped. Otherwise it gives:

- **1.0** when the cap has popped and the bottle stands on the coaster (axis within 3 cm of its centre, base on it,
  within 10 degrees of upright), still, with each gripper's TCP (fingertip centre) at least 6 cm from the bottle's grip
  point on its lower neck, 15.4 cm above its base.
- **0.75** when popped and on the coaster.
- **0.5** when popped.
- **0.25** when caught.
- **0** otherwise.

`success()` ends the episode once 1.0 has held for 1 s.

## Real-world build

Use a full 275 ml long-neck crown-capped glass bottle with a 26 mm crown finish. Use a cast or stainless jaw-type
wall-mount bottle opener (a back plate and a jaw with a catch lip under an upper prong). Screw it to a post or block
that stands on the table, with the catch lip's top at the height in `task.json` and the jaw facing the robot. If no
cast opener matches the jaw's dimensions, bend and weld one from 3 mm steel. With the bottle held upright and pressed
back into the jaw, a pull down pops the cap. To judge the 0.25 level, film the jaw from the side as well. Score
from a video and a ruler. Opening the bottle is safe: the arm only pulls the bottle down a few millimetres with the
cap in the jaw. Use a crown-capped PET bottle if glass on the bench isn't allowed.

## Licence

This task is released under the MIT licence. `partkit.py` comes from `tasks/din-rail-mount` in this repository. It is
unchanged and released under the same licence. No third-party models or assets are used. Every object is built from
primitive shapes.
