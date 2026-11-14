# Fit the last magnet into a motor rotor, holding down the loose magnet beside it so it cannot jump out

A motor rotor, a stack of steel laminations 20 cm across, stands on its assembly fixture with four magnet slots round
its rim. Two magnets are set in resin. The third, a blue-topped magnet beside the one empty slot, was only just pushed
in and sits loose. A wooden tray holds two more magnets: one with its red (north) face up and one with its blue (south)
face up. The poles must alternate round the rotor, so the empty slot takes the red-topped magnet.

Neighbouring rotor magnets attract each other. As the new magnet comes over its slot, it pulls the loose magnet up out
of its own slot. One hand therefore presses the loose magnet down. The other brings the red-topped magnet over the empty
slot, lowers it in, lets go and presses it in flush with the rotor's top. Then both hands let go and both arms go back
to where they started.

The task is one task file,
[`fit_the_last_magnet_into_a_motor_rotor.py`](fit_the_last_magnet_into_a_motor_rotor.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

**What it tests:**
- picking the magnet with the right pole by its colour;
- holding one part still against a pull while the other hand assembles next to it;
- pressing a part home;
- finishing with both hands free.

**Why two hands:** an unheld loose magnet is lifted out of its slot by the new magnet's pull. The new magnet has to come
down over the slot to go in, and the pull is strongest at 6 to 10 cm above it, so the loose magnet must be held down
the whole time. The one-hand check scored 0.25: the right hand did its part as in the demo while the left never moved.
The loose magnet jumped 24 mm out of its slot as the new magnet came in.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The rotor, with the two potted magnets, the shaft
stub and the fixture plate, is one kinematic body made of shapes from `bodykit.py`. The tray is kinematic too. The
three loose magnets are rigid bodies.

- **The pull:** after every control step, the task file applies a force along the line between the loose magnet's
  middle and each tray magnet's middle. It is 8.5 N at 11 cm, falls as the fourth power of the distance, and is capped
  at 12 N. The red-topped magnet attracts the blue-topped loose magnet; the blue-topped one repels it. The opposite
  force goes on the tray magnet. The potted magnets pull on nothing.
- **The steel:** the steel round the empty slot pulls a magnet whose bottom is within 2 cm above the slot's mouth
  (and within 2.5 cm of its middle line). The pull is toward the slot's middle line (1,200 N/m, at most 8 N) and down
  into the slot (1 N).

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s; `success()` means full score (which
includes both arms back where they started) with the red-topped magnet and the loose magnet still. Otherwise it gives
`score()` at the end of the episode: 0, 0.25, 0.5 or 1.0 (see `task.json`).

- `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
  layout.
- The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending
  it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fit_the_last_magnet_into_a_motor_rotor.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the last magnet into the motor rotor: take the red-topped magnet from the wooden tray (the poles must alternate red, blue, red, blue around the rotor) and put it into the empty slot, pressed in flush with the rotor's top. The blue-topped magnet in the slot next to it is loose and will jump out toward the new magnet as it comes near, so hold it down with your other hand until the new one is in. Then let go and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/rotor-dream
```

The scripted demo reaches full score:
1. The left hand comes down on the loose magnet with its fingers closed, leaning a little outward, and presses it down.
2. The right hand comes straight down on the red-topped magnet in the tray, fingers closing across it, and lifts it.
3. The right carries the magnet to just above the rotor's edge, brings it in over the empty slot (aiming 1.7 cm wide of
   it, against the pull that bends the arm) and lowers it half way in.
4. The right lets go; the magnet sinks in flush. The right closes its fingers and presses on it.
5. Both hands rise and both arms go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fit_the_last_magnet_into_a_motor_rotor.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the last magnet into the motor rotor: take the red-topped magnet from the wooden tray (the poles must alternate red, blue, red, blue around the rotor) and put it into the empty slot, pressed in flush with the rotor's top. The blue-topped magnet in the slot next to it is loose and will jump out toward the new magnet as it comes near, so hold it down with your other hand until the new one is in. Then let go and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/rotor-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/rotor-dream/*.json; do inspect-robots video "$log"; done
```

Set `ROTOR_DEBUG=1` to print what the score is built from twice a second:
- how far the loose magnet has risen, and whether it has jumped;
- the pull on it;
- where the red-topped magnet is, and whether either tray magnet is half way into or flush in the empty slot;
- where each TCP is;
- whether both arms are home.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 900 steps (194 s on the clock, with Dream answering in 1.00 s on average), and about 1 minute for the scripted demo (401 steps, 20.1 s of simulated time, 68 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
