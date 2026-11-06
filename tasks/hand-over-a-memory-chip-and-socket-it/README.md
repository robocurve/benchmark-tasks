# Hand a memory chip from one hand to the other to turn it pins down, then seat it in its socket

A memory chip lies upside down, pins up, on a pink anti-static foam block. One hand picks it up and hands it over in
the air to the other, which takes it so that, turned over, it holds the chip pins down; that hand seats it in the
socket on a fixed circuit board, its pin-1 notch at the socket's marked end, and lets go. The socket's two slots have
sloped mouths, so pins brought within a few millimetres slide in. The task is one task file,
[`hand_over_a_memory_chip_and_socket_it.py`](hand_over_a_memory_chip_and_socket_it.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

Why two hands: a hand that takes the chip as it lies, from above, holds it with its pins pointing back at the hand.
Turned pins down in that hand, the chip has the hand underneath it, where it cannot go into the socket (each arm can
point its gripper up; that does not help). To seat it, a hand has to hold it from above, pins down, so the chip has to
be turned over while out of that hand: in the other hand. One hand alone could only tip it over on the table first
(stand it on edge against something and push it over) and take it again, a slow and chancy way round; the score
judges only where the chip ends up, so such a way round is not ruled out, but it earns nothing until the chip is seated.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The chip (package and 16 pins) and the board (with
the socket's ridge, rails, end walls and sloped mouths, the printing and a few other parts) are rigid bodies made of
several shapes from `bodykit.py`; the board is a kinematic body, fixed where it is put. Importing `bodykit.py` hooks
`yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()` each body's own pose, and lets
the task file look at the simulation after every control step, where it notes whether the chip has been turned over
in the air and where the grippers are.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the chip still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hand_over_a_memory_chip_and_socket_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pick up the memory chip, hand it to your other hand so that its pins point down, and seat it in the socket on the circuit board with its notch at the socket's marked end, then let go." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/chip-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the chip and grips it by its two
ends, lifts it and turns its gripper to point at the right hand, so that the chip stands on edge, pins toward the left.
The right hand slides in level from its side and grips the chip's long edges, fingers closing up and down; the left
lets go and backs away. The right hand turns its gripper to point straight down, which turns the chip pins down,
carries it over the socket, lowers it in, lets go and lifts away:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hand_over_a_memory_chip_and_socket_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pick up the memory chip, hand it to your other hand so that its pins point down, and seat it in the socket on the circuit board with its notch at the socket's marked end, then let go." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/chip-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/chip-dream/*.json; do inspect-robots video "$log"; done
```

Set `CHIP_DEBUG=1` to print what the score is built from (the chip's position against the seated one, how far its
pins are from pointing down, its turn about the vertical, whether it has been turned over in the air, and where each
gripper's TCP is from it) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (135 s on the clock, with Dream answering in 0.69 s on average), and about 1 minute for the scripted demo (428 steps, 21.4 s of simulated time, 60 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.
