# Slide a repair coupling across a pipe joint

A drain line of 1 1/2-inch PVC pipe is being repaired. A new length of pipe has its far end pushed home in a fixed
elbow; its near end rests on a block, sagging 1.5 cm below the end of the installed pipe it has to join. A slip
coupling (a repair sleeve with no stop inside) sits on the installed pipe. The task is to close the joint the way a
plumber does: lift the new pipe's loose end into line with the installed pipe, slide the coupling along the installed
pipe across the joint until it is centred, and let go, so that the coupling holds the new pipe in line.

The new pipe pivots in the elbow and cannot slide along its length (its near end already reaches the installed pipe),
so the coupling cannot be put on it first and pushed on end-first; while its loose end sags, the coupling's mouth
meets the pipe's end face instead of passing over it, and when the pipe is let go before the coupling is across, it
drops back onto the block. One hand has to hold the new pipe in line while the other slides the coupling. The task is
the task file [`slide_repair_coupling_across_pipe_joint.py`](slide_repair_coupling_across_pipe_joint.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres, upright. The pipes lie on their side and the
coupling and the elbow's socket are sleeves, so `bodykit.py` adds rigid bodies built from several shapes, some of them
fixed in place: importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives
`score()` each body's own pose.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run, which is
the best stage reached (0.75, 0.5, 0.25 or 0) unless a rule was broken: pulling the new pipe 2 cm or more out of the
elbow, or a pipe or the coupling falling onto the table, sets it to 0 for the rest of the run. `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=slide_repair_coupling_across_pipe_joint.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Join the new pipe to the installed pipe: lift the new pipe's loose end into line with the installed pipe, slide the coupling along the installed pipe until it is centred over the joint, then let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pipe-dream
```

The scripted demo, which reaches full score (the left gripper closes on the sagging new pipe 10 cm from its far end
and lifts it into line, turning it about the elbow's socket; the right gripper comes down behind the coupling with
its fingers astride the installed pipe and pushes the coupling's end along it, across the joint, until it is centred;
the right gripper backs off, the left lets go, and both rise clear):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=slide_repair_coupling_across_pipe_joint.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Join the new pipe to the installed pipe: lift the new pipe's loose end into line with the installed pipe, slide the coupling along the installed pipe until it is centred over the joint, then let go." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/pipe-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/pipe-dream/*.json; do inspect-robots video "$log"; done
```

`PIPE_DEBUG=1` prints, every 10 steps, how far the new pipe's near end is from the installed pipe's axis, how far each
pipe reaches into the coupling, where the coupling and both grippers are, and the stage reached so far.

`PIPE_ONE_ARM=1` runs the same demo with the left arm kept still: the right gripper pushes the coupling exactly as
before, its mouth stops against the end of the sagging new pipe (the new pipe reaches 0 mm into it), and the run
scores 0. Holding the pipe in line is what lets the coupling across.

## Runtime per trial

On one RTX PRO 6000 (Blackwell): about 4.5 minutes for a Dream-1 trial of 2,400 steps (268 s on the clock, with
Dream answering in 0.67 s on average), and about 1 minute for the scripted demo while rendering 1280 x 720 video
(315 steps, 16 s of simulated time; 69 s). Isaac Sim's first start on a new machine adds a few minutes while it
compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A length of 1 1/2-inch PVC drain pipe clamped to a post so that its end sits 10 cm above the table, a 25 cm length of
the same pipe, a 1 1/2-inch PVC elbow fixed on the table with its socket in line with the clamped pipe, 25 cm away, a
1 1/2-inch PVC slip (repair) coupling, and a block of pine 6 x 5 x 6.2 cm under the loose end of the new pipe.
