# Hold the side plates up with one hand and slide the dinner plate out from under them

Three glass side plates sit stacked on a dinner plate in a full kitchen cupboard. The bowls fill the floor on one side,
and the strip on the other side is too narrow for a side plate, so there is nowhere to set the small plates down
inside the cupboard. Taking them out to the counter first would mean two extra trips with loose glass, and it scores 0. One
arm lifts the three side plates a few centimetres and holds them level; the other arm slides the dinner plate out from
under them. The side plates go back down on the cupboard floor where they were, and the dinner plate goes on the
counter mat. One arm holds a load up while the other draws an object out from under it.

## Requirements

The official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), which provides
`inspect-robots` and the `yam_arena` embodiment. For Dream-1, a Dream API key in `~/.dreamlabs/credentials` and the
Dream-1 bridge on port 8211. Everything else is in this folder; `platedemo.py` holds the scripted demo's hooks.

## Run and score a trial

From this folder:

```bash
INSTR="Hold the three side plates up while you slide the dinner plate out from under them, then put the dinner plate on the counter mat. The side plates stay stacked in the cupboard where they are: do not set them down anywhere else, and do not drop them."
ARGS=(--embodiment yam_arena -E task=plate_under_stack.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360
      --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames)

# Dream-1
inspect-robots run "${ARGS[@]}" --policy dream -P server_url=http://127.0.0.1:8211 --seed 0 --log-dir ~/yam-eval/plates-dream

# The scripted demo, which reaches full score (seeds 0, 1, 2 shown)
inspect-robots run "${ARGS[@]}" --policy yam_scripted -P hold_s=3 --seed 0 --log-dir ~/yam-eval/plates-demo-s0

# The same demo made wrong on purpose (see Results): fixed, yank, drop, nomat
PLATE_FAIL=yank inspect-robots run "${ARGS[@]}" --policy yam_scripted -P hold_s=3 --seed 2 --log-dir ~/yam-eval/plates-yank
```

The run prints `task_score` at the end. The trial is scored at its last step; the zero-score conditions are checked
at every control step (20 times a second) and stay set once tripped. One trial takes 1,200 steps or fewer (the scripted demo stops
itself when both arms are back at the ready pose, after about 1,000 steps); about 4 minutes of wall time on one GPU.
With `PLATE_DEBUG=<file>` set, every few control steps add one line to that file: each plate's and bowl's position
and tilt, which zero-score condition (if any) has tripped, and the score so far; `<file>.plan` gets the planned and
measured tool poses. `dev/run.sh <name> <policy> <seed> <steps> [ENV=VAL ...]` wraps the command with these logs.

The scripted demo reads the plates' start poses from the simulator to plan its moves, and the dinner plate's pose
once it leans on the cabinet's edge. It moves the plates only through the grippers, by contact.

## Scoring

Tools: tape marks round each plate and bowl at the start, a ruler, a front photo for tilt, and the video.

| Check | Real trial | In the simulator (`report` in `plate_under_stack.py`) |
|---|---|---|
| Side plates never parked, carried off, separated, tipped or dropped (else 0) | at no time does a side plate's centre leave a 5 cm circle round its tape mark, come 2.5 cm off the plate under it, tip past 15 degrees (video), or fall 1.3 cm or more | checked every control step; a drop is the centre moving down faster than 0.5 m/s, or 12 mm within 0.1 s |
| Dinner plate drawn out (else 0); 0.35 | its centre gets 23.5 cm from the side plates' start centre (it is then fully clear of them) | the same distance, from the simulator's poses |
| Dinner plate on the mat, +0.25 | at the end: flat (under 5 degrees), resting on the counter, centre within 6 cm of the mat's centre, never dropped, no gripper within 3 cm | foot within 1 cm of the counter, tilt from the pose, gripper distance from the finger points |
| Side plates back, +0.25 | at the end: bottom plate resting on the cupboard floor, each within 4 cm of its mark and 1.5 cm of the one below, each under 5 degrees, no gripper within 3 cm | foot within 5 mm of the floor, poses from the simulator |
| Bowls undisturbed, +0.15 | each bowl moved less than 1 cm, upright (under 5 degrees), top bowl still nested (centres within 1 cm) | poses from the simulator |

Outcomes: side plates set down elsewhere, carried out, left riding on the dinner plate, tipped or dropped: 0. Dinner
plate never fully out: 0. Dinner plate out but left in the cupboard or off the mat: 0.75 at most. Side plates not put
back down: 0.75 at most. Bowls knocked: 0.85 at most.

## Setup

Exactly as in `task.json`. Every trial's seed moves the layout (`layout` in `plate_under_stack.py`): the bowls stand
left or right of the plates (the plates then move 13 cm the other way), and the plates, the bowls and the mat move
within the ranges in the setup text. The framework turns `--seed N` into its own episode seed. Each trial's draw is
written to the first line of the `PLATE_DEBUG` log.

| Seed | Episode seed | Bowls | Dinner plate centre (cm) | Side plates centre (cm) |
|---|---|---|---|---|
| 0 (setup image, video) | 1415158438 | left | (57.07, -5.24) | (57.36, -4.52) |
| 1 | 2566101560 | left | (58.45, -6.25) | (58.26, -6.06) |
| 2 | 377265627 | right | (58.59, 6.63) | (58.46, 5.67) |

## Results

| Run | Seed | What it does | Steps | Score |
|---|---|---|---|---|
| Dream-1 | 0 | the model policy, full run limit | 1,200 | 0.0 |
| Scripted demo (video) | 0 | hold up, slide out, lean, carry to the mat, set the side plates back | 971 | 1.0 |
| Scripted demo | 1 | the same | 971 | 1.0 |
| Scripted demo | 2 | the same, bowls on the right (roles mirror) | 972 | 1.0 |
| Fixed plan (`PLATE_FAIL=fixed`) | 3 | the seed-0 plan replayed blind | 1,065 | 0 (a side plate tipped 33 degrees) |
| Fixed plan (`PLATE_FAIL=fixed`) | 2 | the seed-0 plan replayed blind | 1,065 | 0 (dinner plate never fully out) |
| Nobody holds them (`PLATE_FAIL=yank`) | 2 | the dinner plate is pulled out with the side plates on it | 1,007 | 0 (side plates rode 32 cm out) |
| Let go up high (`PLATE_FAIL=drop`) | 1 | the side plates are released 4.5 cm up | 1,013 | 0 (they fell back onto the dinner plate; one tipped 16 degrees) |
| Not to the mat (`PLATE_FAIL=nomat`) | 1 | the dinner plate is left leaning on the cabinet's edge | 624 | 0.75 |

The scripted runs stop themselves when both arms are back at the ready pose. Runtime per trial: about 4 minutes of
wall time on one GPU (up to 10 minutes on a shared one).
