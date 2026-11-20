# Lower an infant car seat's carry handle: both hub buttons held in while it swings back

Two YAM arms turn the carry handle of an empty infant car seat (carrier) from upright back to its rear position, leave
it locked there, and move clear. The handle turns only while BOTH release buttons, one on each arm of the U-handle,
44 cm apart, are pressed in. Each gripper squeezes one button against its handle arm and both hands then travel the
same arc together. Let either button go before the handle passes the 35 and 65 degree holes and the lock pin drops
into that hole: the handle stops partway.

## Requirements

The official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), which provides
`inspect-robots` and the `yam_arena` embodiment. For Dream-1, a Dream API key in `~/.dreamlabs/credentials` and the
Dream-1 bridge on port 8211. Nothing else.

## Run and score a trial

From this folder:

```bash
INSTR="Lower the infant car seat's carry handle from its upright carry position back to its rear position over the head end, and leave it locked there. Do not move the seat. Then move both grippers clear."
ARGS=(--embodiment yam_arena -E task=car_seat_handle.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360
      --instruction "$INSTR" --scorer task_score --max-steps 1200)

# Dream-1
inspect-robots run "${ARGS[@]}" --store-frames --policy dream -P server_url=http://127.0.0.1:8211 --seed 0 --log-dir ~/yam-eval/csh-dream

# The scripted demo, which reaches full score (seeds 0, 1, 2 shown)
inspect-robots run "${ARGS[@]}" --policy yam_scripted -P hold_s=2 --seed 0 --log-dir ~/yam-eval/csh-demo-s0

# The same demo made wrong on purpose (see Results): fixed, early, nopress, holding
CSH_FAIL=early inspect-robots run "${ARGS[@]}" --policy yam_scripted -P hold_s=2 --seed 0 --log-dir ~/yam-eval/csh-early
```

The run prints `task_score` at the end. A trial takes about 6 minutes of wall time on a shared GPU (the demo uses
about 380 of the 1,200 steps). With `CSH_DEBUG=<file>` set, every control step adds one line to that file: the
handle angle, each button's travel, the seat's pose, the grippers' clearance and the score so far; `<file>.plan` gets
the planned and measured tool poses. `dev/run.sh <name> <policy> <seed> <steps> [ENV=VAL ...]` wraps the command with
these logs.

## Scoring

Scored at the last step. Angles are the handle's turn back from upright, read on a protractor sticker on the hub.

| Part | Points | Condition |
|---|---|---|
| Angle | 0.5 x (angle - 5) / 80 | linear from 5 to 85 degrees; the full 0.5 within 3 degrees of the rear position (85) |
| Locked | 0.3 | at the rear position with both buttons fully out (each pressed in less than 1 mm of its 6 mm), so both pins are in their rear holes |
| Seat still | 0.2 | also locked, and the seat moved under 30 mm, turned under 5 degrees and tilts under 5 degrees from its start |
| Forced lock | score 0 | at any step the handle was more than 2.5 degrees off a lock position while a button was pressed in less than 2 mm (its pin still at least 2 mm deep in the hub cap): the handle was forced past the lock |
| Hands on | score 0 | a gripper ends less than 10 mm from the handle or a button (it must have let go) |

Real trial: a protractor sticker on the left hub, a ruler on the buttons (a pressed button sits flush with the arm;
an out button stands 6 mm proud), the seat's outline taped on the table, and a 30 fps video for the forced-lock check.

## Setup

See `task.json` (`setup`). The seat's start is drawn per seed: hub centre moved up to 40 mm along x and 6 mm along y,
turned up to 6 degrees, redrawn until one handle arm's grip point is at least 25 mm off its nominal place along x.
The sideways shift is small because the seat's hubs pass 15 mm from the grippers at the arms' ready pose, where every
trial starts.

## How the scripted demo works

The demo reads the seat's pose from the simulator once, after the start (`handledemo.py`, `plan_stage` in the task
file); it moves the handle only through the grippers, by contact. Each hand comes in level, from the robot's side,
with its fingers closing along the hub axis, and closes across the handle arm and its button (22 mm, closed to
16 mm): the squeeze pushes the button in 6 mm. Both hands then move together along a 10 cm radius arc round the hub
axis, the hand turning with the arm (pointing down at the end), from 0 to 85.5 degrees. At the rear stop they let go,
both pins drop into the rear holes, and the hands back off, move outward over the handle arms and return to the ready
pose. A bounded integral correction from the arms' own joint readings takes up the arms' sag.

## Results

All runs on the final code (logs in `runs/`, no frames), 1,200-step limit, `task_score` as printed.

| Run | Seed | Score | Steps | What happened |
|---|---|---|---|---|
| Dream-1 | 0 | 0.0 | 1,200 | the model policy for the full run limit |
| Scripted demo (video) | 0 | 1.0 | 342 | both buttons pressed 6 mm, handle 0 to 86 degrees, both pins back in, seat still, hands clear |
| Scripted demo | 1 | 1.0 | 342 | same |
| Scripted demo | 2 | 1.0 | 342 | same |
| Fixed plan (`CSH_FAIL=fixed`): seed 0's plan made for the reference layout | 0 | 0 | 491 | the right hand pressed its button only 4 of 6 mm; the handle was forced past a lock with that pin still in (score 0); it ended at the 35 degree hole and the seat was shoved 9 cm |
| Let go early (`CSH_FAIL=early`) | 0 | 0.382 | 430 | handle locked at the 65 degree hole: 0.5 x (66.1 - 5) / 80 |
| No press (`CSH_FAIL=nopress`): closed hands push the handle | 0 | 0 | 382 | pins hold; the handle stayed within 1 degree of upright and the seat slid 12 cm |
| Keep holding (`CSH_FAIL=holding`): hands stay on at the rear stop | 0 | 0 | 431 | handle at 86 degrees but both buttons still pressed and a gripper 0.5 mm from the handle |

## Fidelity

- Rigid plastic shell (tub, two runners), rigid U-handle, real size and mass (4.3 kg shell, 0.45 kg handle).
- Each button is one sprung part (cap, stem and lock pin; 40 g in the model) on a slide joint in its handle arm:
  1,500 N/m, 1 mm preload (1.5 N at rest, 10.5 N pressed 6 mm), 6.5 mm stop. The lock is plain contact between the
  pin (5 x 5 mm, 4 mm deep) and holes 7 mm wide in a fixed hub cap ring. No rule unlocks or locks the handle.
- Handle hinge: stops at -5 and 86 degrees (the shell rim in front, the head end behind), viscous damping
  0.004 N m s per degree instead of the real dry friction. The handle does not collide with the shell (the stop
  stands in for the rest on the head end).
- Not maker data: the button's place (here on each arm's outer face, 10 cm up from the hub, where a parallel gripper can
  squeeze it against the arm; the maker's manual says only "press the handle adjustment buttons on both sides of
  handle"), the lock pin and hole sizes, the 35 and 65 degree hole
  angles. The fabric seat pad, harness and canopy are not modelled.

## Files

- `car_seat_handle.py`: the scene, the scorer, the seeded start and the demo's plans.
- `handledemo.py`: the demo's planning hook and arm-sag correction for `yam_scripted`.
- `dev/run.sh`: one trial with logs. `runs/`: the logs behind the Results table (no frames).
