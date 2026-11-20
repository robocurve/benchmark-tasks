# Lower an infant car seat's carry handle: both release tabs held in while it swings back

Two YAM arms turn the carry handle of an empty Graco SnugRide Lite LX infant car seat (carrier) from upright back to
its rear position, leave it locked there, and move clear. As the maker's manual shows, each arm of the U-handle has a
release tab on its front edge where the arm meets its hub, pressed rearward to release that side; the handle turns
only while BOTH tabs, 44 cm apart, are pressed in. Let either tab go before the handle passes positions B and C and
the handle locks there: it stops partway.

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
about 250 of the 1,200 steps). With `CSH_DEBUG=<file>` set, every control step adds one line to that file: the
handle angle, how far each tab is pressed, the seat's pose, the grippers' clearance and the score so far; `<file>.plan` gets
the planned and measured tool poses. `dev/run.sh <name> <policy> <seed> <steps> [ENV=VAL ...]` wraps the command with
these logs.

## Scoring

Scored once, after the trial, with the robot stopped. Angles are the handle's turn back from upright, read on a
protractor sticker on the left hub marked at positions A to D.

| Part | Points | Condition |
|---|---|---|
| Angle | 0.5 x (angle - 5) / 95 | linear from 5 to 100 degrees; the full 0.5 within 3 degrees of D (100) |
| Locked | 0.3 | at D, and the handle passes the manual's push-and-pull check: a luggage scale hooked on the middle of the top bar pulls 20 N toward the foot end; the handle turns less than 3 degrees and stays at D |
| Seat still | 0.2 | also locked, and the seat is within 30 mm of its taped outline at every corner, turned under 5 degrees, both runners flat |
| Hands off | score 0 | a gripper ends less than 10 mm from the handle or a tab (it must have let go) |
| Lock check | score 0 | after scoring, press both tabs, turn the handle back to A and let go: if it does not lock at A and hold the same 20 N pull, or a tab is cracked or loose, the trial forced or broke a lock |

Nothing has to be read while the grippers cover the tabs: every check is made on the stopped scene.
Examples: locked at D, seat still 1.0; locked at D, seat dragged 4 cm 0.8; let go early and locked at C 0.32, at B
0.16; never turned 0; hands left on the tabs 0.

The simulator scores the same from poses. Locked: the handle within 3 degrees of D with both tabs out (each pressed in
under 1 mm of its 6 mm), so both teeth sit in their D notches; a handle held there only by a pressed tab would fail
the pull. The simulated ring cannot break, so the lock check always passes there; pushing the handle arm instead of
the tab does not open the simulated lock (the no-press run below).

## Setup

See `task.json` (`setup`). The seat's start is drawn per seed: hub centre moved up to 40 mm along x and 6 mm along y,
turned up to 6 degrees, redrawn until one tab is at least 25 mm off its nominal place along x. The sideways shift is
small because the seat's hubs pass close to the grippers at the arms' ready pose, where every trial starts.

## How the scripted demo works

The demo reads the seat's pose from the simulator once, after the start (`handledemo.py`, `plan_stage` in the task
file); it moves the handle only through the grippers, by contact. Each hand closes its fingers, comes in level from
the robot's side and puts its fingertips on its tab's front face, then pushes 7.5 mm rearward: the tab goes in 6 mm
and the tooth lifts clear. The same push then carries the handle round, the hands following the tabs on a 4 cm arc
round the hub axis. Once unlocked the handle tends to run ahead under its own weight and catches at B or C; the hands
catch up and press the tabs again. At 72 degrees, past C, they let go and back off; the handle drops back to D, where
both teeth fall into their notches. A bounded integral correction from the arms' own joint readings takes up the
arms' sag. A two-finger squeeze across the arm, the way a hand works the tab, is out of the arms' reach at this
height (checked on the arm model), so the demo pushes the tab in its real press direction instead.

## Results

All runs on the final code (logs in `runs/`, no frames), 1,200-step limit, `task_score` as printed.

| Run | Seed | Score | Steps | What happened |
|---|---|---|---|---|
| Dream-1 | 0 | 0.0 | 1,200 | the model policy for the full run limit |
| Scripted demo (video) | 0 | 1.0 | 252 | both tabs pushed in, handle carried to 72 degrees (catching at B and C on the way), dropped to D, both tabs out, seat moved under 1 mm, hands 48 mm clear |
| Scripted demo | 1 | 1.0 | 252 | same |
| Scripted demo | 2 | 1.0 | 252 | same |
| Fixed plan (`CSH_FAIL=fixed`): seed 0's plan made for the reference layout | 0 | 0 | 387 | on the shifted seat the fingertips missed the tabs and pushed the arms: both locks held, the handle stayed at 2 degrees and the seat was shoved 3 cm |
| Let go early (`CSH_FAIL=early`) | 0 | 0.327 | 354 | let go at 45 degrees: the handle dropped back and locked at C, 0.5 x (67.1 - 5) / 95 |
| No press (`CSH_FAIL=nopress`): closed hands push the arms 6 cm above the tabs | 0 | 0 | 320 | both locks held (handle 2 degrees, tabs untouched); the seat slid 6 cm |
| Keep holding (`CSH_FAIL=holding`): hands stay where they pressed the tabs | 0 | 0 | 387 | the handle dropped to D and locked, but a gripper ended 1 mm from the handle |

## Fidelity

- Carrier: Graco SnugRide Lite LX (sold as SnugRide 35 Lite LX in the manual used), rigid shell (tub, two runners) and
  rigid U-handle, about 70 x 44 cm, 3.3 kg in all as published (7.2 lb).
- From the owner's manual (NWL0000993719A, sections 2-A and 2-D), read off its drawings, not measured: each tab on the
  arm's front edge where the arm meets its round hub, centre 3 cm from the hub axis, pressed rearward; hub 5 cm across;
  four locked positions, A to D at about 0, 35, 65 and 100 degrees.
- Not maker data (the parts are inside the hub): each tab is one arm of a sprung lever (20 g) whose other arm carries a
  5 mm tooth; the tooth sits 4 mm deep in an 8 mm notch of a ring on the shell's hub boss; a 6 mm press lifts it 2 mm
  clear. Tab spring 1 N at rest, 4 N pressed. Locking and unlocking are plain contact; no rule.
- Handle hinge: stops at -5 and 100.8 degrees (the shell rim in front, the head end behind), viscous damping
  0.004 N m s per degree for dry friction. The handle does not collide with the shell (the stop stands in for its rest
  on the head end). The fabric pad, harness and canopy are not modelled.

## Files

- `car_seat_handle.py`: the scene, the scorer, the seeded start and the demo's plans.
- `handledemo.py`: the demo's planning hook and arm-sag correction for `yam_scripted`.
- `dev/run.sh`: one trial with logs. `runs/`: the logs behind the Results table (no frames).
