# Connect a garden hose to its tap's quick-connect coupler

A garden tap on a standpipe post has a brass ball-lock quick-connect coupler on its spout, mouth toward the robot. A
green garden hose is clamped to the table off to the robot's right, its brass plug lying on the table. The
instruction is: **"Connect the hose to the tap: pull back the coupler's collar, push the hose's brass plug fully in,
let the collar spring forward to lock it, then let go of everything."**

Clicking a hose onto an outdoor tap is something most people have done. With a ball-lock coupler it takes both hands
working together:
- one hand holds the coupler's sprung collar back toward the tap (with the collar forward, the locking balls are held
  in and the plug's nose stops on them);
- the other picks the plug up by its hex, lines it up with the bore (the hose drags on it) and pushes it home: its
  pointed nose lifts the balls, which ride along the shaft and then drop into its groove;
- the collar is let go, so it springs forward over the balls and locks them in the groove;
- both hands let go. Let the collar go early and the balls ride on the shaft and hold it back: nothing locks.

## Credit

This task takes up the idea of PR #1868 ("Connect a garden hose to its tap with a quick-connect coupler and let it
lock", TK-GD5BJP / TK-LP9ERQ), which was not accepted and whose hold lapsed without a revision. It is our own
implementation (new assets, mechanism, scorer and demo) addressing both of that task's reviews:

| Review point | This version |
|---|---|
| The pawl could miss the groove; full credit did not require real locking or the 10 N pull | Three balls round a round bore; full credit needs the collar home with all three balls down in the groove; the real trial's 10 N pull is part of the score |
| The demo video did not show the operation clearly | The demo video is recorded at 1280 x 704 and shows the right wrist camera's close-up of the plug going into the collar beside the top view |
| (Not raised, but simplified) square bore, coupler 3 cm above the table, 20 cm stiff hose | Round chamfered bore on a tap 11 cm up on a standpipe post; a 72 cm soft hose clamped well away |

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the coupler and hose, the monitor (the ball lock, the plug's depth, letting go), `success()` and `score()` |
| `geometry.py` | Dimensions, positions and the mechanism's numbers |
| `build_assets.py` | Writes `assets/coupler.usda` and `assets/hose.usda` (needs `pxr`) |
| `assets/` | The coupler (one articulation, fixed) and the hose with its plug (one articulation, fixed at its clamp) |
| `scenekit.py` | USD bodies and articulations with per-joint drives for yam_arena task files, from `child-resistant-cap`, with one addition (`fix_root`, for articulations fixed where they are spawned) |
| `demokit.py` | Helpers for the scripted demo (unchanged from `child-resistant-cap`) |
| `demo.py` | The scripted full-score demo, and a variant that skips the collar and scores 0 |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. Tested with yam-isaac-lab-arena at `bd62771`
(Isaac Lab-Arena 0.3.0, Isaac Sim 6.1, Python 3.12).

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Connect the hose to the tap: pull back the coupler's collar, push the hose's brass plug fully in, let the collar spring forward to lock it, then let go of everything." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/hose-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- **Seeds:** one fixed layout and no randomisation (`object_jitter=0`); the seed has no effect.
- **Runtime:** the scripted demo succeeds after 587 control steps (29 s of simulation); Isaac Sim takes about a
  minute to start.

## Scripted demo

```bash
python build_assets.py                            # once: writes assets/
python demo.py task.py demo_out --hires           # full score: demo_*_cam.mp4 (1280 x 704), setup.png, result.json
python demo.py task.py nocollar_out --no-collar   # the same pushes without pulling the collar back: 0
```

1. **Pick up the plug.** The right arm takes the plug from above by its hex, near the hose end, and lifts it.
2. **Turn it into line.** Where it was picked up, clear of the tap, it turns the plug's axis onto the bore's direction,
   measuring the plug each time (the wrist lags its command under the hose's drag).
3. **Bring it to the bore.** It carries the plug onto the bore's axis 8 cm out, then slides it along the axis to 5 cm
   out, measuring where the plug sits in the hand at each stage.
4. **Hold the collar back.** The left arm comes in from the robot's left, grips the collar at its tap end (fingers
   above and below) and draws it back onto its 9 mm stop.
5. **Push home.** The right arm pushes the plug along the bore 2 mm a step, steering on the measured plug, until its
   flange reaches the mouth: the nose lifts the balls and they drop into the groove.
6. **Lock and let go.** The left hand lets the collar go: it springs forward and holds the balls down. Then the right
   hand lets go and both move clear.

The demo succeeds (1.0) in 587 steps. The demo reads the plug's and the coupler's poses from the simulator to steer
(as a person watches the plug and the bore); the arms are moved only by joint commands through the standard 14-value
interface. The `--no-collar` variant makes the same pushes without the left arm: the balls stop the plug's nose 12 mm
into the bore, nothing locks, and it scores 0.

## Scoring

Measured along the bore. *Flange*: how far the plug's flange face is from the mouth. *Locked*: flange within 3 mm of
the mouth with the plug's axis within 10 degrees of the bore's, the collar back within 1.5 mm of rest and all three
balls down (within 0.8 mm of their seated position, which they can only reach in the groove). *Let go*: every finger
pad of both grippers (each pad's tip and a point 3 cm back along it) at least 3 cm from the coupler and collar (a 40 mm
rod from the collar's front to the tap) and from the plug (a 24 mm rod from its boot to its nose). *Still*: the plug
moved less than 3 mm over the last second.

| Score | When |
|---|---|
| 1.0 (success) | Locked, let go and still, together for 1 s. This ends the trial |
| 0.75 | At the end, locked but still touched or held |
| 0.5 | At the end, the plug's nose at least 15 mm into the bore (past the balls, which takes the collar held back) on the bore's axis, but not locked |
| 0.25 | Its nose got 15 mm in at some point, but it is out at the end |
| 0 | Anything else, including pushing the plug in without holding the collar back |

- **Real trial:** read the flange against a steel rule along the coupler and the collar against a scribed 6 mm line
  in the top video; then hang a spring scale on the hose just behind the boot and pull 10 N along the bore for 5 s:
  for full credit the plug must move less than 1 mm and stay in (if it comes out, the score is at most 0.5).
- **In simulation,** the balls held down in the groove by the collar are the equivalent of the pull test: with the
  collar home the balls cannot rise, and the groove's walls bear on them.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left.

- **Tap and coupler.** A brass garden tap on a 5 x 5 cm post (or a tap mounted on a board), with a brass ball-lock
  quick-connect coupler (female, spring sleeve) screwed onto its spout, mouth toward the robot at (32, -4), 11 cm above
  the table, the spout turned 15 degrees toward the robot's right. Common 3/4-inch garden-hose brass quick-connect sets
  fit; check that the sleeve must be held back for the plug to go in.
- **Hose.** About 75 cm of 1/2-inch green garden hose, clamped to the table at (-4, -48) and laid in a curve round the
  right arm's base, with the set's brass male plug fitted to its end, lying on the table with its flange at about
  (21, 0), nose forward.
- **Arms** start at the ready pose with open grippers.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control.
- **Coupler:** fixed in place: a round bore (rings of 16 convex segments), 18 mm, behind a mouth chamfered from 26 mm
  in three steps, wet brass (friction 0.1). Three 4 mm steel balls, real colliders, each on its own radial slide with a
  light spring (40 N/m), greased (0.1), 8 mm in from the mouth; at rest they stand 3 mm into the bore and can rise
  3.5 mm. The collar is a slider on a 300 N/m spring preloaded to 1.2 N, 9 mm of travel, friction 0.9.
- **The collar's land and relief** are a rule acting through joint limits, applied after every control step: with
  the collar less than 6 mm back the balls cannot rise (the land holds them); while any ball is raised the collar
  cannot come forward past 6 mm (the ball holds it); with all balls down it springs home and holds them.
- **Hose and plug:** the hose is 16 rigid 45 mm links (12 g each) on alternating yaw and pitch hinges, with soft
  springs (0.001 N m/rad) and damping whose rest shape is the curve it lies in, fixed at its clamp. The plug is a rigid
  100 g brass part on the last hinge, friction 1.2: a 15.5 mm shaft with a pointed nose and an 11 mm groove, a flange
  and a hex the hand takes on two flats.
- Dimensions and forces are representative of common brass garden quick-connect sets, not measured from one product.

## Reset between trials

Pull the collar back and draw the plug out; lay the hose back in its curve with the plug at its mark.
