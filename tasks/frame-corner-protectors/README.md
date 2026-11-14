# Finish protecting a framed picture: push foam corner protectors onto its two bare corners

A framed picture lies face down on two carpet-covered blocks, ready to be packed for a move. The blocks run under its
two 30 cm sides, so all four corners stick out past them. Its two right-hand corners already have white foam corner
protectors on them. The other two protectors lie on the table beside the two bare left-hand corners, each with its
pocket facing its corner.

The instruction is: "Fit the two loose foam corner protectors onto the picture frame's two bare corners, then let go of
them."

Movers and packers fit corner protectors to framed pictures and mirrors before wrapping them, so that a knock on a
corner doesn't split the mitre or crack the glass. A foam protector is a block with an L-shaped pocket cut into two
adjacent sides. The pocket is as tall as the frame is thick, and its foam squeezes the frame. So the protector has to
be held level with the frame, lined up with its edge within a few millimetres, and pushed on until the corner reaches
the back of the pocket. The push moves the frame unless something stops it. Once on, the protector stays on by itself.

## Scene

Positions in cm from the midpoint between the arm bases (x forward, y left). The layout is fixed.

| Object | Description |
|---|---|
| Padded blocks (2, fixed) | Pine 2x4 offcuts covered in grey carpet: 16 long (x), 8.9 wide, 3.8 tall, centred at x 30, y ±18.5 |
| Picture frame (1.1 kg) | Face down on the blocks. Oak moulding 3 wide and 2.5 thick; 30 (x) by 40 (y), centred at x 30, y 0. Glass, mount and hardboard back fill the opening. Corners at x 15 and 45, y ±20, each sticking out 7 past the blocks' ends |
| Corner protectors (4, 20 g each) | White cross-linked polyethylene foam: a block 7 x 7 and 4.5 tall, with an L-shaped pocket in two adjacent sides, 4.5 deep each way and 2.65 tall, 1 cm of foam above and below. The open edges are rounded (5 mm radius). The pocket's floor stands 2 mm proud until pressed (about 1.25 N once on the frame). Two are already pushed fully onto the right-hand corners (y -20). The other two lie on the table beside the bare left-hand corners, pocket facing their corner, the back of the pocket at the corner's x and at y +30 |

In the sim the two fitted protectors are part of the frame's rigid body; the two loose ones each have a sprung floor.

## Scoring

The task file follows the trial after every control step:

- **Dropped:** the frame tilted more than 15 degrees at any time, or its front went 1.5 cm below the blocks' top (it
  slid off a block).
- **A bare corner counts** when a protector is fully on it and let go:
  - the corner within 4 mm of the back of the pocket in both directions. On a real rig: each closed wall's outer face
    2.5 cm past the frame's edge, within 4 mm, measured with a tape measure;
  - the pocket's floor under the frame's front and its roof over its back, with the foam squeezed onto the frame. In
    the sim: the frame's front within 3 mm of the floor's line and the sprung floor pressed down at least 0.3 mm. On a
    real rig: no gap a 1 mm feeler gauge, slid in at the pocket's open mouth, passes between the frame's front and
    the pocket floor;
  - the protector square and level with the frame (its rotation within 6 degrees of the frame's, turned for that
    corner);
  - let go: no part of either gripper within 1.5 cm of the protector (in the sim: each arm's TCP, the point
    between its finger pads, at least 4 cm from the protector's block; the finger surfaces reach about 2.4 cm from it).

`score()` gives 0 if the frame was dropped or doesn't end flat on both blocks (tilted more than 3 degrees). On a real
rig a trial also scores 0 if a right-hand protector was pulled off or moved more than 5 mm along either edge; in the sim they are part of the frame's
rigid body, so they can't come off.
Otherwise it gives 0.5 for each bare corner that counts. With both counting it gives 1.0 once the frame and both
protectors are still (each moving under 1 cm/s and turning under 0.5 rad/s; on a real rig, no visible movement for 1 s), and 0.75 before
that. `success()` ends
the episode once 1.0 has held for 1 s.

Either loose protector can go on either bare corner, as long as its pocket faces that corner.

## Run a trial

Following the task board's simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Fit the two loose foam corner protectors onto the picture frame's two bare corners, then let go of them." \
  --scorer task_score --no-prompt --max-steps 1800 --store-frames --log-dir ~/yam-eval/corner-protectors-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`), so no seed is needed. A full 1,800-step trial takes about 13 minutes of wall time with frames stored; the scripted demo ends on success at step 897 (about 7 minutes).

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`. The left arm fits the two loose protectors, far one
first, while the right arm holds the frame down, pressing on its back beside the middle. For each protector the left
arm:

1. pinches it across its closed wall (the fingers close along y on solid foam, clear of the pocket);
2. lifts it and brings it beside its corner at the frame's height, 5 cm out along y and 4 mm out along x so the closed
   wall clears the frame's end;
3. pushes it about 2 cm onto the corner along y, over the lip, where the foam's squeeze holds it;
4. eases off, lets go, moves its fingers off the protector and lifts clear of it;
5. closes its fingers, turns its wrist so they lie along x, and presses both fingers flat on the protector's closed
   outer face, aiming 1 cm past seated, so the back of the pocket stops it on the corner; then backs off and lifts.

Set `CORNERS_DEBUG=1` to print the frame and, for each loose protector, where its corner sits in the pocket and how far
the floor is pressed, every 5 steps. `CORNERS_SAG="dx,dy,dz"` shifts every fitting pose to make up for the loaded
arm's sag.

Shortcuts, with `DEMO_VARIANT` set in the environment. `no_hold` shows that the frame doesn't need holding: the
protectors slide on without shoving it off its blocks, so holding it is the demo's choice, not part of the task.

| `DEMO_VARIANT` | What the arms do | Score |
|---|---|---|
| `full` | as above | 1.0 (success at step 897) |
| `nothing` | wait | 0.0 |
| `near_only` | fit the near protector only | 0.5 |
| `keep_holding` | fit both, but keep the fingers pressed on the second to the end | 0.5 |
| `no_hold` | fit both with nobody holding the frame | 1.0 |

## Offline checks

`test_task.py` checks the geometry (the protector fits the open gripper, the squeeze, the bevels' height window, the
clearances to the blocks), the scoring rules on hand-built states, and the demo's reach with the robot's inverse
kinematics, without Isaac Sim:

```bash
PYTHONPATH=<path to yam-isaac-lab-arena> python -m pytest test_task.py -q
```

## Real-world build

Use a 30 x 40 cm oak picture frame (moulding 3 cm wide and 2.5 cm thick) with its glass, mount and back, and two pine
2x4 offcuts covered in carpet, 16 cm long, screwed or clamped to the table. Use four foam corner protectors made for
frames up to 1 inch (2.5 cm) thick: a 7 cm square block with a 4.5 cm deep pocket, which grips the frame when pushed
on (or cut them from 4.5 cm cross-linked polyethylene foam). Push two onto the right-hand corners before the trial and
mark where the other two lie. Score with a tape measure at the end. Fitting them is safe: the arms push a 20 g foam
block onto a frame lying on blocks.

## Licence

This task is released under the MIT licence. `partkit.py` comes from `tasks/din-rail-mount` in this repository, with
sphere colliders added; it is released under the same licence. No third-party models or assets are used. Every object
is built from primitive shapes.
