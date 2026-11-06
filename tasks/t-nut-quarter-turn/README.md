# Drop a T-nut into the profile slot, quarter-turn it, slide it to the mark

Fitting a drop-in T-nut into an aluminium T-slot profile, the way a maker-space technician or a machine builder
does when mounting a bracket, a guard or a sensor on a frame. The nut is narrower than the slot's opening only one
way: it goes in lengthwise, and a quarter turn inside the channel puts its ends under the slot's lips, where it
locks. Then it slides along the slot to where the part will be bolted, marked with red tape, and is let go there.

| File | What it is |
|---|---|
| `t_nut_quarter_turn.py` | The `yam_arena` task file: scene, scoring (`success`, `score`), the finger watch and the scripted `demo()` |
| `compound.py` | `Compound`: a task-file box that collides as several smaller boxes of one rigid body (the nut and its stud) |
| `fingers.py` | Where the grippers' fingers are, from the robot's collision meshes, so the scorer knows if the nut is still held |
| `bonds.py` | The spawn, reset and step hooks the other modules use (`on_spawn`, `on_reset`, `on_step`) |
| `simready.py` | Loads the NVIDIA SimReady props (pinned revision) and makes the marker's bodies one |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`profile.usdc`, `tnut.usdc`, `mat.usdc`), their textures, and `make_looks.py`, the Blender script that models and exports them |
| `test_t_nut_quarter_turn.py` | Offline checks: the file loads, the nut and the channel fit as the lock needs, every scoring outcome, the demo's turn, heights and reach (no GPU) |
| `task.json` | The task's details for the board |

## Assets

- **Profile, T-nut and cutting mat:** modelled in Blender by `looks/make_looks.py` and drawn on the task file's boxes.
  The T-nut's mesh fits inside its box (a test checks it). SimReady's T-slot extrusions
  (`extrusion_tSlot_aluminum_b06` to `_b09`) are 6.8 m rack frames, not a bench profile, so the profile is modelled.
- **Speed square and marker:** NVIDIA SimReady `measurement_ruler_speedsquare_a02` (scaled to 0.6) and
  `stationery_marker_alcohol_prussianBlue_a01` (its cap and body merged into one rigid body), dataset
  `nvidia/simready-assets` at revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` (CC-BY 4.0), downloaded on first use
  into `~/.cache/simready`. Both are fixed scenery.

## The lock

The profile collides as fixed boxes: a solid core under the channel (its floor), two channel walls, two lips and two
end caps. The T-nut is one rigid body (`compound.py`) of two boxes: the nut, 24 x 10 x 6 mm, and the stud, 8 mm square
and 40 mm tall (drawn round and threaded).

| Nut | Opening (16 mm) | Under the lips |
|---|---|---|
| lengthwise (10 mm across) | passes, 3 mm of play each side | slides out again straight up |
| crosswise (24 mm across) | doesn't pass: it rests on the lips | its ends sit 4 mm under each lip: it can't come up |

The nut's diagonal (26 mm) turns freely in the 30 mm channel, and the 9 mm channel leaves it 3 mm under the lips. The
end caps close the channel, so the nut can't be slid in from an end.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=t_nut_quarter_turn.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Drop the yellow T-nut into the slot on top of the aluminium profile, turn it a quarter turn so it locks under the lips, then slide it along the slot to the red mark and let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/t-nut-quarter-turn-dream

# Scripted demo (right arm), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=t_nut_quarter_turn.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Drop the yellow T-nut into the slot on top of the aluminium profile, turn it a quarter turn so it locks under the lips, then slide it along the slot to the red mark and let go." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/t-nut-quarter-turn-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The layout keeps the work in the right arm's half: the T-nut starts at (0.21, -0.14), it goes in at y = -0.04 and
  the mark is at y = -0.12. The left arm stays in its ready pose. The profile reaches into the left half, so either
  arm can do it.
- The demo: the right arm comes straight down with its fingers closing along y on the top 14 mm of the stud, lifts
  the T-nut, turns it a quarter turn while carrying it to the profile (the nut now lies along the slot), lowers it
  through the opening into the channel, turns it back a quarter turn there (crosswise: locked), slides it 8 cm along
  the slot to the mark, lets go and rises. It reaches 1.0 and ends on success at step 330 (16.5 s at 20 Hz). A trial
  takes about 2 minutes on an L40S, including startup.
- The same demo without the quarter turn in the channel (`TURN = 0`) leaves the nut lengthwise at the mark and scores
  0.25 (in the channel, not turned). Pulling straight up after the turn instead of sliding (`PULL = 1`) scores 0.5:
  the lips hold the nut and the arm can't lift it. The same pull without the turn (`PULL = 1`, `TURN = 0`) lifts the
  nut out of the slot and scores 0.
- Videos: `inspect-robots video ~/yam-eval/t-nut-quarter-turn-demo/<log>.json` writes one MP4 per camera next to the
  frames.
- Offline checks: `python -m pytest -q test_t_nut_quarter_turn.py`.

## Scoring

All positions are in metres, in the station frame (x forward, y to the robot's left, from the midpoint of the arm
bases), and heights are above the table top. The scorer reads the nut's centre (under the stud) from the T-nut's
centre and orientation.

- **In the channel:** the stud within 20 degrees of vertical, the nut's centre within 8 mm of the slot's line
  (x = 0.30), below the underside of the lips and between the end caps.
- **Locked:** in the channel, with the nut's 24 mm side within 10 degrees of perpendicular to the slot.
- **Let go:** no point of any finger's collision mesh within 3 mm of the T-nut's box (read every control step).
- **0** if the T-nut has fallen off the table (its centre more than 2 cm below the top) or the state is not finite.
- **1.0 (success, held 1 s):** locked, the nut's centre within 1 cm of the mark (y = -0.12) along the slot, let go,
  and the T-nut still (under 1 cm/s and 0.5 rad/s).
- **0.5:** locked, but 1 cm or more from the mark, or still held, or moving.
- **0.25:** in the channel but not turned (lengthwise, or turned less than 80 degrees).
- **0** otherwise, including the T-nut standing on the mat, lying on top of the profile or across the opening.

In a real trial, a photo from above shows the nut's turn and a tape measure gives its distance from the mark. Lifting
the stud gently tells a locked nut (it stays) from one that is not turned (it comes out).

## Deviations from a real 2020 profile

The profile and the nut are about twice the size of a 2020 profile's 6 mm slot and its 10 x 6 x 4 mm T-nut: PhysX
contacts start a few millimetres out and a 6 mm slot leaves the gripper no play. The proportions are kept: the nut
passes the opening lengthwise only and locks under the lips crosswise.
