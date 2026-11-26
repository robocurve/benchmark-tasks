# Refit a popped-out sprung bed slat

Sprung (bowed) beech slats sit in plastic holders on a bed's side rails, each end cap tucked under a small lip. A slat
pops out when a child jumps on the bed or the frame flexes, and refitting it is a two-handed job: push one end into
its holder, then bow the slat up so it is short enough to drop the second cap inside the other holder's lip, and let
it spring back to length so both caps go home under their lips. A slat left resting on top of the holders pops out
again under the mattress; one bent too far cracks.

Here a replacement sprung beech slat (53 x 8 mm, 755 mm over its plastic end caps, 20 mm rest bow, 0.25 kg; the size
sold for small single and toddler beds) lies crown up on the bench in front of a bed-frame section. The frame's two
pine side rails carry three pairs of holders; the outer pairs hold fitted slats and the middle pair is empty. At rest
the slat is 0.3 mm longer than the holders' wall-to-wall span, so it cannot be laid in flat: the second cap only
drops inside its lip with the slat bowed about 7 mm shorter (about 50 mm of bow, an estimated 20 N of lift near a
third of its length). Bowing it past about 75 mm, or kinking it, cracks it.

## Rubric (exactly what `score()` checks)

- **0** at any point (gate): the slat falls off the bench (mid-span more than 5 cm below the bench top); the slat is over-bent (mid-span more than 75 mm above the
  straight line between its two end-face centres, or any of its hinges bent more than 0.20 rad past its rest angle);
  a neighbouring slat moves more than 10 mm; the bed frame moves more than 20 mm.
- Otherwise, at the end, judged in the frame's own coordinates and only with the two caps at opposite holders of the
  empty pair. A cap is **in** its holder when its end-face centre lies within the 64 mm pocket width and within
  27.5 mm of the end wall (at most 1 mm past it), with the whole end face below the lip's underside and no more than
  2 mm below the floor. It is **seated** when, in addition, its end-face centre is within 2 mm of the end wall (under
  the 4 mm lip) and its outer bottom edge is within 3 mm of the floor.
- **1.0** (success): 0.75 has held for 1 s (20 steps), both grippers' centre points are at least 5 cm from the slat
  (the two straight lines from its end-face centres to mid-span) and the slat moved less than 2 mm over that second.
- **0.75**: both caps seated and the slat crown up, mid-span 10 to 40 mm above the line between its end-face centres.
- **0.5**: one cap seated and the other in its holder (a seated cap also counts as in).
- **0.25**: one cap seated.
- **0** otherwise, including doing nothing and laying the slat flat on top of the holders.

## Files

- `task.py`: scene, seeded start, scoring (yam_arena task file). `score()` and `success()` are the rubric above and
  in `task.json`. The slat is five rigid beech segments on four sprung hinges (a serial articulation); the frame and
  the two fitted slats are loose rigid parts. Caps seat by contact only; nothing latches or snaps them in.
- `robokit.py`: rigid parts built from boxes with sprung joints (here extended with serial chains). No change to
  yam_arena is needed.
- `demo.py`, `script.txt`: the scripted demo (below).
- `script_noop.txt`, `script_random.txt`, `script_reach.txt`: baselines (do nothing, random joint targets, random
  reaches and grasps); `BASELINE_SEED` seeds them. `script_naive.txt`: the demo's grasp and carry, then the slat laid
  flat on top of the empty holders and let go.
- `run_dream.sh`, `mkdream.py`: the Dream-1 run (one episode, seed 0) and its videos.
- `media/checks.md`: demo and baseline runs, real-part sources, the observable-inputs check and the overlap check.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

The score is `task_score` in the log: 1.0 on success (held 1 s), otherwise `score()` from `task.py` at the end.

Scripted demo (one MP4 per camera, `setup.png` and `result.json` per seed; run one seed per process):

    DEMO_CAM_W=640 DEMO_CAM_H=352 python demo.py task.py demo_out --seeds 0 --max-steps 2400

The demo uses only what a policy has: top-camera images, the arms' joint angles (forward kinematics, including both
finger joints) and the frame layout and slat dimensions in the setup text. It finds the slat on the bench from its two
grey end caps in the top image, grips it with both hands (left 13 cm left of mid-span, right 10 cm in from the
right-hand cap), carries it over the empty holders, lowers the left-hand cap into its holder inside the lip and
pushes the slat end-wise until that cap is home. It then lifts the slat 30 mm at the left hand while drawing the
right-hand end 9 mm in, so the slat bows and shortens until the right-hand cap clears the lip, lowers that cap onto
the holder floor, lets the slat spring back out while holding the cap down, and lets go. Each hand is re-aimed from
its measured joints (the arms sag under the slat's spring), and the finger joints show where the slat sits between
the jaws, so the hands keep the slat on the holders' centre line. It never reads simulator poses or the slat's
joints.
