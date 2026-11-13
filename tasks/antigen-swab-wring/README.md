# Wring a strep-test swab in its tube: squeeze the tube on the head, turn the swab 5 times, draw it out still squeezed

The swab step of a CLIA-waived rapid strep A test, as a medical assistant runs it in a clinic. The insert (ACON Flowflex Plus Strep A, FDA CLIA waiver
CW250006) says: "Rotate swab 5 times while squeezing, keep swab in test tube for 1 minute, and remove swab while rotating and squeezing the tube before
testing extracted material." The squeeze wrings the sample out of the swab head into the buffer.

The count of five is given in the instruction and nothing is read or chosen; the difficulty is in the hands:

- **A squeeze on a part that turns.** One hand keeps the soft tube squeezed onto the swab head at the tube's bottom, hard enough to bear on it but not
  so hard that the swab cannot turn, while the other turns the swab inside the squeeze.
- **Turning a thin shaft.** A parallel gripper turns the 3 mm shaft a third of a turn at a time: grip, turn, let go, turn back, grip again. Between strokes
  the squeeze alone holds the swab, and each grip must not pull it out of the squeeze.
- **Out through the squeeze.** The swab is then drawn from the bottom up through the still-squeezed tube, turning, so the head is wrung as it leaves.
  Let the squeeze go part-way, and the wring does not count.

**The arms.** In the demo the right arm squeezes the tube and the left arm works the swab. The scoring asks for the result only: any division of the work
is allowed.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`).
- `mechkit.py` and `demokit.py`, in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without modifying them.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 4000 \
    --instruction "Do the strep test's swab step: put the swab head down to the bottom of the tube, squeeze the tube on the head and turn the swab 5 times, then take the swab out while squeezing and turning, drop it in the bin, let go of the tube and move both hands away."
```

The scene is the same for every seed.

## The demo

From this folder:

```bash
python demokit.py task.py OUT                                   # the full-score run, with video
python demokit.py task.py OUT_checks --mode open,yank,resqueeze,holdbin,shallow --fast    # the failure checks
```

The demo is a scripted reference, not a policy. It reads the swab's and the tube walls' poses from the simulator; all of them are in view of the cameras.
It never reads the scorer.

1. The left hand takes the swab by its handle, over the box's edge, lifts it and swings its head down, then lowers the head to the tube's bottom.
2. The right hand comes level from the front and squeezes the tube over the head, at the tube's bottom.
3. The left hand lets go, turns down above the swab and takes the shaft again 10.5 cm above the head's tip. It turns the swab in 21 strokes of 120
   degrees (grip, turn, let go, turn back), each time first centring its open fingers on the shaft where it stands.
4. It then draws the swab up 8 cm in one stroke while turning, the squeeze kept on until the head has left the tube. (The insert's 1-minute wait is
   left out.)
5. The right hand lets go of the tube; the left lifts the swab clear of the rim and drops it in the bin. Both hands back away.

It scored 1.0 in GPU physics in 88.5 s of simulated time (1,769 steps): 5.2 turns counted at the bottom while squeezed, then the squeezed draw
from the bottom to the rim. A CPU run without cameras takes under a minute.

Failure checks (`--mode`, CPU physics):

| mode | what the script does | score |
|---|---|---|
| `open` | turns the swab in the tube with the tube not squeezed | 0.0 |
| `yank` | turns the swab 5 times squeezed, then lets go of the squeeze, takes the swab and pulls it straight out | 0.6 |
| `resqueeze` | turns 5 times squeezed, lets the squeeze go, lifts the swab near the rim, squeezes again and draws it the last part, turning, without letting go of it | 0.6 |
| `holdbin` | does everything, but stands the swab in the bin still held by its top | 0.8 |
| `shallow` | lowers the head only 2 cm into the tube before squeezing; let go, the swab slides to the bottom, so its turns there count | 0.6 |
| `none` | does nothing | 0.0 |

## Scoring

`--scorer task_score` prints the total. The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold: `score()` is 1.0 only once the 1.0 state has
held 20 control steps (1 s) in a row, and `success()` is true from then on, so the two agree.

A turn counts while the tube is squeezed onto the swab head with the head's tip within 10 mm of the tube's bottom: both walls pressed in 2.3 to 3.4 mm
(they touch a 5 mm head at 2.5 mm; with no head between them they close fully, to 4.5 mm), the head at least 5 mm into the walls, and the swab within the
bore and upright within 15 degrees. Turns back count against the total.

- **0.12 per full turn**, **0.6** at 5 turns.
- **0.8:** after 5 turns, the head drawn from the bottom up and out of the tube with the squeeze on the whole way (the walls bearing on the head every
  step while it is in them), turning at least 0.15 turn as it rises.
- **1.0:** after that, the swab in the bin, every fingertip at least 5 cm from the tube's axis and from every point along the swab, held 1 s.
- **0:** if the swab falls off the table (latched).

Real trial: turns are counted from the blue tab near the handle's end on video; the squeeze is checked on a side camera, the tube walls touching the
head; the draw is filmed from the side.

## Real trials: the build

- **Test kit.** A CLIA-waived rapid strep A kit with its extraction tube, buffer and sterile throat swab (for example ACON Flowflex Plus Strep A). Measure
  the tube (modelled 75 mm long, 13 mm across, 10 mm bore) and the swab (modelled 15 cm long, a 3 mm shaft, a 5 x 20 mm flocked head). Fix a small blue
  tab to one side of the swab 3 cm from its end. Each trial uses a fresh tube with buffer and a fresh swab.
- **Squeeze check.** Before a trial, check on the bench that squeezing the tube's lower half with about 6 N brings its walls onto the swab head (the model's
  walls give 1 N per mm, up to 4.5 mm on each side).
- **Tube stand.** A steel foot 5 x 5 x 0.8 cm at (31.5, 8) cm with a 10 cm post behind the tube and a spring clip at 6.9 to 8.5 cm holding the tube by its
  upper half, upright at (28, 8) cm with its bottom 1.5 cm above the table.
- **Kit box and bin.** The kit's box (about 10 x 10 x 5 cm) centred at (20, 20) cm with the swab lying on it along y, its head tip at (20, 25) cm, its
  handle 5 cm over the edge. A waste bin 12 x 12 x 8 cm centred at (7, 20) cm.
- **Safety.** Unused swabs and buffer only; no patient samples. The buffer is a mild irritant: eye protection for anyone near, and wipe spills.

## The model behind it

The tube is one articulation fixed in its clip. Its frame is the tube's floor and two rigid side walls 3 mm thick, 1 mm from the head; its other two walls
are links on sprung slides (1 N/mm, 4.5 mm travel), from the floor to the rim, which a gripper squeezing the tube pushes in until they bear on the head.
Each wall moves as a whole, so where along the tube the hand squeezes is not modelled; the demo squeezes over the head.
The buffer and its extraction are not simulated: a turn counts while both walls bear on the head. The swab is a free rigid body of 8 g (a real swab is 1 to
2 g; the heavier body keeps contact steady). Friction: the wet walls 0.1, the swab 0.3 (assumed). The insert's 1-minute wait is left out. The stand is painted steel, the bin
polypropylene.

## Attribution

The task statement quoted in `task.json` is from the O*NET database (O*NET OnLine, task statements for 31-9092.00) by the U.S. Department of Labor,
Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence. O*NET is a trademark of USDOL/ETA. The insert's wording is quoted from
the FDA's public CLIA waiver record CW250006.

## Assets

Every mesh is generated by `task.py` and `mechkit.py`. There are no textures and no third-party assets.

## Licence

MIT
