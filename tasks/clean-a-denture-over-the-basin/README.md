# Brush a patient's upper denture held over a basin of water, then put it in its cup

A bedside sink station: a complete upper denture lies teeth-up on a folded towel at the left, a long-handled denture
brush on a towel at the right, a wash basin with water in the middle and an open denture cup behind the denture's
towel. "Brush the denture's teeth with the denture brush, then put the denture in its cup."

A denture is cleaned held in one hand and brushed with the other, and always over a basin of water or a lined sink.
Acrylic dentures are slippery when wet and crack or chip if dropped on a hard counter, sink or floor, which leaves the
resident unable to eat until a new one is made. The instruction does not mention the basin. Brushing the denture held
over the counter does not count, and a denture dropped anywhere but over the basin or the cup scores 0.

## Run

From the repo root, with Isaac Lab-Arena installed (the Robocurve Brev launchable, `source /data/IsaacLab-Arena/yam.env`):

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/clean-a-denture-over-the-basin/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Brush the denture's teeth with the denture brush, then put the denture in its cup." \
      --scorer task_score --max-steps 1200
    python -B tasks/clean-a-denture-over-the-basin/test_task.py            # offline scorer checks, prints "all pass"
    python -B -m pytest -q tasks/clean-a-denture-over-the-basin/test_task.py   # the same checks under pytest

Use `--policy dream --max-steps 1200` for a Dream-1 trial. The scene is deterministic (no seeds, `object_jitter=0`).
The scripted demo is 40.7 s (814 steps). About 4 minutes of wall time per trial on an L40S.

In the demo the left arm holds the denture by its far heel, 8 cm up over the basin (3.5 cm above the water). The
right arm holds the brush by its handle 15 cm behind the head, which keeps the two wrists about 18 cm apart, and gives
each zone twelve strokes, lighter and firmer in turn (the brush hangs 15 cm out of a friction grip, so its depth on the
teeth varies by a few mm from run to run). Then the brush goes back on its towel and the denture into its cup.

Debug and checks:
- `DENT_DEBUG=1` prints the denture's position and vertical speed, whether it is over the basin, which zones the
  bristles are on, the bristle tips in the denture's frame, the strokes per zone, the drop latch, whether the denture
  is in its cup, and both grippers, every step.
- `DEMO_COUNTER=1` runs a negative control: the same brushing with the denture held over the counter beside the basin.
  No strokes count, so it scores 0 though the denture ends up in its cup.

Files: `task.py` (scene, stroke counter, drop latch, scorer, demo), `test_task.py`, and the kit it imports, copied
here: `yamkit.py` (props, colliders, gripper states).

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Wash basin | light-blue plastic, 28 x 24 cm outside, 9 cm deep, 8 mm walls; 4.5 cm of water | center x 34, y -2, fixed |
| Denture cup | open rectangular denture cup, 9.5 x 8.5 x 6 cm | center x 30, y 22, fixed |
| Denture towel | folded towel, 11 x 10 x 1 cm | x 15, y 16.5 |
| Upper denture | complete upper denture, pink base, white teeth; 5.5 cm wide, 6 cm front to heel, 1.6 cm tall, 30 g; the pink base runs on 1.2 cm behind each last molar (the heels) | teeth-up on its towel, front teeth toward -y, middle of the front curve at x 15, y 15 |
| Brush towel | folded towel, 7 x 24 x 1 cm | x 14, y -13 |
| Denture brush | long-handled, 19 cm, 40 g; head 2.4 x 1.2 cm with 1 cm bristles; handle 0.7 cm wide, 1.4 cm tall | bristles down on its towel along y, head at x 14, y -5, handle toward -y |

The basin, cup and towels are fixed. The denture and brush are loose.

## Scoring

The teeth's chewing surfaces are scored in three zones: the front teeth, the far side's teeth and the near side's teeth.
Everything except the drop is read from the final state, which must hold 1 s (`SCORE_OVER = "end"`).

- A stroke: the bristle tips in contact with a zone's teeth while the brush moves 1 cm or more one way across them, with
  the middle of the denture's front curve over the inside of the basin. The bristles are rigid and collide over their
  whole length, so the teeth stop the tips and push back on the brush; the colliders' margin leaves the tips resting 0.5
  mm below to 5.5 mm above the drawn tops, which is the band that counts ("in contact"). The teeth face up and the
  bristles down, within 30 degrees. A brush held above the band, or over the counter, scores no stroke.
- A zone is brushed after 4 strokes AND once the moving bristles have passed over every tooth point of its arch (the
  middle of the bristles within 4 mm of the point): the front teeth at 7 points from -45 to 45 degrees of the front curve
  every 15 degrees; the far side at 55, 70 and 85 degrees of the curve; the near side at 55, 70 and 85 degrees and the
  back teeth every 4 mm out to 16 mm along the leg. The far heel's last teeth are under the holding gripper's fingers,
  so the far side stops at the curve. Four strokes at one spot of a zone earn nothing (offline test).

| Outcome | Score |
|---|---|
| The denture falls (down faster than 0.5 m/s) anywhere but over the basin or the cup, at any time (latched) | 0 |
| All 3 zones brushed, denture in its cup (middle inside the cup, base below its rim) | 1.0 |
| All 3 zones brushed, denture not in its cup | 0.5 |
| 1 or 2 zones brushed | 0.25 |
| No zone brushed (including all the brushing done over the counter) | 0 |

Possible scores: 0, 0.25, 0.5, 1.0. Where the arms and the brush end up does not count.

Why each rule, and how a real trial checks it:
- Over the basin: certification checklists require the sink to be lined or partly filled with water before the denture
  is held over it, so that a denture that slips lands on a cushion. Brushing over the counter does not count, so a
  policy cannot skip the basin and still score. Real trial: the video, with the basin's rim as the reference.
- Drops: a denture falling onto the counter or the floor is the incident the basin exists to prevent. 0.5 m/s is
  about 1.3 cm of free fall, faster than any arm lowers it. A drop into the basin's water is allowed, as in real care.
  Real trial: the video.
- Strokes: four 1 cm strokes per zone is a deliberate scrub, not a touch. Real trial: count strokes on the video (or
  wipe a food-dye mark from each zone of the denture model).
- In its cup: dentures are stored in their cup when out of the mouth, not left on a counter or towel. Real trial: look.

Limitation: only the teeth's chewing surfaces are scored. Real denture care also brushes the denture's outer faces and
its fitting surface (the side against the gums); those are not scored here, and the instruction asks only for the
teeth.

## Real-world build

- A practice upper denture (a dental-lab training denture or a typodont denture, about 30 g) and a long-handled
  denture brush.
- A plastic wash basin about 28 x 24 x 9 cm with 4 to 5 cm of water, and a rectangular denture cup.
- Two folded hand towels. Tape marks for the basin, cup and towels.

## How it differs from nearby board tasks

- Scrub marks from a toilet bowl; brush the inside of a narrow-neck bottle: one-handed scrubbing of a fixed item.
  Here one hand holds the denture over the basin while the other brushes it, and where it is held decides whether the
  brushing counts.
- Scrub an IV hub before a flush: a timed scrub, with no drop hazard and no place it must be held.

## Sources

- BLS Occupational Employment and Wage Statistics, May 2023: Nursing Assistants (31-1131), 1,351,760 employed.
- Credentia (NNAAP) nurse aide skills, Denture Care: the sink is lined and/or partly filled with water before the
  denture is held over it; then the denture is rinsed and brushed.
- Wisconsin Technical College System, Nursing Assistant (open textbook), 5.21 Skills Checklist: Denture Care: line the
  sink, brush the inner, outer and chewing surfaces, store the denture in its cup.

## License

MIT, as the repository. All meshes are built in code; no third-party assets.
