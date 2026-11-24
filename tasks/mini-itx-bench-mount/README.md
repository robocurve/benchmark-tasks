# Seat a Mini-ITX server board on four locating standoffs

Robots that build and service data-center hardware would speed up AI compute build-out. Mini-ITX server boards such
as Supermicro's X11SDV-4C-TLN2F (Xeon D, 170 x 170 mm) are the motherboards of compact edge servers like the
SuperServer E300-9D. Integrators and field technicians seat a new or replacement board in the chassis on its four
standoffs, with its rear I/O through the chassis opening, before screwing it down. This task is that placement: the
robot lifts a full-size 170 mm Mini-ITX board by two protective edge clamps, lowers it flat so all four locating pins
enter their holes together, seats it on the standoffs with its rear I/O housing in the opening, and lets go.

The four C/F/H/J hole positions and the 3.96 mm round hole diameter come from
[the Mini-ITX addendum, Figure 3](https://content.instructables.com/ORIG/FJD/TETT/GU59XBKK/FJDTETTGU59XBKK.pdf); every
Mini-ITX board, the X11SDV-4C-TLN2F included, uses this pattern. The scene's +X edge is the rear I/O edge: C and F are
the rear-side holes, F recessed 22.86 mm farther into the board than C. The 2.4 mm round locating pins leave 0.78 mm
of radial clearance. The pins and the PCB collide (each round hole is approximated by thin collision strips), so a
board set down off the pins rests on them and is not seated.

## Real trial

- Board: a Supermicro X11SDV-4C-TLN2F, or an unpowered retired unit of it. Handle it with an ESD wrist strap and
  nitrile gloves; pad the grippers with ESD-safe foam; grip only the two protective polymer edge clamps.
- Fixture: as in `task.json` setup (ESD mat, 12 mm aluminium plate, four brass standoffs with 2.4 mm locating pins on
  the Mini-ITX pattern, the rear wall with its I/O opening).
- Checks at the end, all by eye or with a feeler gauge:
  - Pins engaged: from above, each pin's tip is seen inside its hole's outline.
  - Seated: a 1.0 mm feeler gauge does not enter between the board and the top of any of the four standoffs.
  - Square: tilt and yaw at most 3 degrees (a photo from above with a scale and a protractor overlay).
  - Rear I/O: the I/O housing's tip has passed the rear wall's inner face, centred in the opening within 30 mm (ruler).
  - Released: both grippers open and at least 60 mm from the board, and the board still for 1 s (video).

## Scoring (`scoring.py`, final state; `success()` must hold 1 s)

| Score | Condition |
|---|---|
| 0 | Board centre more than 20 mm from its seated position, tilt over 8 deg or yaw over 12 deg |
| 0.25 | Inside that zone, but a pin has not entered its hole, or a hole is more than 6 mm above its standoff |
| 0.5 | Every pin has entered its hole (each pin axis within 0.98 mm of its hole centre at both PCB faces, the 0.78 mm clearance plus 0.2 mm for the collision strips) and every hole within 6 mm of its standoff, but not seated: a standoff more than 1.0 mm below the board, tilt or yaw over 3 deg, or the I/O housing not through the opening |
| 0.75 | All four pins engaged and the board seated (every standoff within 1.0 mm of the board), square within 3 deg, I/O housing through the opening, but the board moving or a gripper closed or within 60 mm |
| 1.0 | As 0.75, held still for 1 s with both grippers open and clear |

## Results

Both runs are on this task.py (SHA-256 prefix 24da1ad9817a) with this scoring.py (24088191f7a7), seed 0,
`object_jitter=0`, 2,500-step limit; their logs are committed in `runs/`.

| Run | Steps | Wall time | task_score | End | Log |
|---|---|---|---|---|---|
| Scripted demo (`./run.sh demo`) | 299 | about 2 min | 1.0 | `success`: all four pins engaged, seated, I/O housing in the opening, released and still for 1 s | `runs/demo_log.json` |
| Dream-1 (`./run.sh dream`) | 2,500 | about 17 min | 0.0 | `max_steps` | `runs/dream_log.json` |

The submitted demo video shows the top camera enlarged beside both wrist cameras; the setup screenshot is the demo's
first top-camera frame. Runs of earlier revisions do not validate this one.

## Claimed revision

Review TK-WMXND9 (not accepted) on pull request #2029, whose hold lapsed without a revision:

| Reviewer point | What changed |
|---|---|
| The team-assembler total does not establish the workers who regularly install these boards; ground the task in Impact on AI timelines with a real data-center server board and setting | Category is now Impact on AI timelines. The board is the Supermicro X11SDV-4C-TLN2F Mini-ITX server board, the motherboard of the SuperServer E300-9D edge server, seated by integrators and field technicians before it is screwed down. The setup names the board and its ESD handling (also the reviewer's note on protective handling). The simulated geometry is unchanged: it already uses the Mini-ITX hole pattern and size that board has. |
| The 0.77 mm hole-to-pin limit at both faces, 0.01 mm inside the clearance, cannot be checked with simple tools; use observable checks for all four pins engaged and the board seated, with practical checks for the partial tiers | `scoring.py` now scores "pins engaged" (each pin's tip seen inside its hole; in the simulation the pin axis inside the 0.78 mm clearance plus 0.2 mm for the collision strips) and "seated" (a 1.0 mm feeler gauge does not enter under any standoff). Each tier lists its real check (table above). The tests cover the engaged limit at 0.97 vs 0.99 mm, the seated limit at 0.9 vs 1.1 mm, a pin outside its hole and a diagonal offset. The demo was rerun under the new scorer (Results). |

## Run and score

Up to 2,500 control steps at 20 Hz. With the YAM Isaac Lab Arena environment active, from this folder:

```bash
./run.sh dream
OUT=~/yam-eval/mini-itx-bench-mount/demo ./run.sh demo
```

Scorer tests and the sampled CPU collision and reach check:

```bash
export YAM_ARENA=/path/to/yam-isaac-lab-arena
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$YAM_ARENA:$PWD" python3 test_scoring.py
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$YAM_ARENA:$PWD" python3 check_local.py \
  --yam-arena "$YAM_ARENA" --modes demo settle no_board --report local_checks.json
```

Reset: put the board back flat on the ESD mat at its start pose and check that every hole and locating pin is clear.

## Origin

This task builds on robocurve/benchmark-tasks pull request #2029, whose hold lapsed without a revision; its scene,
demo and scorer are used with the changes listed under "Claimed revision".
