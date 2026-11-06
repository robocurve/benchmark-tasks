# Index a workpiece one of 7 equal divisions on a dividing head

Before cutting gear teeth, splines or a bolt-hole pattern, a machinist turns the workpiece through exact equal
fractions of a turn on a dividing head. The head's 40-tooth worm wheel turns the spindle 1/40 of a turn for each turn
of the index crank. For N equal divisions the crank turns 40/N times. When that is not a whole number, the fraction is
counted in holes on a hole circle of the index plate whose hole count divides it exactly. From US Army Ordnance
subcourse OD1644, *Milling Machine Operations*
([page 28](https://armyordnance.tpub.com/od16448/od164480028.htm), [29](https://armyordnance.tpub.com/od16448/od164480029.htm),
[31](https://armyordnance.tpub.com/od16448/od164480031.htm)):

> "Since there are 40 teeth in the worm wheel, one turn of the index crank causes the worm wheel, and consequently the
> index head spindle to, make one-fortieth of a turn" ... turns per division = "40 by the number of equal divisions
> desired" ... the sector "indicates the next hole in which the pin is to be inserted and makes it unnecessary to count
> the holes".

Done by: Machinists (O*NET 51-4041.00).

Here the job is one division of 7: 40/7 = 5 5/7 = **5 turns and 15 holes on the 21-hole circle**. The plate also has a
20-hole circle, and the plunger starts there. So the robot must pull the spring plunger, move it to the right circle,
count 5 whole turns and 15 holes while holding the pin out, then let the pin drop into that hole and let go. A wrong
hole or wrong circle leaves the divisions unequal, and the part is scrap.

The task is a task-local plugin for the stock `yam_arena` adapter (Isaac Lab-Arena 0.3.1, Isaac Lab 3.0, Isaac
Sim 6.1, the calibrated `station` scene). It registers:

- the embodiment `yam_dividing_head`: the stock robot, cameras and 14-D joint action and observation, unchanged, plus
  the head's bodies and joints, scored from the measured part poses;
- the policy `dividing_head_scripted`: a privileged scripted demonstration, plus the failure paths below.

## Setup
Coordinates: x forward from the line between the arm bases, y left from their midpoint; heights above the table top.

| Part | Description | Position |
|---|---|---|
| housing | dark-blue steel box, 15 x 31 cm, 5 cm tall, fixed; hides the worm and wheel | x 20.5 to 35.5 cm, y -6.5 to +24.8 cm |
| clamp blocks | two dark steel blocks, 3 x 4 x 1.5 cm, fixed | x 19 and 37 cm, y 9 cm |
| index plate | grey steel disc, 14 cm across, 11 mm thick, fixed. Inner circle: 20 holes at 42 mm radius. Outer circle: 21 holes at 56 mm radius. Each hole is a 7 mm square pocket, 5 mm deep. Hole 0 of each circle is red; both lie on a red zero line | centre (28, 17) cm, face 6.1 cm high; zero line points forward (+x) |
| index crank | orange arm, 18 mm wide, 6.7-7.3 cm high, turning about the plate centre; 250 g; bearing drag 0.02 N m s/rad | starts on the zero line |
| plunger | rides in a slot along the arm between the two circles' radii; a light spring pulls it toward the inner circle (0.4 N there, 1.0 N at the outer). 6 mm steel pin, 4 mm deep in its hole, spring-loaded down (1.5 N seated, 2.6 N at 7 mm lift, 10 mm travel) | starts in hole 0 of the 20-hole circle |
| knob | yellow sleeve, 20 mm across, 36 mm tall, spinning freely on the pin | bottom 8 cm high |
| workpiece | aluminium disc, 100 mm across, 16 mm thick, on the spindle; red line from its centre to its rim | centre (28, 0) cm, top 7.8 cm high; the line points forward (+x) at a fixed green pointer |

- **Start:** both grippers open at the ready pose. One fixed layout; nothing is randomized.
- **The lock:** with the pin seated, the pocket's walls stop the crank (0.5 mm of slack each side). Lifted 5 mm or
  more, the pin clears the plate and the crank turns freely. Released over a hole, the spring drops it in.
- **The worm:** the workpiece follows the crank's rotation / 40, one way. Pushing the workpiece does nothing.

## Scoring
Measured at 20 Hz from the part poses and the robot's measured joints.

- **Rest state:** the pin down in a hole (its bottom at least 2.5 mm below the plate face, which only a hole allows),
  both grippers at least 80% open with their tool points at least 5 cm from the knob, and the crank under 1 degree/s.
- **On a circle:** the pin's centre within 1.5 mm of that circle's radius.
- **Spindle angle:** the workpiece's rotation from its start, either direction (no backlash is modelled).
- A tier is awarded, and kept (best so far), only after a rest state has held for 1 s.

| Score | Condition | How to check on a real table |
|---|---|---|
| 1.0, success, the trial ends | pin in the 21-hole circle; spindle within 0.2 degrees of 360/7 = 51.43 degrees (5 turns + 15 holes; one hole here is 0.43 degrees) | count the crank turns in the video (the orange arm passing the red zero line); photo: knob on the outer circle, 15 holes from the red hole 0, counted in the direction the crank turned; protractor on the workpiece line |
| 0.5 | pin in the 21-hole circle and 5 whole turns (spindle 44.8 to 53.8 degrees), wrong hole | the same count and photo |
| 0.25 | 5 whole turns (spindle 44.8 to 53.8 degrees), pin in either circle | the same count and photo |
| 0 | anything else: every state with fewer or more than 5 whole turns, including the workpiece unmoved | |
| 0, latched | the crank turned more than 3 degrees while the pin stayed in a hole (forced), or a non-finite state | |

- **Credit follows the procedure:** 0.25 needs the 5 whole turns done, and 0.5 adds the right circle. Moving the
  plunger to the 21-hole circle without turning earns nothing (the `plungeronly` run).
- **Why the circle is a condition:** 5 turns + 14 holes on the 20-hole circle is 5.70 turns, 51.30 degrees: nominally
  0.13 degrees short (0.12 in the run below), inside the 0.2-degree band. Seven such divisions add up to 0.9 degrees,
  and the last one is wrong. So it scores 0.25: the turns are right, the circle is wrong.
- **No credit for touching:** gripping, pulling or moving the knob earns nothing by itself. Each tier needs 5 whole
  turns, the pin back in a hole, the hands off and the crank still.
- **Unintended strategies:** the workpiece turns only through the crank. Pushing or dragging it scores 0 (the `twist`
  run). Turning the crank without pulling the pin is blocked by the pocket (the `force` run). Forcing it more than
  3 degrees with the pin seated latches 0. That rule is a guard: in the probe below, a 0.3 N m push moved the
  seated crank 1.08 degrees and no further.
- The scorer is pure Python, in the package's scoring module.

## Run a trial
Install the pinned adapter, which installs Isaac Lab-Arena and Inspect Robots, then this package into the same
environment. From the repository root:

```bash
mkdir -p .deps
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git .deps/yam-isaac-lab-arena
git -C .deps/yam-isaac-lab-arena checkout 0059d02a1a91bc6834c1d0846022891df0c78730
ARENA_DIR="$PWD/.deps/IsaacLab-Arena" .deps/yam-isaac-lab-arena/setup.sh
source .deps/IsaacLab-Arena/yam.env
uv pip install --no-deps -e tasks/dividing-head
```

On the Robocurve launchable the environment already exists: `source /data/IsaacLab-Arena/yam.env`, then
`uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e tasks/dividing-head`.

Run from `tasks/dividing-head`. Every run below uses the same settings; only `--policy` and `-P` change:

```bash
cd tasks/dividing-head
INSTR="$(python3 -c 'import json; print(json.load(open("task.json"))["instruction"])')"
COMMON=(--embodiment yam_dividing_head -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176
        --instruction "$INSTR" --scorer task_score --max-steps 4800 --store-frames)

# The scripted demonstration (full score)
inspect-robots run "${COMMON[@]}" --policy dividing_head_scripted --log-dir runs/demo

# Dream-1 (needs a Dream API key in ~/.dreamlabs/credentials)
inspect-robots run "${COMMON[@]}" --policy dream --log-dir runs/dream

# Failure paths
inspect-robots run "${COMMON[@]}" --policy dividing_head_scripted -P mode=wrongcircle --log-dir runs/wrongcircle
inspect-robots run "${COMMON[@]}" --policy dividing_head_scripted -P mode=wronghole --log-dir runs/wronghole
inspect-robots run "${COMMON[@]}" --policy dividing_head_scripted -P mode=plungeronly --log-dir runs/plungeronly
inspect-robots run "${COMMON[@]}" --policy dividing_head_scripted -P mode=twist --log-dir runs/twist
inspect-robots run "${COMMON[@]}" --policy dividing_head_scripted -P mode=force --log-dir runs/force

# Two episodes with a reset between them
inspect-robots run "${COMMON[@]}" --policy dividing_head_scripted --epochs 2 --log-dir runs/epochs2

# Lock probe (diagnostic, arms idle): a drive on the crank tests the seated lock, the free turn and the spring drop
inspect-robots run "${COMMON[@]}" --policy dividing_head_scripted -P mode=idle -E probe=lock --max-steps 500 \
  -E telemetry=runs/probe/telemetry.jsonl --log-dir runs/probe

# One MP4 per camera, next to each run's frames
for log in runs/*/*.json; do inspect-robots video "$log"; done
```

- The embodiment loads its own task file (`dividing_head_task.py`: the clamp blocks) and always uses a fixed layout.
- `-E telemetry=<file.jsonl>` writes a per-step trace: crank angle and turns, spindle angle (measured and commanded),
  plunger radius, circle and hole, pin lift, grippers, joint-anchor errors and score.
- The run prints `task_score` at the end: 1.0 on success, otherwise the best tier reached.
- Policies: `-P mode=wrongcircle` stays on the 20-hole circle and turns 5 turns + 14 holes; `-P mode=wronghole` moves
  to the 21-hole circle but stops one hole short (5 turns + 14 holes); `-P mode=plungeronly` moves the plunger to
  hole 0 of the 21-hole circle and lets go without turning the crank; `-P mode=twist` presses the right gripper on the
  workpiece and drags it 4 cm along its rim; `-P mode=force` grips the knob and tries to turn the crank 30 degrees
  without pulling the pin; `-P mode=idle` holds both arms still.

## Results
Measured 2026-11-05 23:01 to 2026-11-06 00:10 UTC, one L40S, the settings above (one episode each unless noted).
After the scoring change of 2026-11-06 (the 0.25 tier now needs 5 whole turns), the demo, `wronghole`,
`wrongcircle` and the new `plungeronly` run were repeated between 18:32 and 18:48 UTC. The first three gave the same
crank readings and scores as before; `plungeronly` is new. Dream-1 was not re-run: its instruction did not change, and
it never moved the plunger or the crank, so it scores 0 under both versions.

| Run | Score | Steps | What happened |
|---|---|---|---|
| scripted demo | 1.0 | 1,316 (65.8 s) | pin moved to the 21-hole circle; crank turned 5.7143 turns (goal 5 15/21); pin dropped into hole 15; spindle 51.441 degrees (target 51.429) |
| demo, `--epochs 2` | 1.0, 1.0 | 1,316, 1,314 | the same in both episodes; spindle 51.4415 degrees each time |
| `-P mode=wronghole` | 0.5 | 1,401 | 21-hole circle, 5 turns + 14 holes: hole 14, spindle 51.013 degrees |
| `-P mode=wrongcircle` | 0.25 | 1,423 | 20-hole circle, 5 turns + 14 holes: hole 14, spindle 51.308 degrees (0.12 short, inside 0.2, but the wrong circle) |
| `-P mode=plungeronly` | 0.0 | 345 | plunger moved to hole 0 of the 21-hole circle, pin seated, hands off; the crank never turned (spindle 0.001 degrees) |
| `-P mode=force` | 0.0 | 568 | knob gripped, crank pushed 30 degrees without pulling the pin: it moved at most 1.72 degrees, the pin stayed seated |
| `-P mode=twist` | 0.0 | 267 | right gripper pressed on the workpiece and dragged 4 cm along it: the spindle stayed at 0.000 degrees |
| Dream-1 | 0.0 | 4,800 (the limit) | the left gripper came within 3.2 cm of the knob; the pin was never lifted; the crank moved at most 0.77 degrees |

- **Runtime per trial:** 66 s of simulated time for the demo, 240 s for a full 4,800-step run. Measured wall time
  on an L40S shared with one other simulation, including Isaac Sim's start: demo 282 s, two-episode demo 496 s,
  Dream-1 873 s.
- **Seeds:** one fixed layout; the seed changes nothing. The simulation is deterministic: the two demo episodes and a
  separate demo run gave the same crank readings to 0.01 degrees.
- **Reset:** in the two-episode run, the part poses written after the second reset matched the first to 1e-6 m.
- **Lock probe** (2026-11-06 00:15 UTC, the command above): a velocity drive on the crank (30 degrees/s target,
  0.01 N m per degree/s, so 0.3 N m on a stalled crank, capped at 0.5 N m), arms idle.
  - With the pin seated, the 0.3 N m push moved the crank 1.08 degrees and no further.
  - Lifted by its own drive, the pin cleared the plate and the crank turned freely at 30 degrees/s.
  - Released while the crank turned on at 8 degrees/s, the pin rode on the plate face. The spring then dropped it into
    the next hole it met (hole 14 of the 20-hole circle), and it stopped the crank.
- **Joint stretch:** in the `force` run the arm pulled harder than the probe. The crank moved 1.72 degrees, and the
  knob's and pin's joints stretched by up to 3.2 and 0.8 mm (PhysX joint softness under the arm's pull). In the demo
  the largest joint error was 0.97 mm (the knob, while the pin was pulled).
- **First demo, before the knob correction:** scored 0.0. The pin reached the right angle (hole 15), but it sat
  1.45 mm inside the outer circle (radius 54.55 mm, 56 needed) and landed on the plate face beside the hole.

## Simulation notes and limits
- **Joints:** the crank (revolute, with a declared bearing drag), the plunger slide (prismatic along the arm, limits
  at the two circles' radii, a light inward spring), the pin (prismatic, vertical, a spring drive pushing it down to
  its seated limit) and the knob (revolute, free) are native PhysX joints outside the robot's articulation. They are
  disabled while a reset writes the start layout and enabled afterwards.
- **Contact only:** the pin locks the crank only through contact with the pocket walls. The pin, the knob and the
  plate's pockets are the only colliding parts of the crank; the arm, hub and slide are visual, so the crank can be
  turned only through the knob.
- **The spindle:** kinematic. Each control step its yaw is written from the crank's measured, unwrapped rotation / 40,
  one way, so the worm cannot be back-driven. This build has PhysX's gear joint, but a gear joint is a two-way
  constraint, so a push on the workpiece would turn the crank; it is not used.
- **The demonstration is privileged:** it reads the measured crank angle (as a machinist watches the arm pass the zero
  line and counts holes) and the knob's measured position. The YAM arm is compliant under the stock gains: in a first
  demo without correction (2026-11-05), the pin's lift swung between 5.1 and 7.8 mm (8.5 commanded) and the plunger's
  radius between 51.4 and 55.5 mm (56.7 commanded) around the circle, and the pin missed its hole. So the demo
  adds an integral of the measured knob error to its commanded point (rate-limited and capped, frozen while the
  error is large) and waits for the pin to line up with the hole before it lowers it.
- **Simplified:** no worm, wheel or backlash; the plate and the spindle both face up (a real head has the crank at
  right angles to the spindle); square pockets; a spring-held slide instead of a clamped plunger arm; no sector arms;
  rigid primitives, nominal masses and friction. Physics at 240 Hz, control and scoring at 20 Hz.
