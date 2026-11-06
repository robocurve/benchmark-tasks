# Find an engine's top dead centre with a piston stop

Before a camshaft is degreed, an engine builder finds the crankshaft's true top dead centre (TDC) with a positive
piston stop. They turn the crank until the piston touches the stop and mark the degree wheel. Then they turn it the
other way until it touches again and mark the wheel again. They remove the stop and turn the crank to the midpoint
of the two marks. Quoted from PROFORM's degree-wheel kit instructions (Specialty Auto Parts U.S.A., 2017,
[ProForm-68786.pdf](https://cdn.vividracing.com/file/vr/3351/2021/ProForm-68786.pdf)):

> "Continue rotating the crankshaft in the same direction until the piston rises and contacts the piston stop.
> Marking with a pen or pencil on the degree wheel, mark the degree number ..." ... "rotate the crankshaft back in
> the direction of motor rotation until the piston rises and contacts the piston stop." ... "remove the piston stop,
> and turn the crankshaft to the midpoint between the two marks: this is top dead center for cylinder #1."

Done by: automotive service technicians and mechanics (O*NET 49-3023.00, which lists "Rebuild parts, such as
crankshafts and cylinder blocks").

Here the engine is a single-cylinder trainer with a vertical crankshaft and a horizontal cylinder, the layout of a
small vertical-shaft engine, at that engine's size (60 mm stroke). One gripper turns the crank handle and feels for
the stop from each side. The other pulls the stop out of its slot and sets it in a tray. The first turns the crank to
the midpoint and lets go. The throw, rod and piston are hidden in the housing. The handle is set 37 degrees from the
throw, and the degree wheel's zero is 23 degrees from TDC, so TDC can't be read by eye; it has to be measured.

The task is a task-local plugin for the stock `yam_arena` adapter (Isaac Lab-Arena 0.3.1, Isaac Lab 3.0, Isaac
Sim 6.1, the calibrated `station` scene). It registers:

- the embodiment `yam_piston_tdc`: the stock robot, cameras and 14-D joint action and observation, unchanged, plus
  the trainer's bodies and joints, scored from the measured part poses;
- the policy `piston_tdc_scripted`: a privileged scripted demonstration, plus the failure paths below.

## Setup
Coordinates: x forward from the line between the arm bases, y left from their midpoint; heights above the table top.

| Part | Description | Position |
|---|---|---|
| base plate | dark steel, 13 x 23.4 x 0.6 cm, under the housing | fixed to the table |
| housing | closed box of dark-blue painted 6 mm steel, 11.2 x 21.6 x 7.2 cm; hides the crank throw, rod and piston | crank axis at (25, 14) cm; housing from y +19.6 to -2.0 cm |
| crankshaft | vertical; 30 mm throw (60 mm stroke), 95 mm connecting rod, 4 x 3 x 3 cm steel piston | hidden |
| degree wheel | yellow-painted aluminium disc, 17 cm across, 4 mm thick; black tick every 10 degrees, blue every 90, red zero tick 23 degrees from TDC | on the crankshaft, top 7.8 cm high |
| handle | dark steel arm (9.6 x 1.6 x 0.6 cm) and an orange nylon sleeve (20 mm across, 40 mm tall) spinning on its steel pin, 7 cm from the axis, 37 degrees from the throw | sleeve 8.8-12.8 cm high |
| pointer | dark steel post (1 x 1 x 7.8 cm) with a green-painted steel needle (triangle 18 mm long, 12 mm wide, 3 mm thick), tip 2 mm outside the rim | (34.9, 14) cm |
| piston stop | red steel T (80 g): 36 x 10 x 60 mm shank in a slot in the lid, 50 x 30 x 8 mm plate on the lid, 20 x 30 x 35 mm grip block | (25, -0.5) cm, top 11.5 cm high |
| cylinder head wall | fixed, right behind the stop's shank: it carries the piston's load, so the stop can't be pushed out | inside the housing |
| tray | blue plastic, 16 x 14 cm inside, 8 mm walls 12 mm high, 4 mm floor, fixed | (21, -17) cm |

- **Start:** crank 75 degrees past TDC (the piston mid-stroke, clear of the stop), stop in its slot, both grippers
  open at the ready pose. One fixed layout; nothing is randomized.
- **The stop:** with it in, the piston meets it 28.5 degrees before or after TDC, so the crank turns freely over the
  other 303 degrees only.
- **Drag:** the crankshaft has a little bearing drag, so it stays where it is let go.

## Scoring
Crank angle is measured from true TDC at 20 Hz. On a real trainer the TDC reference is a scribe mark on the underside
of the degree wheel facing a mark on the lid: it is hidden under the wheel and read only after the trial (lift the
wheel, or photograph from below). The sim scores from the crank's measured pose.

| Score | Condition | How to check on a real table |
|---|---|---|
| 0.25 per side, latched | the piston is in contact with the seated stop on that side (piston-to-stop gap under 0.3 mm), with the crank at rest (under 2 degrees/s) within 1 degree of the contact angle (28.5 degrees before or after TDC) | feeler gauge or the crank stalling; photo of the wheel at each stall, read against the hidden TDC mark after the trial |
| 1.0, success, the trial ends | the crank within 2 degrees of TDC, the stop resting in the tray, both grippers at least 80% open with their tool points at least 5 cm from the handle sleeve, the crank slower than 1 degree/s, all for 1 s | photo of the wheel against the TDC mark; the tray; the grippers open and away |
| 0, latched | the crank more than 3 degrees past a contact angle (toward TDC) while the stop is seated: forced through the stop (on a real engine, a bent rod or a damaged piston) | the wheel reads inside the blocked arc while the stop is in |
| otherwise | the best partial score so far | |

- **Every outcome:** nothing done, 0; one side, 0.25; both sides, 0.5; the final condition, 1.0 (with or without the
  sides: the result is what counts). A crank released 3 degrees off TDC keeps only its side credit.
- **Seated** means the stop's pose within 2 mm and 3 degrees of its seated pose. **In the tray** means its centre
  inside the tray's inner footprint, less than 8 cm above the tray floor, and moving slower than 5 cm/s.
- **No credit for touching:** a side counts only when the piston is measured touching the seated stop (gap under
  0.3 mm) and the crank is at rest there. Touching the stop or the handle, or a shove that stops short of the stop,
  earns nothing.
- **Unintended strategies:** pulling the stop first and guessing earns nothing unless the guess is within 2 degrees
  (the handle and the wheel's zero are both off TDC, so the obvious guesses are 23 or 37 degrees out); the `guess` run
  shows it. Forcing the crank through the stop scores 0. That rule is a guard, geometrically unreachable with the
  rigid stop: in the probe the piston sank into it by at most 0.05 mm, while 3 degrees past contact would take about
  1 mm.
  A stop dropped anywhere but in the tray, or knocked off the table, never reaches
  1.0.
- **Seeing 2 degrees:** 2 degrees is 2.4 mm at the handle, about half a top-camera pixel. The intended route is the
  arm's own joints: note the gripper's position at each stall and bisect. The wheel's ticks also show in the wrist
  cameras. Once the stop is out, the piston crown shows in the open slot, up to 6 mm of it at TDC; it moves less
  than 1 mm over the last 10 degrees either side, so it locates TDC only to about plus or minus 10 degrees.
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
uv pip install --no-deps -e tasks/piston-tdc
```

On the Robocurve launchable the environment already exists: `source /data/IsaacLab-Arena/yam.env`, then
`uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e tasks/piston-tdc`.

Run from `tasks/piston-tdc`. Every run below uses the same settings; only `--policy` and `-P` change:

```bash
cd tasks/piston-tdc
INSTR="$(python3 -c 'import json; print(json.load(open("task.json"))["instruction"])')"
COMMON=(--embodiment yam_piston_tdc -E control_hz=20 -E cam_width=320 -E cam_height=176
        --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames)

# The scripted demonstration (full score)
inspect-robots run "${COMMON[@]}" --policy piston_tdc_scripted --log-dir runs/demo

# Dream-1 (needs a Dream API key in ~/.dreamlabs/credentials)
inspect-robots run "${COMMON[@]}" --policy dream --log-dir runs/dream

# Failure paths
inspect-robots run "${COMMON[@]}" --policy piston_tdc_scripted -P mode=wrongmid --log-dir runs/wrongmid
inspect-robots run "${COMMON[@]}" --policy piston_tdc_scripted -P mode=guess --log-dir runs/guess

# One MP4 per camera, next to each run's frames
for log in runs/*/*.json; do inspect-robots video "$log"; done
```

- The embodiment loads its own task file (`piston_tdc_task.py`: the tray) and always uses a fixed layout.
- `-E telemetry=<file.jsonl>` writes a per-step trace: crank angle, speed, stop state, piston-to-stop gap, grippers,
  joint-anchor errors and score, and prints a status line every 0.5 s.
- The run prints `task_score` at the end: 1.0 on success, otherwise the best partial score.
- Policies: `-P mode=wrongmid` measures both sides, then stops at the midpoint of the arc it swept (bottom dead
  centre); `-P mode=guess` pulls the stop first, then turns the handle to point at the stop and lets go; `-P mode=idle`
  holds both arms still.

## Results
Measured on 2026-11-05, one L40S, the settings above (one episode each). Re-run on 2026-11-06 with the contact-gap
rule, on the pinned adapter commit (0059d02): the demo, `wrongmid` and `guess` gave the same scores and steps, and a
byte-identical demo video; each side was credited with the piston touching the stop (gap -0.01 mm). Dream-1 was not
re-run (same instruction).

| Run | Score | Steps | What happened |
|---|---|---|---|
| scripted demo | 1.0 | 1,215 (60.8 s) | marks at +28.39 and -28.46 degrees (the true contact is +-28.47); midpoint 359.96; released 0.16 degrees from TDC |
| `-P mode=wrongmid` | 0.5 | 1,491 | both sides measured, stop in the tray, crank left at bottom dead centre (179.8 degrees) |
| `-P mode=guess` | 0.0 | 669 | stop pulled first, handle pointed at the stop: the crank ends 36.9 degrees from TDC |
| Dream-1 | 0.0 | 2,400 (the limit) | a gripper came within 4.3 cm of the handle; the crank never moved |

- **Runtime per trial:** 61 s of simulated time for the demo, 120 s for a full 2,400-step run. Measured wall time on
  an L40S shared with one other simulation, including Isaac Sim's start: demo 231 and 268 s (two runs), Dream-1 460 s.
- **Seeds:** one fixed layout; the seed changes nothing. The simulation is deterministic: two demo runs gave
  byte-identical videos.
- **Reset:** a two-episode demo run (`--epochs 2`) scored 1.0 in both episodes; the part poses after the second
  reset matched the first to 1e-6 m.
- **Closed chain at rest** (measured 2026-11-05 on an earlier development build that had a diagnostic crank motor,
  since removed): with the crank turned by that torque-limited motor and the arms idle, the crank
  rested against the stop at +28.467 and -28.467 degrees with a 0.00 mm piston-to-stop gap. Every joint anchor stayed
  within 0.005 mm over 15 s at rest (peak 0.062 mm at impact). In the demo the peak was 0.07 mm.

## Simulation notes and limits
- **Joints:** the crank (revolute, with a declared bearing drag of 0.02 N m s/rad), the rod's big end (revolute), the
  wrist pin (spherical, which removes the loop's redundant constraints), the bore (prismatic) and the handle sleeve
  (revolute) are native PhysX joints outside the robot's articulation. They are disabled while a reset writes the
  start layout and enabled afterwards.
- **Contact only:** the piston meets the stop only through contact, and the cylinder head wall behind the stop takes
  the load. Nothing is teleported or locked in software.
- **The demonstration is privileged:** it reads the crank's measured angle, the way a mechanic reads the wheel, and
  never the true TDC. It finds each contact when the crank stops following the handle, and bisects its own two
  readings. The YAM arm is compliant under the stock gains, so while it turns the crank the demo corrects its
  commanded angle by integrating the measured error (rate-limited and capped, and frozen against the stop).
- **Simplified:** rigid primitives, nominal masses and friction, no compression, oil film or wear. Physics at 240 Hz,
  control and scoring at 20 Hz.
