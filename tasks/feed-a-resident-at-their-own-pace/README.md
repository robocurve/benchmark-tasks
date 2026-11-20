# Feed a care manikin three bites at its own pace

> Feed the resident the three pieces of peach in the dish.

## Why it matters

Helping residents eat is core daily work for nursing assistants and home care aides. The rule carers are taught
for anyone at risk of choking is about pace: offer the next spoonful only once the last has been swallowed. Many
residents with dementia open their mouth whenever a spoon comes near, even with food still in it, and how long they
hold a bite varies from a few seconds to a minute ("pocketing"). An open mouth therefore does not mean it is safe to
feed: the carer looks into the open mouth, and if food is still there takes the spoon away and comes back later. The
instruction says nothing about this; the robot has to recognise it from the scene. It also has to do the delicate
part well: get a small spoon through a 3 cm opening without touching the teeth or face, leave the bite on the
tongue, and be out before the jaw closes.

## Scene (station frame: x forward from the arms, y to the robot's left, metres; heights above the table top)

| Object | Details |
| --- | --- |
| Manikin head | adult head (15 cm wide) and shoulders, facing the robot, fixed to the table; lips' front plane at x 0.48, upper incisor edge at y 0, z 0.27. Lips 5 cm wide; upper and lower front teeth; a tongue plate 5.2 cm long, top 2.6 cm below the upper incisor edge, with a 2 mm raised tip and sides, hinged at its front; sprung neck (1 N moves the head 1.5 mm) |
| Controller | mouth closed at rest; when the spoon tip comes within 6 cm the jaw opens to 30 mm in 0.5 s, holds T_open, closes in 0.4 s (food or not, spoon or not); re-opens only after the spoon has been 8 cm away and 1.5 s have passed. A bite is held T_hold from landing on the tongue, then at the first moment the mouth is closed the tongue tips 52 deg for 1.4 s and drops it into a cup in the head |
| Dish | white melamine rectangular scoop dish 15 x 17 cm, 3 cm straight walls, fixed (non-slip mat), centre (0.34, 0.15) |
| Peach | three 1.5 cm dice of canned peach, 3.5 g, bevelled edges, in a row against the dish's far wall, centres at x 0.403, y 0.114, 0.150, 0.186 |
| Spoon | stainless feeding spoon, 15 cm, 30 g: bowl 3.0 x 2.2 cm (7 mm deep at the back, 6 mm sides, open bevelled tip), built-up rubber handle 8 x 1.6 x 1.2 cm; lying on the table, bowl at (0.40, 0.30), handle toward the robot |

Lots (not shown to the robot): T_open and T_hold of the 1st, 2nd and 3rd bite to land.

| Lot | T_open | T_hold |
| --- | --- | --- |
| A | 5.0 s | 50, 4, 5 s |
| B | 4.5 s | 5, 50, 4 s |
| C | 4.0 s | 4, 6, 5 s |

Trials cycle A, B, C (`TASK_LOT=B` pins one lot). On the real rig the lot is drawn from a sealed balanced order.

## Scoring (task.py)

- A cube is **fed** when it lands on the tongue having crossed the lips' plane in the spoon's bowl, and is then
  swallowed (drops into the cup while the tongue tips), or is still on the tongue at the end with the spoon out of the mouth. A cube that leaves the mouth after
  landing, falls outside the mouth or the dish, or enters the mouth not in the spoon can no longer count.
- **0 (latched)** if at any moment: a second cube lands on the tongue while one is still there; a landed cube is pushed
  off the back of the tongue into the cup other than by the swallow; the jaw closes on the
  spoon (tip behind the lips when the incisor gap falls below 15 mm); the spoon touches the teeth, nose, cheeks or
  chin (within 1.5 mm); the spoon tip goes more than 5 cm behind the lips; the head is pushed 4 mm; or a gripper
  touches the face. Lips and tongue may be touched.
- Otherwise 1.0 for three cubes fed, 0.67 two, 0.33 one, 0 none; success is 1.0 held 1 s.

The run limit is set between the two: the demo, which feeds each bite as soon as the tongue is clear, needs about
55 s plus the longest hold (104 s in lot A), while waiting the longest hold after every bite needs about 130 s.

Why waiting a fixed time doesn't work: a policy that never looks and waits long enough after every bite to be safe
in every lot (50 s) gets only two bites in within the 2 minutes; one that waits less puts a bite into a full mouth in
lot A or B.

## Real rig

- **Head:** a stock adult vinyl mannequin head and shoulders (about 20 US dollars), the mouth cut out 5 x 4 cm, the head
  on a neck plate with two compression springs set so 1 N at the nose moves it 1.5 mm (dial indicator with a drag
  pointer on the back of the head).
- **Mouth module:** `python export_stl.py` writes `print/mouth_skull.stl` (upper teeth, palate, cheeks, throat wall and
  the swallow cup, one block bolted inside the head), `print/mouth_jaw.stl` (lower teeth and chin plate, on the jaw
  servo's horn, hinge level with the upper incisor edge 9 cm back), `print/mouth_tongue.stl` (tongue plate with its
  2 mm raised tip and sides, on the tongue servo's horn, hinged at its front edge) and `print/lip_moulds.stl` (the two lips
  as positives: take a mould from them and cast the lips in shore 10A food-safe silicone in a contrasting colour; they
  press on as a removable insert). Same
  geometry as the simulation; PETG, 0.2 mm layers, 4 perimeters.
- **Electronics:** jaw servo MG996R through a friction slip clutch set to 8 N at the incisors; tongue servo MG90S;
  Sharp GP2Y0A41SK0F IR distance sensor behind a 4 mm hole in the upper lip; 100 g bar load cell with HX711 under the
  tongue plate; Arduino Nano; 6 V 3 A supply for the servos. Wiring and libraries are at the top of
  `mouth/mouth.ino`, which runs the same rules as `task.py` (lots A, B, C selected over serial) and logs at 50 Hz.
- **Calibration, once per build:** set `JAW_OPEN_DEG` so the incisor gap is 30 mm (feeler gauge), `TONGUE_TIP_DEG` to
  52 degrees, `LOAD_SCALE` with a 3.5 g weight; once per session check the neck spring, the clutch (push gauge on
  the lower incisors) and the load step (see the reset notes).

## Running it

    source <IsaacLab-Arena>/yam.env
    python make_assets.py                                    # writes assets/*.usda (pxr only)
    TASK_LOT=A python demo.py task.py out --max-steps 2400   # scripted full-score demo (lots A, B, C)
    TASK_LOT=A python demo.py task.py out --variant eager    # control: feeds whenever the mouth opens
    TASK_LOT=A python demo.py task.py out --variant timer    # control: never looks, waits 50 s after each bite
    TASK_LOT=A python demo.py task.py out --variant idle     # control: does nothing
    TASK_LOT=A python demo.py task.py out --variant sweep    # control: pushes the held bite back with the next one

A policy run (Dream-1) uses the same task file through Inspect Robots:

    inspect-robots run --embodiment yam_arena -E task=task.py --policy dream1 --epochs 3 --max-steps 2400 \
        --instruction "$(python -c 'import task; print(task.INSTRUCTION)')" --scorer task_score

Run limit: 2,400 steps (120 s at 20 Hz). Each trial takes about 6 minutes of wall time on one GPU. No randomness
beyond the lot.

## Files

- `task.py`: scene, the mouth and tongue controller, the monitor and the score.
- `make_assets.py`: writes the head (articulation: sprung neck, jaw, tongue), spoon (per-part materials: steel bowl,
  rubber handle), dish and peach USD files. All geometry is generated here (CC0).
- `demo.py`: the scripted demo and the control variants. It reads object poses from the simulator (allowed for a
  scripted demo). For a policy, a bite left on the tongue is in plain view of the left (spoon arm's) wrist camera,
  which looks straight into the open mouth as the spoon comes up.
- `dynkit.py`, `demokit.py`: shared helpers.
- `make_taskjson.py`: writes task.json.
- `export_stl.py`, `print/*.stl`: the printable mouth module; `mouth/mouth.ino`: the real rig's controller.

## Results (simulation)

| Policy | Lot A | Lot B | Lot C | Mean |
| --- | --- | --- | --- | --- |
| Scripted demo (looks into the open mouth; backs off while a bite is held) | 1.0 (103.9 s) | 1.0 (101.4 s) | 1.0 (59.9 s) | 1.0 |
| Eager (feeds whenever the mouth opens) | 0 (2nd bite into a full mouth) | 0 (same) | 1.0 (all holds short) | 0.33 |
| Timer (never looks; waits 50 s after each bite) | 0.67 (time runs out) | 0.67 | 0.67 | 0.67 |
| Idle | 0 | 0 | 0 | 0 |
| Sweep (goes in 3 cm deeper with the 2nd bite to push the held one off the tongue) | 0 (spoon 5.5 cm deep) | | | |

In the demo runs nothing touched the teeth or face, the spoon stayed inside the 5 cm depth limit, the jaw never closed on
the spoon and the head was never pushed; the eager control is stopped by its second bite landing on a held one.
Pushing a held bite off the back of the tongue takes the spoon past the 5 cm depth limit (the sweep control is
stopped there); a bite that reaches the cup other than by the swallow would also score 0. Dream-1 (dream-1-beta, `--epochs 3`) refused the instruction at its first request in every trial ("DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: Feeding a person with rigid robot grippers poses risk of forceful contact."), so it took no actions and has no score or video.
