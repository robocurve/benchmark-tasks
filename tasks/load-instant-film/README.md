# Load a film pack into an instant camera

An instax mini 12 instant camera sits on a stand on the table, empty, its back towards the right arm and its lens
towards the left, leaning 30 degrees towards the lens so that its back faces up, as a hand holds it to load. A new film
pack lies flat beside it. The instruction asks for the pack to be loaded and the camera
made ready to shoot. That is the camera's own loading sequence, each step a different kind of contact:
1. Press the back cover lock down: the back cover springs ajar.
2. Swing the cover open round its hinge.
3. Pick the pack up, turn it so its yellow mark is at the camera's yellow mark (yellow end up, the back with the two
   holes facing out), and put it fully home in the chamber.
4. Swing the cover shut and press it until it locks.
5. Grip the lens ring and turn it to ON.
6. Press the shutter button once: the black film cover comes out of the slot on top.

A pack in upside down or back to front, or a cover that isn't locked, spoils the film; a shutter press on a camera that
isn't loaded right ejects nothing.

## The real step
Fujifilm's instax mini 12 User's Guide (FI020-EN-WW-01, pages 3-4, drawn steps; part names on page 6: "Back cover
lock", "Back cover", "Lens ring / Power", "Shutter button", "Film ejection slot"):
1. Slide the back cover lock down to open the back cover.
2. Insert the film pack, aligning the yellow marks.
3. Close the back cover.
4. Turn the camera on by rotating the lens ring to ON.
5. Press the shutter button: the film cover is ejected.

Fujifilm's instax printer manuals add, for the same mini film pack, never to press the two rectangular holes on the
pack's back. That rule is not scored here: a video can't show reliably whether a fingertip touched a hole.

Done by: anyone who owns an instant camera, at a party, a wedding or at home; the same step comes with every new
pack (10 shots). What goes wrong: fogged or jammed film, a wasted pack. A photo of the camera shows whether the film
cover came out.

## The camera, the pack, and what was simplified
| Size | Value used | Source |
|---|---|---|
| camera body | 104 x 66.6 x 122 mm (W x D x H) | Fujifilm's instax mini 12 specifications page (306 g without batteries, strap and film) |
| film pack | 92 x 61 x 20 mm, 45 g | size: Wikipedia's instax article (mini film pack); mass: **estimate** |
| film chamber | 67 x 98 mm, 20.5 mm deep (3 mm play round the pack), floor 11 mm up | **estimate** |
| back cover | 3 mm plate, 30 g, hinged on its far edge, a stop at 95 degrees, springing 20 degrees ajar when unlocked | **estimate** from the guide's drawings (which warn against opening it further) |
| back cover lock | 12 x 12 x 7 mm tab, 3 mm travel, unlocks at 2 mm | **estimate** |
| lens ring | 62 mm across, 22 mm deep; ON at 30 degrees or more | **estimate** |
| shutter button | 18 mm across, 4 mm proud, 2.5 mm travel | **estimate** |
| pack's holes | two 20 x 10 mm holes, either side of the back's middle | **estimate** |

How the sim models it:
- The camera is a fixed articulation, built with `mechanism.py` (MIT, from robocurve#223; changes listed at its top),
  described upright and then tilted as a whole, 30 degrees about the x axis (`FILM_TILT`), on its stand:
  - fixed boxes and cylinders for the stand, the body, the rear frame round the chamber, the chamber's floor and the
    marks;
  - the back cover on a revolute joint with a spring whose target the task file sets every control step: locked, the
    target is 0 (held shut, 20 N m/rad); once the lock tab is 2 mm down, the target is 20 degrees (it springs ajar),
    then follows the cover wherever it is put; pushed back within 2 degrees with the tab up, it relocks;
  - the lock tab and the shutter button on sprung prismatic joints; the lens ring on a revolute joint with friction;
  - the film cover, a thin black plate inside the body on a prismatic joint, driven 45 mm up out of the slot by a
    shutter press made while the camera is ready.
- The pack is one loose rigid box at its true size; its holes and yellow mark are drawn on its back, not cut.

What we simplified:
- The camera is fixed on a stand, leaning 30 degrees; a person holds it in one hand, back up, to load it. Real
  trials: a wedge 30 degrees steep, the camera taped to it. WHY the lean: standing upright, a pack let go part-way
  into the chamber slid straight back out in the sim every time (2026-11-23, six variants); leaning, its weight
  holds it in.
- The film cover is ejected by a joint, not a motor and rollers; nothing is exposed or printed.
- The pack is a plain box: the real pack's shape keys it, so in a real camera a wrong-way pack sits proud and the cover
  won't shut; here it fits and is scored 0.
- The chamber has 3 mm of play round the pack [estimate].
- The sizes marked estimate above, until a camera is measured; the batteries are left out.

## Setup
Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left, heights above the table.
Places on the camera are given as if it stood upright on its base on the table (its own frame); the stand then tilts
it 30 degrees about the x axis, top towards +y, and lifts it so the lens ring's lowest point is 2.5 cm up.

| Object | Description | Position |
|---|---|---|
| camera | instax mini 12, mint green, on a grey stand, leaning 30 degrees towards the lens; fixed | centre (38, 0), back towards -y (right arm) and up, lens towards +y (left arm) and down |
| back cover | mint, 3 mm, hinged along its far edge (x 43.2) | covers the back; opens towards -y |
| film chamber | dark floor, yellow mark at its top far corner | x 34.65-41.35, 1.1-10.9 cm up, behind the cover |
| back cover lock | grey tab | x 34.0-35.2, y -4.5 to -3.3, 11.2-11.9 cm up |
| lens ring | silver, black front, index mark on top | centre (38.6, y 3.3-5.9), 4.8 cm up |
| shutter button | silver | (34.6, y 3.3), 9.2 cm up |
| film pack | black; two grey holes and a yellow corner mark on its back | flat, back up, centre (24, -18), length along y, yellow end away from the arms |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`. For real trials: the camera
empty, cover locked, ring at OFF, a fresh pack placed as in the setup photo.

## Scoring
- **Seated:** the pack in the chamber with its yellow end up and its back out (each axis within 10 degrees), standing
  on the chamber's floor and centred across it (each within 3.5 mm), its back within 1.5 mm of flush with the mouth.
- **Locked:** the cover pushed shut within 2 degrees with the lock tab up.
- **ON:** the lens ring turned 30 degrees or more.
- **Hands clear:** no point along either gripper's pads, and neither fingertip's end, within 2 cm of the camera.

| Score | When |
|---|---|
| 1.0 | the film cover ejected by a shutter press made with the pack seated, the cover locked and the ring at ON; still so, hands clear, the pack still; held for 1 s, which ends the trial |
| 0.75 | seated, locked and ON, but not yet ejected (or a hand near the camera) |
| 0.5 | seated and locked |
| 0.25 | seated |
| 0 | anything else, including a pack in the chamber the wrong way round, a fingertip ever within 2 mm of either hole on the pack's back (and up to 3 mm above it), and the cover ever pulled 5 degrees open while locked |

In a real trial, the film cover in the slot and the camera's film counter (S, then 10) show the result; a ruler gives
the pack's seating.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder. `mechanism.py` must sit beside the task file, which
imports it:

```bash
INSTR="$(python3 -c 'import json; print(json.load(open("task.json"))["instruction"])')"
COMMON=(--embodiment yam_arena -E task=load_instant_film.py -E object_jitter=0 -E control_hz=20 -E cam_width=320
        -E cam_height=176 --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames)

# Dream-1
inspect-robots run "${COMMON[@]}" --policy dream --log-dir ~/yam-eval/load-instant-film-dream

# The scripted demo, which reaches full score
inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/load-instant-film-demo

# Wrong ways (probes of the scoring rules)
FILM_PACK_TURNED=1 inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/lif-turned   # yellow end down
FILM_DEMO=noring inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/lif-noring     # ring left OFF

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/load-instant-film-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. With `FILM_DEBUG=<file>` set, each control step adds a line to that file: the
pack's place and turn, whether it is in the chamber and seated, the cover's angle, the lock's travel, the ring's angle,
the shutter's travel, the film cover's height, the lock state, presses, ejection, and where the pad
tips are.

## The scripted demo
It plays a fixed plan from the fixed layout with the arm's own inverse kinematics, and reads nothing from the
simulator.

Moves at the camera are planned as if it stood upright, then carried by its 30-degree tilt; how far the hand's own
turn follows the tilt is set per move, to where the arm reaches (offline IK, 2026-11-23).
1. The right hand, pointing down, presses the lock tab 3 mm; the cover springs 20 degrees ajar.
2. Its two closed fingertips along the cover, the hand leaned 65 degrees in the cover's plane, fingertips towards the
   hinge, it comes down into the gap behind it and pushes it round its hinge, in 5-degree steps, to its stop at 95
   degrees. Past upright, the cover's weight holds it there.
3. It pinches the pack across its width, holding 4 mm of its 20 mm sides, lifts it and turns it a quarter turn about
   the line between its fingers, to point at the chamber: yellow end up, back out.
4. It carries the pack 14 mm into the chamber, opens its fingers just past the pack's sides, slowly, and backs out;
   the pack stays, leaning in. It pushes it home with its closed fingertips on the pack's back.
5. It goes round the open cover's free edge, low, and swings the cover shut from outside to about 35 degrees, then
   turns to point at it and pushes it straight shut until it locks, and parks.
6. The left hand, pointing level at the lens, grips the ring and turns it, lets go, then presses the shutter button
   2.2 mm and parks. The film cover comes out of the top.

## Runtime and seeds
- **Official runs (2026-11-23, this version: camera on its stand, no notches):** the scripted demo scored 1.0, ending
  on success at step 1,135 (57 s): pack seated the right way, cover locked, ring at 56.9 degrees, one shutter press,
  film cover 44.7 mm out, hands clear.
  - Let go 14 mm into the leaning chamber, the pack stayed, and the closed hand pushed it home.
  - Two runs of this version failed first. The hand, turned with the camera's tilt only part-way, met the cover with
    its back and opened it to 45 degrees; the cover drifted shut and the pack, carried in, shut it in front of the
    chamber (grade 0). Now the hand leans 65 degrees in the cover's own plane. Then the lens ring, gripped level about
    its tilted axis, slipped and stopped at 23.7 degrees (grade 0.5); the hand now turns 70 degrees.
  - Probes (the same demo, one thing wrong, 1,300 steps): `FILM_PACK_TURNED=1` loaded the pack yellow end down, then
    locked the cover, turned the ring and pressed the shutter: grade 0, nothing ejected. `FILM_DEMO=noring` left the
    ring OFF: grade 0.5 (seated and locked), the shutter press ejected nothing.
  - Dream-1 (2026-11-23, the task's instruction, 320 x 176 cameras, 2,400 steps; the run used the task file before the
    demo's ring turn changed, the same scene): 0. It moved both arms for the full run without touching the lock, the
    cover or the pack.
- **The version first submitted (2026-11-22, robocurve#3895):** camera upright, a finger notch in each side of the
  chamber; demo 1.0 at step 991. Reviewed "Not accepted": the notches aren't on the real camera, and the holes rule
  can't be checked from video. A notch-free upright version failed: the pack slid back out each time it was let go
  (six variants). See `docs/robocurve/load-instant-film-spec.md`.
- **Offline checks:** the scoring, lock logic and reach checks run without the simulator:
  `python -m pytest -q test_load_instant_film.py` (needs `yam_arena` importable).
- **Seeds:** one fixed layout (`-E object_jitter=0`), so the seed doesn't change the start.
