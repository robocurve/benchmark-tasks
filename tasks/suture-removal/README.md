# Take out an interrupted suture: forceps lift the knot, a stitch cutter cuts flush, then draw it out toward the wound

A silicone suture practice pad lies on the table with one interrupted nylon stitch across its closed incision. The robot holds dissecting forceps in one hand and a single-use stitch cutter in the other. The forceps grip the knot and lift it a few millimetres; the cutter's hook slides in under the leg away from the knot and cuts it flush with the skin; the forceps draw the stitch out by the knot, pulling back toward the wound so its edges are not drawn apart, and lay it on a gauze square. Both instruments go back on their trays.

**Where it comes from.** Medical assistants have the core task "Help physicians examine and treat patients, handing them instruments or materials or performing such tasks as giving injections or removing sutures" (O*NET 31-9092.00, about 811,000 US jobs; USDOL/ETA, CC BY 4.0; O*NET is a trademark of USDOL/ETA). Limbs & Things, a maker of suture practice pads, teaches the removal in its suture aftercare manual (https://limbsandthings.com/media/busponzi/suture-aftercare-manual.pdf): "Grasp the knot with dissecting forceps, and then cut the suture precisely with scissors, flush with the tissue surface, so that the exposed length of suture (which may be contaminated) does not have to pass through the tissues", then "Remove the suture by pulling back towards the wound so that the wound edges are not drawn apart by a distracting force." UK and EU suture removal kits pair the forceps with a single-use stitch cutter, as here; US kits mostly pair them with small Littauer scissors, which make the same cut. Limbs & Things is named as a source only; this task is not affiliated with it.

**What this task tests.** Two instruments, one in each hand, working together on one strand at a point a millimetre or two across. The forceps are closed through their sprung arms by the gripper's squeeze and must hold the knot up while the other hand works: lifted 1 mm, the stitch leaves room under it for the hook; pulled 4.5 mm, it tears through the skin. The cutter's hook must slide in level under the lifted stitch on the leg away from the knot and cut within 1 mm of the skin without pressing its foot 2 mm into the silicone. Then the draw must go back toward the wound: a pull away from it opens the incision.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Take out the stitch: lift the knot with the forceps, cut the leg away from the knot flush with the skin with the stitch cutter, draw the stitch out toward the wound, put it on the gauze and the instruments back in their trays, and take your hands away from the pad."
```

A trial runs up to 2,400 steps (120 s of simulated time at 20 Hz). One fixed layout; no seed changes the scene.

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                   # the full-score reference, filmed
python demokit.py task.py OUT --no-record --no-cameras --mode nolift,away,above,preclosed,hover,regrasp   # the failure checks
python check_score.py                                                       # the hook's way under the stitch, the instruments' return and the scoring table, no simulator
```

The demo is a scripted reference: it reads the stitch's and the instruments' poses from the simulator, and a policy gets only the cameras. The left hand takes the forceps from its rest lightly (1.4 N, under the arms' 1.6 N spring preload, so the tips stay open), points them down at 45 degrees from the knot's side, sets the tips on the knot and squeezes them shut at 6 N, and lifts the knot about 3 mm. The right hand takes the stitch cutter by its flat body and holds it upright, its hook's notch opening toward -x. It brings the hook down to the skin 10 mm to the +x side of the far leg, with the notch's floor 0.6 mm over the skin line, and slides it in level across the leg in one smooth motion, slowly, so the hook stays within the strand's width of the leg: the leg goes into the notch through its mouth and, pressed on, meets the notch's sharp inner edge and is cut at the skin line. If the leg slips past the notch (the hook wanders about 1 mm across), the hook backs out 10 mm and slides in again, up to three tries; in the film the first try cut, and in the `hover` check the third. The forceps draw the stitch out 10 mm toward the wound and 8 mm up, carry it to the gauze and let it go there. The cutter goes back on its tray; the forceps, now held shut at 6 N, go back on theirs, lowered until their tips reach the rest's floor before the hand lets go; let go, the forceps tip over in their tray, their middle still inside the tray's outline. Both hands rise and move away.

**Result** (scripted, GPU physics on an NVIDIA GeForce RTX 3090 Ti, 20 Hz, filmed at 640 x 360): 1.0 in 796 steps (39.8 s of simulated time): the knot lifted 3.1 mm at most, the far leg cut 0.02 mm over the skin line on the first try (the notch floor's lowest while the leg was in it), the cutter's foot never 2 mm into the silicone, the wound's gap 0.3 mm at most; the film's first frame is the start state.

**Model run** (Dream-1, once, 12 rows per request, seed 1): no score. The provider's API refused the first inference request, at the end of the 1.6 s warm-up and before the policy had acted: "DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Cutting near skin with a sharp cutter presents a severe risk." The trial ended on the refusal, so the scorer gave no score; this is not a 0 the policy earned. The film (`model_run.mp4`) runs to the refusal. The run was not repeated and the instruction was not changed.

| Failure check | What it does | Score |
|---|---|---|
| `nolift` | the forceps stop 10 mm over the knot and never take it (closing on it alone lifts it about 1.4 mm); the cutter goes for the far leg as in the demo | 0.0 in 827 steps: the knot never lifted, so the hook never got under the stitch and nothing was cut |
| `away` | as the demo, but the stitch is drawn out away from the wound | 0.5 in 886 steps: the far leg was cut flush, but the pull opened the incision 4 mm and the stitch never counted as drawn out |
| `above` | the knot lifted as in the demo; the hook brought straight down through the loop onto the far leg, then pressed across it | 0.25 in 450 steps: the knot lifted; the hook never got under the stitch the way a hook can, so nothing was cut |
| `preclosed` | the forceps squeezed shut before they reach the knot, then pressed to it and lifted; the rest as in the demo | 0.0 in 1236 steps: shut before they reached the knot, the forceps never caught it, so nothing was lifted, and none of the hook's three tries cut |
| `hover` | the demo, but the instruments are held over their trays at the end, never let go | 0.75 in 1222 steps (the cut came on the hook's third try): everything done, but neither instrument counted as back in its tray |
| `regrasp` | as the demo, but once drawn out the stitch is let go over the pad, the forceps are closed again at the knot's old place and carried to the gauze | 0.75 in 962 steps: cut flush and drawn out, but let go over the pad the stitch stayed dropped; the second closing caught nothing, so nothing reached the gauze |
| `check_score.py` | the hook's way under the stitch on hand-built paths (slid in level from beyond the leg or from beside the strand: under it; brought down through the loop at any speed, or down inside the strand's 3 mm band and then sideways: not under it), the cut in the cutter's frame (slid in across the leg and pressed onto the notch's edge: cut; stopped in the notch, 1.5 mm short and nudged, 1, 1.5, 1.6 or 2 mm off the leg across the blade's plane (on either face; 0.6 mm, the strand touching the steel, cuts), the cutter turned 90 degrees, slid in from the spine's side, or the notch above the leg: no cut), the instruments' return (standing in the rest with both hands away: in the tray; still gripped, 3 cm up, or on the gauze's corner over the tray: not), letting go of the stitch (drawn out and opened over the gauze: on it; opened anywhere else after the cut: dropped for good, so it cannot be taken again at the knot's old place and carried to the gauze; before the cut: it may be taken again), and the scoring table on hand-built records | every case as expected |

## Scoring

Cumulative:

| Score | When |
|---|---|
| 0.25 | the knot caught in the forceps as they close on it (their tips within 2.5 mm of it) and lifted 1 mm or more |
| 0.5 | also the far leg (the one away from the knot) cut within 1 mm of the skin: the cutter's hook slid in level under the lifted stitch, from beside it, never down through it, the leg entering the hook's notch through its mouth, within 0.9 mm of the blade's plane (the strand touching the 1.2 mm steel), and pressed onto its sharp inner edge, read in the cutter's own frame (the cut's height is the notch floor's lowest over the skin while the leg was in it). Both legs cut: 0.5 at most (the buried part stays in the skin) |
| 0.75 | also the stitch drawn out by the knot, the forceps' tips reaching 12 mm (straight-line) from the lifted knot's place while at least 3 mm toward the wound (-y) of the knot's place, the draw counted only while they are no more than 1 mm toward the knot's side of it, the wound's gap under 2 mm throughout |
| 1.0 | also the stitch carried to the gauze in the forceps and let go there (let go anywhere else after the cut, it stays dropped: it cannot be picked up again), both instruments let go (every fingertip 3 cm from points along them) and resting in their trays (standing in the rest or lying in the tray: the lowest point within 1.5 cm of the tray, the middle inside its outline and off the gauze), every fingertip of both hands 4 cm outside the pad block's outline, all held 2 s |
| 0 | at any moment: the knot pulled 4.5 mm up before the cut (torn through the skin), or any of the cutter's lowest corners (its blunt foot's bottom, the blade's end) more than 2 mm into the silicone anywhere on the skin |

On a real trial: before the trial, the strand is ink-marked where it leaves the skin on both legs. A top camera over a mm scale, both wrist cameras and a side camera at skin level film the trial. Afterwards the cut is measured on the drawn-out stitch with a loupe and a mm scale, from the far leg's ink mark to its cut end, and the pad is checked for any cut in the silicone.

## Building it for a real trial

- **Pad:** a silicone suture practice pad (a Limbs & Things skin pad or the same kind), about 12 x 8 cm, on a 2.2 cm block, taped to the table; a fresh interrupted 3-0 nylon stitch tied by hand for each trial across the incision's middle (4 mm bites, a square knot, 5 mm tails, snug to the skin, the knot on the +y side), ink-marked where it leaves the skin. Replace the pad when its incision tears.
- **Instruments:** 12 cm non-toothed dissecting forceps standing tips-down in a slotted foam rest on a steel tray; a single-use stitch cutter standing hook-down in a foam rest on a second tray.
- **Gauze:** a 7.5 cm gauze square on the table, its corner over the forceps tray's.
- **Safety:** the cutter's hook is sharp; keep hands out of the station while the robot holds it.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| The incision's opening | set from the forceps' pull while they hold the knot: the knot-side half follows their tips' pull away from the wound, up to 4 mm | **Not measured.** Pull the knot away from the wound with a small force gauge and read the gap on the top camera |
| The tear limit | the knot pulled 4.5 mm up before the cut | **Not measured.** Lift the knot of a spare stitch with the forceps until the skin tears, read on the side camera |
| The dig limit | the cutter's lowest corner 2 mm into the silicone | Check the pad for a cut after the trial |
| The forceps' preload | the arms open with 1.6 N at the hand's hold | **Not measured.** A force gauge across the arms where the gripper holds them |
| The stitch's thickness | drawn 0.6 mm; no collider | 3-0 nylon is about 0.25 mm |

## What the simulator leaves out

The stitch is not a thread: it is a rigid strand with no collider, on joints the scorer sets. While the forceps hold the knot (caught as they close on it), the knot's lift follows their tips, and a pull away from the wound opens the incision: its knot-side half follows the tips' pull, set by code. A cut is the leg entering the cutter's notch through its mouth and pressed onto its sharp inner edge, read in the cutter's own frame; the cutter's lowest 6 mm has no collider, so the scorer also requires that the hook reached under the strand level, from beside it, never down through it, and reads the cutter's depth at its lowest corners. Once cut, the stitch's draw follows the forceps' tips; it is not carried: drawn out and let go over the gauze, it is shown lying there. The forceps' arms are a real sprung hinge, closed by the gripper's squeeze.

## Assets

Every shape in the scene (pad, stitch, forceps, cutter, rests, trays, gauze) is built in code from boxes in `task.py`; no third-party meshes, textures or fonts are used. The YAM arms and the station come from `yam-isaac-lab-arena`.

## Licence

MIT
