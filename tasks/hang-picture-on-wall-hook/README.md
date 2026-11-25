# Hang a framed picture on a wall hook by the wire behind it, level, then let go

A framed picture (30 x 40 cm, 0.20 kg poster frame) stands on the table leaning back against a wall panel, just below
one picture hook nailed into the wall. Braided stainless steel picture wire runs between two D-rings on the back of the
frame. The robot is told to hang the picture on the hook by its wire so that it hangs straight, and to let go.

**Ability tested:** lifting and carrying a wide, flat object upright with two hands, then catching a wire that cannot
be seen. Once the frame is near the wall, the wire is hidden behind it (from the top camera and from both wrist
cameras), so the robot has to keep the frame upright and within about a centimetre of the wall, tip its top slightly
toward the wall so the slack wire falls back against it, and lower the frame until the wire drops into the hook's
throat and takes the weight, which it can only feel through the grippers or see as the frame stopping. Then it has to
let go without pulling the wire off the hook, and, if the frame hangs crooked (a wire caught off-centre stays where
friction holds it), push a lower corner until the top rail is level within 1.5 degrees, and park both arms clear.

**Why two arms:** the frame is 30 cm wide and the arms can only pinch its side rails. Held at one side alone, the
frame's weight acts 15 cm from the pinch, which a two-finger pinch on a 22 mm rail cannot resist: the frame swings
round the grip, and in the simulator a one-arm attempt drops it on the table. The middle of the top rail, the only
place one pinch could hold it level, is above where either arm can reach with its gripper pointing down.

## How this differs from existing tasks

- **BEHAVIOR-1K picture-hanging activities** (e.g. `hanging_pictures`) place a rigid picture so that it attaches to a
  wall nail when near the right spot. Here nothing snaps on: a flexible braided wire, free to sag and swing, has to be
  caught blind in a real hook's 9 mm throat behind the frame where no camera sees it, the frame's weight has to be
  handed over to the wire without sliding off, and the result is scored by its measured tilt angle on its own.
- **RLBench `hang_frame_on_hanger` and VLABench `hang_picture_on_specific_nail` / `hammer_nail_and_hang_picture`** use
  one arm to place a rigid frame whose own geometry engages a visible peg or nail. Here the frame hangs by a flexible
  braided wire hidden behind a 30 x 40 cm frame that requires both hands to carry upright, and the score checks the
  wire seated in the hook's throat, top-rail tilt at most 1.5 degrees, bottom rail flush within 1.0 cm, and both
  grippers parked at least 10 cm clear.
- **Other tasks that drape a cable over hooks** hand the robot the cable itself to lay over a visible hook. Here the
  robot never holds the wire: it holds a 30 x 40 cm frame with both hands and the wire follows out of sight behind it.
- **Hanging a small planter by its ring on a wall bracket** lands a visible rigid ring on a visible hook and scores the
  load clear of the table. Here the loop is a flexible braided wire hidden behind a wide frame, caught blind in a flat
  wall hook's throat, and judged on explicit tilt angle and wall alignment.
- **Pegging a hanging photo back onto a display wire** squeezes a clothes peg onto a taut line beside a small print.
  This task has no pegs: it tests bimanual carrying of a framed picture, blind wire-on-hook seating, and angle
  levelling.

## Real-world setup (materials)

- **Wall:** white-painted 12 mm MDF sheet, 90 cm wide x 60 cm tall, standing on the table along y with its front face
  45 cm in front of the arm-base line (`x = 45 cm`), braced behind by two 150 mm steel L-brackets screwed to its back
  and clamped to the table with two F-clamps. Draw a spirit-levelled horizontal pencil line across the panel at 50 cm
  height as an optical level reference.
- **Hook:** one **OOK Professional 30 lb Picture Hanger, part #533024** (brass-finished steel wall hook with blue steel
  nail, 30 lb working load limit; published 32 mm tall x 14 mm wide x 10.4 mm deep; modelled as a 3-box steel hook with
  13 x 32 mm plate, 12 mm lip, 9 mm throat), nailed into the MDF at `y = 0` so the throat seat is **39 cm** above the
  table.
- **Picture:** a black plastic poster frame, 30 x 40 cm outer size (portrait), moulding 25 mm wide x 22 mm deep,
  styrene glazing (no glass, so a drop cannot shatter), hardboard back with a white mat and printed picture inside,
  **0.20 kg** total mass (weigh on a scale; tape flat washers inside the back if lighter).
- **D-rings and purchasable picture wire:**
  - **D-rings:** two **OOK Narrow 1-Hole D-Ring Picture Hangers, part #50206** (zinc-plated steel, 11/16 in / 17.5 mm
    long, **30 lb** rating, fastened with the supplied 1/2 in wood screws), screwed to the back of the side rails
    **10 cm** below the top edge and **2 cm** in from each outer side edge (wire eye ~6 mm behind the back face).
  - **Picture wire:** **OOK DuraSteel #3 Braided Stainless Steel Picture Hanging Wire, part #50112** (Hillman ref
    **#534616**; 7-strand braided stainless steel, ~0.8 mm diameter, **20 lb** working load limit, sold in a 9 ft
    coil). Cut ~40 cm and tie between the two D-rings so the active span between the rings is **27 cm**, threading each
    end through its D-ring and wrapping 5 tight turns back onto the standing part. Hung on the hook, the wire's V-peak
    sits about 6.4 cm (~6.1 cm including ring give) below the top of the frame.
- **Start:** the frame stands on its bottom edge on the table, centred at `y = 0`, face to the robot, leaning back at
  **25 degrees** with its top against the panel just below the hook (bottom-back edge at `x = 28.0 cm`, bottom-front
  edge at `x = 26.0 cm`, marked with tape on the table), the wire hanging in a loose V behind it. Both arms start in
  the ready pose, grippers open. Layout is fixed (`object_jitter=0`).

## Run a trial

Requires `yam-isaac-lab-arena` on Isaac Lab-Arena 0.3.1 (`cablekit.py` and `cableshape.py` in this folder). Trial
limit: **900 steps (45 seconds at 20 Hz)**.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 900 \
  --instruction "Hang the picture on the wall hook by its wire so it hangs straight, then let go."
```

## Real-trial scoring

Tools: the trial video, a millimetre ruler, and a digital angle gauge / torpedo level (0.1 degree resolution, e.g.
Klein Tools 935DAG) or smartphone inclinometer zeroed on the bench and held lightly along the top rail without bearing
weight on the frame (cross-checked on a front photo against the spirit-levelled line on the panel). Judged at the end
of the trial once the picture and both arms have been at rest for **2 s** (or at the 45 s limit).

- **0** if the frame touches or rests on the table (lowest corner within **5 mm** of the table), if it does not hang
  from the hook's throat by its wire alone with both wire ends attached to their D-rings (checked behind the frame
  afterwards), if the picture does not face forward, if any part of either gripper (TCP or finger pads) is within
  **5 mm** of the frame, or if the top-rail tilt about the wall normal exceeds **15.0 degrees** (e.g. sideways, upside
  down, or hung by one corner).
- **1.0** when it hangs from the hook by its wire facing forward, the top rail's tilt angle from horizontal about the
  wall normal is **at most 1.5 degrees** (`<= 1.5 deg`; ~0.79 cm height difference across the 30 cm top rail), the
  bottom rail's back edge is within **1.0 cm** of the panel at both corners (ruler), both grippers are at least
  **10 cm** clear of the frame, and both the frame and both arms are at rest (held 2 s to end early).
- **0.5** when it hangs from the hook by its wire facing forward, clear of the table (`> 5 mm`) and grippers
  (`>= 5 mm`) with top-rail tilt at most `15.0 deg`, but is crooked (**over 1.5 degrees and up to 15.0 degrees**),
  stands more than **1.0 cm** off the panel at the bottom rail, has a gripper between **5 mm and 10 cm** from the
  frame, or is still moving at the time limit. Example 0.5 end states: wire caught 2 cm off-centre so the frame hangs
  at about 8 degrees tilt; hung level within 1.5 degrees with a gripper parked 4 cm away.

## Fidelity

The frame is one rigid box with the real outer dimensions (30 x 40 x 2.2 cm) and 0.20 kg mass (linear damping 0.2,
angular damping 1.0 for air and joint drag); no grasp weld or proximity latch is used anywhere. The wire is a chain of
thirteen 2 cm rigid segments with bending springs (`k = 0.0005`), no stretch or axial twist; for reliable collision on
the 3 mm hook seat without tunnelling, it is 3 mm in diameter and 60 g (the real #3 7-strand wire is ~0.8 mm and a few
grams). Each wrapped wire end is a 6 mm, 30 g stub held at its D-ring (6 mm behind the back face) by a symmetric
1,500 N/m spring (`c = 3.0`) that gives ~3 mm under the hung load at both ends. The hook is a fixed 3-box steel shape
(13 x 32 mm plate, 12 mm lip, 9 mm throat; 30 lb nail never pulls out) with static and dynamic friction 0.4, so an
off-centre catch holds the frame crooked just as braided wire does on a steel hook. Panel and table are rigid.

**Simulated scorer and quaternion conversion.** The scorer reads the frame's orientation from the embodiment's
`ObjectState.quat_xyzw`, which `yam_arena` (Isaac Lab 3.0) populates in `(x, y, z, w)` order, normalises it and
converts it to a rotation matrix `R` expecting `(x, y, z, w)`, then computes the top-rail tilt about the wall normal as
`degrees(atan2(|R[2, 1]|, R[1, 1]))`. Verified live at reset on the 25-degree start lean about `+y`, both `quat_xyzw`
and `root_quat_w` read `(0.0, 0.2171, 0.0, 0.9761) = (0, sin 12.5 deg, 0, cos 12.5 deg)`, reading `0.00 deg` top-rail
tilt. Unit-tested offline across 22 poses: upright `0.00 deg`, `+1.00 deg`, and boundary `±1.50 deg` score `1.0`;
`+1.60 deg`, `+5.00 deg`, `-8.40 deg` (2 cm off-centre), `+14.50 deg`, bottom rail `2.0 cm` off the wall, gripper
`4.0 cm` away, and a still-swinging frame score `0.5`; `+16.00 deg`, `90 deg`, `180 deg`, facing the wall, resting on
the table, unhooked wire, detached D-ring, gripper within `2 mm`, and missing robot state score `0.0`.

## Demo video

The scripted demo pinches both side rails physically, lifts the frame upright, carries it close to the panel with its
top tipped slightly in so the hidden wire hangs against the wall, lowers it until the wire seats in the hook's throat,
opens both grippers, and parks both arms well clear (`> 14 cm` from the frame) for over 2 s until the episode ends on
the 2 s resting hold within the 900-step limit.
