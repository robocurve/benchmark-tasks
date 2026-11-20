# Hang a wall clock back on its screw by the keyhole hanger hidden behind it

A 12 in round wall clock stands on its rim on a console table, its back to the wall, after its battery was changed. Above
it a #8 pan-head screw stands 5 mm proud of the wall, where the clock hung. The instruction is:

> **"Hang the clock on its screw."**

The files are `task.py` (scene, scorer and scripted demo) and its helper modules: `yamkit.py` spawns the modeled props,
`gripkit.py` gives the finger pads their rubber friction, and `yamgeom.py` holds the arm links' collision hulls for the
contact checks. The embodiment is `yam_arena`, scene `station`.

## Why it matters (Societal relatability)

A wall clock comes off its screw for each battery change and, in most of the US, twice a year to reset it for daylight
saving time. Hanging it back is a blind job. The hanger is on the clock's back and the screw is behind the clock, so neither can
be seen while you do it. People hold the clock flat on the wall, press it on the screw's head, slide it until the head
drops through the hanger's round hole, then let it down so the shank rides up the slot into its narrow end. Set it
against the wall, or rest it on the screw's head, and it falls when you let go.

The skill is finding a hidden fixing by feel and locking onto it with a slide. Hanging a poster or a picture by its
wire, D-ring or sawtooth hanger on a nail or hook is a different job: the wire or ring catches anywhere over the hook
across centimeters, and gravity seats it. A keyhole must put the 7.9 mm head through an 11 mm hole within about 1.5 mm,
blind, then slide the clock at right angles to that push so the plate traps the head. Miss the hole by 2 mm and it
cannot engage.

## The scene

| Part | Model | Source |
|---|---|---|
| Clock | Amazon Basics Classic 12 in round wall clock, black: 305 mm across (12 in), 44 mm deep (1.75 in), 0.64 kg (1.4 lb) | the product listing (Sources) |
| Frame ring | 12 mm wide, standing 12 mm in front of the lens | estimated from maker photos |
| Keyhole hanger | 1 mm steel plate flush in the back near the top; a round hole 11 mm across; a slot 5 mm wide whose narrow end is 11 mm above the hole's center and 35 mm below the clock's top edge; a 6 mm pocket behind the plate for the screw's head | estimated from maker photos of such hangers, sized for a #8 screw |
| Screw | #8 pan-head screw: head 7.9 mm across and 2.9 mm high, shank 4.17 mm; level, its head's face 5 mm proud of a painted plywood wall, its axis 31.75 cm above the table | ASME B18.6.4 (#8 recessed pan head) |
| Start | the clock upright on its rim on the table under the screw, its back 3 mm off the wall; arms at the ready pose, grippers open | |

The head fits only the hole; the shank fits the slot. Every hold is contact. The plate is built from convex pieces around
the keyhole, the screw is a fixed head and shank, and the clock is a free rigid body at its listed mass. Nothing
captures, snaps or pulls the clock onto the screw: if the head is not in the hole, the clock slides down the wall.
Friction (static / dynamic, model values): steel on steel 0.45 / 0.40, the clock's plastic on painted wood 0.40 / 0.35,
the YAM's rubber pads 1.0 / 0.8. The clock's movement is a lumped mass a little below the middle.

## Scoring

`task_score` reads `score()`; `success()` is true on each step that counts toward the 2 s hold. The score is taken at
the end of the run, except the fall check, which is latched.

| Score | Condition |
|---|---|
| 1.0 | Hung: the screw's head behind the hanger plate, the shank in the slot's narrow end (within 3 mm of it), the clock at least 1 cm clear of the table, every arm link at least 5 mm from the clock, and the clock still (under 1 cm/s) for 2 s. |
| 0.5 | At the end, the head through the hanger (in the hole or the slot), but the clock not hanging free for 2 s (still held, or held with the head in the hole). |
| 0.25 | At the end, the clock lifted with its back within about 13 mm of the wall (1 cm past the end of the screw's shank) over the screw, the head not through. |
| 0 | Anything else, or, at any moment, the clock's face tipped more than 30 degrees off the wall's plane while no arm link touches it (it tipped over with the hands off it; latched). |

Why each threshold:
- **Head behind the plate:** the head's middle deeper into the clock than the plate's front face, inside the keyhole's
  outline. A clock resting on the screw's head (not through) fails this, so balancing it on the head scores at most 0.25.
- **Within 3 mm of the narrow end:** the shank at least 8 mm up the slot from the hole's center. The head can only
  come back out through the hole within about 1.5 mm of the hole's center, so a clock in the narrow end is locked on.
- **1 cm off the table:** the clock must hang from the screw, not stand on the table.
- **5 mm clear and still for 2 s:** the clock stays up on its own. A link closer than 1 mm counts as touching.
- **30 degrees, unheld:** a clock that falls off the wall tips face out or face down. Tilting the clock while holding it
  is allowed, and a clock that slips down onto its rim upright can be picked up and hung again. Hanging off level is not
  penalized: on one screw the clock swings plumb by itself.

Real trial, judged from the video: the clock hangs on the screw and stays up with the robot's hands off it for 2 s
(1.0); the clock is on the screw but the hands still hold it (0.5); the clock is held flat on the wall over the screw
(0.25); otherwise, or if it tipped over with the hands off it, 0. The head is hidden, so 0.5 and 0.25 are told apart at
the end of the trial: with a hand ready under the clock, the operator pulls its top gently straight out from the wall.
Caught on the screw: 0.5; comes away: 0.25.

## Failure table

Each mode is a scripted demo variant (`CLOCK_VARIANT=<mode>` in the environment) and an offline scorer test.

| Mode | What it does | Score in sim |
|---|---|---|
| Full demo | `CLOCK_VARIANT` unset | 1.0 |
| Do nothing | `nothing`: both arms stay home | 0 |
| Never let go | `hold`: hangs it, keeps gripping it | 0.5 |
| Head in the wide end only | `partial`: presses it flat with the head in the hole, never slides it down, keeps holding | 0.5 |
| Head beside the hole | `miss`: aims 25 mm to the side, slides it down and lets go | 0 |
| Rested on the screw's head | offline test | at most 0.25 |
| Tipped over, face out or face down, with no hand on it | offline test | 0 (latched) |

## Results

- Scripted demo: 1.0 (637 steps; the head drops through the hole at about step 440, the clock hangs in the narrow end by about step 490, the hands are clear by step 600). Two further runs of the same task.py gave the same score and step count.
- Dream-1 run: 0 (2,400 steps).

## Run a trial

From the repository root, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/rehang-wall-clock-on-its-screw/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the clock on its screw." \
  --scorer task_score --max-steps 800 --store-frames --log-dir clock-demo
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`. Video: `inspect-robots video <log>.json`.
A failure mode: prefix the command with `CLOCK_VARIANT=<mode>`.

## Runtime and seeds

The demo takes about 640 steps (32 simulated seconds); the model run limit is 2,400 steps (2 minutes
at 20 Hz). The layout is fixed (`object_jitter=0`); the scripted demo is open loop for it and never reads the clock's
pose. It grips the frame ring at 3 and 9 o'clock and aims the hands 22 mm higher while they carry the clock, because the
arms sag under its weight.

## Real-world build

A low-cost setup: the clock as sold, a #8 x 1-1/4 in pan-head wood screw, and a painted plywood panel (90 x 70 cm,
18 mm thick) clamped upright at the back of a table. Drive the screw level at the height in `task.json`, leaving its
head's face 5 mm proud (a 5 mm spacer under the head while driving sets it), and check by hand that the clock's hanger
slides onto it. Mark the clock's start spot on the table with tape. Reset: lift the clock off its screw and stand it on
its rim on the mark, back to the wall; under a minute. A clock that misses the screw drops a few centimeters onto the table.

## Sources

- Amazon Basics Classic 12 in round wall clock, product listing (ASIN B07PXWTYWL): 12 in across, 1.75 in deep, 1.4 lb,
  one AA battery. https://www.amazon.com/dp/B07PXWTYWL
- ASME B18.6.4, tapping screws: #8 recessed pan head 0.311 in across and 0.115 in high; #8 screw diameter 0.164 in.

## License

Same as the repository.
