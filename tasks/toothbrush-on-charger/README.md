# Stand an electric toothbrush on its charger: the handle's hollow base lowered straight over the hidden charging post

An Oral-B electric toothbrush handle lies on its side on the counter. Its charger, a Braun Oral-B Type 3757, has a short post on its top that fits a recess in the handle's base. One gripper picks the handle up across its body, turns it upright, centres its base over the post, lowers it straight down until the base sits on the charger's top with the post inside the recess, and lets go. The handle must then stand on its own: nothing guides it but the post, 10 mm across, inside a recess 11 mm across, and once the handle is close its own base hides the post. A handle let go tilted rests on the post's tip and topples, and a push off-centre slides the light charger across the counter. The other arm is not needed; a person might steady the charger with the other hand, and the task does not ask for it.

What sets it apart from a slot-guided dock is the release: there are no walls. The handle is 232 mm tall and 27 mm across on a post 13 mm tall, so it stands only if it is let go plumb and seated, and it has to stay standing for 2 s with both grippers clear.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,hold,beside,counter,tilted,drop
```

The harness's instruction (Dream-1 run): `--instruction "Stand the electric toothbrush on its charger: pick up the handle, lower its base straight down over the charger's post, and let go so it stands on the charger by itself."`

`TB_CX`, `TB_CY` (default 0.40, -0.02 m) set where the charger stands; `TB_TRACE=1` prints the scorer's terms every second.

**Result** (scripted, GPU physics, 20 Hz): the full mode scores 1.0 at 640 x 360 on an RTX 3090 Ti (404 steps, 20.2 s, against a limit of 1,200; 0 blank frames), and 1.0 at 320 x 176 (402 steps). Every route in the table below ran on this scene on GPU.

**Dream-1 run** (the harness's own policy, one trial, 1,200 steps, one session, valid): 0.

## Scoring, and the routes that score less

The score is read from the state at the end of the run. Only the failures latch; every tier is read from the state now, so a step undone loses its credit.

- **0.33, lifted upright:** the handle's base more than 10 mm above the counter and its axis within 15 degrees of vertical (base down). Real trial: by eye on the video.
- **0.67, on the post:** upright as above, the base's centre within 3 mm of the post's axis and at most 12 mm above the charger's top, so the post is at least 1 mm inside the recess. Real trial: a side photo against a millimetre scale shows the handle centred on the charger with its base no more than 12 mm above the charger's top (the post is hidden; a base that low over the post can only be on it).
- **1.0, standing on the charger:** on the post with its base within 2 mm of the charger's top, tilted under 5 degrees, still, and every part of both grippers (fingertips, pads, palm, housing and wrist link: a lattice 2 cm apart over each gripper from 12 cm behind its frame to the fingertips, plus points along both pads) 20 mm or more from the handle's exact envelope (a capped cylinder 13.5 mm in radius from its base to the head's top; 20 mm is more than the 17 mm a point of the gripper can lie from its nearest lattice point), for 2 s running (a count that restarts if any of these breaks). Real trial: the brush stands on the charger by itself with its base flat on the charger's top, a phone level app on the handle reads under 5 degrees, and the handle's charge light flashes green.
- **Void (0):** the charger pushed more than 20 mm off its mark or tipped over 20 degrees, or the handle dropped after it has been lifted: its base falling faster than 0.3 m/s and more than 0.15 m/s faster than both hands move down, so a release (a free fall of about 5 mm), not a controlled move that carries it down; read after every physics step. Real trial: tape marks round the charger's feet; in the side video, the frame where the fingers open shows the base no more than 5 mm above the charger's top on the mm scale (more is a drop).

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | picks the handle across its body, lifts it, turns it upright, carries it over the charger, levels it by its own tilt, centres its base over the post, lowers it at 15 mm/s until the base sits on the charger, lets go and backs away | 1.0 (404 steps, 20.2 s) |
| hold | as full, but keeps hold once seated | 0.67 (the hands never clear) |
| beside | as full, but lowered onto the charger's top 20 mm beside the post and let go: it stands there | 0.33 (upright off the counter, post not in the recess) |
| counter | stood upright on the counter beside the charger and let go | 0 |
| tilted | as full to over the post, then tilted 12 degrees, lowered onto the post's tip and let go | 0 (the 12 degree tilt, near the wrist's limit, drags the charger 55 mm, and the handle let go on the post's tip topples) |
| drop | as full, but let go with the base 20 mm above the charger's top, centred | 0 (a drop: it lands seated, but its base fell at over 0.3 m/s away from the hands) |
| none | nothing | 0 |

## Sizes

| What | In the scene | Published or estimated | Source |
|---|---|---|---|
| Charger | Braun Oral-B Type 3757: a rounded block 65 x 55 x 33 mm, white, 0.10 kg, free on the counter, friction 0.6 static, 0.5 dynamic (rubber feet) | 65 x 55 x 33 mm | reseller listing of the 3757 (Best Buy Canada); mass and friction estimates |
| Charging post | 10 mm across, 13 mm tall, its top 3 mm tapered to 6 mm across, at the middle of the charger's top | | estimate; no maker's figure found |
| Handle | Oral-B Pro-series handle with its brush head fitted, one rigid body: body 27 mm across and 170 mm long (tapering over its top 15 mm), the head's neck and head to 232 mm, 0.13 kg, centre of mass from uniform density (about 95 mm up), friction 0.6 static, 0.5 dynamic | | estimates; listings give only boxed sizes |
| Recess in the handle's base | 11 mm across (a 16-sided bore, flats 5.49 mm from the axis: 0.5 mm clearance a side round the post), 14 mm deep | | estimate |
| Layout | the charger centred at (0.40, -0.02) m; the handle lying on its side along y, its base's centre at (0.40, -0.32), its head toward the charger (+y, the robot's left), its body's middle about 21 cm right of the charger | | |

Metres; x forward, y to the robot's left, z up; origin midway between the arm bases (0, +/-0.305); the counter is the bench top (z = 0.7475).

## Colliders

Every body is built in `task.py` from prisms and lathed profiles (no third-party meshes).

| Body | Colliders | Free or fixed |
|---|---|---|
| Charger | the rounded block as one convex hull; the post as a second convex hull (its tapered tip included) | free, 0.10 kg |
| Handle | 16 convex ring segments (the base from 0 to 14 mm, round the 16-sided bore); the body from 14 to 170 mm as a 16-sided tapered hull; the neck and the head as two more hulls | free, 0.13 kg |
| Counter | the harness's bench top | fixed |
| Fingers | the harness's YAM finger colliders | the robot |

## Robot figures the demo keeps within

The YAM's jaws open to 94 mm and grip about 40 N a finger; the demo closes on the 27 mm handle with about 15 N a finger, 95 mm up its body, near its centre of mass (0.13 kg x 9.81 x a few mm of offset is far under the grip's 1.2 N m). The right arm alone does the act, its hand within 0.5 m of its base.

## Real trial

Put an Oral-B Pro-series handle with a head fitted and a Braun Oral-B Type 3757 charger, plugged in, on the bench at the positions above; tape round the charger's feet. Lay the handle on its side. Film the side of the charger beside a printed millimetre scale. After the run, look for the handle standing on the charger by itself and its charge light flashing green (the manual: "place the toothbrush handle (c) on the charging unit"), and check the tilt with a phone level app.

| Real-trial kit | Use | Price (about) |
|---|---|---|
| Oral-B Pro 3 (or any Pro-series) electric toothbrush with its 3757 charger | the objects | $50 |
| Phone tripod, printed millimetre scale, masking tape | the side video and marks | $15 |
| Total, with a phone already to hand | | about $65 |

The robot is the program's YAM pair; nothing else is needed.

## Safety for a real trial

The 3757 is a low-voltage inductive charger in a sealed case. Keep its cable and plug away from the arm's path and off the bench's edge, and keep water away from the plug.

## What is modelled, and what is not

- Rigid bodies. The handle is one body with its head fitted; its mass is spread uniformly over its colliders, so its centre of mass (about 95 mm up) may sit a little higher than a real handle's, whose battery and motor sit low.
- Charging is not modelled. A seated handle is read from the geometry: its base on the charger's top with the post inside the recess.
- The demo reads the handle's and the charger's poses to centre the base over the post and to level the handle (a policy would see them from its cameras).
- Unpublished sizes (the post, the recess, the handle's length and mass) are estimates, marked so above.

## Sources

- Braun Type 3757 charger manual (https://www.manualslib.com/manual/20477/Braun-3757.html?page=4): "place the toothbrush handle (c) on the charging unit"; "A full charge takes at least 17 hours".
- Charger size: Best Buy Canada, model 3757 replacement charger listing (65 x 55 x 33 mm).
- UK use: Dentistry.co.uk, 2025-08-07, "Dentistry's guide to electric toothbrushes" (https://dentistry.co.uk/2025/08/07/dentistrys-guide-to-electric-toothbrushes/): 59% of UK adults use an electric toothbrush (OHID survey); Dentistry.co.uk, 2020-06-01 (https://dentistry.co.uk/2020/06/01/adults-electric-toothbrushes-manual/): two in three (Oral Health Foundation and Oral-B).
- US use: YouGov (https://yougov.com/en-us/articles/51427-how-often-do-americans-brush-a-look-at-oral-care-habits): 23%.

Oral-B and Braun are named for identification only; this task is not affiliated with them.

## Licence

MIT
