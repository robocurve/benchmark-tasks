# Pull the hot dog past its hold time off a running roller grill with spring tongs and drop it in the discard pan

An Avantco RG1830NS roller grill runs on the counter, its 11 rollers turning. Twelve franks lie in four of its grooves; a red hold-time tag on the front rail marks the darker one at the right end of groove 2 as past its hold time. Spring tongs lie on a folded bar towel beside the grill, and a 1/3-size stainless pan stands between them and the grill as the discard pan. One hand takes the tongs, closes their tips on the tagged frank at its middle while the rollers keep it turning, lifts it straight up out of its groove and lets it go into the pan; the demo then lays the tongs back on the towel, which is not scored.

In the demo the right hand comes down over the tongs, its fingers tilted 25 degrees forward and either side of the arms 11 cm from the hinge, and squeezes them shut. It lifts them and turns its wrist so the tongs reach down 50 degrees along the tagged frank's axis, eases the squeeze so the tips spring 3.4 cm apart, lowers them beside the frank until their lower edges are 1.5 mm below its middle, steered by the tongs' own pose, and closes them on it with about 25 N a finger against the tongs' spring. It lifts the frank 8 cm straight up, carries it over the pan, turning the tongs 30 degrees on the way, lowers it, and eases the squeeze, so the tips spring apart and the frank drops in. It lifts the tongs out, squeezes them shut, lays them back on the towel and lets go, so they spring open where they lie.

The other hand has no part in the act: a person does it with one hand on the tongs. The task tests the tool-held pinch, not two-handed work.

**Why it matters.** Societal relatability: most people have lifted hot food off a grill with tongs, a sausage off a home barbecue, a chop out of a grill pan, food from a buffet tray, or a hot dog off a roller grill at a petrol station counter, where customers often use the tongs themselves. The grip is the same each time: the tips close on round food that will not stay put and lift it clear without dropping it. On a roller grill the rollers "continually rotate" (NACS Magazine, June 2024, "Let It Roll"), so the frank keeps turning as the tips close; roller grills stand in "well over two-thirds of all c-stores" in the US (supporting colour only). A convenience-store roller grill sanitation audit asks that "Food-contact utensils, tongs, and serving tools are clean" and that held items are discarded at the required interval, which is why a frank past its hold time is pulled with tongs.

**Ability tested:** a tool-held pinch of a round item that a moving surface keeps turning: the tong tips must close on the frank no higher than its middle, where it sits between two driven rollers, then lift it straight up and drop it in the pan.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 \
  --instruction "Use the tongs to take the tagged hot dog off the roller grill and drop it in the discard pan."
```

## Scripted demo

The demo stays inside the YAM's figures: it squeezes with about 25 N a finger (the YAM grips with about 40 N), carries 167 g (payload 1.5 kg), and works within 0.5 m of the right arm's base (reach about 0.6 m).

From this folder:

```bash
python demokit.py task.py OUT --mode full                    # full score, with a video
python demokit.py task.py OUT --no-record --mode none,gripper_direct,high_pinch,push_off,push,scoop,wrong_frank,hover,undo,regrip,no_release,scoop_pinch,tongs_in_pan,set_back_held,top_touch,cradle,two_hand_shadow,release_touching,drag_pinched   # the check runs below
python demokit.py task.py OUT --no-record --no-cameras --mode full --jitter 0.005 --seed 1   # a jittered run (seeds 1 to 4)
```

The demo reads the tongs' pose and joint to take them and to steer the tips (the hand's own pose does not show where the tips are once the tongs have shifted in the grip), and the tagged frank's pose to aim at it. It reads no contact force and moves nothing except through the gripper.

## The scene and what is simplified

Every part is at true size. Positions are in cm from the midpoint between the arm bases (x forward, y left), heights above the counter, which is the station table.

| Part | In the scene | Source |
|---|---|---|
| Roller grill | Avantco RG1830NS: 58.1 cm long (y -18 to 40.1), 46.4 cm deep (x 26 to 72.4), 22.2 cm tall, 11 rollers, 30 hot dogs | Avantco spec sheet |
| Rollers | 25 mm across, 49.6 cm long, on a 3.4 cm pitch, the front axis at x 29, tops 19 cm up; each turns at 3 rpm, its top moving toward the back | estimates (the sheet gives none); ranges 25 to 28 mm, 3.2 to 3.5 cm, 17 to 21 cm, 2 to 6 rpm |
| Franks | 12, each 15 cm long and 2.2 cm across, 57 g (8 to the pound), in grooves 2, 4, 6 and 8 at y -7, 9.5 and 26 | standard beef franks (estimate) |
| Tagged frank | groove 2 (x 34.1), y -7, darker; a red polypropylene clip tag, 50 x 38 mm, on the front rail in line with it | |
| Tongs | 9 in (22.9 cm) stainless utility tongs, two 20 mm arms on a sprung hinge, tips 25 mm wide with round ends, opening to 7 cm, 110 g; spring about 4 N at the tips held 5 cm apart, 8 N closed on a frank | tong length from the 9 in size; spring an estimate, checked with a luggage scale |
| Towel | a folded cotton bar towel, 30 x 10 cm and 1 cm thick, at x 27 to 57, y -51 to -41; the tongs lie on it on their arms' edges, the hinge at (53, -46), tips toward the robot | |
| Discard pan | a 1/3-size stainless steam-table pan, 32.4 x 17.5 cm and 10.2 cm deep (12 3/4 x 6 7/8 x 4 in), 0.5 kg, centred at (44, -30), long side along x | standard pan size |

The towel and tongs lie on the counter just right of the discard pan, on its side away from the grill. In the setup view the right arm, at its ready pose, hangs over them and hides them.

Size check: `python -c "import task; [print(r) for r in task.size_check()]"` prints each key size in the scene beside the published one (all match; see `size_check()`).

Departures, each stated once:
- Franks are rigid capsules (real: slightly soft); their own turning on the rollers is real friction from the driven rollers.
- The rollers are joints driven at a set speed; there is no heat. A real trial runs the grill switched on and unheated, so the rollers turn.
- The tong arms are 2.5 mm thick (real about 1 mm), for stable contact; their tips are flat plates with round ends (real: scalloped; with a soft frank, real tips may allow a slightly wider pinch height than the simulation's, an estimate). The arms do not collide with each other, so a joint stop stands in for the tips meeting.
- The towel is a rigid pad.
- Physics: rigid bodies at 240 Hz on the GPU.

## Scoring

Three tiers, one per key substep, and one failure that voids the run. Terms, each with its real-trial check:
- **Tongs picked up:** a fingertip within 15 mm of them and they are 5 cm or more above where they lay (real: the video).
- **Closed on:** both tong tips squeezing the tagged frank, in contact with it on opposite sides of its axis, one frank diameter apart, anywhere on the flat tips (in the simulation, each tip within 1 mm of its surface and the two contacts at least 154 degrees apart round the axis; real trial: the front-right video shows both tips on its sides), and no gripper finger within 6 mm of the frank at any time it counts as carried. A frank held by the gripper, or lying across the tips' edges in a cradle, is not closed on.
- **Lifted:** its centre 4 cm or more above where it lay (real: the video, against the 3.4 cm roller pitch behind it).
- **Carried to the pan:** it was lifted in the closed tips and held in them until it was over the pan, the tips never off it for more than 0.5 s; a regrip starts a new carry. A frank set down off the pan and left is no longer being carried, so one pushed or scooped in afterwards is not (real: the video).
- **In the pan:** its centre over the pan's floor and below the rim, no part of the tongs (both arms' centre line and edges, tips, hinge and ring) within 5 mm of it, and it moving slower than 5 mm/s (real: a photo into the pan; the video, by eye).
- **Dropped:** let go and falling. The simulation reads the frank's centre at every physics step (240 Hz), from the moment nothing holds it up (it speeds up downward at 0.6 g or more for 3 physics steps) to its landing (it loses over half its downward speed in one physics step): the height it lost in free flight, plus the drop its downward speed at that contact loss is worth (v²/2g), so a throw gains nothing. Real: the video, the height it was let go above where it lands.

| Score | Condition, read from the state now |
|---|---|
| 0.33 | the tongs are picked up |
| 0.67 | the tongs are closed on the tagged frank and it is lifted (or it was carried to the pan and lies in it, whether or not a tong still touches it) |
| 1.0 | the frank was carried to the pan and lies in it, no part of the tongs touching it, still, held for 1 s |

The one failure: the tagged frank dropped 3 cm or more anywhere but into the pan scores 0 for the whole run. Free: regrips, retries, time, the other franks, the rollers and what the tongs do after the drop. So a frank pushed, scooped or batted into the pan, or set back on the grill and pushed in, never earns the last two tiers. Possible scores: 0, 0.33, 0.67, 1.0.

## Building it for a real trial

Real trial (about US$355): the RG1830NS (about $330), 9 in utility tongs ($5), a 1/3-size pan ($10), a bar towel ($3), and for each trial a fresh standard frank from the refrigerator (4 C), handled with gloves and discarded after, or a 2.2 cm hardwood dowel 15 cm long, sanded, finished with food-grade mineral oil and weighted to 57 g. Before the trials, measure the roller diameter and pitch with a caliper, the roller speed by timing 10 turns of a taped roller, and the tongs' spring with a luggage scale at the tips. Run the grill switched on with both heat zones at off, so the rollers turn cold; check that they turn before the first trial. If a unit turns its rollers only with heat on, set the front zone to its lowest, use the dowel, and fit heat-resistant silicone finger sleeves (about $8) over the gripper pads; the gripper touches only the tongs' hinge half, well away from the rollers. Reset, about 2 minutes: discard the frank in the pan (or wipe the dowel); lay a fresh one in groove 2 at y -7 by a tape mark on the front rail; return any moved frank to its rail mark; lay the tongs on the towel with the hinge on its tape mark; send the arms to their ready pose. Film one 60 fps phone video from the front-right, at rail height, about 60 cm away, framing the right half of the grill, the pan and the towel: it sees the tips close, the lift, the lift against the 3.4 cm roller pitch, the pan and the last 1 s.

## Results

Run on GPU physics, at the README's commands. The check runs and the tongs drops are at 320 x 176 on lab PCs with an RTX 4070 Ti SUPER or RTX 3090 Ti. A fall is read from the moment nothing holds the part up to its landing, at the point of the tongs that fell furthest (a tip end, as the tongs tilt).

| Run | What it does | Score |
|---|---|---|
| Dream-1 | the README command with the Dream-1 policy, 320 x 176, 13 rows a request, RTX 5090 | 0.0 (1,800 steps, the step limit), run on the final code 52be54e63, 11-20 |
| `full` (the film) | the whole task, 640 x 360, RTX 5090 | 1.0, at step 492 (24.6 s) |
| `none` | nothing done | 0.0 |
| `gripper_direct` | the bare gripper takes the frank | 0.0 |
| `high_pinch` | the tips closed 10 mm higher, on the frank's upper half: it rolls out of them and stays on the grill | 0.0 |
| `wrong_frank` | the frank beside it in groove 2 taken instead | 0.0 |
| `hover` | the open tips held round the frank, never closed | 0.33 (tongs picked up) |
| `top_touch` | the tips held narrower than the frank, resting on its top | 0.33 (not closed on it) |
| `push_off` | the shut tips push the frank toward the grill's front | 0.33 |
| `push` | the shut tips push the frank along its groove toward the pan | 0.33 |
| `scoop` | the shut tips sweep the frank up and over the pan, never closing on it | 0.33 (the frank never reached the pan) |
| `undo` | lifted, then set back in its groove and let go | 0.33 |
| `set_back_held` | lifted 8 cm, set back in its groove, the tips kept closed on it to the end | 0.33 (not lifted, not in the pan) |
| `no_release` | held in the tongs over the pan to the end | 0.33 (below 4 cm above where it lay, tongs on it) |
| `scoop_pinch` | the frank carried over the pan at under 4 cm and let go there (no carry), then the tips closed on it in the pan and lifted it a little | 0.33 |
| `tongs_in_pan` | the frank let go in the pan, then the tongs held over it and lowered | 1.0 at step 460: the frank was in the pan, clear and still for 1 s before the tongs came down, and the run ends at the first 1.0 |
| `cradle` | the tips 1.6 cm apart, swept under the frank's side and over the pan, to rest it on their edges | 0.33. In this run the tips missed the frank (the right arm's IK was 32 mm off), so it was never cradled; the scorer's squeeze test fails a cradle's contacts by their numbers (about 94 degrees apart round the axis, against the 154 needed) |
| `two_hand_shadow` | the right hand holds the tips touching the frank, no squeeze; the left hand takes the frank bare and carries it as far as it reaches | 0.33. The left arm cannot reach the frank's place or the pan (IK 190 to 270 mm off), so a bare-handed carry to the pan is out of its reach; the scorer also ends a carry whenever a gripper finger is within 6 mm of the frank |
| `release_touching` | the frank carried to the pan, set on its floor and the tips eased to a touch, left on it | 0.67 (in the pan, a tip still touching) |
| `drag_pinched` | lifted 8 cm, set back in its groove pinched, then lifted 4.5 cm and carried over the housing into the pan | 1.0 (it was lifted and carried in the tips: correct) |
| `regrip` | lifted, set back in its groove and let go for 1 s, then closed on again and carried on as in `full` | 1.0 (630 steps) |

The check runs are at 320 x 176. The film has no dark frames at the start of any camera, and `demo.mp4` is written with faststart. A frank in the pan scores only if it was carried there in the closed tips: a frank set down off the pan and left for more than 0.5 s is no longer being carried, while a regrip that closes on it again and carries it on scores in full.

## Runtime and seeds

The demo takes 24.6 s of simulated time (492 steps) of the 1,800-step limit. At 640 x 360 the slowest 30 steps took 10.1 s (about 0.34 s a step), so a policy served 13 rows per request stays under 5 s a request. With `object_jitter=0` nothing is random, so no seed is needed; the jittered runs pass `--seed`.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Grill: Avantco RG1830NS 30 hot dog roller grill with 11 non-stick rollers, spec sheet (https://www.webstaurantstore.com/documents/specsheets/avantco_equipment_hd-g11-n.pdf) and product page (https://www.webstaurantstore.com/avantco-rg1830ns-30-hot-dog-roller-grill-with-11-non-stick-rollers-120v-910w/177RG1830NS.html): "They continually rotate to make sure that your foods are evenly cooked from all sides."
- Workers: BLS OEWS May 2023, NAICS 457100 Gasoline Stations (https://www.bls.gov/oes/2023/may/naics4_457100.htm). O*NET 35-3023.00, Fast Food and Counter Workers (https://www.onetonline.org/link/details/35-3023.00). This includes information from O*NET OnLine by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence; O*NET is a trademark of USDOL/ETA.
- Roller grills in c-stores and hold time: NACS Magazine, June 2024, "Let It Roll" (https://nacsmagazine.com/issues/june-2024/let-it-roll). Store count and jobs per store: NACS, "U.S. Convenience In-Store Sales Top $340 Billion", 15 April 2026 (https://www.convenience.org/stay-current/press-releases/2026-press-releases/u-s-convenience-in-store-sales-top-$340-billion).
- Procedure: convenience store roller grill sanitation audit (https://www.mangoapps.com/templates/inspections/convenience-store-roller-grill-sanitation-audit).
- Pan: the standard 1/3-size steam-table pan, 12 3/4 x 6 7/8 in, 4 in deep.
- Avantco is named here for identification only; the name is its owner's trademark.

## Licence

MIT
