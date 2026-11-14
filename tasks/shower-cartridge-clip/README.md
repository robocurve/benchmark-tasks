# Push a shower valve's cartridge flush against its O-rings and hold it there while sliding the U-clip down past its ears

A shower valve stands in a cut-out of a wall board, its cartridge back in the bore but resting on its O-rings 15 mm proud. The robot pushes the cartridge home until it is flush and keeps it there, while the U-shaped retainer clip is slid down its slots across the cartridge's two ears.

**What is new here,** compared with other hold-a-part-while-fitting tasks: the part is held home against a drag that pushes it back out (the cartridge's O-rings), and the retainer is a U-clip slid down across the part's ears, which passes only while the part sits fully home and square. It is not holding a sprung drive flat while the other hand turns its latch, nor handing a sprung ball over from a tool into a bore mid-motion.

**Ability tested:** a hold and a fit that depend on each other. The cartridge goes in against its O-rings' drag and, released, creeps back about 1 mm (assumed, not measured: see the calibration table). Its ears must sit fully home in the valve face's notches, and turned true, for the clip's legs to pass in front of them; a cartridge a millimetre proud or a few degrees turned stops the clip on an ear. So the cartridge is held flush until the clip is fully down. Unlike a snap fit, which holds itself once home, the retainer passes only while a held preload keeps the part home: one hand must hold while the other fits. In the demo one hand holds the cartridge and the other fits the clip; the score does not check which hand does what.

**Why it matters:** replacing a leaking single-handle shower cartridge is routine repair work for plumbers and general maintenance workers (O*NET 47-2152: "Repair and maintain plumbing by replacing defective washers, replacing or mending broken pipes, or opening clogged drains"), in apartments, hotels and homes. Moen's own instructions for this cartridge ("1222 Cartridge: Single Handle Shower, PosiTemp", solutions.moen.com) say: "If you encounter any resistance, make sure the cartridge is flush."

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
  --instruction "Push the cartridge home, fit its retainer clip, then let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                              # full score, with a video
python demokit.py task.py OUT --no-record --mode letgo                 # the failure checks below,
python demokit.py task.py OUT --no-record --mode proud                 # each in its own session
python demokit.py task.py OUT --no-record --mode none
```

The demo is a scripted reference. It reads the poses of the cartridge and the clip, which the cameras see; a policy gets only the cameras. The right hand, fingers closed and level, pushes the stem's end face along the bore until the cartridge is flush, and keeps pressing. The left hand takes the clip from its holder by its crossbar, squares it in the pinch by its measured tilt, lowers it pinched until its leg tips are past the ears, lets go, and drives the crossbar down onto the stops with closed fingertips. Then both hands let go.

## Building it for a real trial

- **Valve:** a Moen Posi-Temp valve body (or the same pattern) fixed upright in a short wall section, its bore horizontal and toward the arms, the valve face about 28 cm in front of the arms' bases and 7 cm above the table; copper stubs in the supply and outlet ports.
- **Cartridge:** a Moen 1222 cartridge (105 mm body, 25 mm across), its O-rings greased, set in the bore 15 mm proud with its ears level with their notches.
- **Clip:** the matching retainer clip (Moen 179107 pack; stainless steel, about 1.5 mm thick and 5 g; measure), standing in a small slotted holder at (20, 10) cm: a black plastic block 30 x 60 mm on the table with two 4 mm plates 30 mm tall, 3.2 mm apart.
- **Wall section:** a 12 mm cement board 300 x 360 mm with an opening behind the valve, on a 2x4 timber stud (38 x 89 mm, 400 mm tall) standing on the table behind it, the valve fixed to the stud.

Calibrate before trials:

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| The O-rings' push-in force | 15 N while the cartridge moves in, over its last 15 mm | **Not measured; approximated** (no maker figure). Push a greased cartridge in with a push gauge or luggage scale and read the force |
| The O-rings' spring-back | 1 mm when released short of home | **Not measured; assumed** (no published figure found). On a greased 1222, release the cartridge at flush with no clip and read how far the stem creeps out with a dial gauge: this is why the hold is needed |
| The ears' depth | 17.5 mm off the axis (35 mm across the two), 3 mm under the clip's legs | **Not measured; approximated.** One retailer lists the 1222 at 2-1/8 in (54 mm) across its widest; measure the ears with a caliper and set `EAR` |
| The clip's clearance in its slots | 2.2 mm slots for 2 mm legs (0.2 mm of play) | **Not measured; assumed.** With the clip seated, a feeler gauge between a leg and its slot's face; set `SLOT_W` |
| Clip thickness and mass | 2 mm, 10 g, in 2.2 mm slots | Measure the clip; a real one is about 1.5 mm and 5 g. The simulator's clip is heavier (with 96 solver iterations) because at 5 g its legs let the pulled ears through |

## Safety for a real trial

The valve section is heavy: clamp it to the table. The clip is small and springy: keep the area clear and wear eye protection. Keep an emergency stop in reach.

## Scoring

See `scoring` in `task.json`, and `score`, `success`, `state_now` and `legs` in `task.py`. `python3 check_score.py` runs that scorer on eleven hand-built states (the full result and each lazy route: a clip beside its slots, turned, on the ears, dropped and picked up again) with no simulator. In a real trial:

- **Flush:** the cartridge's ears fully home in their notches (within 0.5 mm) and the cartridge turned true (under 3 degrees), read before any pull. Mark the stem in pencil where it meets the valve face with the cartridge flush; "pushed flush at some moment" (the 0.25 tier) is that mark reaching the face in the trial's video.
- **Clip started:** each of the clip's two legs inside its own slot (within 0.8 mm across the slot and 2 mm along the face) and below the lugs' tops. A clip lowered anywhere else, even at the slots' height, has not started.
- **Clip down:** both legs on their stops in their slots (each tip within 1.5 mm of its stop) and the clip square (its crossbar within 5 degrees of the face's line, upright within 5 degrees).
- **The pull:** a 20 N outward pull on the stem with a push gauge, after the hands are clear: during the pull the cartridge stays within 1 mm of home, its ears held by the clip's legs (the 0.5 mm flush check is the one before the pull).
- **Hands clear:** no gripper within 3 cm of the clip's crossbar, the cartridge's ears or the end of its stem, all of this for 1 s: 1.0.
- **0.5:** the cartridge flush and the clip started in its slots, without the 1.0 result. **0.25:** the cartridge pushed flush at some moment, or the clip started in its slots. **0** otherwise, or if the clip falls to the table or off it at any moment of the trial (its crossbar under 2 cm above the table top), even if it is picked up again.

In the simulator the scorer itself pulls the stem outward with 20 N for the whole hold, and the hold restarts if anything breaks it. Under the pull a clipped cartridge sits a few tenths of a millimetre out (the legs' play in their slots), so the hold asks for 1 mm, after the cartridge has been pushed flush to 0.5 mm. The ears reach 17.5 mm off the axis (35 mm across) and the clip's 4 mm legs run from 14.5 to 18.5 mm (centres 33 mm apart, inner gap 29 mm), so each ear sits 3 mm behind its leg.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | Cartridge pushed flush (+0.3 mm) and held; clip squared and lowered with each leg in its own slot, past the ears, and driven onto both stops (0.0 mm over each, square); under the scorer's 20 N pull the cartridge stays within 1 mm; held 1 s | 1.0, at step 585 (29 s) |
| `letgo` | The cartridge pushed flush and let go before the clip comes: the O-rings push it back 1 mm, and the clip stops on its ears | 0.25 |
| `proud` | The cartridge not pushed (15 mm proud); only the clip is fitted: it goes fully down behind the ears, which stand outside the slots, and then blocks the cartridge from going home | 0.25 |
| `aside` | The cartridge not pushed; the clip lowered 20 mm beside its slots, at their height, and let go: it falls over there, neither leg in a slot | 0.0 |
| `none` | The arms do nothing | 0.0 |
| Dream-1 | One run on the revised scorer, seed 1, 12 actions per call, one session throughout: ran to the 1,200-step limit without pushing the cartridge home or fitting the clip | 0.0 |

No seed is needed: nothing in the scene is random. Runtime per trial, on one shared GPU: the demo's 585 steps took about 10 minutes while recording three cameras (about 1 step per second); a run to the 1,200-step limit takes about 20 minutes.

## Assets

All meshes are made by the task's own code from plain shapes. No third-party assets, no textures. Occupation codes and the task statement quoted above are from the O*NET 30.0 Database by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 license. O*NET is a trademark of USDOL/ETA.

## Licence

MIT
