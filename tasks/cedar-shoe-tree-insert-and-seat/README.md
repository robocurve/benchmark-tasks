# Hold an oxford shoe steady on an entry mat while inserting and seating a spring-loaded cedar shoe tree

Instruction: "Seat the twin-tube cedar shoe tree inside the oxford shoe on the entry mat, keeping the shoe upright on the mat and leaving both grippers clear."

Files in this folder: `task.json`, `cedar_shoe_tree_insert_and_seat.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_cedar_shoe_tree_insert_and_seat.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.
Task file SHA-256 (first 12): see freeze log.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_cedar_shoe_tree_insert_and_seat.py` with `yam_arena` on `PYTHONPATH`: verifies initial spawn state scores 0.0, all documented stateless scoring tiers (`0.0`, `0.35`, `0.70`, `1.0`) and post-seating undo drop, gripper proximity hover cap (`0.70`), displaced/one-hand shoe slide failure (`0.0`), sub-gate ablations, shortcuts scoring `0.0`, top-camera sight-line clearance, scoring numeric parity, demo purity, and kinematic reachability.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=cedar_shoe_tree_insert_and_seat.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Seat the twin-tube cedar shoe tree inside the oxford shoe on the entry mat, keeping the shoe upright on the mat and leaving both grippers clear." \
  --scorer task_score --max-steps 640 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=cedar_shoe_tree_insert_and_seat.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Seat the twin-tube cedar shoe tree inside the oxford shoe on the entry mat, keeping the shoe upright on the mat and leaving both grippers clear." \
  --scorer task_score --max-steps 640 --seed 0
```
`demo()` reads fixed scene geometry constants only (never live simulator state). One arm clamps the 50 mm tapered cap-toe nose ahead of the 68 mm brogue shoulders to counter-brace the unbolted 380 g oxford shoe on the mat while the other arm pinches the 28 mm cedar handle crest for 1.0 s, lifts the 320 g twin-tube shoe tree clear, threads the contoured forepart diagonally (-12 deg) under the 62 mm cap-toe ceiling to the +139 mm toe stop, compresses the 250.0 N/m twin-tube spring past 28 mm against the braced shoe so the 54 x 44 x 32 mm heel block clears the -131 mm collar lip, drops the heel onto the 16 mm insole, eases aft into an ~18 mm (~4.5 N) locked preload beneath the 52 mm collar undercut, and retracts both grippers to park for the 3 s hold. Open-loop plan duration is 20.7 s, finishing early at ~19.8 s once 1.0 holds for 3 s. Episode `run_limit` is 640 steps (32 s at 20 Hz).

## Scoring
Judged statelessly at the end of the trial from pure observable end-state geometry (verified in real trials with a caliper/ruler and digital protractor); succeeds once 1.0 holds for 3 s:
- `1.0`: both shoe and shoe tree inside the entry mat (20 mm edge margin); shoe upright within 10 deg, within 8 mm vertically of the mat surface and 65 mm horizontally of its start mark; cedar forepart tip >= 122 mm forward in the shoe cavity (within 18 mm laterally, 10 to 30 mm above outsole bottom, aligned <= 12 deg, roll <= 15 deg); heel block rear base 125 to 144 mm aft in the heel counter pocket (within 18 mm laterally, 10 to 30 mm above outsole bottom, beneath the 52 mm collar lip) with 8 to 24 mm active spring compression; both grippers' TCPs and pad tips >= 80 mm clear of shoe and shoe tree.
- `0.70`: fully seated and preloaded on the mat, gripper within 80 mm.
- `0.35`: shoe upright on the mat and forepart seated in the vamp, but heel unseated/cocked on collar rim or spring preload outside 8 to 24 mm.
- `0.0`: otherwise (noop, hover, 1-hand unbraced push, sweep, or off-mat drop).
Possible scores: `0.0`, `0.35`, `0.70`, `1.0`.

## Runtime per trial
Runtime per trial: 640 control steps at 20 Hz (32 s of simulated time, about 3 min wall clock on one A10 GPU); scripted demo completes once the 3 s clear hold finishes (~395 steps / 19.8 s).

## Seeds
Seeds: 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Compress the cedar shoe tree forward by its upright crest, lift the heel block out from beneath the 52 mm heel collar lip, withdraw the forepart from the vamp, and set the oxford shoe at (0.250, 0.042) m and the uncompressed shoe tree at (0.370, 0.005) m on their chalked marks on the entry mat.

## Mechanism and sources
| part | real item | model | precedent |
|---|---|---|---|
| entry mat | 360 x 560 x 5 mm entryway shoe tray (~$6 retail reference: https://www.amazon.com/dp/B07P6X8L8R) | fixed 11-geom compound mat with 2.5 mm perimeter curb, drainage ribs, and corner bosses (`mu = 0.12`) | station bench mat |
| cap-toe oxford shoe | single men's size-10 leather cap-toe oxford, 305 x 108 mm, 380 g (~$22 single/half-pair reference: https://www.amazon.com/dp/B07B4J7K9D) | 39-geom loose rigid body (`380 g`, `mu = 0.12`): stacked outsole, welt, 16 mm insole, 50 mm cap-toe nose, 68 mm brogue shoulders, 4-lace saddle, 66 mm pocket, 8 mm collar undercut | unbolted hollow receptacle |
| twin-tube cedar shoe tree | men's red cedar twin-tube spring shoe tree, 292 mm, 320 g (~$24 retail reference: https://www.amazon.com/dp/B000HJBXMM) | 19-geom 2-body articulation (`145 g` contoured forepart + `175 g` rounded heel slide) on 42 mm prismatic `spring_tube` (`250.0 N/m`, `5.5 N` at `22 mm`) | spring-preloaded internal lock |

## Real-world build
Parts: men's twin-tube aromatic red cedar shoe tree (~$24, https://www.amazon.com/dp/B000HJBXMM) + single men's size-10 cap-toe leather oxford dress shoe (~$22, https://www.amazon.com/dp/B07B4J7K9D) + 360 x 560 x 5 mm dark entryway shoe mat (~$6, https://www.amazon.com/dp/B07P6X8L8R; ~$52 total BOM). Chalk start marks for the shoe at `(0.250, 0.042) m` and the shoe tree at `(0.370, 0.005) m` relative to the robot station origin; verify seated forepart depth, heel-pocket seating beneath the collar lip, and active tube-spring travel with a caliper/ruler and digital protractor.

## Fidelity
Rigid-body simulation (Isaac Sim 6.1, PhysX, 20 Hz control, `SelfCollisions=False` within articulations). Simplified: leather upper flex and moisture absorption are omitted; the oxford shoe, mat, cedar forepart, and cedar heel slide are rigid compound primitives with measured masses and dry friction (`0.8x` to `1.5x` friction and contact-softness margin verified), and the twin telescoping spring tubes are represented by a single 42 mm prismatic joint (`250.0 N/m`, `5.5 N` at `22 mm`) with zero scripted welds.

## Licence
All shapes are procedural primitives authored in `cedar_shoe_tree_insert_and_seat.py`; no external third-party mesh, texture, or font files are included (original own work).
