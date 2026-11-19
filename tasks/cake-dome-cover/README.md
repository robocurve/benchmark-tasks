# Swap a cake in under a pastry display dome and cover it, without touching the cake

A clear polycarbonate pastry display dome (American Metalcraft LFTD8 size: 206 mm across, 168 mm tall overall, a
rounded top with a ball knob on a short neck) rests on an empty plate on the display spot. A 6 in cake waits on an
identical plate to the right. The cake must go where the empty plate is, under the dome, and the dome must come down
over it, and nothing may touch the cake. How the plates and the dome are moved is up to the robot: the demo holds the
dome up with one hand while the other swaps the plates; sliding the empty plate aside with the dome still on it and
moving the dome across afterwards is also allowed.

Why it matters: covering a cake under a cake dome is an everyday kitchen and party chore, and cafe counters do the same
with display domes many times a day. The dome hangs from its small knob about 130 mm above its rim, so it sways and the
arm sags under it while it is held; the plates have to trade places on one spot; and the dome has to come down square
round the cake, where the rounded wall leaves about 15 mm at the cake's top edge. A touch by a finger or by the dome's
wall marks the icing, so the trial scores 0.

Instruction given to the policy:

`Put the cake where the empty plate is, under the dome, and cover it with the dome. Don't touch the cake.`

## This revision

This task revises board task TK-QTH3SB (robocurve/benchmark-tasks#2318), whose authors did not revise it within the
48 h window, after which the program opens the idea to other teams. Its scene, demo and `labkit.py` are reused and
credited to that pull request. The review of TK-QTH3SB asked for four things; each is addressed here:

1. **Importance: re-file as Societal relatability.** Done: the category is Societal relatability, and the context
   explains the familiar cake-covering activity (home kitchens, parties, cafe counters).
2. **Construct validity: the robot could touch or mark the cake without moving it 3 mm or tilting it 3 degrees.**
   The scorer now detects a touch even when the cake does not move. Every control step it samples both fingers of both
   hands as plates (16 mm wide at the tip, 68 mm wide 60 mm up, 30 points each, plus the gripper body's lower corners),
   and any of them within 4 mm of the cake counts as a touch. It also measures the gap between the cake's surface
   (72 points on its top edge, side and top) and the dome's wall (skirt, spherical top and top disc, wall thickness
   excluded), and the gap from the dome's rim to the cake; under 1 mm counts as a touch. A touch is latched and the
   trial scores 0, as before for a knocked cake.
3. **Scoring: the movement-only check could award full credit after a damaging contact.** It cannot now: see 2. Tests
   cover a finger 3 mm from the cake's side (touch), 5 mm above its top (clear), the dome 10 mm off centre (clear), 20 mm
   off centre (touch) and the dome's rim set down on the cake's top (touch).
4. **Clarity: the perishable sponge cake had no repeatable source or handling.** The cake is now a repeatable stand-in
   with the same size and mass: a 6 x 2.4 in (152 x 60 mm) round polystyrene cake dummy, as sold for bakery display and
   cake practice, ballasted to 0.50 kg by a 140 mm steel disc 4 mm thick (0.48 kg) glued under it. Before each trial its
   top and side are dusted evenly with icing sugar through a sieve, so any touch leaves a visible mark; it is photographed
   from above and the front before and after. The real-trial touch check is that photo pair and the video.

The instruction is unchanged. The scripted demo is unchanged apart from two negative variants (below).

## Files

- `task.py`: scene, scoring and the scripted demo plan (`demo()`, played by the yam_scripted policy).
- `demo.py`: runs that demo through `inspect-robots` and writes `result.json`, the videos and `setup.png`.
- `labkit.py`: parts built from several shapes. It hooks `yam_arena.scene` and the embodiment from the task's
  import, so no change to yam_arena is needed.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets.
- `task.json`, `README.md`. Offline tests: `tests/test_cake_dome_cover.py` in the team repository (20 tests).

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed, from this folder:

    inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
        -E cam_width=320 -E cam_height=176 \
        --instruction "Put the cake where the empty plate is, under the dome, and cover it with the dome. Don't touch the cake." \
        --scorer task_score --max-steps 1500 --store-frames --log-dir logs

Scripted demo and its negative variants:

    python demo.py task.py demo_out                                   # full score
    DEMO_VARIANT=touch python demo.py task.py touch_out --no-record    # fingertips down on the cake's top: 0
    DEMO_VARIANT=noswap python demo.py task.py noswap_out --no-record  # the cake plate is never pushed in: 0

## Scoring

`task_score` is `score()` in `task.py` at the end of the run (1.0 once `success()` has held).

- 0 for the whole trial once the cake has been touched (a finger within 4 mm, or the dome's wall or rim within 1 mm)
  or knocked (moved 3 mm or tilted 3 degrees relative to its plate). Moving the plate the cake stands on is fine.
- From the final state: 0.25 the cake's plate centred within 15 mm of the display spot; 0.5 also the dome covering the
  cake (its rim lower than the cake's top, the whole cake inside its wall and under its top, tilt under 10 degrees);
  1.0 also the dome's rim resting wholly on the cake's plate (within 2 mm of its top, centre within 20 mm, tilt under 2
  degrees).
- Every positive score needs every fingertip and each hand's pad midpoint at least 80 mm from the dome, both plates and
  the cake, and all four still for the last 1 s (21 samples at 20 Hz, no two positions of a tracked point 1 mm apart).

The scripted demo reads only what a camera sees: the dome's and the plates' positions. The left hand lifts the dome by
its knob, holds it to the left of the spot and raises itself by the arm's seen sag; the right hand pushes the cake plate
in, stopping short and closing the last gap to the spot as seen; the left hand then brings the dome over the cake plate,
centres it as seen and lowers it until the rim is seen resting on the plate before letting go.

## Scores

- Scripted demo: 1.0 (success at step 1,162 of 1,500).
- Negative variants: fingertips down on the cake's top 0.0; cake plate never pushed in 0.0.
- Dream-1 run (1,500 steps): 0.0.

## Runtime and seeds

- 1,500 control steps (75 s of simulated time) per trial; the demo finishes in about 1,130.
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.

## Fidelity

Rigid bodies. The dome's rounded top is drawn as five rings of flat panels (within 1.3 mm of the sphere) and the knob as
stacked discs; knob sizes are scaled from the product photo and the mass from the wall's area. The cake is a rigid
cylinder; touches are found geometrically, as the dusted sugar shows them in a real trial.

## Real-world build

- Dome: American Metalcraft LFTD8 polycarbonate display dome (8 in class: 206 mm rim, 168 mm tall), 0.43 kg.
- Plates: two flat glazed ceramic plates, 250 mm across, 12 mm thick, 0.60 kg each (weigh them; 0.55 to 0.65 kg).
- Cake: 6 x 2.4 in round polystyrene cake dummy, a 140 mm x 4 mm steel disc glued centrally underneath (0.50 kg in all),
  icing sugar dusted on before every trial; brush clean between trials.
- Table: smooth laminate counter. Mark the display spot (x 0.30, y 0.10 m from the arm-base midpoint) and the cake
  plate's start (x 0.30, y -0.19 m) with tape crosses.
- Scoring: a ruler for the plate's position, the photo pair for touches, the video for stillness and release.
