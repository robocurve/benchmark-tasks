# Make a waffle in a rotating waffle maker: pour the batter, latch the lid, turn it over, turn it back and open it

A rotating Belgian waffle maker of the kind found on hotel breakfast buffets (a round 7-inch Belgian grid set in a
housing that turns 180 deg on its stand, worked by one handle on the lid, like the Waring WW180) stands on the table,
unplugged and cold, its lid open. A stainless batter cup beside it holds a cup of batter (30 beads). The instruction:
"Make a waffle: pour the cup of batter onto the waffle grid, close and latch the lid, turn the waffle maker over and
leave it to bake, then turn it back and open the lid."

Hotel guests make their own waffles this way at breakfast buffets every morning. The Waring WW180 instructions
([Waring WW180/WW200 manual][manual]): "Fill measuring scoop with batter to fill line. Pour batter evenly into the
center of the waffle grid ... Close the lid and rotate the unit 180°." "Once the waffle is ready and the waffle maker
has beeped 3 times, rotate the waffle maker 180°, open the lid and remove waffle". On rotating models whose lid
latches, the latch is closed before the turn ([Wayfair's guide][guide]): "Make sure your lid has latched into place
... Turn the handle so that the waffle plates flip 180°." Turned over unlatched, the lid swings open underneath and the
batter falls out onto the counter and the hot base.

[manual]: https://res.katom.com/products/141/141-WW200/141-WW200_manual.pdf
[guide]: https://www.wayfair.com/sca/ideas-and-advice/guides/how-to-use-a-waffle-maker-for-perfect-waffles-T5721

## The hand skill

- The cup of batter is carried over the housing's front and tipped over its lip onto the 18 cm grid, none over the
  grid's edge.
- The lid (1.2 kg), standing open just past its balance point, is tipped over from behind, low near its hinge, so it
  falls shut onto the batter.
- The lid is pressed down at its handle until the latch clicks (10 N).
- The handle is pinched at its end (fingers above and below it) and turned 180 deg about its own axis: a full wrist
  roll, with the whole housing turning on the handle. The arm lags under the load, so the wrist turns on past 180 deg
  until the housing sits on its stop.
- The hand lets go and keeps clear while it bakes; then it takes the handle again and turns the housing back.
- The handle is pulled up along the lid's arc against the latch (about 17 N), unlatching it and lifting the lid open.

One arm does most of it; the other tips the open lid shut from behind (from the front the lid can only be reached by
its handle, which then stands 45 cm up). The task does not claim that two hands are needed.

The task is the task file
[`make_a_waffle_in_a_rotating_waffle_maker.py`](make_a_waffle_in_a_rotating_waffle_maker.py) for the `yam_arena`
embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the waffle maker, the cup and the beads are built in these files from boxes, cylinders and spheres.

`yam_arena` task files place single boxes, cylinders and spheres and cannot put one on a joint. The waffle maker (a
fixed base and post; the housing's bottom half on a revolute joint about the turning shaft; the lid on a revolute joint
on the bottom half) is built with `bodykit.py`, which adds such bodies and joints: importing it hooks `yam_arena.scene`
when the simulator starts, without changing yam_arena, and gives the task file each joint's position. This copy of
`bodykit.py` starts a joint away from its built pose (`init`: the lid is built shut and starts open) and reports a
part's contact forces (`report_contacts`, on the lid). Every control step the task file reads the lid's net contact
force and sets the lid hinge's drive for the latch through Isaac Lab's articulation API.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Waffle maker, fixed (about 6 kg on rubber feet), unplugged and cold. Base plate 26 x 30 x 2 cm centred at x 25,
  y 12, with the dial and lights drawn on it. The housing turns on a shaft along y at x 25, 14.5 cm above the table,
  carried by a post at the rear (y 24.5), against light friction (0.25 N m). The housing is 23 cm across, centred at
  y 9.5: two halves 4.5 cm thick that meet at the shaft's height, each holding an 18 cm (7-inch) Belgian grid 1.2 cm
  deep. Bottom half 2.4 kg; lid 1.2 kg, hinged along x at the housing's rear edge (y 21, 14.5 cm up), its hinge damped
  (0.5 N m s/rad: it falls shut in about 2 s). The lid's handle, 11 cm long, 3 cm wide and 2.6 cm deep, runs out
  from the lid's front edge along the shaft's axis to y -12.5, so it stays in place as the housing turns. The housing
  rests on stops at 0 and 180 deg (turned over, the lid is underneath). At the start the lid stands open, just past
  its balance point.
- Latch: a spring catch between the lid and the bottom half. It clicks shut when the closed lid (within 1 deg) is
  pressed down with 10 N or more (PhysX's net contact force on the lid, for two control steps in a row, with the
  housing within 10 deg of upright). Latched, it holds the lid shut in any position (a drive of 300 N m/rad toward
  0.3 deg past touching) until the lid is pulled open 0.5 deg against it, about 17 N up at the handle's end with the
  housing upright; then it lets go.
- Batter cup: stainless, no handle, 7.6 cm across the top, 6.0 cm at the base, 7.5 cm tall, 0.12 kg, at x 21, y -18.
  In it 30 batter beads, 10 mm across, 8 g each (240 g, about a cup of batter).
- Both arms start at the ready pose, grippers open. The layout is the same in every trial, as in the screenshot.

What is modelled, and how: the housing, the lid and the latch as above; the batter as 30 rigid beads, as liquids are
modelled with beads in accepted tasks; nothing heats (the real trial runs the unit unplugged and cold), and the bake is
shortened to 3 s with the hands off the unit. A bead is on the grid when its centre, in the housing's frame, lies
within 9 cm of the grid's centre and between the two grids (within 1.3 cm of the parting plane; up to 4.3 cm above
the bottom grid while the lid is open).

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the best stage reached (or 0, below).
Every control step the task file reads the housing's and the lid's angles, the lid's contact force, the beads'
positions and the fingers' collision points. "The batter in" means 27 or more of the 30 beads on the grid.

- 0.25: the lid closed (within 2 deg) with the batter in.
- 0.5: the lid latched with the batter in.
- 0.75: the housing turned over (175 deg or more) with the lid latched and the batter in, and left there for 3 s
  with both hands off it (every finger point 3 cm or more from the handle and the housing): the bake, shortened.
- 1.0 (success, held 1 s): after that, the housing turned back (within 5 deg of upright), the lid unlatched and open
  30 deg or more, and the batter in.
- 0 for the whole run once the lid opens 20 deg or more while the housing is turned more than 60 deg from upright:
  the batter falls out of an upside-down waffle maker (on a hot unit, onto the counter and the hot base).

Doing nothing scores 0. A real trial is scored from the trial video (the lid shut and latched before the turn, the
housing turned over and left, turned back, the lid opened; whether the lid ever opened upside down) and a look at the
grid at the end (the batter on it, none spilled).

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=make_a_waffle_in_a_rotating_waffle_maker.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Make a waffle: pour the cup of batter onto the waffle grid, close and latch the lid, turn the waffle maker over and leave it to bake, then turn it back and open the lid." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/waffle-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The right gripper takes the cup round
its top (fingers closing along x), lifts it, tips it 30 deg toward the grid and carries it over the housing's front,
tips it on to 100 deg so the beads run out over its lip onto the grid, and sets it back down. The left gripper,
pointing down with its fingers closed, pushes the open lid's back low near the hinge until it tips past its balance
point and falls shut. The right presses the lid's handle down with its closed fingers (aimed 3.5 cm below the
handle's top: the latch clicks at about 15 N), then pinches the handle's end, level and pointing along it, fingers
above and below, and rolls its wrist 192 deg: the housing turns over onto its stop. It lets go, backs off and keeps
clear for the bake, takes the handle again and rolls back, then pulls the handle up along the lid's arc, unlatching
the lid and lifting it to about 45 deg. It reaches success at about step 1410 (71 s); the run stops once success has
held 1 s.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=make_a_waffle_in_a_rotating_waffle_maker.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Make a waffle: pour the cup of batter onto the waffle grid, close and latch the lid, turn the waffle maker over and leave it to bake, then turn it back and open the lid." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/waffle-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/waffle-dream/*.json; do inspect-robots video "$log"; done
```

`WAFFLE_DEBUG=1` prints, every 5 steps (`WAFFLE_DEBUG_EVERY=1` for every step), the housing's and the lid's angles,
whether the lid is latched, the lid's vertical contact force, the beads on the grid, the nearest finger's distance from
the unit, the hands-off time, the stage, the beads' mean position and height range, the cup's position, and where each
gripper is and how far it is open.

The wrong way, `WAFFLE_NOLATCH=1`: the same demo without pressing the lid down, so the lid is shut but never latched.
Turned over and let go, the lid swings open underneath (23 deg, onto the base), the beads fall out, and the run scores
0 (it had reached 0.25 with the lid closed over the batter).

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs: debug runs of the scripted demo
without video (about 1,430 steps, 72 s of simulated time) took about 8 to 10 minutes from start to finish. Isaac Sim's
first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A rotating Belgian waffle maker with a lid latch and a single lid handle along the turning axis (a hotel-style unit
like the Waring WW180, or a household rotating model whose lid latches), set on the table with its turning axis along
y at x 25, 14.5 cm up, its handle toward the robot's right, unplugged and cold; tape its feet down or set it on a
non-slip mat. Check the latch with a spring scale: it should click shut at about 10 N pressed at the handle and let go
at about 17 N pulled up. A stainless 1-cup portion cup (about 7.6 cm across the top, 7.5 cm tall) at x 21, y -18,
filled before each trial with 240 g of pancake-mix batter (about 1 cup, mixed fresh to the box's directions); or, to
keep batter off the rig, 30 smooth 10 mm beads of about 8 g. Before each trial: wipe the grids, set the housing upright
and the lid open on its hinge, put the filled cup on its mark. Score from the trial video and a look at the grid.
