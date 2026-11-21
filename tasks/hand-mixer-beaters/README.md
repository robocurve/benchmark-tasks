# Fit both beaters into a hand mixer, the right one in each socket

Two YAM arms fit the two beaters of a hand mixer (modelled on a KitchenAid KHM512 and its KHM2B beaters), each fully
in and locked, and let go. The mixer stands on its heel
rest on the table. The two beaters look alike, but only the plain one fits both sockets: the collared one fits only
the larger socket, so the plain one must go in the smaller socket. Inside each socket a drive spindle with four slots
rests at an unknown angle; the four fins on a beater's stem pass only when they are turned to the slots. A spring clip
then has to be pushed past (15 N preload on its ball). The 0.9 kg mixer slides on the table at about 4 N (estimate),
so the other hand must hold it while the push is made.

## Requirements

The official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), which provides
`inspect-robots` and the `yam_arena` embodiment. For Dream-1, a Dream API key and the Dream-1 bridge on port 8211.
Nothing else: the rest is in this folder.

## Run and score a trial

From this folder:

```bash
INSTR="Fit both beaters into the hand mixer, each fully in and locked in its own socket, and let go. The mixer stays standing on its heel rest on the table."
ARGS=(--embodiment yam_arena -E task=hand_mixer_beaters.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360
      --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames)

# Dream-1
inspect-robots run "${ARGS[@]}" --policy dream -P server_url=http://127.0.0.1:8211 --seed 0 --log-dir ~/yam-eval/mixer-dream

# The scripted demo (seeds 0, 1, 2 shown)
inspect-robots run "${ARGS[@]}" --policy yam_scripted -P hold_s=2 --seed 0 --log-dir ~/yam-eval/mixer-demo-s0

# The same demo made wrong on purpose (see Results): fixed, nobrace, swap, nothing (a fifth mode, holding, was not run on the final code)
MIX_FAIL=nobrace inspect-robots run "${ARGS[@]}" --policy yam_scripted -P hold_s=2 --seed 0 --log-dir ~/yam-eval/mixer-nobrace

# The mechanism probe (no arms): push each beater along its stem with a rising force
MIX_PROBE=a inspect-robots run "${ARGS[@]}" --policy yam_scripted --max-steps 260 --seed 0 --log-dir ~/yam-eval/mixer-probe-a
```

The run prints `task_score` at the end. The trial is scored at its last step; it ends early once the full score has
held for 1 s. With `MIX_DEBUG=<file>` set, every control step adds one line to that file (each beater's depth, offset
from the socket axis, angle, clip ball lift, spindle angle, the mixer's pose and the score); `<file>.plan` gets the
planned and measured tool poses. `dev/run.sh <name> <policy> <seed> <steps> [ENV=VAL ...]` wraps the command with these
logs. A trial takes about 4 to 7 minutes of wall time on the shared GPU machine, start-up and the demo's planning included
(measured on the final runs: 3.6 to 6.8 minutes).

## Scoring

Scored at the end of the run (`score()` in `hand_mixer_beaters.py`):

| Part | Points | Condition (simulation) | Real trial |
|---|---|---|---|
| Each beater: in a socket | 0.1 | stem tip at least 6 mm past the socket face, on the socket axis within 1.5 mm and 6 deg | ruler: the stem's tip is inside the hole (showing length L - 6 mm or less; L, F, B from the calibration below) |
| Each beater: fins (real: tabs) in the slots | +0.1 | tip at least 30 mm deep (fins off the slots stop the tip at 28 mm) | showing length shorter than B - 2 mm |
| Each beater: clipped | +0.2 | tip within 1.5 mm of full depth (36 mm) and the clip ball back in the groove (within 0.3 mm of its stop) | showing length within 2 mm of F, and it stays in when the mixer is lifted 10 cm by its handle for 3 s |
| Mixer standing and held | +0.2 | at least one stem in a socket (row 1), tilt at most 5 deg, footprint centre within 5 mm of its start | tape measure from the marked start; it stands on its heel rest |
| Hands clear | gate | both grippers at least 20 mm from the mixer and both beaters, else the trial scores 0 | photo |

The collared beater counts only in the larger socket; the plain beater counts in either (the maker: a beater without a
collar fits either opening). So a plain beater put in the larger socket first can still earn its 0.4, but the collared
one then has nowhere to go: 0.6 at most. The mixer's 0.2 counts only once a stem is in a socket, so the untouched
scene scores 0 (run p-nothing-s0).

### Real-trial calibration (once per mixer and beater pair, before the trials)

The real-trial thresholds are not computed from estimated sizes. The operator reads them off the real parts with a
steel ruler, for each beater in its own socket:

1. Read the beater's length L (stem tip to the end of the cage).
2. Push it fully home by hand until it clicks. Read the length showing from the socket face to the end of the cage:
   the full-seat reading F. Press the ejector.
3. Turn the beater so the stem's tabs are off the spindle slots, push until it stops, and read again: the blocked
   reading B.

A trial is then read with the same ruler: in a socket = showing length L - 6 mm or less (the tip is inside the hole);
tabs in the slots = shorter than B - 2 mm; clipped = within 2 mm of F and the beater stays in when the mixer is lifted
10 cm by its handle for 3 s.

Expected values from the published figures (the operator's readings replace them):

| Value | Expected | Source |
|---|---|---|
| L | about 178 mm (7 in) | heima.com parts page for W10490648 ("approx. 1-1/2 in wide x 7 in"); an amazon.ca parts supplier ("approximately 7 inches long") |
| F, collared beater | about 137 mm (L less 41 mm) | collar 40.7 mm from the tip, read from the Speedy Appliance Parts W10490648 photo on a 1 in grid (about +/-1 mm) |
| F, plain beater | not published | no collar; it seats on the spring ring |
| B | F plus the slot depth (not published) | cross pin (tabs) 29.6 mm from the tip in the same photo |

## Setup

See `setup` in `task.json` for every object with its size, material and the random start ranges, and the table
under Fidelity for the published figures of the real KHM512 and KHM2B. In short: a hand
mixer (0.9 kg) stands on its heel rest with its socket face toward the right arm and its handle toward the left arm;
the two beaters lie flat in front of it, stems pointing at it. Each trial moves and turns the mixer (+/-30 mm,
+/-20 deg), sets each drive spindle to a random angle (0 to 90 deg), swaps which beater lies where at random, and
moves and turns each beater (+/-12 mm, +/-8 mm, +/-12 deg). `layout(seed)` in the task file gives every pose.

## How the scripted demo works

`mixdemo.py`. **Scripted demo, reads object poses from the simulator** (mixer, beaters, spindle angles) and the arms'
own joint readings to plan its motions; it moves the parts only through the grippers, by contact.

1. Left hand: closed fingers come down behind the mixer's handle and stop 4 mm short of its back, at 150 mm height. It
   holds its place; when a push moves the mixer toward it, it pushes back (the brace).
2. Right hand: picks the collared beater by its stem just ahead of the blades. Before the pick it runs the arm's
   inverse kinematics over the whole path (pick, lift, every insertion pose) for both ways round of the hand and all
   four fin alignments, and keeps the one the arm reaches best (the hand also must not lean back toward the robot,
   where its wrist would crowd the mixer).
3. It measures the beater in the hand, draws it back from the mixer, lines it up 13 cm out on the larger socket's
   axis with the fins turned to the slots (spindle angle read from the simulator), and brings it to 8 mm short of
   the face. There it measures the beater in the hand again (it shifts a few millimetres in the fingers on the way)
   and plans the last stretch.
4. It pushes the stem in until the fins rest on the spindle (30 mm, light push), turns the beater a few degrees each
   way if the fins have not dropped in, then pushes past the clip (target 6 mm past full depth) and holds, lets go,
   and lifts the hand away.
5. The same for the plain beater in the smaller socket. Then the left hand backs away and both arms return.

The simulated arms are soft, so each settles short of its target under load; the demo closes that gap from the joint
readings (a bounded integral correction of the tool pose), except during the light push of the slot search.

## Results

All runs below are on one code version (the `sha256.txt` in each `runs/` folder matches the files in this folder),
1,200-step limit. The scripted policy ends the trial once its script is done, or once the full score has held for 1 s.

| Run | Seed | What it does | Score | Steps | What happened |
|---|---|---|---|---|---|
| p-demo-s0 | 0 | scripted demo, reads object poses from the simulator | 1.0 | 872 | both clipped (36.0 / 36.1 mm); mixer moved 0.5 mm (the demo video) |
| p-demo-s1 | 1 | scripted demo, reads object poses from the simulator | 1.0 | 872 | both clipped (36.0 / 36.2 mm); mixer moved 1.4 mm |
| p-demo-s2 | 2 | scripted demo, reads object poses from the simulator | 1.0 | 871 | both clipped (36.0 / 36.2 mm); mixer moved 0.4 mm |
| p-nothing-s0 | 0 | nothing done: both arms hold their start pose | 0.0 | 47 | no stem in a socket, so the standing mixer earns nothing; the scripted policy has no motions and ends the trial |
| p-fixed-s0 | 0 | the plan made for the reference layout (spindles at 0 deg), no sensing | 0.0 | 1,005 | neither beater reached a socket (tips 25 and 20 mm short of the face, 63 and 89 mm off its axis); the mixer was pushed 64 mm |
| p-nobrace-s0 | 0 | demo with the left hand kept away (no brace) | 0.8 | 985 | both clipped, but the pushes slid the mixer 11.5 mm (limit 5 mm) |
| p-swap-s0 | 0 | demo with the order swapped: plain beater in the larger socket | 0.6 | 1,005 | plain clipped in the larger socket (0.4); collared stops on its collar at 31.3 mm in the smaller one (0); mixer held (0.2) |
| p-dream-s0 | 0 | Dream-1 | 0.0 | 1,200 | the arms moved toward the beaters but touched nothing: neither beater nor the mixer moved; a gripper ended 18 mm from a part, inside the 20 mm gate (the model_run video) |

Each score is the sum of the parts in the Scoring table: a scripted pass needs the right beater in each socket, the
fins found by turning, the clip pushed past, the mixer braced, and both hands clear.

## Fidelity

Rigid PhysX bodies. Everything works by contact, joints and springs: the face openings, collar, shoulder, fins,
spindle slots and stem groove are collision shapes; each drive spindle is a hinge held at its angle by a spring
(0.005 N m/deg, its gear train); each clip is a 4 mm steel ball on a sprung slider (15 N preload) that the stem's
rounded tip pushes aside and that drops into the groove. There is no rule that attaches anything.

The real lock is the same kind of latch. iFixit's teardown of this mixer ("KitchenAid KHM512ER Beater Rings
Replacement") shows a spring ring on each drive spindle ("the plastic knob"); iFixit's answer "Beaters keep falling
out!" says the beater clicks in when the ring's small metal tabs catch the tabs on the beater's stem. The model uses
one sprung ball in a stem groove; its click force is an estimate.

| Part | Simulated | Published or photo figure | Status |
|---|---|---|---|
| Mixer body | standing 200 mm tall, 90 mm wide, 125 mm from socket face to handle back; 0.9 kg | 8 x 3-1/2 x 6 in, 203 x 89 x 152 mm (AJ Madison, KHM512BM); 2.0 lb, 0.9 kg (The Brick, KHM512IC) | height, width and mass match within 3 mm; depth an estimate, 27 mm short; shape simplified |
| Socket holes | square, 14.5 mm and 11.8 mm | round holes in a round plastic cap (iFixit photo); diameters not published | estimate; needs a ruler |
| Socket spacing | 38 mm | about 36 to 40 mm, read from the iFixit photo against the 89 mm body width | photo figure; needs a ruler |
| Cage | 38 mm across, 70 mm long | about 1-1/2 in (38 mm) across (heima.com; amazon.ca supplier) | matches |
| Beater length | 190 mm | about 7 in (178 mm) (same sources) | estimate, 12 mm long |
| Beater mass | 60 g and 55 g | 0.2 lb for the pair, about 45 g each (Amazon listing for KHM2B) | estimate, heavier |
| Stem | 6.0 mm | 1/4 in, 6.35 mm (Speedy Appliance Parts, W10490648) | estimate, 0.35 mm thin |
| Stem key | four fins 10 mm across, 24-32 mm from the tip | a cross pin (two tabs, about 9 mm across) 29.6 mm from the tip (Speedy photo) | different shape, same job |
| Collar | 13 mm, 32-36 mm from the tip | about 11.7 mm, centre 40.7 mm from the tip (Speedy photo) | estimate |
| Latch | sprung 4 mm ball into a stem groove, 15 N preload | spring ring whose metal tabs click over the stem's tabs (iFixit) | same kind; force estimated |

Simplified: the wire blades are straight bars; the two beaters' blades do not collide with each other (on a real mixer
they mesh); no motor, cord or ejector button. Measured in simulation: a beater with its fins off the slots stays at
28.1 to 28.8 mm under a 12 N push; the collared beater in the smaller socket stops at 31.7 mm (collar on the face)
under 12 N; a beater with its fins lined up clips in at about 5 N and pulls out at about 4 N with a 9 N preload (the
probe, `MIX_PROBE=a`). The preload was then raised to 15 N; its pass force was not probed again.

## Why it matters, and the sources

Fitting the beaters is an everyday home-baking step: it comes first every time a hand mixer is used to whip cream,
beat eggs or mix batter at home.

How the skill differs from two close cousins:

- A stand mixer's (or bakery mixer's) beater: one beater on one shaft, slid up until its bayonet pocket meets the
  shaft's pin, then turned to lock, on a heavy base that does not move. No choice of socket, no hidden slot angle, no
  brace.
- A hand blender's attachment swap: one attachment pushed or twisted onto one coupling, while the other hand holds the
  blender's body in the air.
- Here: two near-identical parts and two sockets that only one order fills; a blind slot search under a light push;
  and a free-standing body that one hand must brace on the table while the other pushes past a sprung clip.

- KitchenAid product help, "Inserting the Beaters - Hand Mixer" (producthelp.kitchenaid.com, opened 2026-11-19): the
  beater with the collar goes in the larger opening; align the flanges on the shaft with the slots; push to lock; the
  beater without a collar goes in the smaller opening.
- AHAM consumer blog, "All Mixed Up: 6 Facts about Stand Mixers" (blog.aham.org, 13 Aug 2018, opened 2026-11-19):
  43% of consumers report owning a stand mixer.
- iFixit, "KitchenAid KHM512ER Beater Rings Replacement" (ifixit.com/Guide/KitchenAid+KHM512ER+Beater+Rings+Replacement/61130,
  opened 2026-11-19): a spring ring on each drive spindle holds the beater; step 1's photo shows the round cap with two
  round holes.
- iFixit Answers, "Beaters keep falling out!" (ifixit.com/Answers/View/293834, opened 2026-11-19): the beater clicks
  in when the ring's metal tabs catch the tabs on the stem.
- AJ Madison listing for KHM512BM (ajmadison.com, opened 2026-11-19): 6 x 3-1/2 x 8 in. The Brick listing for
  KHM512IC: 2.0 lb.
- heima.com parts page and an amazon.ca parts supplier for W10490648 (opened 2026-11-19): about 1-1/2 in wide, 7 in
  long. Speedy Appliance Parts W10490648 page (opened 2026-11-19): 1/4 in shaft, stainless, photos on a 1 in grid.
- Daily Advocate, "Million mixer milestone" (dailyadvocate.com, 16 Nov 2016, opened 2026-11-19): KitchenAid's plant
  in Greenville, Ohio, the only place that makes its hand mixers, built 1 million hand mixers in 2016, and more than
  2 million stand mixers a year.

## Real trial and safety

A real KitchenAid KHM512 hand mixer, unplugged, standing on its heel rest on the table; its two KHM2B beaters. Nothing sharp or hot. Do the calibration under Scoring once first. Mark the
mixer's start and each beater's place on paper under a clear mat; turn each drive spindle by hand to the seed's angle
(a paint dot on each spindle). Spindle angles (large, small socket) as each run's `debug.log` prints them on its first line: seed 0: 17.0, 53.5 deg; seed 1: 33.6, 6.7 deg; seed 2: 65.5, 84.4 deg.

## Files

- `hand_mixer_beaters.py`: the scene (mixer, spindles, clips, beaters), the scorer, the seeded start and the M1 probe.
- `mixdemo.py`: the scripted demo's planning and its hooks into `yam_scripted`.
- `dev/run.sh`: one trial with logs. `dev/geom_check.py`: a static check that the parts fit. `dev/reach_check.py`: the
  demo's planning on several seeds without the simulator. `dev/frame_png.py`, `dev/build_media.py`: pictures and the
  video from stored frames. `runs/`: the logs behind the Results table (no frames).
