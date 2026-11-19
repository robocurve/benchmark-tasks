# Fit both beaters into a hand mixer, the right one in each socket

Two YAM arms fit the two beaters of a hand mixer, each fully in and locked, and let go. The mixer stands on its heel
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

# The same demo made wrong on purpose (see Results): fixed, nobrace, swap (a fourth mode, holding, was not run on the final code)
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
| Each beater: in a socket | 0.1 | stem tip at least 6 mm past the socket face, on the socket axis within 1.5 mm and 6 deg | ruler: 114 mm or less of stem showing between the face and the top of the blades |
| Each beater: fins in the slots | +0.1 | tip at least 30 mm deep (fins off the slots stop the tip at 28 mm) | 90 mm or less showing |
| Each beater: clipped | +0.2 | tip within 1.5 mm of full depth (36 mm) and the clip ball back in the groove (within 0.3 mm of its stop) | 84 mm showing and it stays in when the mixer is lifted 10 cm by its handle for 3 s |
| Mixer standing and held | +0.2 | tilt at most 5 deg, footprint centre within 5 mm of its start (braced demos: 0.4 to 1.8 mm; with no brace the locking pushes slid it 11.6 mm on seed 0) | tape measure from the marked start; it stands on its heel rest |
| Hands clear | gate | both grippers at least 20 mm from the mixer and both beaters, else the trial scores 0 | photo |

The collared beater counts only in the larger socket; the plain beater counts in either (the maker: a beater without a
collar fits either opening). So a plain beater put in the larger socket first can still earn its 0.4, but the collared
one then has nowhere to go: 0.6 at most.

## Setup

See `setup` in `task.json` for every object with its size, material and the random start ranges. In short: a hand
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
| v2-demo-s0 | 0 | scripted demo, reads object poses from the simulator | 1.0 | 872 | both clipped (36.2 / 36.1 mm); mixer moved 0.8 mm (the demo video) |
| v2-demo-s1 | 1 | scripted demo, reads object poses from the simulator | 1.0 | 872 | both clipped (36.0 / 36.2 mm); mixer moved 1.8 mm |
| v2-demo-s2 | 2 | scripted demo, reads object poses from the simulator | 1.0 | 884 | both clipped (36.0 / 36.3 mm); mixer moved 0.4 mm |
| v2-fixed-s0 | 0 | the plan made for the reference layout (spindles at 0 deg), no sensing | 0.0 | 1,006 | neither beater reached a socket (tips 15 and 21 mm short of the face, 62 and 93 mm off its axis); the mixer was pushed 64 mm |
| v2-nobrace-s0 | 0 | demo with the left hand kept away (no brace) | 0.8 | 998 | both clipped, but the pushes slid the mixer 11.6 mm (limit 5 mm) |
| v2-swap-s0 | 0 | demo with the order swapped: plain beater in the larger socket | 0.6 | 1,030 | plain clipped in the larger socket (0.4); collared stops on its collar at 31.9 mm in the smaller one (0); mixer held (0.2) |
| v2-dream-s0 | 0 | Dream-1 | 0.0 | 1,200 | neither beater reached a socket (both 17 to 19 cm off the socket axes); the mixer did not move; a gripper ended touching a part (model_run video) |

Each score is the sum of the parts in the Scoring table: a scripted pass needs the right beater in each socket, the
fins found by turning, the clip pushed past, the mixer braced, and both hands clear.

## Fidelity

Rigid PhysX bodies at real size and mass. Sizes and masses are estimates for a KitchenAid-class hand mixer and its
wire beaters, to be measured on a bought unit. Everything works by contact, joints and springs: the face openings,
collar, shoulder, fins, spindle slots and stem groove are collision shapes; each drive spindle is a hinge held at its
angle by a spring (0.005 N m/deg, its gear train); each spring clip is a 4 mm steel ball on a sprung slider (15 N
preload) that the stem's rounded tip pushes aside and that drops into the groove (the real part is a wire spring clip).
There is no rule that attaches anything. Simplified: the wire blades are straight bars; the two beaters' blades do not
collide with each other (on a real mixer they mesh); no motor, cord or ejector button. Measured in simulation: a beater
with its fins off the slots stays at 28.1 to 28.8 mm under a 12 N push; the collared beater in the smaller socket stops
at 31.7 mm (collar on the face) under 12 N; a beater with its fins lined up clips in at about 5 N and pulls out at
about 4 N with a 9 N preload (the probe, `MIX_PROBE=a`). The preload was then raised to 15 N; its pass force was not
probed again.

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
- Daily Advocate, "Million mixer milestone" (dailyadvocate.com, 16 Nov 2016, opened 2026-11-19): KitchenAid's plant
  in Greenville, Ohio, the only place that makes its hand mixers, built 1 million hand mixers in 2016, and more than
  2 million stand mixers a year.

## Real trial and safety

A real hand mixer, unplugged, standing on its heel rest on the table; its two beaters. Nothing sharp or hot. Mark the
mixer's start and each beater's place on paper under a clear mat; turn each drive spindle by hand to the seed's angle
(a paint dot on each spindle). Spindle angles (large, small socket) as each run's `debug.log` prints them on its first line: seed 0: 17.0, 53.5 deg; seed 1: 33.6, 6.7 deg; seed 2: 65.5, 84.4 deg.

## Files

- `hand_mixer_beaters.py`: the scene (mixer, spindles, clips, beaters), the scorer, the seeded start and the M1 probe.
- `mixdemo.py`: the scripted demo's planning and its hooks into `yam_scripted`.
- `dev/run.sh`: one trial with logs. `dev/geom_check.py`: a static check that the parts fit. `dev/reach_check.py`: the
  demo's planning on several seeds without the simulator. `dev/frame_png.py`, `dev/build_media.py`: pictures and the
  video from stored frames. `runs/`: the logs behind the Results table (no frames).
