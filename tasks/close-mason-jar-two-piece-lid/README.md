# Screw a canning band onto a filled pint Mason jar over its loose flat lid

**Instruction:** *Screw the band on over the lid, fingertip-tight, ready for the canner.*

A filled Ball regular-mouth pint jar stands on the counter with its flat lid (the disc with the sealing compound) already set on the
rim, centred, as home-canning guides direct. Its screw band stands flange-up on the counter beside it. The robot has to lift the band,
bring it down square over the loose disc, start it on the jar's continuous glass thread, and turn it down, regripping, about one and a
quarter turns until the flange clamps the disc on the rim. Then it lets go.

## Importance and sources

Closing a jar with a two-piece canning lid is the step every home-canning reference spells out, and a familiar one in any kitchen
that puts up jam or pickles. The guidance is consistent:

- North Dakota State University Extension, *Put a Lid On It! Best Practices for Using Closures for Home-based Canning* (FN2028):
  "Place the metal screw band over the flat lid and apply fingertip tight", that is, "turn it just until you feel resistance, then turn
  the band one-quarter turn more", and "Screw bands that are applied too loosely or too tightly can cause jars to not seal properly."
  https://www.ndsu.edu/agriculture/extension/publications/put-lid-it-best-practices-using-closures-home-based-canning
- Clemson Cooperative Extension, Home & Garden Information Center, *Canning Tips: How tight should your lids be?*: place the lid on
  the filled jar, centre it and hold it with the fingers, then screw the band down fingertip-tight without force; "If the screw band is
  too tight, air cannot escape", and buckled lids may not seal.
  https://hgic.clemson.edu/canning-tips-how-tight-should-your-lids-be/
- The product: Ball 16 oz pint regular-mouth canning jar with lid and band, MFR #61000, 12 per case, about $17 a case
  ($1.42 a jar). Listed dimensions: height 5 1/8 in, top (mouth) diameter 2 3/4 in, maximum diameter 3 1/8 in.
  https://www.webstaurantstore.com/16-oz-regular-mouth-glass-canning-jar-with-silver-metal-lid-and-band-case/52761000.html

**The knack.** The band is a 74 mm thin ring that has to start square on a continuous glass thread while a loose 6 g disc sits under
it. Brought down tilted, its thread climbs over the glass thread on one side (cross-threads), and the band cocks the disc or pushes it
off the rim. Started square, it needs about one and a quarter turns, and a parallel gripper turns it at most about 150 degrees before
it must let go, turn back and grip again, each time without lifting the band off its thread. The second hand holds the jar, as people
do. The task stops at "resistance": the demo stops turning once the band no longer turns with the hand.

**Distinct from** the nearest tasks we know of:

| Nearest | What it tests | Difference here |
|---|---|---|
| Flashlight tail cap screwed back on (PR #281, accepted) | a small cap on a tube, after placing a battery | a 74 mm band over a separate loose disc that it must not disturb; regrips through 1.25 turns |
| Spin-on oil filter threaded on by hand (PR #1405, accepted) | a heavy canister onto a nipple under a car | a light ring, a two-piece closure, kitchen context |
| Oxygen humidifier onto a flowmeter (PR #3222, pending) | a bottle threaded onto a fixed outlet | the threaded part is a ring that clamps a separate loose lid |
| E-stop locking ring onto a panel (PR #3180, pending) | a nut-like ring run down a fixed button body behind a rigid panel | the ring has to pass over and clamp a loose 6 g disc without cocking it, on a free-standing filled jar |
| Canning jars: find the unsealed lid by pressing (PR #3326, pending) | inspecting sealed jars | closing the jar before processing |

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/close-mason-jar-two-piece-lid/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Screw the band on over the lid, fingertip-tight, ready for the canner." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/close-mason-jar-two-piece-lid
```

The scripted demonstration is closed loop (it reads the band's and jar's poses, never the scorer), so it runs through `demokit.py`
from the task folder:

```bash
cd tasks/close-mason-jar-two-piece-lid
python demokit.py task.py ~/yam-eval/close-mason-jar-two-piece-lid/demo --mode full                     # writes full/demo.mp4, setup.png, result.json
python demokit.py task.py ~/yam-eval/close-mason-jar-two-piece-lid/redteam --no-record --mode none,onearm,careless,placed
```

`result.json` holds `task_score`. The layout is fixed (`object_jitter=0`); there are no seeds or random choices. The demo takes 1,101
control steps (55 simulated seconds); the run limit is 2,400 steps. A GPU-rendered run took 6 to 10 minutes of wall time on a shared
GPU (a CPU `--fast` run, no cameras, about 1 minute).

## How the simulator models it

All in `task.py`; nothing is triggered by proximity, every state change is contact.

- **Jar** (free rigid body, 0.75 kg filled): the glass body is the convex hull of a lathe profile at the Ball pint's published size
  (79.4 mm widest, 69.9 mm across the thread), the neck a cylinder at the thread's root, the rim a 66.6 mm sealing surface 127 mm up.
  The glass thread is a continuous right-hand helix, 6 per inch (4.23 mm pitch), 1.25 turns, made of 30 convex segments of a trapezoid
  section (2.2 mm wide at the root, 1.2 mm at the crest, 1.6 mm deep), running out to its root over the last 29 degrees at each end, as
  a moulded thread does. Glass-on-counter friction 0.5.
- **Band** (free, 11 g): flange (58 mm opening) and skirt (73.6 mm outside, 13.5 mm tall) as 48 annular segments, and its rolled
  internal thread, the same helix, 0.875 turn, 21 segments, also run out at its ends. It stands on the counter turned 125 degrees from
  where its thread would line up with the jar's. Turning it clockwise seen from above draws it down the glass thread; the
  flange stops it on the disc.
- **Disc** (free, 6 g, 67.2 mm by 1.5 mm with its compound ring): loose on the rim. Its edge clears the band's thread crest by 0.6 mm,
  so a square band passes over it; a tilted or off-centre band lands on it.
- **Not modelled:** tightening torque (there is no force rule: fingertip-tight and cranked-down look alike to the scorer), the
  compound's give, the band's knurl, heat. The thread's turn count and section are estimates for a GPI 70 mm continuous-thread finish,
  not a measured drawing.
- **A simulator artefact to know about:** on the GPU, a hand that keeps turning after the band has seated presses the 6 g disc up to
  0.7 mm into the rim (the CPU solver does not). The scorer has no lower limit on rise, so this does not change a score, and it is not
  visible in the video.

## Scoring

Band rise means the height of the band's top above its seated height (flange on the disc on the rim), measured along the jar's axis.

- **0** if at any time the jar tipped more than 25 degrees, or the disc left the rim (its centre more than 12 mm off the jar's axis, or
  its underside more than 5 mm below the rim). This is recorded each control step and cleared only at a reset.
- **1.0:** rise at most 1.0 mm; band within 2 degrees of the jar's axis and centred within 2 mm; disc flat within 2 degrees and centred
  within 2.5 mm; jar within 5 degrees of upright and within 3 mm of the counter; nothing moving (no origin more than 0.5 mm, the band
  not more than 0.5 degree, between control steps); every fingertip point at least 3 cm from jar and band; all for 1 s (20 steps).
  Credit lapses if the state that earned it is later undone.
- **0.5:** band within 2 degrees and 2 mm of centred, rise at most 2.0 mm, disc within 5 degrees of flat (most of the way down).
- **0.25:** band within 4 degrees and 3 mm of centred, rise at most 2.5 mm: threaded on. An unthreaded band resting on top of the glass
  thread stands at least 3.7 mm up (one pitch, less the 45 degrees of spare thread), so it cannot earn this.
- **0** otherwise: the band left on the counter, set on unthreaded, or cross-threaded.

Possible scores: 0, 0.25, 0.5, 1.0.

## Real trial: preparation, reset and measurement

- **Parts:** a Ball regular-mouth pint jar with its new lid and band (MFR #61000), filled cold with jam or water to 1/2 in headspace;
  a kitchen counter or table. No heat, no canner, no sharp edges: safe for the arms' pads.
- **Preparation:** tape marks for the jar's base and the band's place 18 cm from it. Wipe the rim, set the disc on it, centred by
  eye within 1 mm (equal overhang all round). Calibrate once: screw the band on by hand fingertip-tight and measure the band top's height
  above the counter with a caliper or height gauge: that is the seated height. Unscrew it and stand it flange-up on its mark.
- **Measurement (before anyone touches the jar):** caliper the band's top height at three points 120 degrees apart: the mean less the
  seated height is the rise, and the spread over the 66 mm across the band gives the tilt (1.15 mm of spread is 1 degree). Read the
  disc's centring from a top photo (overhang either side) and its flatness from a side photo. Check the jar stands on its marks and the
  video for a tip or a dropped disc. The hold and the hands-clear condition are read from the video's last second.
- **Reset (about 2 minutes):** unscrew the band, stand it on its mark, re-centre the same disc (replace it with a new one if it fell
  to the counter), jar back on its marks.

## Assets and licences

The jar, disc and band are authored meshes (SimReady has no canning jar or band; the thread is the mechanism under test). The four
distractors are NVIDIA SimReady props, downloaded at first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`
of `nvidia/simready-assets` on Hugging Face, © NVIDIA, CC-BY 4.0:

- [food_produce_cucumber_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_produce_cucumber_a01) (two copies): mass set to 0.15 kg each.
- [dishware_bowl_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_bowl_a01): mass set to 0.35 kg.
- [cleaning_cloth_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_cloth_a01): mass set to 0.05 kg.

Nothing large is committed. `simready.py` is the loader from `tasks/cutters-off-pegboard/` (unchanged apart from its docstring), `mechkit.py` the mechanism kit
from `tasks/heat-shrink-lug/` (unchanged apart from a credit line) and `demokit.py` the closed-loop demo runner from
`tasks/heat-shrink-lug/` (unchanged): other teams' open-source files in this repository.

## Validation

All runs on the final `task.py`, GPU physics, rendered (demos) or `--no-record` (red team). Contact sheets and the demo video were
checked by eye: the band is carried clear over the jar, comes down square, and is turned in five strokes; both hands end clear.

| Run | Score | Steps | What happened |
|---|---:|---:|---|
| demo, GPU A | 1.0 | 1,101 | threaded in 1 stroke, down in 4 more; band rise -0.7 mm, square within 0.1 degree, disc centred 1.2 mm |
| demo, GPU B | 1.0 | 1,101 | the same |
| none | 0.0 | 143 | band left on the counter |
| placed | 0.0 | 582 | band set square on the jar and let go unturned: rise 4.6 mm (unthreaded) |
| careless | 0.0 | 1,100 | band brought down tilted 10 degrees: it cocked the disc and pushed it off the rim (harm), then fell off |
| onearm | 1.0 | 922 | the right arm alone, jar not held: the 0.75 kg jar did not turn on the counter, so one hand suffices here |

The one-arm run is an honest result, not a cheat: the scorer does not require the second hand, and in the simulator the filled jar's
friction on the counter holds it against the band's turning. People usually steady the jar; we do not claim it is necessary.
Distractor drift was logged on every run: the bowl and cloth moved under 1 mm and the cucumbers rolled about 2 cm as they came to rest
after spawning, by the same amount in every run; nothing touched them.

**Dream-1 run (2,400 steps): 0.0.** The model brought the right gripper down to the band and nudged it about on the counter, but never lifted it; the band stayed on the counter for the whole run, the jar and disc untouched.
