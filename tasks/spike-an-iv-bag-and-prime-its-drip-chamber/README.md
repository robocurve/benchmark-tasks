# Hold an IV bag still on a slick tray while the other hand uncaps its port, spikes it and primes the drip chamber

A 500 ml bag of saline lies on a smooth steel tray, its spike port closed by a blue pull-off cap; beside it lies an infusion giving set (spike, drip chamber, a short length of tubing). One hand holds the bag still by the stiff hanger tab folded up on its far end, while the other pulls the cap off the port, takes the giving set by its drip chamber and pushes the spike into the port all the way, until it locks, then squeezes the drip chamber twice to draw saline into it. Then both hands let go and both arms go back to where they started. The task is one task file, [`spike_an_iv_bag_and_prime_its_drip_chamber.py`](spike_an_iv_bag_and_prime_its_drip_chamber.py), for the `yam_arena` embodiment (scene, scoring and the
scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: holding a light, slippery part still against pulls and pushes from the other hand; a pull-off, a push-in to a stop and a squeeze, in order; letting go cleanly.

Why two hands: the bag slides on the tray at about 0.25 N, and the spike takes about 1.5 N to push in. With one hand (the
right doing its part as in the demo, the left never moving) the cap came off, but pushing the spike shoved the whole
bag 5 cm along the tray and the spike never got past the port's mouth (0.25).

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The tray is kinematic; the bag (with its ports), the cap and the giving set are loose rigid bodies made of shapes from `bodykit.py`, and the saline in the drip chamber is three kinematic pieces kept under the table until they are drawn in.

- The cap: held on its port by a hinge that breaks for good at 3.5 N (a `Hinge` from `bodykit.py` with a break force).
- The spike in the port: a sliding joint (a `Thread` from `bodykit.py`, a PhysX D6 joint) between the bag and the giving set, switched on once the cap is off and the spike's tip enters the port's mouth in line (within 7 mm and 12 degrees). A spring holds the spike where it last stopped; after every control step, a spike pushed more than 1 mm past its hold has given, and the task file moves the hold there, so it takes about 1.5 N to push it further, six times what holds the bag on the tray. At full depth (its collar on the port's mouth) it locks.
- Priming: after the spike has locked, each squeeze of the drip chamber (the right hand's fingers closed on it, then opened again) draws in a third of a chamber of saline, shown by moving another light-blue piece into the clear chamber (its collision cylinder is not drawn).

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step. This copy of `bodykit.py` also lets a shape be collision only (`color=None`): not drawn, as the clear drip chamber is.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the bag and the giving set still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=spike_an_iv_bag_and_prime_its_drip_chamber.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Get the IV bag ready for an infusion: hold the bag still on the tray by its hanger tab with one hand, and with the other pull the blue cap off its spike port, push the giving set's spike into the port all the way until it locks, then squeeze the drip chamber twice so it fills about halfway. Then let go of both and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/ivbag-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the bag's hanger tab, fingers closing across it, and holds the bag. The right comes down on the blue cap's tab, fingers closing across it, pulls it straight off the port and drops it on the tray out of the way. It takes the giving set by its drip chamber, lifts it, lines the spike up with the port and pushes it in to the collar, where it locks. It lets go, closes on the drip chamber and squeezes it twice, then rises. The left lets go of the bag, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=spike_an_iv_bag_and_prime_its_drip_chamber.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Get the IV bag ready for an infusion: hold the bag still on the tray by its hanger tab with one hand, and with the other pull the blue cap off its spike port, push the giving set's spike into the port all the way until it locks, then squeeze the drip chamber twice so it fills about halfway. Then let go of both and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/ivbag-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/ivbag-dream/*.json; do inspect-robots video "$log"; done
```

Set `IVBAG_DEBUG=1` to print what the score is built from twice a second: whether the cap is off, how deep the spike is in the port and how far it is off the port's axis, whether it is caught and whether locked, how many thirds of the drip chamber are filled, the right hand's finger joint, whether both hands are clear, and where the right TCP is from the drip chamber and the cap.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (240 s on the clock, with Dream answering in 1.03 s on average), and about 2 minutes for the scripted demo (554 steps, 27.7 s of simulated time, 118 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
