# Pour a full 3 kg pot of soup into a tureen without spilling and set the pot back on the hob

The skill is two arms lifting, carrying and tipping one heavy two-handled pot together and setting it back down. A
full pot of soup stands on a hob; the job is to pour all of the soup into a tureen in front of it without spilling and
set the pot back on the hob. The pot is a 3 L stainless pot (inside 18 cm across and 12 cm deep, 1.2 kg empty) with two
flat loop handles, holding 1.8 L of soup, so the full pot weighs 3.0 kg. A real trial uses 1,800 ml of room-temperature
water tinted orange and scores the millilitres that reach the tureen and the millilitres spilled, by weighing; the
simulation stands in 120 rigid beads for the soup, each 15 g and 15 ml, that pour and roll but do not slosh or flow like
liquid. Held by one handle the full pot swings down about that handle, so the arms have to move as one. The size is
chosen so the full pot, about 1.5 kg per arm, stays inside the YAM's ~2 kg per-arm payload, while one arm alone would
have to carry all 3 kg about a single handle. The task is one task file,
[`pour_a_pot_with_both_hands.py`](pour_a_pot_with_both_hands.py), for the `yam_arena` embodiment (scene, scoring and
the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The pot (floor, round wall, rim, two loop handles)
and each bead are loose rigid bodies built from several shapes, and the hob and the tureen are fixed bodies built the
same way, so `bodykit.py` adds such bodies: importing it hooks `yam_arena.scene` when the simulator starts, without
changing yam_arena, and gives `score()` each body's own pose. It also lets the scripted demo give a grip as a full
rotation, since pouring needs level grips that roll, not just straight-down ones.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the pot and every bead still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0 by the soup in the tureen, the soup spilled and
where the pot ends, each bead counting as 15 ml; see `task.json`). `-E object_jitter=0` keeps every object exactly
where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales
every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pour_a_pot_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pour all of the soup from the full pot (about 3 kg) into the white tureen without spilling, and set the pot back down on the hob." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/pot-dream
```

The scripted demo, which reaches full score: both arms turn their grips level in front of the handles, pointing at the
pot with their fingers one above and one below each handle's plane, slide back so the handle's outer bar comes between
the fingers (the fingertips inside the U) and close on it, so the lower finger carries the weight. They lift the pot
together, carry it over the tureen, roll both wrists to tip it 110 degrees about the line through the handles, tip it
back, carry it back, set it down on the hob and slide out forward. The robot's gains are the stock ones, so the loaded
arms sag 4-5 cm at the handles under the full pot; the waypoints aim 4 cm high while lifting and carrying, 2 cm while
pouring (the lip then dips just inside the tureen's rim) and 3 cm with the pot empty:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pour_a_pot_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pour all of the soup from the full pot (about 3 kg) into the white tureen without spilling, and set the pot back down on the hob." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/pot-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/pot-dream/*.json; do inspect-robots video "$log"; done
```

Set `POT_DEBUG=1` to print what the score is built from (beads in the tureen, in the pot and spilled, and the same in
ml; the pot's tilt and position) twice a second.

## A real trial

The rig: a cold portable single hob, a 3 L stainless pot with two loop side handles (inside 18 cm across, 12 cm deep,
1.2 kg empty) and a 2.7 L ceramic tureen (inside 20 cm across, 8.7 cm deep) on a non-slip mat, laid out as in the
setup. The soup is 1,800 ml of room-temperature tap water tinted orange with food colouring (1,800 g), about 7 cm deep
in the pot. Before the trial the pot and the tureen are each weighed empty and dry on a kitchen scale (1 g resolution,
5 kg range). After it each is lifted off, its outside and base are wiped dry, and it is weighed again; 1 g is 1 ml.

| Measure | Real trial | Simulation (each bead = 15 ml) |
| --- | --- | --- |
| Transferred | the tureen's gain in grams | beads with their centre inside the tureen |
| Left in the pot | the pot's gain in grams | beads with their centre inside the pot |
| Spilled | 1,800 ml minus transferred minus left in the pot (hob, table, floor, outsides) | the other beads |

| Score | Real trial | Simulation |
| --- | --- | --- |
| 1.0 | at least 1,530 ml transferred, at most 75 ml spilled, the pot back upright on the hob | at least 102 beads in, at most 5 spilled, the pot back upright on the hob |
| 0.5 | at least 900 ml transferred, at most 180 ml spilled | at least 60 in, at most 12 spilled |
| 0.25 | at least 180 ml transferred | at least 12 in |
| 0.0 | anything less, or the pot tipped 30 degrees or more within 6 cm of the table at any time, or left tilted 30 degrees or more | the same |

"Back upright on the hob" means tilted less than 10 degrees, its centre within 3 cm of the hob's centre and its
underside within 1 cm of the hob's top, judged from the video or with a ruler and a phone level.

## Why two hands

The full pot weighs 3.0 kg and its handles stick out 9.4 to 12.9 cm from its axis on opposite sides. Lifted by one
handle, the pot swings down about that handle. A one-hand check, the same scripted demo with the left arm left at its
ready pose, scores 0.0: as the right arm lifts, the pot swings down and outward about its handle, tips up to 36
degrees with its body within 6 cm of the table, ends tilted 34 degrees off the hob, and none of the soup reaches the
tureen.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,200 steps (569 s on the clock, with Dream answering in 0.78 s on average), and about 7 minutes for the scripted demo (670 steps, 33.5 s of simulated time, 409 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the pot and the beads) shift.

## Changes in this revision

This is a revision of TK-KZMJV4. Its review said:

> Construct validity (not met): The specified real-world soup consists entirely of 24 mm rigid beads, which do not test liquid flow, sloshing or spills.
>
> What to change: Specify a safe liquid, such as room-temperature dyed water, for the real-world trial and replace bead counts with numeric transferred and spilled-volume thresholds. The simulation may retain its explained bead approximation, but the physical evaluation should measure controlled liquid pouring.

The review before that (TK-5U3XAT) asked for a more explicit soup weight and a larger pot that presents a realistic challenge; that was done in the previous revision (a 3 L pot, inside 18 cm across and 12 cm deep, 1.2 kg empty, holding 1.8 kg of soup, 3.0 kg full) and is kept.

What changed in response:

- **Construct validity: a real liquid in the real trial.** A real trial now pours 1,800 ml (1,800 g) of room-temperature tap water tinted orange with food colouring, about 7 cm deep in the 3 L pot, on a hob that is switched off. The 24 mm beads stay only as the documented simulation stand-in: each of the 120 beads stands for 15 ml (and weighs 15 g, so the full pot weighs 3.0 kg in both the real trial and the simulation).
- **Bead counts replaced by numeric transferred- and spilled-volume thresholds.** Every tier is now stated in millilitres: 1.0 needs at least 1,530 ml transferred into the tureen, at most 75 ml spilled and the pot back upright on the hob; 0.5 needs at least 900 ml transferred and at most 180 ml spilled; 0.25 needs at least 180 ml transferred. The 0 rule for tipping the pot over on the hob (30 degrees or more within 6 cm of the table) is unchanged.
- **How a real trial measures them.** The tureen and the pot are each weighed empty and dry on a kitchen scale (1 g resolution, 1 g = 1 ml) before the trial, and again after it with their outsides and bases wiped dry. The tureen's gain is the volume transferred; the pot's gain is the volume left in it; 1,800 ml minus both is the volume spilled (on the hob, the table, the floor or the outside of either vessel).
- **Simulation measurement, matched to the millilitres.** At 15 ml per bead the simulation thresholds are exactly the same volumes: at least 102 beads in the tureen and at most 5 spilled (1,530 ml / 75 ml) for 1.0, at least 60 in and at most 12 spilled (900 / 180 ml) for 0.5, at least 12 in (180 ml) for 0.25. The scoring text, the README (a table pairs each tier's real-trial measurement with its simulation count), the code's docstrings and the fidelity note all say so; the fidelity note states that the beads pour and roll but do not slosh, splash or cling.
- **Also changed:** the category is now "Societal relatability" (the renamed category). The instruction and title now state only the goal: "Pour all of the soup from the full pot (about 3 kg) into the white tureen without spilling, and set the pot back down on the hob." (the method "by both handles" is gone). The scene, the scorer's thresholds and the demo are unchanged; the demo was re-run and scores 1.0 (all 120 beads, 1,800 ml, in the tureen, none spilled, the pot back on the hob), a fresh one-hand check (left arm idle) scores 0.0 (the pot swings down about the single handle, tips up to 36 degrees low over the table, and no soup reaches the tureen), and the Dream-1 video is new.
