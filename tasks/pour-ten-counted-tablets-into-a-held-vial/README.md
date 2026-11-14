# Dispense ten counted tablets from a counting tray into a vial

A plastic pharmacy counting tray lies on the table. Ten counted tablets wait in the trough along its right side, which
opens at the back into a short chute; six more lie on the tray's counting area, kept out of the trough by a low divider.
The job: get exactly the ten counted tablets into an empty plastic vial, with the vial off the table while they go in,
leaving the six others on the counting area and spilling none. Too many or too few in the vial is a miscount. The task
is one task file, [`pour_ten_counted_tablets_into_a_held_vial.py`](pour_ten_counted_tablets_into_a_held_vial.py), for
the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: an exact count, kept by emptying only the trough and never the counting area (tipped sideways or too
far, the six others can slide over the divider or a rim), with the vial off the table so that no tablet bounces out.

Why two hands: the vial has to be off the table each time a tablet goes in (pharmacy staff hold it up to the chute so
none bounce out), and the tray only pours once it is lifted and tipped. Holding the vial up takes one hand and tipping
the tray the other. One hand alone can at best stand the vial under the chute and pour into it on the table, which the
score caps at 0.5. In a one-hand check, the left hand stood the vial where the chute would pour, let go, then lifted and
tipped the tray into it: 8 of the 10 went in and 2 bounced out onto the table, which scores 0.0.

## Scoring

Only the 10 counted tablets, those that started in the trough, earn credit, and only on a clean trial:

- 0.0 for the whole trial (latched, never reset) if any of the 6 others is ever in the vial, or any tablet is ever
  spilt (outside the vial and the tray, its centre at most 1 cm above the table, or on the floor);
- also 0.0 if at the end a tablet is neither in the vial nor on the tray, or fewer than all 6 others are on the counting
  area (the tray's floor left of the divider);
- otherwise 1.0 with all 10 counted tablets in the vial and the vial's underside at least 2 cm off the table each time
  a tablet went in; 0.5 with 5 to 9 of them in the vial, or all 10 but not all with the vial held up; 0.25 with 1 to 4;
  else 0.0.

In the vial: a tablet's centre inside the vial, up to 4 mm above its rim. On the tray: its centre over the tray's floor
or chute and at most 2 cm above the tray's underside. `success()` is the full score with the tray, the vial and every
tablet still.

**In a real trial** the tablets look alike, so which ones moved is read from counts and the three videos. Full credit
needs, at the end, exactly 10 tablets in the vial, exactly 6 on the counting area, and none in the trough or the chute,
on the table or the floor; at no time a tablet on the table or the floor, or one entering the counting area; and the
vial's underside at least 2 cm up as each tablet drops in. This pins down which tablets moved: the 6 started apart on
the counting area and the 10 in the trough, and a tablet can reach the counting area only over the divider or a rim,
which the videos would show. So the 6 still there are the 6 others, and the tablets in the vial are counted ones. The
same counts give the partial tiers. The simulation knows each tablet's identity and applies the same rules directly.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The tray (floor, rims, divider, chute and grip fin)
and the vial (a square open box) are rigid bodies made of shapes from `bodykit.py`; each tablet is a small rigid disc.
The tablets are slick, like film-coated tablets: they take the lower of the two frictions in every contact (0.15), so
they slide down the trough once the tray is tipped back past about 8.5 degrees.

After every control step the task file notes where each tablet is, and latches a tablet going in with the vial's
underside less than 2 cm up, any of the 6 others in the vial, and any tablet on the table or the floor.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with everything still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pour_ten_counted_tablets_into_a_held_vial.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put exactly the ten counted tablets from the counting tray's trough into the vial, keeping the vial off the table while they go in. The six other tablets stay on the tray's counting area, and no tablet may be spilt." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tablets-dream
```

The scripted demo, which reaches full score with fixed waypoints: the right hand comes straight down on the vial,
fingers closing on its left and right, lifts it and holds it up where the chute will pour. The left hand comes straight
down on the tray's grip fin, grips it and lifts the tray, tipped back a little (6 degrees: the tablets stay put), and
brings the chute in between the right hand's fingers until its lip is over the vial. Then it tips the tray on back: the
ten tablets in the trough slide out into the vial, and the six others slide only to the back of the counting area. Then
it tips the tray forward again and draws it away:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pour_ten_counted_tablets_into_a_held_vial.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put exactly the ten counted tablets from the counting tray's trough into the vial, keeping the vial off the table while they go in. The six other tablets stay on the tray's counting area, and no tablet may be spilt." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tablets-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tablets-dream/*.json; do inspect-robots video "$log"; done
```

Set `TAB_DEBUG=1` to print what the score is built from (where the tray and the vial are, how far the tray is tipped
back, where the chute's lip is from the vial, how many counted tablets and others are in the vial, how many others are
on the counting area, how many tablets are off the tray and how many of those are down, and the latches) twice a second.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,200 steps (408 s on the clock, with Dream answering in 0.65 s on average), and about 3 minutes for the scripted demo (476 steps, 23.8 s of simulated time, 163 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-J3KV8V. Its review said:

> Construct validity (not met): Pouring the six uncounted tablets directly into the vial earns credit without transferring any tablets from the specified trough.
>
> Scoring (not met): The partial scores reward tablets from the wrong side of the tray and permit all remaining tablets to be spilled.
>
> What to change: Revise the partial scores so transferring uncounted tablets or spilling the remaining tablets cannot earn credit. Specify how counted tablets and spills are verified in a real-world trial, given that the tablets look identical.

What changed in response:

- **Uncounted tablets earn nothing:** credit now counts only the 10 tablets that started in the trough. Any of the 6 uncounted tablets in the vial at any moment scores 0 for the whole trial (latched), so pouring them in can't earn credit. Full credit also needs all 6 still on the tray's counting area at the end.
- **Spills earn nothing:** any tablet, counted or not, on the table or the floor at any moment scores 0 (latched), as does a tablet that ends neither in the vial nor on the tray. Partial credit is only for clean progress: 0.5 for 5-9 counted tablets in the vial (or all 10 without the vial held up), 0.25 for 1-4. The vial must now be at least 2 cm off the table as each tablet goes in, not just the first.
- **Real-world check with identical tablets:** the referee counts exactly 10 in the vial and exactly 6 on the counting area. The three videos must show nothing in the trough, chute, on the table or on the floor, and no tablet entering the counting area. The 6 started apart on the counting area and the 10 in the trough, so these counts pin down which tablets moved.
- **Instruction and title** now state only the goal and constraints, with no method. New demo (1.0) and Dream-1 videos.
