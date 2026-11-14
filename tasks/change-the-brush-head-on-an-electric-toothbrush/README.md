# Replace an electric toothbrush's worn brush head with the new one from its pack

An electric toothbrush lies on its back on a dark grey granite bathroom counter, its worn round brush head still on its
steel shaft. To its right lies the new head, shut in its reclosable clear PET clamshell, and a small pedal bin stands
near the front right. The job is the whole real one: open the pack, take the worn head off and drop it in the bin, take
the new head out and push it all the way on, then bring both arms back. The task is one task file,
[`change_the_brush_head_on_an_electric_toothbrush.py`](change_the_brush_head_on_an_electric_toothbrush.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: opening light snap-shut packaging, which has to be held while it is opened; holding an object still
against a pull and then a push; taking a part off and putting its replacement on; lining a part up with a shaft.

Why two hands: the clamshell's two press studs hold its lid shut until it is turned with 0.18 N m (3.2 N at the studs),
but the whole pack weighs 12 g, so pulled with nothing holding its tray, the pack comes along with the lid. And each
brush head is a push fit that takes about 3 N to pull off or push on, while the 140 g toothbrush slides along the
counter at about 0.7 N. With one hand (the right doing exactly its part of the demo, the left never moving) the demo
scored 0: the pull on the lid dragged the whole pack 3 cm across the counter with the lid still shut (it never opened
more than 0.1 degrees), the pull on the worn head dragged the toothbrush 4.9 cm with the head still on, and the hand
then carried the toothbrush off by its head.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The counter and the bin are kinematic, fixed where
they are put; the toothbrush, the two heads and the clamshell's tray and lid are loose rigid bodies made of shapes from
`bodykit.py`, at their real sizes and masses (each head 5 g, tray and lid 6 g each).

- The clamshell: the tray and the lid are joined along the tray's right-hand edge by a hinge (a PhysX revolute joint).
  The press studs are a detent on that hinge: a stiff spring presses the lid shut, capped at 0.18 N m, so the lid
  stays shut until it is turned harder than that; once it has opened 1.5 degrees (1.5 mm at the studs) the studs are
  out, and the hinge becomes the pack's moulded living hinge, a weak spring (4e-5 N m per degree) that swings the lid
  open to its 120-degree stop. Pushed back to within 0.8 degrees of shut, the studs catch again. The real PET is about
  0.35 mm thick; its walls are modelled 2 mm thick (the lid's bubble 4 mm, so a squeezing gripper can't push through)
  at the pack's real mass. They collide but are drawn only along their edges, so the head inside shows.
- The push fit: each head is held on the shaft by a `Thread` from `bodykit.py` (a joint that keeps the head on the
  shaft's axis and lets it slide along it), pulled by a stiff spring toward a hold point. After every control step the
  task file moves the hold point to wherever the head has been pushed or pulled more than 1 mm from it, so a head
  slides only against about 3 N and stays where it is left (stick-slip). The worn head starts on the shaft; once
  pulled out past the shaft's end (2.8 cm), its joint switches off and it is free. A head whose collar is pushed onto
  the free shaft's end (within 1 cm of its axis and 15 degrees of its line) catches the same way. The heads do not
  collide with the handle, so a head's collar can slide over the shaft.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Scoring

All judged at the end of the trial, on live states, never latched. 1.0: the new head all the way on (its base within
3 mm of the shaft's base), the worn head in the bin and both arms back where they started (each gripper's fingertip
centre within 5 cm of its start); 0.75: the same with an arm not back; 0.5: the new head all the way on; 0.25: the
pack open (studs out, lid at least 60 degrees open) and the worn head off the shaft; else 0.0. The new head can only
get out through the opened lid, and it only catches on the shaft once the worn head is off. In a real trial: the new
head is all the way on when it has clicked down and the gap between its collar and the handle is under 3 mm (ruler);
the worn head must lie in the bin; the pack is open when its lid is off both studs and stands at least 60 degrees
open (video or protractor); the arms are checked against marks at their start.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the toothbrush, both heads, the tray and the lid
still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same layout.
The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it,
so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=change_the_brush_head_on_an_electric_toothbrush.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Replace the electric toothbrush's worn brush head with the new one from its pack, and put the worn one in the bin. Finish with both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/brush-dream
```

The scripted demo, which reaches full score: the left hand holds the clamshell's tray across its thumb rest and leans
it to the left, while the right takes the lid's bubble near the hinge and pulls it over toward the hinge (aiming 3.2 cm
past, as the arms give): the studs let go, the right lets go and rises, and the hinge swings the lid open. The left
sets the tray back where it was and takes the toothbrush's handle near its middle, its grip pitched back so its wrist
stays clear of the right's. The right, pitched the other way (fingers ahead of the wrist, to reach the shaft's end),
takes the worn head by its neck, pulls it straight off the shaft (aiming 2 cm past, as the arm gives under the push
fit), lifts it and drops it in the bin. The left lets go of the handle and stands aside while the right takes the new
head out of the tray by its neck (the two wrists would meet over the tray), then holds the handle again while the right
lines the head up in front of the shaft and pushes it straight on, all the way. Both hands let go and both arms go back
to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=change_the_brush_head_on_an_electric_toothbrush.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Replace the electric toothbrush's worn brush head with the new one from its pack, and put the worn one in the bin. Finish with both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/brush-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/brush-dream/*.json; do inspect-robots video "$log"; done
```

Set `BRUSH_DEBUG=1` to print what the score is built from four times a second: the lid's angle and whether the studs
are in, where the tray, the toothbrush and both heads are, how far each head is along the shaft and whether its joint
is on, how far the new head is from the shaft's axis and at what angle, whether the worn head is in the bin, the
finger openings and where each TCP is.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,200 steps (461 s on the clock, with Dream answering in 0.67 s on average), and about 5 minutes for the scripted demo (715 steps, 35.8 s of simulated time, 309 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-UMKUUZ. Its review said:

> Task appears simple as-is, would be more impressive if the task involved taking the new toothbrush head out of its packaging

What changed in response:

- **The new head now has to come out of its packaging:** it starts shut in a reclosable clear PET clamshell (13 x 6.4
  x 4.8 cm, 12 g), lying on two cradles under the lid's bubble. The lid is hinged along one long edge; two press studs
  along the other edge hold it shut until it is turned about the hinge with 0.18 N m (3.2 N at the studs). Once it
  has opened 1.5 degrees the studs are out and the moulded living hinge swings the lid open to its 120-degree stop;
  pushed back shut, the studs catch again. Nothing is peeled, torn or cut. The whole pack weighs 12 g, so a hand that
  pulls the lid drags or lifts the pack unless the other hand holds the tray, as a person's would.
- **Real sizes and masses:** each brush head is now 5 g (it was 10 g), the pack 12 g (tray 6 g, lid 6 g), the
  toothbrush 140 g. The only deviation: the PET walls are modelled 2-4 mm thick instead of about 0.35 mm, at the real
  mass, so the contacts are stable. This is stated in fidelity.
- **Instruction, title, category:** the instruction now gives only the goal and a constraint: "Replace the electric
  toothbrush's worn brush head with the new one from its pack, and put the worn one in the bin. Finish with both arms
  back where they started." It no longer says which hand does what, or in what order. The title names the job only,
  and the category is Societal relatability.
- **Scoring from live joint states:** the push fits and the lid's studs are joints, and their state is read at the
  end; nothing is latched and nothing is credited for proximity. 1.0: the new head all the way on (its base within
  3 mm of the shaft's base), the worn head in the bin and both arms back; 0.75: the same with an arm not back; 0.5:
  the new head all the way on; 0.25: the pack open (studs out, lid at least 60 degrees open) and the worn head off the
  shaft. Each tier states its real-trial check: a ruler gap under 3 mm at the collar, the worn head lying in the bin,
  the lid off both studs and at least 60 degrees open by video or protractor. success() needs the full score with the
  toothbrush, both heads and both halves of the pack still.
- **Demo:** fixed waypoints only. The left holds the tray by its thumb rest while the right pulls the lid over toward
  its hinge until the studs let go. The left then holds the handle while the right pulls the worn head off and drops
  it in the bin, and again while the right pushes the new head on. It ends with the new head on, the worn head in the
  bin, the pack open and both arms home. New demo (1.0) and Dream-1 (0.0) videos.
- **One hand:** with the right doing exactly its part of the demo and the left idle, the score is 0. The pull on the
  lid dragged the whole pack 3 cm with the lid still shut (it never opened more than 0.1 degrees), and the pull on the
  worn head dragged the toothbrush 4.9 cm with the head still on.
