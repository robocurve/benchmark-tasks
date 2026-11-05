# Shelve the light cartons and slide the one too heavy to lift to the person

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [heavy_carton_handoff.py](heavy_carton_handoff.py).

Three identical cartons go on a 14 cm shelf, but one is full of steel screws (1.5 kg) and the arm cannot raise it that high. The robot only finds out by trying; it should shelve the light cartons and slide the heavy one into the handoff zone at the colleague's end of the bench, beside the shelf. `score()` gives credit for each carton in the right place and 0 if a light carton is handed off or any carton tips over.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/heavy-carton-handoff/heavy_carton_handoff.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the cartons on the shelf. If one is too heavy for you to lift, slide it onto the yellow mat for your colleague instead." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/handoff-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`. The left arm shelves the two light cartons, tries to lift the third, finds it will not rise to shelf height, sets it back down and slides it along the bench into the handoff zone, pushing low so it slides rather than tips, then moves clear of the camera's view.

The run prints `task_score` at the end: 0.25 per light carton on the shelf plus 0.5 for the heavy carton in the handoff zone, and 0 if a light carton is handed off or any carton tips. The task defines no `success()`, so the trial is scored on where the cartons end up.

## Runtime and seeds

- Demo: 877 steps (44 s simulated), about two minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop; the simulator is not bit-exact, so where the pushed carton stops varies by a few centimetres between runs, well inside the handoff zone.
