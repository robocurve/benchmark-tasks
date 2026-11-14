# Shelve the light cartons and slide the one too heavy to lift to the person

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [heavy_carton_handoff.py](heavy_carton_handoff.py).

Three identical cardboard cartons stand on a stockroom bench. Two hold packing paper (0.3 kg); one is packed with lead fishing weights (2 kg), more than the arm can raise to the 14 cm shelf from where the cartons stand. Which slot holds the heavy carton is drawn from the trial's seed at every reset, so the cartons' positions say nothing: the robot finds out only by test-lifting them. It should shelve the light cartons and slide the heavy one forward onto the yellow handoff mat in front of the slots, where the colleague can lift it.

## Scoring

`score()` runs every control step:

- **Test lift:** the heavy carton counts as test-lifted once its base has been at least 1 cm above the bench while it was off the mat.
- **Drop:** the heavy carton counts as dropped if it falls faster than 0.6 m/s with its base more than 2 cm up (about a 2 cm fall).
- **Score at the end:** 0 if any carton is tipped more than 30 degrees or off the bench, a light carton is on the mat, or the heavy carton was dropped. Otherwise 0.25 for each light carton at rest wholly on the shelf top, plus 0.5 for the heavy carton upright and at rest wholly on the mat if it had been test-lifted first. "Wholly" allows a 1 cm overhang, for the shelf and the mat alike. The heavy carton anywhere else, including on the shelf, adds nothing.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=heavy_carton_handoff.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the cartons on the shelf. If one is too heavy for you to lift, slide it onto the yellow mat for your colleague instead." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/handoff-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted`. Each arm test-lifts the carton in front of it towards shelf height and reads from its own joint angles whether the carton came up (a 2 kg carton stalls about 3 to 4 cm below the commanded height; 0.3 kg follows within 1 cm). The left arm shelves the light cartons and sets the heavy one back down and slides it onto the mat, pushing low with its fingers half open so the carton runs straight. The right arm slides its carton onto the mat if it is heavy; if light, it holds it clear while the left arm works the middle slot, then sets it into the emptied middle slot for the left arm to shelve. The arms take turns near the middle so they never work side by side. The demo decides only from the arms' joint positions, which a policy observes, not from the simulator's state.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the cartons end up.

`HEAVY_SLOT=0..2` fixes the heavy carton's slot (0 is the left one) and `HEAVY_DEBUG=1` prints the cartons' positions and tilts every step.

## Checks

The demo with the heavy carton in each slot, and with slots drawn from the seed:

| Run | Heavy carton's highest rise | Score |
|---|---|---|
| Slot 0 (left arm) | 11.8 cm | 1.0 |
| Slot 1 (left arm) | 10.5 cm | 1.0 |
| Slot 2 (right arm) | 12.2 cm | 1.0 |
| Three trials, slots drawn from the seed | | 1.0, 1.0, 1.0 |

Wrong strategies, scripted with `HEAVY_VARIANT` (heavy carton in slot 0):

| Variant | Score |
|---|---|
| Slide the heavy carton onto the mat without test-lifting it | 0 |
| Slide a light carton onto the mat | 0 |
| Shelve both light cartons, leave the heavy one where it was set down | 0.5 |
| Nothing done | 0 |

## Real trials

The heavy carton is 2 kg, within the arm's rated payload (about 2 kg). In the simulator the arms stall with it at these slots; a real arm may hold position more stiffly and raise it, so real trials limit the elbow's torque. The limit comes from the joint torques logged in the simulator during the demo's test lifts and pushes:

| Phase (demo, slots 0, 1 and 2) | Elbow (joint 3) | Shoulder (joint 2) |
|---|---|---|
| 0.3 kg carton raised to 14.7 to 15.0 cm (6 lifts) | 5.8 to 6.0 N m held, 6.3 N m peak | 6.8 to 9.6 N m held, 10.9 N m peak |
| 2 kg carton test lift, stalls at 10.4 to 12.2 cm (3 lifts) | 9.7 to 9.8 N m held, 10.2 N m peak | 11.6 to 16.2 N m held, 16.8 N m peak |
| 2 kg carton pushed onto the mat, friction 0.4 as for cardboard on a steel bench (slots 0 and 2) | 6.1 to 6.8 N m median, 7.0 N m peak | |

The simulator's own friction (1.2, rubber-like) makes the push need 8.9 to 10.2 N m at the elbow; at the cardboard-on-steel friction of a real bench (0.4, measured in the same demo with that friction set) it needs at most 7.0 N m. Lifting does not depend on friction.

Only the elbow separates the two loads (the shoulder's light-lift peaks come within 2 N m of the heavy lift), and an elbow limit between 7.0 and 9.6 N m lets the arm shelve the light cartons and push the heavy one but not raise it, so:

1. With the arms unloaded, set the elbow (joint 3) torque limit of both arms to **8 N m**, midway between the light lift's 6.3 N m and the heavy lift's 9.7 N m. Leave the other joints at their defaults. If the controller takes a current limit, convert 8 N m with the elbow motor's torque constant and gear ratio from its datasheet.
2. With an operator's hand on the emergency stop, put a 0.3 kg carton in each slot in turn and run the test lift to 14 cm: it must reach 14 cm at every slot.
3. Do the same with the 2 kg carton: it must stall below 14 cm at every slot (in the simulator it stops 2 to 4 cm short). The arm should hold the carton or lower it slowly; stop the arm if it drops faster.
4. Still at that limit, push the 2 kg carton from each slot onto the handoff mat with the fingers half open, low on its back face: it must reach the mat at every slot.
5. If a 0.3 kg carton falls short or the push stalls, raise the elbow limit in 0.5 N m steps and repeat steps 2 to 4, never above 9.5 N m (just under the heavy lift's 9.7 N m). If no setting passes steps 2, 3 and 4 together, do not run trials.
6. During trials the operator stops the arm if a carton rises above 15 cm or the arm faults. Pick the heavy carton's slot with a die before each trial (1-2 left, 3-4 middle, 5-6 right).

## Runtime and seeds

- Demo: 880 to 1,190 steps (44 to 60 s simulated) depending on the heavy carton's slot (the handover through the middle slot takes longest), three to five minutes on an L40S. The recorded demo has the heavy carton in the right slot.
- Dream-1: 2,400 steps (2 minutes simulated), about 7 minutes on an L40S.
- Each trial draws the heavy slot from its seed (`inspect-robots run --seed N` repeats a sequence of layouts); `object_jitter=0` keeps every other position fixed.

## Licence and third-party material

This task is released under the MIT licence, including [trialkit.py](trialkit.py), a helper written for this task: it draws the per-trial layout from the trial's seed and runs the demo in stages from the policy's observations.

No third-party models, textures or code are used: every object is a box or cylinder declared in the task file.
