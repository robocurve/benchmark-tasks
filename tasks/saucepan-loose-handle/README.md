# Put two saucepans on the burners, leaving out the one whose handle is loose

Three 1-quart stainless saucepans stand in front of a cold double hot plate. One pan's handle screw has worked loose:
its handle rises 5 degrees against the pan when rocked, and looks like the others when left alone. Which pan it is
changes with the seed; nothing else changes. The robot is asked to put two pans on the burners and the third in the
fix-it tray, the tray with the screwdriver in it.

## Files

- `saucepan_loose_handle.py`: the scene, the seeded start, the scorer (`score()`), and the hooks for the demo.
- `layout.py`: every size, mass, friction and position, with its source.
- `pandemo.py`: the scripted demo, planned stage by stage.
- `bodykit.py`: multi-shape rigid bodies and joints for `yam_arena`.
- `dev/run.sh`: runs one trial; `dev/reach.py`, `dev/reach2.py`, `dev/ikcheck.py`: reach checks without the simulator.

## Run and score a trial

On a machine with Isaac Lab Arena and Inspect Robots (`yam_arena` embodiment), from this folder:

    inspect-robots run --embodiment yam_arena -E task=saucepan_loose_handle.py -E object_jitter=0 -E control_hz=20 \
      --instruction "Put two of the saucepans on the two burners of the hot plate so I can heat the soup and the sauce. Put the third one in the fix-it tray." \
      --scorer task_score --max-steps 2400 --policy <policy> --seed <seed>

`dev/run.sh <name> <policy> <seed> <steps>` does the same, waits for free GPU memory and keeps the logs.
The scripted demo is `--policy yam_scripted -P hold_s=1`; `DEMO_VARIANT=<name>` picks a weaker plan (below).

Seeds: the framework turns `--seed N` into an episode seed; the loose pan's slot is the episode seed mod 3. Seeds 0, 1
and 2 put it in slots 2, 3 and 1 (middle, right, left). The run prints the slot (`pandemo: seed ...`).

Runtime: the demo takes 1,391 control steps (70 s of simulated time); 2,400 steps at most. On one shared GPU with four simulations running, a 1,400-step trial took 18 minutes of wall time.

## The test rule

Rocking a handle leaves no trace in the end state, so the scorer keeps one whole-trial record: credit needs the loose
pan's grip rocked up 3 degrees or more against its pan and back down to 1 degree or less, while that pan still stands
at its start place (base centre within 2 cm, on the counter, upright) and before any pan has touched the hot plate.
The play is 5 degrees; the demo's rock lifts the grip to 5.0 degrees. In a real trial the same rule is read from the
video: a gripper on that handle, the handle's tip moving about 6 mm or more against the pan's bracket, the pan staying
put. Without it the run scores 0, even with the right pans in the right places. Rocking the two sound handles is the
careful way but is not scored: a sound handle does not move against its pan, so neither the video nor the simulator
can tell a rocked one from one left alone.

## The scripted demo

It reads the pans' poses from the simulator to plan its moves, and, while it rocks a handle, how far that handle turns
against its pan (a sound handle cannot turn). It moves the pans only through the grippers, by contact. First it tests
every pan: one hand pinches the rim and the other rocks the handle up 16 mm twice; a handle that rose 2 degrees or more
is loose. Then the loose pan is carried by its rim to the tray, and each sound pan by its rim to a burner, turned by its
handle 80 degrees away from the front edge (the turn leaves it 1 to 2 cm off the plate's centre).

## Results

All runs on the final code (same files, same hashes), `dev/run.sh`, run limit 2,400 steps. The part-done plans are
`DEMO_VARIANT` runs of the same demo; each score was written down before the run.

| Run | Seed (loose slot) | Predicted | Score | Steps |
|---|---|---|---|---|
| Scripted demo (full) | 0 (2) | 1.0 | 1.0 | 1,391 |
| Scripted demo (full) | 1 (3) | 1.0 | 1.0 | 1,391 |
| Scripted demo (full) | 2 (1) | 1.0 | 1.0 | 1,391 |
| `fixedguess`: no rocking, slot 3 to the tray, slots 1 and 2 to the burners | 0 (2) | 0 | 0.0 | 917 |
| `fixedguess` (its guess is right: every pan in the right place) | 1 (3) | 0 | 0.0 | 917 |
| `fixedguess` | 2 (1) | 0 | 0.0 | 917 |
| `untested`: first two pans to the burners, no rocking (both sound on this seed) | 1 (3) | 0 | 0.0 | 734 |
| `handlesout`: full test and placing, handles left pointing at the robot | 0 (2) | 0.8 | 0.8 | 1,111 |
| `nothing`: hands stay at the ready pose | 0 (2) | 0 | 0.0 | 131 |
| Dream-1 | 0 (2) | - | 0.0 (no pan moved) | 2,400 |

On seed 1 the fixed guess put the loose pan in the tray and both sound pans on the burners with handles turned in;
without the test rule it would score 1.0. It scores 0 because no handle was rocked.

## Sources (opened 2026-11-20)

- CPSC recall 24-126 (February 22, 2024), camping cooking pots, about 60,655 units,
  https://www.cpsc.gov/Recalls/2024/Cascade-Designs-Recalls-Camping-Cooking-Pots-Due-to-Burn-and-Scald-Hazards:
  "The pot handle can detach or become loose, posing burn and scald risks to consumers"; 22 reports, two with burns.
- Cookware scalds and burns, CPSC NEISS data charted by UTMB Health,
  https://www.utmb.edu/news/article/health-blog/2022/11/21/non-fire-cooking-burns
- Black+Decker cookware help page,
  https://support.blackanddecker.com/hc/en-gb/articles/11288005186445-Cookware-What-do-I-do-if-the-screws-on-my-glass-lid-or-handle-come-loose
- Saucepan: Farberware Classic Series stainless steel 1-quart saucepan, model 50000,
  https://www.amazon.com/Farberware-Classic-Stainless-1-Quart-Saucepan/dp/B000058AK7 and
  https://www.lehmans.com/product/farberware-classic-stainless-steel-saucepan-1-qt ("Handles: Screw Welded Phenolic").
- Hot plate: Elite Gourmet EDB-302BF double cast-iron burner, https://www.amazon.com/dp/B09RX2WGD3 and its manual.
- Tray: Nordic Ware Naturals quarter sheet pan, https://www.nordicware.com/naturals-aluminum-bakeware/

How to set the loose handle for a trial: back the handle screw off one turn, then check with a ruler that the grip's tip
lifts about 6 mm and drops back, and that it rests at the same angle as the two sound handles. The 5 degree play in the
simulation is an estimate until it is measured on a real pan this way.
