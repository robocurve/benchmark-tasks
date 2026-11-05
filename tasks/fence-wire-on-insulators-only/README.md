# Hang an electric fence wire on insulators only

A short electric-fence line stands 30 cm in front of the robot: a grey corner post on the far left and three grey
line posts (25 x 25 mm, 18 cm tall) at y +15, 0 and -15 cm. Each line post carries a white offset insulator whose
open-top slot sits 3 cm in front of the post, 15 cm above the table. A white 6 mm polyrope fence wire is tied off at
a white strain insulator on the corner post; at the start it hangs from there to the table and lies on it, with a
yellow gate handle on its free end in front of the right arm. The robot must hang the wire in all three insulator
slots so that no part of the wire or handle touches a post (a live wire touching a post leaks to ground) or the
table, and let go, with the handle hanging free past the last post.

The defining constraint is contact: the wire must rest on the insulators and nowhere touch a post body, along its
whole length. Draping it over the post tops, missing one insulator, or leaving the tail against a post all score 0
(the last case checked only offline, see below).

Two arms: the layout puts the left insulator on the left arm's side and the right insulator on the right arm's side,
so that one arm cannot lay the whole wire alone (design intent; reach was not checked separately). The right arm
holds the handle and keeps the wire under tension while the left arm takes the wire near the corner; both lift it
level above the slots, pull it straight and lower it into all three slots at once.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "This electric fence wire will be switched on later. Hang it in the white insulator slots on all three posts so that no part of the wire or its yellow handle touches a post or the table: keep it at least 1 cm from every post and let the handle hang free past the last post. Then let go of the wire."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed loop on the live wire state: the right arm takes the handle and lifts it beyond the right post; the left arm
takes the wire 7.5 cm from the corner. Both raise the wire 6 cm above the slots and 4 cm in front of them, the right
arm pulls outwards until the measured sag is small (or its reach limit), both move it over the slot line and lower it
into the three slots. The left lets go; the right lowers the handle so it hangs past the right insulator, lets go and
backs away. Wrist cameras face along the post line towards the insulators (left arm -y, right arm +y).
Success holds for 1 s and ends the episode at about step 500 (~25 s).

Shortcut variants (test only, `FENCE_CHEAT=<name> python cabledemo.py task.py out`); each scored exactly 0 in a
simulated run on the final code:
`none` (do nothing), `tops` (drape the wire over the post tops: it touches them and is in no slot), `two` (full
lay, then the right arm lifts the wire out of the right insulator and lets the handle hang in front of it: wire in
two slots only), `touch` (park the handle on the right insulator's standoff arm against the post: it touches the
post; the move also lifts the wire out of the right slot), `hold` (full demo, but the right arm keeps holding the
hanging handle). In the sim a tail that merely hangs falls back clear of the post, so no simulated run produced a state
with the wire in all three slots *and* close to a post. That case was checked only offline, by calling `score()` on
hand-made synthetic wire states without running the simulation (handle against the post, or tail along the post side:
0; tail 3-10 mm from the post: 0.5; handle on the table: 0). So the 0 and 0.5 post-clearance outcomes for a wire
lying in all three slots have not been exercised in a simulated run.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** unless: both grippers are at least 3 cm from every part of the wire and handle; part of the wire lies inside
  each of the three slots (between the walls, at most 10 mm above the floor); no part of the wire or handle is within
  3 mm of any post body (corner post included; insulators may be touched); every part of the wire and handle is at
  least 2 cm above the table.
- Then **1.0** if the smallest wire/handle-to-post gap is at least 1 cm, else **0.5** (gap 3-10 mm).
- A non-finite wire state (physics blow-up) scores 0.
- `success()` also requires the wire at rest (mean segment velocity under 2 cm/s, mean spin under 0.5 rad/s).
- `distance()`: empty slots + metres short of 1 cm clearance.
- `progress()`: slots filled, posts clear, off the ground, released.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). The scripted demo ran at about 2 steps/s on one GPU (506 steps in about
  4 min including Isaac Sim start-up); a run that uses all 2,400 steps takes roughly 20 min.
- Layout is fixed with `object_jitter=0`; the wire always spawns the same way (straight at wire height from the corner
  insulator, then falls onto the table), so every trial starts identically.

## Simulation stand-ins

The polyrope is a chain of 27 rigid segments of about 2 cm with bending springs (no twist, no stretch); the tie-off at
the corner block is a rigid weld and the handle a rigid box welded to the wire end. The wire spawns straight at wire
height and falls onto the table. Posts and insulators are fixed rigid boxes (printed PLA in a real build); nothing is
electrified, so "touching" a post is a geometric gap check (under 3 mm), and real trials use an unpowered wire and a
feeler gauge.

## Sources

- AHDB Beef & Lamb, "Electric fencing for livestock": "Insulators are used to prevent the conductor or
  wire touching the posts or stakes, which could lead to power leakage."
  https://projectblue.blob.core.windows.net/media/Default/Imported%20Publication%20Docs/BRP-plus-Electric-fencing-for-livestock.pdf
- NJDEP Fish & Wildlife, "Electric fencing" guide: "Both wooden and metal posts require post insulators to prevent the
  wires from touching the posts and thereby grounding the system."
  https://dep.nj.gov/wp-content/uploads/njfw/electric-fencing-guide.pdf
- O*NET 45-2093.00 Farmworkers, Farm, Ranch, and Aquacultural Animals, task: "Inspect, maintain, and repair
  equipment, machinery, buildings, pens, yards, and fences." https://www.onetonline.org/link/summary/45-2093.00
