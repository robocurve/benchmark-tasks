# Fish a cable through a conduit with a fish tape

A straight 30 cm conduit (square 26 mm bore) is fixed on a grey baseboard, running left to right in front of the
arms. A yellow 4.8 mm fibreglass fish tape is already threaded through it: its steel hook (a 4 x 4 mm prong standing
20 mm up at the tip of a 20 mm stem) sticks out of the right-hand (near) mouth, and its handle end sticks out of the
left-hand (far) mouth. An orange 6 mm cable lies on the table in front of the right arm with a steel pulling eyelet
(lug plus square ring with a 12 mm hole) on its free end. Electricians pull cable into a raceway they cannot reach
inside the same way: they fasten it to a fish tape already pushed through and pull the tape back. Here the right
arm hooks the eyelet's ring over the prong while the left arm holds the tape steady at the far mouth. Then the left arm pulls the tape back through hand over hand until the eyelet comes out of the
far mouth. Neither arm can do it alone. The right arm cannot reach the far mouth, the left arm cannot reach the
near mouth, and pushing the floppy cable through by hand is not allowed (and is scored 0).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).
  `task.py` adds one thing on top of the kit: extra box colliders on two end plugs. These are the eyelet ring on
  the cable's lug and the prong on the fish tape's hook stem, each a rigid part of its plug body.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Pull the orange cable through the grey conduit with the yellow fish tape: hook the cable's eyelet onto the fish-tape hook at the conduit's right-hand end, then pull the tape back out of the left-hand end until the eyelet is at least 5 cm out of the conduit."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

The demo is closed-loop: it reads the live cable and tape states, including the end-plug poses.
1. The left arm pinches the bare fish tape 4 cm out of the far mouth and holds it steady.
2. Meanwhile the right arm pinches the eyelet's lug and lifts it. It sets the lug down turned so that the ring
   points at the conduit, then re-pinches it with its wrist camera facing the hook.
3. The right arm lines the ring's hole up over the prong, measuring again just above the prong tip, and lowers it
   over the prong. It checks that the ring went on (up to 3 tries; the recorded run needed 1) and lets go
   (score 0.25).
4. The left arm pulls the tape back hand over hand in 20 cm strokes. Each stroke regrips the tape at least 7 cm
   beyond the hook. Three strokes bring the eyelet into the conduit (0.5), out of the far mouth (0.75) and 23 cm
   out (1.0).
Success holds for 1 s and ends the episode at step 854 (~43 s); task_score 1.0. `demo/` holds its videos,
`setup.png` and `result.json`.

Unintended strategies, selected with `FISH_DEMO=<mode>`:

| `FISH_DEMO=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 (eyelet not on the hook) |
| `tape_only` | left arm pulls the tape all the way through without the cable hooked on | 0 (the hook comes out of the far mouth empty; the cable never moves) |
| `push` | tape pulled out first, then the right arm pushes the eyelet into the empty near mouth by hand | 0 (eyelet not on the hook; the gripper is stopped by the conduit face with the ring ~5 mm inside) |

Pushing the eyelet in by hand and hooking it on at the far end later cannot be scripted with these arms, because
neither arm reaches both mouths. It is still scored 0 by the "inside the conduit while off the hook" rule, which
the offline scorer checks cover (`scratch/test_score.py`). Those checks also cover carrying the eyelet around the
outside and entering through the far mouth; both score 0.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

"On the hook" means the prong passes through the ring's hole. In the sim this is judged geometrically from the end
plugs' poses: the prong's axis (between its bottom and top) must cross the ring's plane within 9.5 mm of the ring
centre, measured along and across the ring. A prong through the 12 mm hole crosses at most 4 mm from the centre;
a prong outside the ring crosses at least 11 mm from it. `task.py` also reads the two end plugs' orientations from
the sim (through a hook on the embodiment's step), because the kit's cable state has positions only.

- **0** if, at any control step, the eyelet's ring centre was 2 cm or more inside the near mouth while the eyelet
  was off the hook for 0.3 s or longer. That means it was pushed in by hand, and the trial cannot recover. This is
  tracked in module state that resets at every episode start.
- **0** if the eyelet is off the hook at the end.
- **0** if the cable does not run from the eyelet back through the bore and out of the near mouth. Examples: the
  eyelet was carried around the outside of the conduit, or entered through the far mouth.
- Otherwise, with the eyelet on the hook:
  - **1.0** if the ring centre is at least 5 cm past the far mouth;
  - **0.75** if it is past the far mouth by less than 5 cm;
  - **0.5** if it is inside the conduit (at least 2 cm past the near mouth);
  - **0.25** if it is still at the near mouth and no part of the cable is inside the conduit.
- `distance()`: how far (m) the ring centre still has to travel to be 5 cm past the far mouth; 1 m after a
  by-hand push.
- `progress()`: stage diagnostics (hooked, entered, out, routed, not pushed). They do not affect the score.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes ~3–5 min of wall time on one GPU of an idle machine,
  including the ~1–2 min Isaac Sim start-up.
- The layout is fixed with `object_jitter=0`. Cables are never jittered (they always start straight).
  `VARIATION` in `task.py` lists the seeded ranges a later runner may apply.
