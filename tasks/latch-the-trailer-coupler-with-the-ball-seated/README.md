# Hitch the trailer: seat the coupler on the ball, then latch it

**Instruction:** "Hitch the trailer to the ball."

A utility trailer's ball coupler, modelled on the **CURT 25153** (2 in straight-tongue coupler with Posi-Lock latch,
2 in ball, 3,500 lb GTW, SAE J684 class 2, zinc; curtmfg.com/part/25153), hovers 25 mm above a **2 in hitch ball** on a
ball mount, latch handle up. The robot must (1) push the coupler straight down until the ball is fully seated in the
socket, and only then (2) push the handle down until it latches, driving the underjaw in under the ball.

## The scored end state is the latch

Lowering the coupler onto the ball is only part credit. Full credit needs the trailer *hitched*: seated **and** latched,
with the arm off the part. That is CURT's own order ("with the coupler fully seated on the ball, engage the latch",
curtmfg.com/hooking-trailer), and its own failure check: "If it comes off the ball, it means that the coupler was not
properly seated before being latched."

| End state (held 1 s, at rest) | Score |
|---|---|
| Seated (≤ 0.5°, socket ≤ 1.5 mm from the ball centre) **and** latched (handle ≤ 3°, underjaw within 2 mm of its 65 mm stroke), no gripper point within 3 cm of the coupler | 1.0 |
| Latched with the ball **not** seated (any time it is true) | 0 |
| Seated, not latched | 0.5 |
| Lowered onto the ball (≤ 3°), not seated | 0.25 |
| Otherwise | 0 |

The score is recomputed from the current state, so undoing the latch drops it again.

## Why the order matters physically

The underjaw is a real slider with two tines and a crossbar under the socket. When the handle is pushed down while
the coupler still hovers, the tines run into the ball's lower half, the jaw stops, and the handle jams part-way
(about 13–15° in sim). The coupler can then still be pushed down and seated, but the handle cannot be finished:
0.5. Pushing the coupler down only part-way and then latching jams the same way (the sim `short` control).

## Parts and real-world counterparts

| Sim part | Real part |
|---|---|
| Zinc socket head, 52.4 mm pocket, flared skirt, top plate | CURT 25153 coupler body (10-3/4 × 2 × 4-1/4 in) |
| Yellow handle, 5.5 cm, hinged on top of the socket | Posi-Lock latch handle |
| Red underjaw slider, 65 mm stroke | Coupler underjaw / latch tongue |
| Blue 2 in channel, 20 cm, pivoting on a stand | Front of the trailer A-frame tongue (the jack holds the rest) |
| 2 in chrome ball, 3/4 in shank, on a 14 × 12 cm base | 2 in hitch ball on a ball mount |

## Fidelity, honestly

- Ball, socket ring, skirt and jaw tines are rigid colliders at real size (0.4 mm radial clearance in the pocket).
- The tongue is one light hinged link (0.35 kg) on a fixed pivot so a YAM arm can push it, not a 200 kg trailer tongue
  resting on a jack. Its 10° range stands in for winding the jack down.
- The Posi-Lock linkage from handle to underjaw is a **position drive in code**: the jaw target follows the handle
  angle and saturates fully in at 2° from closed. The jaw itself is a physical slider, so it is blocked by the ball when
  the coupler is not seated; it is never teleported.
- Joint start states are written in `@m.on_reset` (mechkit's `start=` is not applied by the sim).
- No safety pin or chains; the task ends at the latch.

## Policy controls (scripted, `TASK_DEMO=<variant>`)

| Variant | What it does | Expected |
|---|---|---|
| demo | coupler down, then handle down, arm to rest | 1.0 |
| naive | handle first (jams), then coupler down | 0.5 |
| plate | coupler down only | 0.5 |
| short | coupler 10 mm of 25 mm down (it stays there, 3.9° up), then handle | 0.5: the handle push forces the coupler down and seats it, but the jaw has already met the ball and jams at 15° |

`test_task.py` (offline, no simulator) checks the untouched start state (0), the scoring table, the 1 s hold, the 3 cm release check, undoing, the
latched-not-seated zero (a development run also checked it in sim with the ball moved 45 mm aside: 0.0), the socket
geometry and the task.json limits.

## Reach

All pushes are straight down with the left arm at x 0.38–0.40 m, y 0.16 m, TCP 0.10–0.22 m above the bench. An offline
straight-down IK check of the arm puts the first touch of the open (60°) handle's end pad near the edge of the arm's straight-down envelope;
the arm in the sim reaches it, and the scripted demo scores 1.0. A real trial should check that pose first; the
fallback is to move the whole fixture to about (0.36, 0.22) with no other change.

## Not a duplicate of

- **#1676** (lowering a trailer tongue onto the ball): there the end state is the coupler on the ball. Here lowering is
  0.25; seating 0.5; only the latch, in the right order, is 1.0, and the reverse order is the failure mode.
- **#2841** (custom latch fixture, rejected as niche): this is a named, catalogue consumer part (CURT 25153 on a 2 in ball)
  sold for ordinary utility trailers.
- **#2584** (7-way trailer plug) and other towing tasks: electrical connection, not the mechanical hitch.

## Real trials (no welding, no trailer needed)

| Item | Approx. cost |
|---|---|
| CURT 25153 coupler | $30 |
| 2 in hitch ball (3/4 in shank) | $15 |
| Ball mount or a 1/4 in steel plate to bolt the ball to | $20 |
| 2 in channel stub (2 ft) + hinge/pivot block, bolted to the bench | $35 |
| Gas strut or counterweight so the arm can lower the tongue | $20 |
| Cameras and tape | $80 |
| **Total** | **about $200** |

## Scores

- Scripted demo (yam_arena, 320x176, seed 1): 1.0, success at step 375 of 700.
- TASK_DEMO=naive: 0.5 (handle jams at about 15°; the coupler is then seated but cannot be latched).
- TASK_DEMO=plate: 0.5.
- TASK_DEMO=short: 0.5.
- Dream-1: not run yet.
