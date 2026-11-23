# Return a shopping trolley: push the chain key of the trolley in front home in your coin lock, then take the quarter back

Return the trolley: push the chain key of the trolley in front into this trolley's coin lock until it locks, then take out the quarter the lock gives back and lay it on the rubber mat.

Supermarkets that lend trolleys for a coin (in the US, a quarter) chain them together: each trolley's lock holds the key of the trolley in front. To get the coin back you nest your trolley behind the last one, pick up that trolley's key, which lies at the end of its 26 cm steel chain, turn it the right way round and push it fully into your lock. Only then does the lock latch the key and push your coin out, half of it standing clear, to be pinched by its faces and drawn out. The key is a thin flat steel plate linked to the other trolley by a chain that drags and swings while you line it up with a 17 x 3.5 mm slot, and it must go about 25 mm deep against the lock's springs before anything happens. A key pushed part way earns nothing; the coin cannot be reached until the lock lets it go.

## Scoring

Judged at the end of the run. In the real world: look at the key and the coin, and give the key one firm pull.

| End state | Score |
|---|---|
| Key locked in the near lock, quarter resting on the mat (either way up) | 1.0 |
| Key locked, quarter out of the lock but not resting on the mat (dropped elsewhere, still held above it) | 0.75 |
| Key locked, quarter still in the lock's pusher | 0.5 |
| Key not locked, but in the near lock's slot in line with it, at most 53 mm of the 58 mm key sticking out | 0.25 |
| Anything else (key on the table, in the funnel only, in the wrong lock) | 0 |

"Key locked" is what a person checks by pulling the key: in simulation the key's tip is 22.5 to 25.5 mm past the
slot's mouth (the plunger's stop is at 24.5 mm; held by the ball it backs off to about 22.6 mm), the locking ball is down in
the key's hole, and the coin pusher is out (within 4 mm of its 30 mm travel). "In line" is the tip within 4 mm of the
slot's centre line across and 3 mm in height, the key pointing into the lock; the 0.25 tier's tip at least 5 mm past
the mouth is the same as at most 53 mm of the key outside. "Resting on the mat" is the coin's
centre over the mat (4 mm inside its edge) and its lowest point within 3 mm of the mat's top. "Out of the pusher" is the coin's centre more than 6 mm from its pocket along the pusher or lifted
6 mm out of it. `success()` is a score of 1.0. The tiers do not overlap, every end state falls in one, and at the
start (key on the table, coin locked in) the score is 0.

Why the shortcuts fail: the coin cannot be reached while the lock holds it (the 2.6 mm coin slit is too narrow for a
finger), and the lock only lets it go when the key is pushed all the way home. Measured in simulation: a key slid 17 mm in by hand and left there (the plunger, 18 mm in, never touched) scores 0.25, with the lock shut and the quarter still inside.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=return_a_trolley_key_into_coin_lock_take_quarter.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Return the trolley: push the chain key of the trolley in front into this trolley's coin lock until it locks, then take out the quarter the lock gives back and lay it on the rubber mat." \
  --scorer task_score --max-steps 2400 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model policy, replace `--policy yam_scripted` with the policy. A trial is at most 2,400 control steps (120 s at
20 Hz); the scripted demonstration takes about 40 s. Run the offline checks with `pytest` in this folder. Keep
`object_jitter=0`: the one loose object at the start, the quarter, is held inside the lock, where a random shift would
put it into the lock's walls.

## The scene

Millimetres from the midpoint between the arm bases: x forward, y to the robot's left, heights above the table top. Two Malltek MT-STL04 shopping-cart coin locks (black and red plastic body 90 long x 70 wide x 68 high, 260 mm stainless chain and flat stainless key, takes coins 23-28 mm across) are each clamped on a 300 mm length of 25 mm steel handle tube. Each tube runs along y, its axis 100 mm above the table, on two steel posts 280 mm apart; the locks sit in the middle of their tubes. The returning trolley's handle (near) is at x 150, y -110; the trolley in front's handle is 178 mm further on, at x 328, y -110 (the nesting distance of a tight-nesting cart). On each lock the key slot (17 x 3.5 mm behind a 6 mm funnel, floor 116 above the table) is in the end facing -y and the coin pusher is on the side facing the arms. The near lock holds a US quarter (24.26 mm, 1.75 mm, 5.67 g) captive, on edge in its pusher; its key slot is empty. The front lock's chain leaves its -y end 20 mm toward the near lock and 121 mm up, hangs to the table and lies 140 mm along -y; the key lies flat at its end, tip pointing -y, tip at x 308, y -354. A black rubber mat 110 x 110 x 2 mm lies at x 60, y -40.

## Scripted demonstration

The right arm does everything; the left arm stays at rest. In simulation it reaches 1.0 at step 609 (30.5 s) and the run stops at success: the key is held 22.6 mm in, the ball down, the pusher out, and the quarter rests on the mat.

1. Fingers pointing straight down, closing across the key's 15 mm width 50 mm behind its tip, the hand stops with the
   pad tips 8 mm over the table, reads where it really is, corrects sideways and lowers the tips to 0.5 mm over the
   table; the pads pinch the key's 3 mm long edges.
2. It lifts the key, chain trailing, and turns it half a turn on the way to the near lock, so its tip points into the
   slot.
3. It holds the key 15 mm short of the slot, reads where the tip really is in its fingers and corrects; puts the tip
   3 mm into the funnel, reads and corrects again across the slot; slides it 17 mm in, past the locking ball, reading
   the depth once and making up any shortfall. Further in, the sides of the fingers would meet the lock.
4. It lets go, rises, turns its closed fingers square to the key behind its eye end, comes down to the key's height and pushes the key
   home by its end with the front finger's outer face, as a thumb does, sent 22 mm past the stop so the arm leans
   on it. The ball drops into the key's hole,
   the plunger frees the pusher and the quarter comes out.
5. It pinches the half of the quarter that stands above the pusher across its faces, draws it straight up, carries
   it to the mat and lets go 3 mm above it.

## Simulation and the real objects

Sources: the lock's listing (Malltek MT-STL04: body 90 x 70 x 68 mm, polycarbonate, 0.28 kg, 260 mm stainless chain, coins 23-28 mm
across, fits 25-27 mm handles; black and red), https://www.supermarket-fixture.com/sale-13809265-zinc-alloy-shopping-cart-coin-lock-with-260mm-chain.html ; the coin-lock mechanism of Wanzl's patent EP0427819B1
(key through the side wall under the coin slide, a ~7 mm locking ball riding on the key, coin slide at right angles
pushed out by a compression spring "so that the coin can be removed", key about 3 mm thick), https://patents.google.com/patent/EP0427819B1/en ; nesting
distance 7 in (178 mm) and handle height 37-3/4 in of a tight-nesting cart (Unarco M60TN), https://www.unarco.com/wp-content/uploads/2021/12/unarco_M60TN.pdf ; US quarter
24.26 mm, 1.75 mm, 5.670 g, https://www.usmint.gov/learn/coins-and-medals/circulating-coins/quarter .

- The lock: one rigid body (base) clamped on its tube, plus three sprung slides that push each other through contact:
  the coin pusher (30 mm travel, spring 40 N/m preloaded: 2.4 N in, 1.2 N out), the plunger the key pushes (6.5 mm to
  a hard stop, 0.8-1.3 N), and the locking ball (lifted 1.5 mm by the key's front edge, 0.3-0.4 N). The pusher is held
  in by a tooth on the plunger's pawl until the plunger has gone 5.5 mm; the ball drops into the key's hole when the
  tip is 22.9-25.1 mm in; once the pusher is out its plate sits over the ball, so the key cannot be pulled out. The
  push needed to latch the key is about 2.5 N in simulation (inner parts' friction 0.2).
- The key and chain: the key is a flat steel plate (58 x 15 x 3 mm, an 8 x 7 mm hole 8 mm behind the tip, a 9 mm wide
  eye lug), 18 g; the chain is 13 rigid 20 mm links (each a row of three 3 mm beads), joined by two crossed hinges, 1.4 g a link, and
  the key hangs on three hinges at its eye. The chain and key collide with everything except their own lock.
- Before trials, measure the real lock's push to latch (lock on a kitchen scale, press the key in until it clicks) and
  record it beside the simulation's 2.5 N.
- Estimates, each checkable in a minute: key size and mass (ruler, kitchen scale), the hole and ball positions, the
  springs and the push to latch (above), the pusher's 30 mm travel (ruler), chain mass (scale), all frictions. The lock's inner layout follows the patent, not a
  drawing of this model. The ball is 3 g in simulation (a 7 mm steel ball is 1.4 g) for solver stability. The quarter is a 32-sided disc, so
  let go on edge it can stay standing on the mat, which a real coin would not: either way counts as resting on the mat.
- The handle sections stand on 100 mm posts, much lower than a real trolley handle (959 mm), so that the arms reach
  them; at this height the 260 mm chain reaches the table and the key lies on it instead of hanging.
- Costs: two coin locks (about 15 USD each, price on request from the seller; any lock taking a quarter works the
  same way), two 300 mm lengths of 25 mm steel tube with posts, a rubber mat, a quarter: well under 100 USD.

## Reset

Push the pusher (with the quarter in its pocket, on edge) back into the near lock: the lock lets the key go; pull it out. Lay the chain and key on their marks (chain hanging from the front lock, along -y on the table, key flat, tip away) and the mat on its marks. About 30 seconds; nothing is consumed.
