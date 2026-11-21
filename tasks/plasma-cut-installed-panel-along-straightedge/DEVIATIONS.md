# plasma-cut-installed-panel-along-straightedge build deviations (2026-11-21)

1. **The arc, kerf and heat are bookkeeping, not physics.** No simulator models a plasma arc. The arc is "on" from the
   torch's pose (held, near square, shield on the deck within -1..+2 mm, over the deck, within 10 mm of the line), the
   way the real trial's supervisor closes the hold-to-run switch. Arc time is summed per 10 mm segment under the nozzle.
   Cut-through and overheating are arc-time thresholds taken from the cutter's published speeds (README, Scoring).

2. **The strip hangs on one weld, released when the last segment is cut through.** mechkit allows one parent joint per
   part, so the uncut bridges are one rigid weld (`SEAM`), not one per segment. It is released only once all 14 segments
   are cut through, so the strip drops only after the whole line is cut. The arms cannot break it: a 3 mm steel bridge
   needs far more force than a YAM can push, so no partial snap-off is modelled (a strip pushed while still attached
   simply does not move).
   In offline MuJoCo the weld does not hold the strip at its start pose: it settles onto the dross pan's rim (centre
   at 761.5 mm) before the cut and falls to 750.5 mm when released, so the MuJoCo start check reports the strip
   falling 42.8 mm. In the GPU simulator (Isaac) the weld holds it at 799.8 mm until the last segment is cut. The
   demo scores 1.0 in both.

3. **Overheating threshold is a modelling choice.** The manual gives the maximum speed for 3 mm (1,245 mm/min) and the
   rated speed (762 mm/min) but no slowest speed; "too slow" is set at half the rated speed (6.35 mm/s, 1.575 s on a
   10 mm segment). The real trial is judged by the edge (dross, melted corner), and the pre-trial test cuts on scrap
   (README) confirm the threshold.

4. **Torch shape and weight are stand-ins.** The Powermax30 AIR's hand torch has an angled head on a pistol-style
   handle; the scene's torch is a straight 190 mm handle along y with a vertical head and a 20 mm square drag shield at
   its +y end. A 30 x 30 x 24 mm grip block clamped on top of the handle at the torch's centre of volume is what the hand
   holds (real trial: printed or machined nylon block, two screws). The torch weighs 0.10 kg at the hand: the overhead
   balancer carries the lead and the rest of its weight, and is set so a spring scale at the grip reads 0.10 kg. The
   AIR has no separate drag shield; the simulated shield stands for its retaining cap with the snap-on deflector.

5. **The shield is square in the simulation.** mechkit builds parts from boxes, so the 20 mm round drag shield is a
   20 x 20 mm square; the guide gap is measured from the shield's centre minus its 10 mm half-width, which equals the
   round shield's radius along the straightedge's normal.

6. **The straightedge is laid along the line at setup, not fetched from a pad.** In the earlier version a hand carried
   the straightedge from a pad and set it on the plate. In the GPU simulator, closing the fingers on its handle block
   turned the bar 3.2 to 3.8 degrees (the gripper itself turned under 1 degree), which put the cut 7 mm off the line;
   in MuJoCo the same grasp turned it 0.9 degrees. The straightedge therefore starts laid on the deck one shield radius
   from the line, as a worker sets a guide before picking up the torch. It is still loose: the left hand presses it to
   the deck with closed fingertips on its handle and nothing else holds it (variant `unheld` scores 0.629). For full
   credit it must still be where it was laid at the end (20 mm in plan, 6 mm in height).

7. **Demo pace and aim.** The scripted drag has a knot every 5 mm reached on the clock at the rated speed (each stock
   waypoint eases in and out; one long waypoint would crawl at both ends and overheat them). The first centimetre onto
   the deck is drawn 1.3 times faster, because at full reach the arm lags (in the GPU simulator the first segment took
   1.55 s of arc at the plain pace against the 1.575 s limit); the run-off past the near edge is drawn at 60 % of the
   pace while the arm catches up. The shield is aimed 4 mm into the straightedge's edge so it bears on it, and set down
   6 mm high past the far end (the unsupported hand sags there). Fixed layout constants only; the demo reads no
   simulator state.

8. **Installed skid at table scale, 140 mm line.** The skid is two 2 x 2 x 1/4 in steel angles carrying the 3 mm deck,
   53.8 mm tall, so the cut sits inside both arms' reach on the station table; the line is 140 mm (14 segments of
   10 mm) so the far end stays within the torch arm's reach with the control cabinet beyond it. The access constraints
   are the cabinet 100 mm past the deck's far end (no run-in: the torch starts in the 100 mm gap), the junction box
   still mounted on the deck (the pressing hand works beside it), and the skid's own angle under the overhang (the jet
   must clear it). A 300 mm wall at the far end was tried first and dropped: the torch arm's fourth link hit it.

9. **Torch hand closes along x.** The torch hand's fingers close across the grip block along x, the drag direction, so
   the drag friction cannot pitch the torch in the hand. Closing along y let it tilt 12 degrees in the GPU simulator.
