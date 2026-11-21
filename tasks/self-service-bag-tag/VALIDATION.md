# Validation — physical-pressure revision

## Response to the previous review

1. **Last sampled frame showed both grippers at the tag and no visible sealed loop.** The demo now ends with both grippers open, both arms back at their ready poses (TCP error < 0.71 mm) and the closed loop on the handle with its barcode outward, held still for 13.8 s. This was checked on the decoded final frame of the submitted video (`encoded-video-audit.json`, `closeup-video-audit.json`), not inferred from the trace. The video also adds a magnified wrist view, because the top camera's view of the squeeze is blocked by the hands.
2. **The 0.85 tier was awarded at zero stable seconds.** `outcome_score` itself now requires at least 0.25 s of released stability, so inconsistent caller flags cannot bypass it. The regression covers 192 combinations, including 0, 0.249, 0.25, 1.99 and 2 s. README and task.json state the same rule.
3. **The seal used alignment, overlap, proximity and time without a physical press.** A bond now requires a measured PhysX normal force of at least 1 N on the two designated coated patches, sustained for 0.4 s. Missing contact data never falls back to geometry. Controls with the same code:
   - aligned hover at 3.4–4.2 mm separation: 0 N, no bond, score 0;
   - touching but weak: 0.89 N, no bond;
   - 20 g coupon at 0.20 N: no bond;
   - force on the wrong place (3.89 N between the strips, but outside the coated patches): no bond;
   - hover, then close and press: bonded at 2.85 N, score 1.0.
   In the continuous demo, the layers touch at 0.40 N without bonding; the bond forms only once the jaws close (2.78–2.93 N), at step 2095.
4. **Full-score run under the revised rule.** A fresh reset with no object-state writes scores 1.0, with a released stable loop.

## Current full demonstration

`runs/full-demo-057/result.json` (bit-identical rerun of full-demo-053 with wrist frames also recorded): fresh-reset continuous run, score **1.0**, 2,756 control steps / 137.8 simulated seconds. Both grippers open and withdrawn; intact linked loop, exposed barcode, final stable interval 13.8 seconds. Seal triggered after the physical contact dwell, with 2.919 N measured coated-patch normal force. No intermediate object-state restore, scripted object teleport or gripper attachment was used in this run.

The corrected motion loosens the right grasp before changing approach, regrips, opens around the two layers, remeasures their positions after opening, aligns, then closes to press. Scoring thresholds were not relaxed. Post-bond contact buffers are not reported as live pressure because PhysX can retain stale impulses after pair collision is disabled.

`final-review.json` verifies final score, arm clearance/ready positions and source hashes. `encoded-video-audit.json` verifies encoded frame count and the actual final frame. `all-panel-pressure-audit.json` and `all-label-clearance.json` record sampled CPU collision geometry checks; they are not exhaustive continuous-collision or real-material certification.

## Regression evidence

- `test_score_outcomes.py`: 192 synthetic combinations, including release boundaries 0, 0.249, 0.25, 1.99 and 2 seconds.
- `test_score_geometry.py`, `test_hold_window.py`: barcode exposure, integrity and hold resets.
- `test_press_gate.py`: no force, weak force, wrong patch, tangential contact, interrupted dwell and nonfinite values.
- `runs/press-weak-negative-026`: staged physical 20 g coupon contact peaked at 0.1962 N without bonding.
- `runs/full-demo-044/failed-contact-review.json`: failed continuous attempt had 3.89 N total pair contact but zero qualifying patch force and overlap; bond and score correctly stayed zero. This was a failed attempt, not a deliberately staged counterexample.
- `runs/full-demo-047`: closed-jaw turn trapped adjacent panel01; stopped with no bond and score0. Retained to document why open-jaw reorientation is required.

Historical full-demo-013/019 and Dream-01 used the superseded geometric-only bonding rule. They are not current revision evidence. The revised Dream-1-beta run (`runs/dream-02`) used the same task source as the demo. It was one valid session of 4,800 steps with no errored trials and five recovered protocol retries, and it scored 0.0: the model never extracted the tag. Its video is `runs/dream-02/model_run.mp4`. Adhesive chemistry, force/dwell and break limits are uncalibrated approximations; see ADHESION_REALISM.md.

## Deliberate closing motion
Continuous full-demo-053 closes the fingers over 0.8 s after alignment and holds closure for 3 s before release. No score, scene, contact or adhesive parameters changed. Pressure-window sampled panel/finger overlap peaked at 0.171 mm; this is a bounded sample, not a continuous collision proof. Encoded final frame independently inspected: hands home, linked tag retained, barcode outward.

## Collision-free arm withdrawal
The right hand now opens and backs away diagonally (-60 mm X, -120 mm Y, +45 mm Z) before rotating, keeping it clear of the left hand. Sampled convex-hull audit of the recorded joints over the whole release and final hold (`runs/full-demo-053/withdrawal-clearance.json`, 122 samples): 0.0 mm arm-to-arm and 0.0 mm right-arm-to-scene depth. The superseded route, which rotated before retreating, planned about 20.5 mm of arm overlap on CPU (`runs/release-route-cpu.json`); in its recorded rollout the arms touched (0.6 mm sampled depth, `runs/full-demo-051/withdrawal-clearance.json`). Sampled checks are not exhaustive continuous-collision proof. Encoded video: 689 frames / 137.8 s, terminal frame matches the final simulation frame (`encoded-video-audit.json`).

## Hover-versus-press controls (staged, restored start; not demos)
All three use the same restored start and route as the release diagnostic, and scoring thresholds were not changed.
- `runs/hover-negative-054`: the patches were aligned inside open jaws (1,762 mm² overlap, normal dot ≥ 0.990) and hovered 3.8 s without squeezing. Measured patch force was 0 N and no bond formed. After release the tag fell off: final score 0. The layers stayed 3.4–4.2 mm apart, so this tests aligned hovering at close range, not touching.
- `runs/hover-then-press-055`: hover, reopen the jaws to 8 mm, then close at the old target. The layers had shifted, so although they were touching, the sustained patch force was only 0.89 N (below 1 N). The gate refused to bond and the run stopped without releasing (score 0). This also shows that a real but too-weak squeeze does not pass.
- `runs/hover-then-press-056`: hover (0 N, no bond), reopen, re-measure and re-align, then close and press. The bond formed at step 393 at 2.846 N after the 0.4 s dwell. The release was collision-free (`withdrawal-clearance.json`, 0.0 mm sampled), and the run ended with score 1.0 and 5.8 s of stability. The video `staged-hover-then-press-diagnostic.mp4` shows this staged diagnostic.

## Close-up video
The top camera's view of the squeeze is blocked by the hands. `runs/full-demo-057/successful-demo-closeup.mp4` (960×352, 689 frames, 137.8 s) places the unmodified top view beside a magnified right-wrist crop and the left-wrist view, all from the same simulation step. Run 057 repeats 053's robot commands; its recorded panel trajectory is identical (0.0 m maximum difference). Only extra wrist frames were saved. Its final review, encoded-frame audits, withdrawal (0.0 mm), whole-run arm clearance (0.0 mm), pressure-window (0.171 mm) and label (0.798 mm transient) audits match 053. The wrist panel shows two separated layers at step 2072, the jaws closing at 2080, squeezed layers at 2088, and the seal formed at step 2095 at 2.919 N.

## No gripper adhesion
The simulated bond can only join the two tag end bodies (panel 00 and panel 09). The contact view is filtered to that pair, and `create_bond` locates only those two bodies. Gripper contact is ordinary friction and can never create a bond. This matches the reference stock's linerless coating, which bonds only coating-to-coating. In run 057, as the right fingers open (steps 2146–2165) and then rise about 45 mm while withdrawing, the bonded ends move at most 3.2 mm. That motion is a slight downward settle under gravity, not following the fingers. After the left release, the loop settles about 5 mm onto the handle and then stays still for 13.8 s. Real gripper-pad tack is not modelled.
