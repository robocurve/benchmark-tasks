# Validation of the simplified cradle-supported reference

Current source `942fe98a9030e07f0862b65d3c6d7cc563c4e8ae50d940bc687a117a271974a8` has two complete native full-score results. No left-hand base hold is used. The apparatus and scoring thresholds are unchanged from the preceding validated geometry. The reference now uses right for rod operation and left only for refill exchange, with a one-second pause after ratchet re-engagement. The instruction explicitly allows the rod handle to remain extended behind the dispenser.

| Check | Actual result |
| --- | --- |
| Native recorder, seed 0 | 1.0 in 1,090 actions / 54.50 simulated seconds; 334.75 wall seconds. Source unchanged; no violations. |
| Registered Inspect Robots `task_score`, seed 0 | 1.0 in 1,089 actions / 54.45 simulated seconds; 346.83 wall seconds. Normal success termination and zero errored trials. |
| Single hosted Dream-1 trial, seed 0 | 0.0 in 2,400 actions / 120 simulated seconds; 502.26 wall seconds. Normal action-limit termination, zero errored trials, 50 returned chunks through 51 request attempts. One expired request was retried within this same episode; no second trial was started. |
| Bounded pull without rotating the rod, also without a left-hand hold | Final score 0, rod retained after about 0.965 mm retraction. Released demo reaches its 189 mm physical stop. |
| Mathematical checks | Twelve scorer/envelope tests and all four offline IK plans pass. Four unchanged USD assets pass native schema/structure checks. These checks do not substitute for native feasibility. |

The maximum measured frame translation in the direct demo was 2.620 mm. Final rod rotation was 0.004585 rad relative to its working zero, pawl angle -0.039262 rad, piston gap 4.547 mm, and whole-robot clearance 79.715 mm. The extended rod therefore remains in its declared engaged state. Both grippers are released.

Current evidence is in `demo-native.json`, `inspect-robots-native.json`, `dream-native.json`, `engaged-pull-native.json`, `current-offline-checks.json` and `current-offline-usd.json`. JSON files with earlier source fingerprints are preserved baseline/history records, not current-controller results.

All native trials use the dedicated Python 3.12 uv environment, Isaac Sim 6.1.0.0, Isaac Lab metadata 17.0.2, Arena metadata 0.3.0, yam-arena 0.1.0 and Inspect Robots 0.60.0. Device is local GPU 0 (RTX PRO 6000 Blackwell Max-Q); GPU 1 is not used. Physics runs at 240 Hz and control/scoring at 20 Hz. The direct run audited 13,080 physics samples. Native Kit shutdown can leave shell status zero after a failed recorder attempt; actual numeric scores and success flags are authoritative.

Updated full-score video: actual native top/two-wrist composite, H.264/yuv420p, 640 x 528, 20 fps, 1,091 frames, 54.55 seconds, 1,995,869 bytes. Its PNG is the matching initial top frame. The private workspace preserves the full traces, original streams and receipts under `artifacts/sealant-cradle-only-*`, plus the prepared media and manifest under `artifacts/sealant-cradle-only-ready`. There are no generated frames or team identifiers.

The one paid model trial is complete, with actual results in `dream-native.json`. Its untrimmed H.264/yuv420p composite has all 2,401 native observation sets at 20 fps: 120.05 seconds, 640 x 528, 6,631,176 bytes. All three initial camera images are pixel-identical to the demo start; the setup PNG is byte-identical. No generated frames or team identifiers were added. There were zero predicted rows outside the declared action limits. The submission includes this model attempt, the successful scripted demo and their matching setup image. These results establish fixed-layout simulated feasibility and the observed result of one model trial, not commercial-product calibration or real-hardware performance.
