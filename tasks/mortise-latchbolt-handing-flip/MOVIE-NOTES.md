# MOVIE-NOTES -- mortise-latchbolt-handing-flip

| gate | what changed | numbers | frames opened |
|---|---|---|---|
| 1 real models | lock case, armored front, latchbolt (beveled head + anti-friction tongue), holder, oak jamb with stainless strike and dark strike box, aluminium guide: builder-made true-to-size PBR (no public library model of a mortise lock, door latch or lock case); props: 2 NVIDIA SimReady hand tools (end-cutting pliers, claw hammer), physics stripped; screwdrivers and wrench left out (maker mark in their geometry) | sizes 1:1 from the maker's installation instructions; no word marks | Blender previews (persp: case, front screws, head + tongue); official 1280 x 704 demo run, first and last frame |
| 2 top-camera size | lock moved toward the image centre within reach: (0.32, -0.15) | faceplate ~170 px long at 1280 x 704 (>= 80); head ~27 x 16 px (proxy: SUBACTION_END + close-up) | same frames |
| 3 no hand over the result | right arm parks at (0.16, -0.32, +0.20 m) after the flip; left arm closes the jamb, backs off and parks at (0.20, +0.26, +0.20 m); clear at the end (1.0 needs both grippers off the lock and jamb for the 1 s success hold) | mj_check VISUAL end_occluded_by_arms {} | last frame: lock fully visible, both grippers away |
| 4 realistic bench | 2 real hand tools lying flat around the lock, footprints disjoint, all outside the arms' paths | bboxes in assets/props/props.json | frame 0 |
| 5 PBR | brushed stainless front, satin chrome head, plated steel case, anodised holder, rubber pads; SimReady UsdPreviewSurface textures (<= 512 px) | -- | -- |

