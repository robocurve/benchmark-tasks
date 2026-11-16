# SCENE grease-gun-cartridge-load

Station coordinates (m): +x forward, +y left, TOP = table top. All objects authored to size (README: sources, materials).

| Object | Real product / source | Dimensions | Mass | Material | Position | Physics |
|---|---|---|---|---|---|---|
| Bench mat | ribbed rubber workbench mat | 560 x 640 x 2 mm | fixed | rubber | centre (0.270, 0.000, TOP) | fixed, friction 0.9 |
| Grease gun barrel + end cap + holder (`gun`) | lever grease gun cylinder for 14 oz cartridges (maker service guide in README) | barrel 56 mm OD, 52 mm bore, 250 mm; cap collar 62 mm; keyhole 16 mm hole + 6.5 mm x 16.5 mm slot; holder rail 72 x 450 mm | fixed (clamped) | zinc-plated steel, oxide-black cap, rubber V-blocks, aluminium rail, steel strap | barrel axis along y at x 0.320, 38 mm above the table; mouth y +0.125, cap y -0.128; loading V y +0.135..+0.330 | fixed |
| Follower + rod (`plunger`) | follower cup, spring, follower rod with neck and T-handle | follower 47 mm; rod 10 mm; neck 5 mm x 9 mm; T-handle 100 mm x 12 mm | 0.03 + 0.06 kg | leather cup, bare steel, red vinyl grips | in the barrel; T-handle at y -0.159 at rest | follower: y slide 0..-245 mm, 58.3 N/m spring; rod: z slide on the follower, -14.5..+3 mm, limited by contact with the keyhole |
| Grease cartridge | 14 oz (397 g) grease cartridge, open end first | 50 mm x 240 mm | 0.45 kg | spiral-wound paper tube, tinplate ends | lying on its rest at (0.185, 0.215), axis 34 mm above the table, open end toward -y | loose, friction 0.35 |
| Cartridge rest | two rubber V-blocks | 60 x 30 mm each, 160 mm apart | fixed | rubber | (0.185, 0.215, TOP) | fixed |
| Bench props (visual only) | unscrewed lever-gun head with lever, hose and coupler; folded shop rag | head 64 mm socket, lever 200 mm, hose 300 mm; rag 170 x 130 mm | - | painted cast head, steel lever, black hose, brass coupler; red cotton | head (0.485, 0.110), rag (0.495, -0.185) | no colliders |
