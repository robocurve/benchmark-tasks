# SCENE -- estop-twist-release (stage 3: every object, real source, size, mass, material, position)

Station frame: origin midway between the arm bases, +x forward, +y left, metres; table top z = 0.7475 m.

| object | real product / source | dimensions (mm) | mass | material / look | position (x, y) | physics |
|---|---|---|---|---|---|---|
| e-stop operator | 16 mm-panel 40 mm mushroom push-to-lock / pull-or-turn-to-reset emergency stop, XA-class (IDEC X-series E-stop catalogue, XA series: 40 mm button XA1E-BV4.., push-to-lock 10.5 N, turn-to-reset 0.16 N*m, min stroke for direct opening 4 mm, max operator stroke 4.5 mm, 40 mm button 28 g, drawing height 30.4 mm) | head 40 dia, 14 thick, top 30.4 above the cover | 28 g (12 g plunger + 16 g head) | red glossy PA cap with two white turn arrows, black plunger, chrome bezel | (0.300, -0.110), head top z = TOP + 0.1004 | slide 4.5 mm (2625 N/m = 10.5 N at 4.0 mm), hinge on it (0.31 N*m/rad = 0.16 N*m at 30 deg, stop 40 deg); latch detent + release cam in the step hook |
| 1-hole e-stop station | generic yellow polycarbonate 1-hole push-button enclosure | 80 x 80 x 70 | fixed | yellow PC, 4 lid screws, cable gland | (0.300, -0.110) | fixed collider |
| EMERGENCY STOP legend plate | 60 mm yellow legend plate for 22 mm devices (generic; the catalogue lists 60 mm "Emergency Stop" nameplates) | 60 dia | -- | yellow, black lettering | under the head | part of the station collider |
| 2-hole control station | generic grey 2-hole push-button enclosure for 22 mm devices | 80 x 120 x 70 | fixed | light grey PC, engraved START / RUN legend plates | (0.300, 0.000) | fixed collider |
| START push button | 22 mm momentary START push button, green | cap 24 dia, 14 proud, 4 stroke | 10 g | green glossy cap, chrome bezel | (0.300, -0.028) | slide 4 mm, 1250 N/m (5 N at full stroke; declared) |
| RUN pilot lamp | 22 mm LED pilot light, green (IEC 60204-1 Table 4: green = normal) | lens 21 dia | -- | dark green lens; lit lens emissive green | (0.300, 0.030) | display joint driven by the contactor state |
| grinder-polisher | single-platen 200 mm (8 in) metallographic grinder-polisher, 50-600 rpm class, generic (no maker marks) | base 330 x 290 x 120; platen 200 dia; splash bowl 270 dia | fixed | grey painted base, white ABS bowl, aluminium platen, SiC paper, steel clamp ring with 4 lugs, water arm | (0.520, 0.120) | fixed; platen hinge turned by the hook at 60 rpm while the machine runs |
| cables | 2 control cables, 9 mm OD | -- | -- | black PVC | station glands -> machine side | visual only |
| caliper | NVIDIA SimReady `instrument_caliper_a01` (CC-BY-4.0) | model size | -- | model PBR | (0.300, 0.330) | visual only |
| solvent spray can | NVIDIA SimReady `consumable_spray_solvent_a01` (CC-BY-4.0) | model size | -- | model PBR | (0.760, -0.080) | visual only |
| box of nitrile gloves | NVIDIA SimReady `box_glove_a01` (CC-BY-4.0) | model size | -- | model PBR | (0.770, -0.250) | visual only |

Search log for the main devices (library first): `search.py "emergency stop button"` -> objaverse-clean
`emergency-stop-button-012e4809` and physxverse `9dbf2a60...` (single static meshes, no moving head: unusable for a
latching mechanism); `search.py "push button control station"` -> no hits; `search_cloud.py "fan"` -> SimReady
cooling-system fan shrouds (car radiator shrouds, no blades); `search.py "polishing machine"` -> physxverse
"Pulido" (cc-by-nc). No articulated e-stop or grinder-polisher found -> builder-made true-to-size models
authored in `assets/build_visuals.py`.
