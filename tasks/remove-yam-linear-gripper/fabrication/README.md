# Adapter fabrication

All lengths in the OpenSCAD model are in millimetres. The coordinate datum is the vendor URDF gripper mount, +X toward withdrawal and +Z on the straight driver approach. The fixture places two assumed M3×0.5 ISO 4762 socket heads at X −27 and Y ±15.5 mm. Their axes are 31 mm apart. Those locations come from the training adapter. The pinned I2RT wrist and gripper meshes contain the mating envelope and joint datum. They do not specify a verified stock screw axis at those positions.

Fabricate the saddle and bridge in aluminum, using paired steel M3 thread inserts or captured steel nuts in the bridge. Use the bridge pilot bores to locate the inserts and finish to the chosen insert manufacturer's bore specification. Set 1.0 mm of effective engaged thread using the stop stack, then check that two complete turns clear it. The captive stop allows additional clearance. Deburr and fit both bushings with their tapered bore facing the driver. Mount the non-powered wrist to a subplate or rigid vise using soft jaws. Check saddle alignment with the pinned mating datum and keep the straight Z driver corridor clear. Verify that the gripper is supported before unthreading the second fastener. The exploded OpenSCAD setting shows the sleeve, screw head, stem, bridge and saddle position in that order.

Use a torque-capable HIOS CL-6500, rated 0.30–1.60 N·m with a 37 mm grip and 235 mm overall body, in the 1/4-inch hex-shank configuration with a straight 2.5 mm across-flats insert bit. The simulation uses the same hex size and a compact spring-fed tool envelope. The purchased body is larger than the simulator body. The actual fit and torque response of the fabricated assembly have not been measured. [HIOS specifications](https://hios.com/EN/products/driver/CL-6500/)

The task-local CAD, OpenSCAD drawings and adapter are original. The material choice, insert bores, screw class and release travel are design assumptions.

| Component | Quantity | Source or fabrication |
| --- | ---: | --- |
| YAM v1 donor wrist and Linear 4310 module | 1 each | I2RT hardware. Pinned I2RT asset source and MIT notice are listed in the task README. |
| Aluminum saddle and guide bridge | 1 each | `collar-and-guides.scad`, saddle and bridge settings. |
| Guide sleeve and steel M3 threaded insert | 2 each | Sleeve from the bushing setting. Select a steel insert to fit the finished pilot. |
| M3×0.5 socket-head screw and travel stop | 2 each | Assumed ISO 4762 screw, 8 mm nominal stem, with captive stop set for the assumed training release. |
| Driver and bit | 1 each | HIOS CL-6500 and 2.5 mm straight hex. |
| Driver dock and stow cradle | 1 each | Original station CAD in `assets/driver_stand.usda` and `assets/module_cradle.usda`. Use cradles sized to the chosen physical tool and module. |
