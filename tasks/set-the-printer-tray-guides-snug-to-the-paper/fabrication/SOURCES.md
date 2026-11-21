# Parts/material sources and authored specifications

No purchases, supplier messages, quote requests or stock guarantees. Primary sources were read November 18 Pacific / November 19 UTC, 2026.

| Item | Specification / sourcing route |
|---|---|
| Contact pads | PP-H polypropylene sheet, machine to 3.00±0.05 mm. [Curbell material sheet](https://www.curbellplastics.com/wp-content/uploads/2022/11/Curbell-Plastics-Polypropylene-Data-Sheet.pdf) identifies sheet material; required thickness/flatness are this drawing's requirements, not an asserted stock tolerance. |
| Backplates, cheeks, carriages, gears | POM-C/acetal copolymer plate, machined to the authored cut list. Grade, actual thickness and surface finish recorded at build. No vendor geometry is assumed. |
| Floor, walls, plinth, brackets | 6061-T6 aluminium sheet/plate, machined to drawings. Floor 3, backplates as specified separately, connectors 3. Standard stock may need facing to the drawing tolerance. |
| Ground shafts/pins | Ø6 h6 straight stainless shaft stock for carriage axes; custom Ø3 g6 ×20 pad pins with axial M2 holes as in the drawing. Cut and inspect to the specified lengths; do not substitute threaded rods as running surfaces. |
| Pad screws | Accu **SSK-M2-6-A2**, M2×6 countersunk A2, [manufacturer listing](https://www.accu.co.uk/countersunk-socket-head-screws/5406-SSK-M2-6-A2). Drawing requires flush heads; confirm received head envelope before cutting the 90° countersink. |
| Main fasteners | Accu **SSCF-M3-12-A2**, M3×12 full-thread A2 cap screw, [manufacturer listing](https://www.accu.co.uk/metric-cap-head-screws/3820-SSCF-M3-12-A2); lengths and head styles as listed in `hardware.csv`; M3 washers and nuts, M2 washers/nuts for pin stops and latch tie rods. Use the thread length appropriate to each joint; no tip protrudes into paper or a slide. |
| Springs S1/S2/S3 | Manufacture to `SPRINGS.md`; [Lee custom spring capability](https://www.leespring.com/learn-about-custom-springs) is a possible source. Finished load/rate and envelope tests govern, not an invented stock SKU. |
| Spring material constants | [Lee material table](https://www.leespring.com/spring-materials), ASTM A228 E/G; calculations are predictions. [Torsion design guidance](https://www.leespring.com/learn-about-torsion-springs) supports winding direction, leg clearance and assembled testing. |
| Scales | Original `compression-scale.svg` printed on dimensionally checked adhesive polyester label stock, pointer on the moving pin. No font file or vendor graphic is redistributed. |
| Gear reference | [KHK dimensional reference](https://khkgears.net/new/gear_knowledge/gear_technical_reference/calculation_gear_dimensions.html) supplies the standard module/base/addendum/root relationships. Tooth contours in this package are authored, with the root geometry specified on drawing D05; no KHK product model is copied. |

The complete custom fixture is the sourced trial apparatus. It must not be called an obtainable M404 printer cassette, and its measurements must not be attributed to HP. Standard hardware specifications and original geometry are explicit. Availability and lead time are not guaranteed; finished springs and the assembled cassette are accepted only by the tests in CALIBRATION.md.
