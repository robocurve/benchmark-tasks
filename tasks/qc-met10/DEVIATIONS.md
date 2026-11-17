# Deviations from the real setup

The current model departs from the real wedge grips, stand and specimen in these points. Values marked (unverified)
in README.md and SCENE.md are dimensions not given by the cited sources.

1. Frame length: the stand's frame is modelled 460 mm long; the data sheet of the commercial stand the layout comes from gives 750 mm overall (the real-trial stage travels 200 mm). The grips are at the maker's drawing size, the 76 mm
   loading axis and the control panel come from that data sheet, and the panel sits on the real frame end (set flat; the
   guide says its viewing angle is adjustable).
2. Grip lever: the real rack-and-pinion lever is modelled as a 22 mm spring-return slide whose wedge lifts the
   spring-closed upper jaw to a 6.2 mm gap (inside the real 0-6.4 mm opening). Lever travel is not on the drawing.
3. Grip force: the task ends at mounting, before any pull, so only the jaw spring holds the tab; it bites with a
   declared 1.9 N (no spring rate or clamp force is published). The real grip's self-tightening wedge bites harder only
   as the test load rises, so it is not simulated, and no test load or preload is applied.
4. Crosshead jog: the crosshead is a stiff position servo in the task code; its target moves at the set speed
   (500 mm/min, inside the stand's optional 0.5-1,100 mm/min speed range) only while LEFT or RIGHT is held down (the guide's momentary
   KEYS setting). Travel is modelled 15 mm past the 115 mm position (457 mm is the commercial source stand's travel; the real-trial stage travels 200 mm).
5. Specimen material: the dogbone is LDPE (7.8 g at 0.924 g/cm3), (the task ends at mounting and applies no load). Its
   ivory visual material keeps the asset name `natural_polycarbonate` inside the authored `.usdc` (appearance only;
   natural LDPE is also translucent ivory).
6. Force sensor: the ESM303H takes an indicator with a Series R01/R03 load cell on its crosshead adapter (the maker page:
   "Force gauges cannot be mounted to the ESM303H test stand"). The model shows the indicator and load cell at the
   fixed (+y) end, with grip B on the crosshead; no load is applied in this task, so the sensor position does not
   change the mounting.
   Code identifiers and asset names that say "gauge" (for example `gauge_lcd`, `gauge_keys`) mean this indicator;
   the scoring constants `GAUGE_L` / `GAUGE_TOL` / `GAUGE_CREDIT` are the ASTM gauge length between the jaw faces.
7. Real-trial stand: the scene's layout and dimensions come from a commercial horizontal motorised stand's data sheet (750 mm frame, 1.5 kN; commercial motorised stands start near USD 2,500). Build the real-trial stage from the parts list (REAL-TRIAL-PARTS.md) as a motorised screw stage with a two-button jog box: its jog speed is set by the controller's speed pot (500 mm/min = 100 RPM on the 5 mm lead) and its buttons are Maple PBS111 (the scene's keys: 24 mm square simulation surrogates at 45 mm pitch; the round PBS111 head diameter is not published in the datasheet; bezel 29 mm and block width 30 mm are datasheet values).
