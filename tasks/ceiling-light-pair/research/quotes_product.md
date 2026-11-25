# Product / practice quotes for the ceiling-light-pair task

All items fetched 2026-11-21 unless marked otherwise. Quotes are copied exactly as fetched
(PDF text layer via `pdftotext -layout`, HTML via tag-stripping or the page's embedded JSON).
Where a page's HTML splits a sentence across links, `|`-joins were removed and nothing else changed.
"Wayback" means the live site blocked bots (HTTP 403/429), so the Internet Archive raw snapshot
(`web.archive.org/web/<timestamp>id_/<url>`) was fetched on 2026-11-21. The snapshot date is given,
and the price is the one shown at that date, not today's.
Lines marked **search snippet, not fetched** come from a web-search result summary, so the page's
exact wording and the number are unverified.
Copies of the pages were saved when fetched. They are not included here, because the pages are third-party work.

---

## Note: light count

Some listings blur the two 4-ft models. Every source below agrees: **WR484080LSL is the 4-light-equivalent model (14.13 in wide, 70 W, 8000 lm,
4 screws). WR484056LSL is the 2-light model (8.5 in wide, 52 W, 5600 lm).**

- Spec sheet for WR484080LSL: "Replaces (4) 32W Fluorescent Bulbs"
- Home Depot page for WR484080LSL: "It installs easily and is a perfect 1 for 1 replacement for a traditional 4-Light fluorescent wrap."
- Home Depot page for WR484056LSL: productLabel "1-Pack 4 ft. 5600 Lumens Square Lens Integrated LED White Wrap Light, 4000K (2-Light)", and "perfect 1 for 1 replacement for a traditional 2-Light fluorescent wrap", and "Replaces two 32-Watt fluorescent bulbs"
- The newer (2023) manual lists the items in this order: "Item #1006 805 989 / 1005 578 285 / Model # WR484056LSLV / WR484080LSLV", then "4’ 2 Light Square Lens" / "4’ 4 Light Square Lens". Store SKU 1006805989 is WR484056LSL and 1005578285 is WR484080LSL.
  Its hardware table has columns "2L Quantity | 4L Quantity" with wood screws 2 | 4.

---

## 1. Commercial Electric WR484080LSL: weight, dimensions, lumens, watts, price

### 1a. Official spec sheet (PDF on images.thdstatic.com)
URL: https://images.thdstatic.com/catalog/pdfImages/93/934eb83f-61ae-4190-b373-1ae516470236.pdf
Fetched 2026-11-21.

Text layer, exact:
- "PRODUCT SPECIFICATIONS" / "4FT LED High Output Commercial Wrap Light" / "WR484080LSL" / "SKU#1005578285"
- "Model Number WR484080LSL" | "SKU 1005578285" | "OSMID 313512347"
- "Power 70W"
- "AC Voltage, Frequency 100-277V, 50/60 Hz"
- "Color Temp (nom.) 4000K" | "CRI 80"
- "Lumen Output (min.) 8000 lm" | "Dimming No" | "Lifetime Rating L70/50000Hrs" | "Efficacy 114lm/watt"
- "• Replaces (4) 32W Fluorescent Bulbs"
- "• For Surface and flush mount installation" | "• Row Mount up to 18 Fixtures" | "• Ideal Mounting Height up to 20'"
- Under "PACKAGING":
  - "Weight (±0.5 lb) 10.41lbs"
  - "Size (L x W x H) (±0.1 in) 47.96 in x 14.13 in x 2.82 in"
  - "Box Dimensions 51.77 in x 14.96 in x 3.5 in"
  - "Gross Weight 14.02 lbs"
  - "UPC 853711008311"
- "INCLUDED IN BOX" / "• Fixture (1)" / "• Wire Nuts (3)" / "•Phillips Head wood Screws (4)" / "•Drywall Anchors (4)"
- "SURFACE MOUNT" / "Suitable for indoor locations"

Isometric drawing. These labels are outlined vector text, read from a 400-500 dpi render rather than the text layer:
- Length: "47.96 in" / "1218.20 mm"
- Width: "15.125 in" / "358.8 mm". This label contradicts itself: 358.8 mm = 14.13 in, which matches the table, and 15.125 in = 384 mm.
- Height: "2.65 in" / "67.30 mm". The table gives 2.82 in.

**Housing without lens:** the sheet does not give housing-only dimensions. The only other figures are the drawing's 2.65 in height
and 15.125 in label shown above. Neither is labelled "housing" or "without lens", and the 15.125 in label is internally inconsistent.
**Keyhole size or spacing:** not on the spec sheet.

### 1b. Home Depot product page (Internet # 313512347)
Live URL https://www.homedepot.com/p/Commercial-Electric-1-Pack-4-ft-8000-Lumens-Integrated-LED-White-Wrap-Light-4000K-WR484080LSL/313512347
returned HTTP 403 on 2026-11-21 to both WebFetch and curl.
Wayback snapshot 2025-09-01:
https://web.archive.org/web/20250901064514/https://www.homedepot.com/p/Commercial-Electric-1-Pack-4-ft-8000-Lumens-Integrated-LED-White-Wrap-Light-4000K-WR484080LSL/313512347


Embedded product JSON, exact:
- `"itemId":"313512347","modelNumber":"WR484080LSL","productLabel":"1-Pack 4 ft. 8000 Lumens Integrated LED White Wrap Light, 4000K","storeSkuNumber":"1005578285"`
- `"specName":"Product Weight (lb.)","specValue":"10.4 lb"`
- `"specName":"Product Height (in)","specValue":"2.82"` / `"specName":"Product Length (in)","specValue":"47.96"` / `"specName":"Product Width (in)","specValue":"14.13"` (specTitle "Dimensions")
- `"specName":"Product Width (ft)","specValue":"1.18 ft"` / `"specName":"Product Length (ft)","specValue":"4 ft"`
- `"specName":"Lumens","specValue":"8000"` / `"specName":"Wattage (W)","specValue":"70 W"` / `"specName":"Voltage (V)","specValue":"120/277 V"`
- `"specName":"Recommended Mounting Height (ft.)","specValue":"10 ft"` / `"specName":"Material","specValue":"Acrylic"`
- Price: `"__typename":"Pricing","promotion":null,"value":101.67` (store 8119; **$101.67 on 2025-09-01**)
- Description: "The Integrated LED square lens wrap from Commercial Electric is perfect for any application where high quality long-lasting light is needed. It installs easily and is a perfect 1 for 1 replacement for a traditional 4-Light fluorescent wrap."
- Bullet: "4 LED boards providing 8000 Lumens"
- Reviews: `"averageRating":"4.7895","totalReviews":"551"`
- The embedded HTML/JSON contains no review or Q&A text with "keyhole", "two people", "helper" or "heavy". Reviews load from a separate API.

Search snippet, not fetched: Home Depot price "$101.67", the same as the snapshot.

### 1c. HD Supply "Precision" twin (P232165, MFG PART WR484080LSL)
Live URL https://hdsupplysolutions.com/p/precision-4-8000-lumens-integrated-led-white-wrap-light-4000k-p232165 returned 403.
Wayback snapshot 2026-02-12:
https://web.archive.org/web/20260212133703/https://hdsupplysolutions.com/p/precision-4-8000-lumens-integrated-led-white-wrap-light-4000k-p232165

- "Precision 4' 8000 Lumens Integrated Led White Wrap Light 4000k | Part | 232165 | MFG PART | WR484080LSL | Brand | Precision | $139.99" (**$139.99 on 2026-02-12**)
- "4 Led Boards Providing 8000 Lumens" / "Includes All Necessary Mounting Hardware" / "5-Year Warranty"
- "Wattage | 70 W" / "Lumens | 8000 LM" / "UPC | 853711008311"
- "Assembled Dimensions | Product Height | 2.82 in. | Product Length | 47.96 in. | Product Width | 14.13 in."
- "Packaging Dimensions | Height | 3.900 | Length | 52.000 | Size Measure | IN | Weight | 14.150 | Weight Measure | LB | Width | 15.000"

Search snippet, not fetched: HD Supply "$155.00".

### 1d. Other vendors (search snippets, not fetched)
- dkhardware.com, WR484080LSL: "$218.73" per each. Live page returned 403.
  URL https://www.dkhardware.com/commercial-electric-wr484080lsl-4-ft-8000-lumens-integrated-led-white-wrap-light-4000k-product-4706666.html
- trayco.com lists the same item (313512347, UPC 853711008311, WR484080LSL). It returned 403 and no price was seen.
- Lowe's, Walmart and eBay: no WR484080LSL listing found.

### 1e. Sibling WR484056LSL (2-light, Internet # 318411896)
Wayback snapshot 2025-07-24:
https://web.archive.org/web/20250724131932/https://www.homedepot.com/p/Commercial-Electric-1-Pack-4-ft-5600-Lumens-Square-Lens-Integrated-LED-White-Wrap-Light-4000K-2-Light-WR484056LSL/318411896

- `"modelNumber":"WR484056LSL","productLabel":"1-Pack 4 ft. 5600 Lumens Square Lens Integrated LED White Wrap Light, 4000K (2-Light)","storeSkuNumber":"1006805989"`
- `"specName":"Product Weight (lb.)","specValue":"7.6 lb"`
- `"Product Height (in)","specValue":"3"` / `"Product Length (in)","specValue":"48"` / `"Product Width (in)","specValue":"8.5"`
- `"Lumens","specValue":"5600"` / `"Wattage (W)","specValue":"52 W"`
- Price `"value":74.97` (**$74.97 on 2025-07-24**)

Search snippet, not fetched: dkhardware WR484056LSL "$186.92", 52 W.

### 1f. Installation manuals
**Older manual (2020/2021).** From https://images.thdstatic.com/catalog/pdfImages/33/33f09c44-4c2e-4052-894e-84945253908f.pdf. Header "Item #1005 578 285 … Model #WR484080LSL / WR484056LSL".
- "NOTE: Two people are recommended for the installation of this light fixture."
- "Select a suitable location that can support the weight of the fixture. Determine the method of mounting before drilling."
- "TOOLS REQUIRED": "Phillips screwdriver", "Ladder", "Safety glasses", "Wire strippers", "Wire cutters", "Power drill with drill bits"
- "AA Phillips head wood screws 4" / "BB Dry wall anchors 4" / "CC Wire nuts 3"
- "Position the fixture and mark the location of the mounting holes." / "Using a 1/8 in. drill bit, drill pilot holes at the marked locations."
- "If mounting to wall studs, install the four wood screws (AA) directly into the mounting surface leaving a 3/8 in. gap between the screw head and the mounting surface."
- "Guide the heads of the four installed wood screws (AA) into the keyhole slots of the fixture (A)." / "Slide the fixture (A) until the wood screws (AA) are secured into the narrow ends of the keyhole slots." / "Tighten the wood screws (AA) until the fixture is secured in place."

**Newer manual (2023; models WR484056LSLV / WR484080LSLV).**
URL https://images.thdstatic.com/catalog/pdfImages/4d/4d7ab39c-d45d-4c5c-b61e-9463f3af85b7.pdf

- Header: "Item #1006 805 989 / 1005 578 285 / Model # WR484056LSLV / WR484080LSLV" / "4’ 2 Light Square Lens" / "4’ 4 Light Square Lens"
- The following were read from a page-3 render because the text layer is missing these strings:
  - "NOTE: Two people are recommended for the installation of this light fixture."
  - The HARDWARE INCLUDED table: "AA Phillips head wood screws 2 | 4", "BB Dry wall anchors 2 | 4", "CC Wire nuts 3 | 3", under "2L Quantity | 4L Quantity"
- Text layer: "If mounting to wall studs, install the two or four wood screws (AA) directly into the mounting surface leaving a 3/8 in. gap between the screw head and the mounting surface." / "secured into the narrow ends of the keyhole slots."

---

## 2. Keyhole slots: size and spacing

**Not found for WR484080LSL.** Neither manual nor the spec sheet gives slot size or spacing.
The Home Depot page JSON has no Q&A or review text about keyholes, and no review or Q&A quote was found in search.

Related Commercial Electric manuals:
- **Model 56514141, 4 ft commercial LED wrap.** URL https://images.homedepot-static.com/catalog/pdfImages/32/32213191-06d4-4dca-9038-8960fb559a6c.pdf
 
  - "With help from another person, temporarily hold the fixture housing (A) up to the ceiling to determine the best mounting position and which knock out hole to use for wiring the fixture."
  - "Mark two keyhole locations 40in. apart on the mounting surface."
  - "Align the 2 mounting keyholes on the fixture housing (A) with the 2 toggle bolts (CC) in the ceiling, allowing the bolt heads to come through the large ends of the keyholes."
  - "Slide the housing (A) until the heads of the toggle bolts (CC) slide into the narrow ends of the keyholes."
- **WR2440K20LWL / WR4840K40LWL.** URL https://images.thdstatic.com/catalog/pdfImages/95/95648201-29c7-48f3-b4c6-171c4f283149.pdf
 
  - "NOTE: Two people are recommended for the installation of this light fixture."
  - "Position the fixture (A) to align the knockout hole with the supply wire location while marking the location of the four keyhole slots at each end of the housing."
  - "Install the four wood screws (CC) into the mounting surface, leaving an approximate 3/8” gap between the screw head and the mounting surface."

Search snippet, not fetched: a review of a different Commercial Electric 4 ft wrap says one person can hold it up but it is "much easier with two people".

---

## 3. Push-to-start cordless screwdriver

### Bosch GO (Gen 2), part 0 601 9H2 170
Bosch product data sheet hosted by Farnell: https://www.farnell.com/datasheets/3270282.pdf
 The sheet is dated "14.06.2021".
- "Torque (soft/hard/max.) 2.5/5/- Nm"
- "No-load speed (1st gear / 2nd gear) – 360 rpm". The leading "0" is absent in the text layer. Bosch pages in search snippets show "0 – 360 rpm".
- "Battery voltage 3.6 V" / "Battery capacity 1.5 Ah" / "Weight incl. battery 0.31 kg" / "Max. screw diameter 5 mm"
- "Dual-activation mode – simply push machine forward or press 'on' button to begin"
- "Mechanical clutch – select from 5 torque settings up to 'maximum performance mode'"

Gen 2 price, search snippets, not fetched:
- Bosch UK 06019H2170: "£83.05 incl. VAT, £69.21 excl. VAT"
- ITS UK GO2 kit: "£54.99 ex VAT"
- Costco UK: "£52.99"

The Bosch UK Gen 2 URLs returned 404 on 2026-11-21 (discontinued). **No US price was found for the GO.**

### Bosch GO (Gen 3), part 0 601 9H2 201 (current model)
URL https://www.bosch-professional.com/gb/en/products/bosch-go-gen3-06019H2201.
- "£67.55 | incl. VAT | £56.29 | excl. VAT"
- "Torque, max. (hard/soft)* | 5 / 2.5 Nm" / "No-load speed* | 306 – 396 rpm" / "Battery voltage | 3.6 V" / "Weight excl. battery* | 0.32 kg"
- "7 torque settings and a max power mode – from a fine precision setting for delicate applications up to a powerful 5 Nm"
- Fetched again on 2026-11-22: "£67.55 incl. VAT (£56.29 excl. VAT)"; "Next Gen mechanical clutch provides extra
  precision with 7 torque settings and maximum performance mode". The page does not describe how the tool starts.
- Search snippet, not fetched (galaxus.at, Bosch Professional GO 3, manufacturer number 06019H2201, third generation):
  "either press the ON/OFF switch or press the device against the workpiece".
- Fetched again on 2026-11-24: "£75.28" incl. VAT, "£62.73" excl. VAT ("Lowest price in the last 30 days: £56.50");
  article number 0.601.9H2.201. The page still does not describe how the tool starts.
- No US price was found for the Gen 3 either. At $1.30 to the pound, £75.28 is about $98 (£67.55, the 2026-11-22
  price, about $88).
- Gen 3 push-to-start: only the search snippet above; the Gen 2 data sheet (Farnell) is the fetched source for
  "push machine forward or press 'on' button to begin". Check push-to-start on the Gen 3 unit bought (or use a Gen 2).

### Cheaper alternative: BLACK+DECKER 4V MAX BDCS20C / BDCSFL20C
These are button-actuated. No push-to-start claim was found.
- **Maker page BDCS20C.** Live page returned 429. Wayback 2025-08-15:
  https://web.archive.org/web/20250815020951/https://www.blackanddecker.com/products/bdcs20c
 
  - "180 RPMs and 35 in-lbs. of torque provide plenty of power for a variety of applications"
  - Shopify JSON `"name":"4V MAX* Cordless Rechargeable Screwdriver","price":"34.99","sku":"BDCS20C"` (**$34.99**)
- **Lowe's BDCSFL20C, Item # 864467.** Wayback 2026-09-06:
  https://web.archive.org/web/20260906235110/https://www.lowes.com/pd/BLACK-DECKER-4-Volt-Max-3-8-in-Cordless-Screwdriver-1-Battery-Included-and-Charger-Included/1000329315
 
  - `"key":"Maximum Speed (RPM)","value":"180"` / `"key":"Maximum Torque (Inch/lbs.)","value":"35"`. No price in the snapshot.
- Search snippet, not fetched: Walmart BDCSFL20C "$44.99".

---

### Torque screwdriver (measuring the clutch and screws; 2026-11-22, found by web search, then checked on wihatools.com)
- Wiha US, SoftFinish TorqueVario, SKU 28415: "Sale price $159.90", "torque setting 0.1 - 0.6 Nm", "torque adapter
  1/4 Hex".

---

## 4. Professional and how-to practice: helper or support while fastening

- **Hunker, "How To Hang Fluorescent Light Fixtures"** (modified 2020-10-07). URL https://www.hunker.com/12002091/how-to-hang-fluorescent-light-fixtures/
  - "You may want a helper to hold the fluorescent fixture flush with the ceiling while you drill in the screws."
  - "Note that if you cannot find matching ceiling studs, use toggle bolts to secure the light fixture, not drywall anchors."
- **Home Depot guide, "How to Install a Light Fixture"** ("Last updated May 21, 2024"). Live page returned 403. Wayback 2026-06-18:
  https://web.archive.org/web/20260618020231/https://www.homedepot.com/c/ah/how-to-install-a-light-fixture/9ba683603be9fa5395fab90eec2fb1f
 
  - "You may need to support the weight of the fixture so you can safely hang it."
  - "Use a second ladder and another person to help you. However, keep in mind that coordinating your movements can be difficult with a very heavy fixture."
  - "Have one person support the weight of the existing fixture while the other unbolts any screws and lock nuts that attach the fixture to the ceiling."
  - "Tip: Be safe. Have someone stabilize your ladder while installing the fixture."
- **Family Handyman, "How to Install a Ceiling Light Fixture (DIY)"**, Step 8, crossbar-mounted fixture. URL https://www.familyhandyman.com/project/how-to-hang-a-ceiling-light-fixture/
  - "You may need a helper to support the fixture if you can’t rest it on the top rung of the stepladder."
- **TaskRabbit blog, "How to Install a Light Fixture in 8 Steps"** (2024-03-27). URL https://www.taskrabbit.com/blog/how-to-install-a-light-fixture
  - "Additionally, it’s a good idea to support the weight of the fixture while you’re working for safety."
  - "Alternatively, a second ladder and helper isn’t a bad idea, especially when dealing with a heavy fixture."
  - "Be sure your helper is nearby during this tricky step."
- **Warehouse-Lighting.com, "How to Install a Fluorescent Light Fixture".** URL https://www.warehouse-lighting.com/blogs/lighting-blog/d-i-y-how-to-mount-a-fluorescent-fixture
  - "Flush Mount: Hold the fixture against the ceiling. Drill pilot holes into the studs (or larger holes for toggle bolts) and screw the housing securely into place."
- **NICOR, "WPC4 Surface Mount LED Wrap Installation Instructions"** (models WPC4225WSUS8WH, WPC4446WSUS8WH; dated
  October 7, 2025). URL https://nicorlighting.com/installs/WPC4_ii.pdf. Fetched 2026-11-22.
  - Step 2: "To mount directly to the surface use the luminaire housing as a template and line up the power feed hole
    with the junction box. Then mark the locations of the four (4) screw holes on the feet of the fixture."
  - Step 5: "If mounting directly to surface, place fixture against mounting surface and install four (4) screws into
    marked locations/anchor bolts."
  - This is a maker's own hold-and-drive procedure for a 4 ft wrap, but a smaller one: NICOR's WPC sheets give about
    47.2-47.6 x 5.3 x 2.1 in (search snippets, not fetched), against the WR484080LSL's 47.96 x 14.13 x 2.82 in.
- **Manufacturer sheets.** Commercial Electric 56514141: "With help from another person, temporarily hold the fixture housing (A) up to the ceiling…" (section 2).
  Commercial Electric WR484080LSL and WR4840K40LWL: "Two people are recommended…" (section 1f).
- **Support pole / "third hand"** (search snippets, not fetched; the AgrAbility page failed with a TLS certificate error)
  - AgrAbility "Third-Hand Support Pole": "provides that 'extra hand' for jacking, bracing, clamping, or supporting … e.g., installing drywall, ceiling panels, light fixtures, crown molding, cabinetry"; working range "5 to 12 feet".
    URL https://www.agrability.org/new-toolbox/1436/
  - FastCap 3rd Hand HD: rated "150 pounds", "5' to 12'". A user review, source page not identified, says they "used one like this recently to hold up a florescent 4 bulb light fixture onto the ceiling while I fastened it with toggle screws".
- **Not found:** a Lithonia or Metalux 4 ft wrap/strip/troffer instruction sheet that says "two people" or "support the fixture".
  Several Lithonia PDFs on images.thdstatic.com (82c4c0c9, d27b83b7, bb244ba9) were checked; none contains such wording in its text layer.
  No evidence was found of drywall-panel lifts being used for fixtures.

---

## 5. O*NET 47-2111.00 Electricians
URL https://www.onetonline.org/link/summary/47-2111.00 (page "Updated 2026"). Fetched 2026-11-21.
- Description: "Install, maintain, and repair electrical wiring, equipment, and fixtures. Ensure that work is in accordance with relevant codes. May install or service street lights, intercom systems, or electrical control systems."
- Tasks about fixtures (exact; there are 21 tasks in total):
  - "Work from ladders, scaffolds, or roofs to install, maintain, or repair electrical wiring, equipment, or fixtures."
  - "Assemble, install, test, or maintain electrical or electronic wiring, equipment, appliances, apparatus, or fixtures, using hand tools or power tools."
  - "Plan layout and installation of electrical wiring, equipment, or fixtures, based on job specifications and local codes."
  - "Direct or train workers to install, maintain, or repair electrical wiring, equipment, or fixtures."
  - "Repair or replace wiring, equipment, or fixtures, using hand tools or power tools."
  - "Test electrical systems or continuity of circuits in electrical wiring, equipment, or fixtures, using testing devices, such as ohmmeters, voltmeters, or oscilloscopes, to ensure compatibility and safety of system."
  - "Perform physically demanding tasks, such as digging trenches to lay conduit or moving or lifting heavy objects."
- Wages & Employment:
  - "Median wages (2025) $30.38 hourly, $63,190 annual"
  - "Employment (2025) 821,000 employees"
  - "Projected growth (2025-2035) Much faster than average (7% or higher)"
  - "Projected job openings (2025-2035) 72,700"
  - "Source: Bureau of Labor Statistics 2025 wage data"

---

## 6. I2RT YAM arm: payload and reach

- **Maker store page, YAM.** Live https://i2rt.com/products/yam-6-dof-arm returned 429. Wayback 2026-10-22:
  https://web.archive.org/web/20261022134045/https://i2rt.com/products/yam-6-dof-arm
 
  - "Gripper Throw: 95 mm | Nominal Payload: 2 kg | Max. workspace range: 750 mm | Arm weight (including gripper): 5.02 kg"
  - "Regular price $2,999.00"
- **Maker store page, YAM Pro.** Wayback 2026-09-13:
  https://web.archive.org/web/20260913025357/https://i2rt.com/products/yam-pro-6-dof-arm-copy
 
  - "Nominal Payload: 3 kg | Max. workspace range: 750 mm | Arm weight (including gripper): 5.06 kg"
- **Official docs, live.** https://doc.i2rt.com/products/yam-standard (also -pro, -ultra, -big)
  - "Payload TBD — see i2rt.com" / "Reach TBD — see i2rt.com"
  - The series page lists "YAM Standard $2,999", "YAM Pro $3,499", "YAM Ultra $4,299", "BIG YAM $4,999".
- **GitHub README, live.** https://github.com/i2rt-robotics/i2rt (main @ 8c1b3ab, 2026-11-20)
  - "| YAM | `yam` | 4.292 kg | 0.807 m |"
  - The README explains: "\"Arm mass\" excludes the gripper and tips. \"Chain length\" is the sum of the parent-to-child joint-origin translation norms — a measure of the kinematic chain, not of reach."
  - "`linear_4310` 0.695 kg"
- Search snippet, not fetched: reseller roboticscenter.ai lists YAM "2 kg payload and 780 mm reach". This is not an official source.

---

## 7. Lumber prices

- **Home Depot, Handprint 23/32 in. x 2 ft. x 4 ft. Sanded Plywood Project Panel (model 300969, Internet # 202093836).** Wayback 2025-05-01:
  https://web.archive.org/web/20250501002840/https://www.homedepot.com/p/Handprint-23-32-in-x-2-ft-x-4-ft-Sanded-Plywood-Project-Panel-300969/202093836
  - `"__typename":"Pricing","promotion":null,"value":23.76` (**$23.76**)
  - Search snippet, not fetched: "$23.76" on the product page; a category listing shows "$12.52".
- **4x8 sheet of 23/32 plywood.** Not verified.
  - The Home Depot Plytanium 23/32 4x8 BC Sanded Pine page (100061386), Wayback 2026-11-16, has `"value":null` because the price is store-specific.
  - The Lowe's 23/32 4x8 Radiata Pine ACX page (3551942), Wayback 2026-04-16, has `"price":false`.
  - Search snippet, not fetched: Home Depot "23/32 in. x 4 ft. x 8 ft. RTD Southern Yellow Pine Wood Sheathing Plywood" at "$41.99" ("$37.79" for 48 or more).
- **Lowe's, "2-in x 4-in x 92-5/8-in Kiln-dried Whitewood #2 Prime Stud" (item 1000525943, listed as 2x4x8).** Wayback 2026-02-12, store "Sidney Lowe's":
  https://web.archive.org/web/20260212212945/https://www.lowes.com/pd/2-in-x-4-in-x-8-ft-Whitewood-Pre-Cut-Stud-Common-1-5-in-x-3-5-in-Actual/1000525943
  - `"basePrice":3.44,"finalPrice":3.44,"retailPrice":3.44` (**$3.44**)
- **Home Depot "2 in. x 4 in. x 8 ft. Prime Stud" (058449 / 312528776).** The Wayback 2024-09-30 page shows no price.
  Search snippet, not fetched: "$3.26" ("$2.93" for 100 or more) and, separately, "$5.82" ("$5.24" for 100 or more).

---

## 8. YAM joint motors: rated and peak torque (search snippets, not fetched; 2026-11-22)

- Damiao DM-J4340 (YAM joints 1-3): sellers list "9Nm rated torque, 27Nm peak torque" (openelab.com, aifitlab.com,
  mybotshop.de). One roboticscenter.ai listing gives 9 / 27 N m in its text and 12 / 40 N m in a table; a 24 V
  listing gives about 7 N m rated. The 27 N m peak is the same in every listing.
- Damiao DM-J4310 (joints 4-6): the Damiao 2025 selection handbook (Chinese) lists 3 N m rated, 7 N m peak (24 V).
- yam_arena uses 27 N m and 7 N m as the joints' effort limits.
- DM-J4340-2EC manual (English, foxtech; https://f.foxtech.com/manual/DM-J4340-2EC.pdf), fetched 2026-11-22:
  "Capable of reporting motor speed, position, torque, and temperature via the CAN bus." / "Equipped with dual
  temperature protection." / "*T_MOS: Average temperature of the MOSFETs on the driver board (unit: °C)" /
  "T_Rotor: Average temperature of the motor's internal coil (unit: °C)" / error codes "B – MOS overtemperature",
  "C – Motor coil overtemperature" / "Please strictly operate the motor within the specified working environment and
  the maximum allowable winding temperature range." It gives no temperature number.
- DM-J4310-2EC V1.1 manual (search snippet, not fetched; aifitlab wiki): the motor exits enable mode if the drive
  temperature exceeds 120 °C or the motor temperature exceeds 100 °C.
