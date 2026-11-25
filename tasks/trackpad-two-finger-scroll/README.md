# Two-finger scroll on a Magic Trackpad: a level light-press slide that stops a red line in a screen band, no click

Scrolling a page with two fingertips on a laptop trackpad is done every day by most computer users. IDC counted 262.7 million PCs shipped worldwide in 2024, and two in three US adults or more own a desktop or laptop. On how often,  iD magazine (January 2021) reports Ogilvy finding more than 300 feet of digital content scrolled per person per day (all devices, not trackpad alone). For a robot, both fingertips must stay on the glass over a 70 mm slide, the total press must stay under the click force, and the slide must stop so the page coasts to rest with its red line inside a 24 mm band on the screen.

A 24-inch monitor shows a document: grey text lines, one red line and a blue band. An Apple Magic Trackpad (USB-C) lies on the table in front of the right arm. The right gripper carries a conductive stylus nib on each finger, lowers both nibs onto the glass 22 mm apart, presses them lightly (0.12 to 0.4 N in total) and slides them 70 mm away from the user. The page follows at 1.5 times their travel; the nibs stop and lift, and the red line rests in the band.

## Run

From this folder, on a GPU machine:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,one_tip,flick,press,rigid,brush
```

Instruction given to the model: `Scroll the document with a two-finger slide on the trackpad until the red line rests inside the blue band on the screen. Do not click.`

## Real items

- Apple Magic Trackpad (USB-C), model MXK93AM/A, Apple's technical specifications: https://support.apple.com/en-us/121932 (160.0 x 114.9 mm, 4.9 mm thick at the front edge and 10.9 mm at the back, 230 g; the glass rises 3.0 degrees away from the user). Modelled at those sizes. Price: US$129 from Apple.
- 6 mm conductive silicone stylus nib covers for touch-screen pens, an unbranded 30-piece pack (no maker is printed on the listing), model MN-D7-240814-NR06, Best Buy Canada SKU 18246982: https://www.bestbuy.ca/en-ca/product/replacement-stylus-tips-0-23-6mm-replacement-silicone-nib-cover-for-touch-screen-pens-no-touch-door-opener-tools-30-pcs/18246982 . Any 6 mm conductive silicone nib is equivalent. A capacitive pad does not register bare plastic or metal fingers, so each finger carries one. Price: a few dollars for the pack.
- Dell P2422H, 23.8-inch 16:9 monitor, active area 527.04 x 296.46 mm (Dell's specification table: https://www.manualslib.com/manual/2769390/Dell-P2422ha.html?page=13). The scene draws a 531.4 x 298.9 mm panel (0.8% larger); only page millimetres and the band matter, so any 24-inch monitor does. A Mac and this monitor are lab-owned; the parts bought for the trial (trackpad, nibs, tape, scale, ruler) come to under US$200.
- The robot is the right arm of a bimanual YAM pair with its parallel gripper; the left arm has no role.

## Scoring

Read at the end of the run.

| Tier | Condition |
|---|---|
| 0.33 | the page has scrolled at least 10 mm toward the band (only a two-finger slide moves it) |
| 0.67 | at least half the way, 52.5 mm |
| 1.0 | the page at rest (under 1 mm of movement over 0.5 s, no coast) with the red line within 12 mm of the band's centre |
| Void, 0 | the computer registers a click (sim: total press over 0.8 N on any step), or one touch travels over 5 mm in a single stroke, which moves the pointer |

The page starts with the red line 105 mm below the band's centre (70 mm of finger travel at gain 1.5).

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | both nibs down, press kept 0.12 to 0.4 N by the force report, slide in 4 mm steps, stop 10 mm of page short, lift | 1.0 (13 s of film) |
| none | the arms wait | 0 |
| one_tip | jaw closed, the two nibs 9 mm apart read as one touch, slid the same way | 0 (pointer moved and click) |
| flick | slide at 3 times the speed, lift while moving | 0.67 (the page scrolled half the way, then missed the band; no click) |
| press | fingers pressed 6 mm into the glass, then slid | 0 (click, 4.2 N) |
| rigid | the full script with the nib give taken away (SC_K = 100000 N/m) | 0 (click, 2.6 N, and a pointer move) |
| brush | the nibs skim the glass at the touch force, same slide | 0.33 (the page moved over 10 mm, not half the way) |
| full, nib give SC_K = 350 and 1200 N/m, click SC_CLICK = 0.5 and 3.0 N | the full script at both ends of both bounds, four runs | 1.0 all four (press peaks 0.40 and 0.37 N) |
| Dream-1 | the model run, 1,200 steps | 0 (1,200 steps, one session, no session loss) |

The press window is narrow because of the nibs' give: at 0.6 N/mm the 0.12 to 0.4 N window is 0.47 mm of travel, and a rigid finger has almost none.

## Trackpad model

The trackpad's software is code (`Trial` in `task.py`), read every control step (20 Hz) from PhysX contact reports on the glass. A touch is a nib (the two finger links) pressing the glass's top face, with its contact normal within about 25 degrees of the glass's normal, at 0.05 N or more. Two touches closer than 12 mm read as one. With exactly two touches the page moves 1.5 mm per mm of their mean travel along the pad's depth, in the natural direction (fingers slide away, the page moves up). When the touches lift the page coasts at its speed over the last 0.1 s and slows with a 0.35 s time constant. One touch that travels moves the pointer, not the page. Every robot link counts toward the click force; a total over 0.8 N is a click.

Apple publishes none of the click force, coast constant, scroll gain or nib stiffness, so each is an estimate with a bound, and the scorer is run across the bound. Click force: the only figures found are a vendor's 50 to 75 g for a standard trackpad click (0.5 to 0.7 N, https://www.displaymodule.com/blogs/knowledge/what-is-capacitive-touch-technology, unverified) and Apple's Light, Medium and Firm settings (https://support.apple.com/102309); the sim takes 0.8 N, a little above the vendor's range, and the scorer passes for 0.5 to 3 N (the press is held at 0.12 to 0.4 N, so under the lowest click force); the trial measures it with a kitchen scale (see Real trial). Nib stiffness: conductive silicone stylus tips are sold at 40 to 80 Shore A (https://bettersilicone.en.made-in-china.com/product/lNcxhLkDCiWV/China-Electrically-Conductive-Silicone-Rubber-Stylus-Tips-for-Touch-Screen.html), which for a 6 mm nib gives roughly 0.35 to 1.2 N/mm (a softer nib than 0.35 N/mm, which the force noise of the sim does not hold under the click force, is not covered); the sim takes 0.6 N/mm, and the trial measures it. The glass is rigid otherwise; the nibs are not separate bodies, and their give is the glass contact's compliance.

## Fidelity table

| Number | Sim | Real | Source | Direction of the difference |
|---|---|---|---|---|
| Glass size, slope, mass | 160.0 x 114.9 mm, 3.0 degrees, 230 g | same | Apple 121932 | none |
| Click force (CLICK_N) | 0.8 N | not published; vendor 0.5 to 0.7 N, Light to Firm up to about 3 N | vendor page and Apple settings, bound above | a lower real click leaves less room under the press (harder); scorer passes 0.5 to 3 N |
| Touch force (TOUCH_N) | 0.05 N per nib | not published | estimate; bound 0 to 0.06 N per nib: the window floor is 0.12 N in total, 0.06 N per nib when two nibs share it, so a real threshold up to 0.06 N leaves the window as set | a higher real value raises the floor of the window (harder) |
| Two-touch merge (MERGE) | 12 mm | not published | estimate; bound 6 to 18 mm: not under the 6 mm nib width, and not over about 18 mm because two adjacent fingertips (centres about 18 to 22 mm apart) scroll on the real pad; nibs sit 22 mm apart | a larger real value moves toward one-touch reading (harder) |
| Pointer void (DRAG_VOID) | 5 mm of one-touch travel | any visible movement | stricter in the real check | the real check is harder |
| Scroll gain (GAIN) | 1.5 page mm per mm | macOS speed-dependent | estimate | a lower gain needs a longer slide; the start offset is set to 70 mm of travel |
| Coast (TAU) | 0.35 s | macOS momentum, not published | estimate | longer coast needs an earlier stop; the 12 mm band tolerance absorbs 8 mm of it |
| Nib stiffness (PAD_K) | 0.6 N/mm, damping 10 N s/m | 0.35 to 1.2 N/mm by Shore A bound | supplier range, measured in the trial | stiffer shortens the press window; scorer passes 0.35 to 1.2 |
| Contact normal (NORMAL_COS) | cosine 0.9, within about 25 degrees of the glass normal | not published | estimate; bound cosine 0.7 to 0.95 (18 to 45 degrees): a flat nib face on the flat glass gives a normal equal to the glass normal (cosine 1), and only a rim or edge contact, near 90 degrees, is excluded | none for the demo, which presses straight down |
| Nib spacing | about 22 mm (jaw 20 mm) | two fingertips about 20 to 25 mm apart | measured on the gripper | none |
| Friction | 0.3 static, 0.25 dynamic on the pad | silicone on glass is higher | estimate | a real nib drags more on the slide; the arm drives position, so only the force reading moves |

## Real trial

- Kit, all bought off the shelf: the Magic Trackpad, two nib covers, copper tape or thin wire, a kitchen scale and a ruler, a Mac with QuickTime and a 24-inch monitor.
- Fix the trackpad flat to the table by its rubber feet on a non-slip mat. In System Settings, Trackpad: Click set to Light, Tap to click off, Scroll direction Natural.
- Ground each nib to the arm's metal frame with conductive tape or wire, then check in a test app that both nibs show two touches on the glass.
- Measure the click force: set Click to Light, rest the pad on the kitchen scale, press a nib-tipped probe straight down on the glass until the click is heard and a ring shows, and read the scale; report it (the scorer passes for any value of 0.5 to 3 N: full scores 1.0 with the click at 0.5 N and at 3.0 N, at both ends of the nib stiffness; a measured click below 0.5 N needs the 0.12 to 0.4 N press window lowered first, and a press-hard run scores 0 at 0.5 N).
- Measure the touch force: lower a nib on the kitchen scale onto the glass and note the reading at which the test app first shows a touch; accepted range 0 to 0.06 N per nib. Measure the merge distance: bring the two nibs together in the test app and note the spacing at which two touches become one; accepted range 6 to 18 mm. The normal needs no measurement: tilt a nib to see that only an edge contact (over 45 degrees) stops registering.
- Measure the nib stiffness by pressing a nib on the kitchen scale against a ruler, and report the measured N/mm (the scorer passes for any value of 0.35 to 1.2 N/mm: full scores 1.0 at 0.35 and at 1.2 N/mm, and fails at 100 N/mm).
- Measure the scroll gain and the coast from the recording: slide two fingers 70 mm and read the page travel (the sim takes 1.5 mm of page per mm of finger travel) and the time the page takes to stop after lift-off (the sim 0.35 s time constant); the start offset and the stop point are set from these two readings.
- Record the screen with QuickTime with Show Mouse Clicks in Recording on. A click ring on any frame voids the run; any visible pointer movement in the recording is the pointer void (stricter than the sim's 5 mm of single-touch travel, so a run that passes the real check passes the sim rule). Page travel and rest are read from the recording against the band.

## Sources

- Apple, Magic Trackpad technical specifications, https://support.apple.com/en-us/121932 : sizes and mass above.
- Apple, 'Use trackpad gestures on Mac' (https://support.apple.com/guide/mac-help/mchlp1226/mac): 'Slide two fingers up or down to scroll.'
- Pew Research Center, February 2021 survey (https://www.pewresearch.org/short-reads/2021/07/16/home-broadband-adoption/ and its device tables): eight in ten White, 69% of Black and 67% of Hispanic US adults own a desktop or laptop computer.
- iD magazine, January 2021, 'How many feet of content are scrolled through in one day?' (https://www.pressreader.com/usa/id-magazine/20210101/282187948483902): source line: "The marketing company Ogilvy has found that [people] scroll through more than 300 feet of digital content each day." (all devices).
- IDC press release, 8 January 2025, 2024 worldwide PC shipments (https://www.idc.com/resource-center/press-releases/the-pc-market-closed-out-2024-with-slight-growth-and-mixed-views-on-what-2025-will-bring-according-to-idc/): 262.7 million units, almost all with a trackpad or mouse.

Apple and Magic Trackpad are trademarks of Apple Inc., named for identification only; this task is not affiliated with them.

## Licence

MIT
