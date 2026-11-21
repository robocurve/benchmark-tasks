# Take a bank card from a wallet, insert it chip-first into an ATM, press RETURN CARD, and put the card back

An ATM front at true size, laid out as an NCR SelfServ 22e (screen with four labelled side keys a side, receipt slot, keypad, green-lit card reader, cash dispenser), stands on its shelf in front of the right arm. The left hand holds a leather vertical card wallet open and upright just in front of the shelf; a bank card stands in its pocket, chip end down. The right hand slides the card out, turns it chip end first and chip up, and pushes it into the reader's closed 58 x 3 mm throat until the reader draws it in. The screen lists services beside its side keys; the hand presses the key beside RETURN CARD, the reader ejects the card with 30 mm out of the bezel, and the hand pinches that stub, draws the card straight out and slides it back into the wallet. Bank guides give the customer's steps: insert the card chip end first, the ATM holds it during the transaction, take the card when prompted (thepennyhoarder.com/bank-accounts/how-to-use-an-atm/, sofi.com/learn/content/how-to-use-an-atm/).

## Run

From this folder, with the harness's Python:

```
python demokit.py task.py OUT --mode full                 # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,lift_only,short,chip_down,no_key,wrong_key,leave,drop,hold,drop_in
python3 check_score.py                                    # the tiers on hand-built states, no simulator
```

Set `FFMPEG` to the venv's imageio_ffmpeg binary first (the system ffmpeg has no libx264).

**Result** (scripted, GPU physics, 20 Hz): task_score 1.0, 1,049 steps (52.45 s), RTX 4070 Ti SUPER; a second run on an RTX 3090 Ti with a slower final push also scored 1.0 (1,078 steps).

**Dream-1 run**: task_score 0.0 (1,400 steps, 70 s, one session, 20 rows per request, RTX 3090 Ti).

## Scoring, and the routes that score less

One tier per key substep (card out of the wallet, the reader takes it, the named key, the card back in the wallet), plus the voiding failure (the card dropped).

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | the left hand closes on the wallet's left leaf; the right hand pinches the card's top edge, slides it out, turns it chip end first and chip up, lines it up on the throat by the card's pose, pushes it 28 mm in and lets go; the reader draws it in; the closed fingertips press the key beside RETURN CARD; the reader ejects the card; the hand pinches the 30 mm stub, draws it 100 mm straight out, turns it upright, squares it and lines its chip end up over the pocket by the card's pose, lowers it along the pocket's centre line, steering by the chip end, to the pocket floor, and lets go | 1.0 |
| none | nothing | 0.0 |
| lift_only | slides the card out and puts it back | 0.0 (the card tips out of the pocket: void) |
| short | pushes the card only 12 mm into the throat and lets go | 0.0 (the reader never takes it; the card falls onto the shelf: void) |
| chip_down | the card turned over, chip face down, pushed 28 mm in | 0.25 |
| no_key | inserts it right, never presses a key | 0.5 |
| wrong_key | inserts it right, presses the key beside STATEMENT (just above RETURN CARD) | 0.5 |
| leave | presses RETURN CARD, never takes the card | 0.75 |
| drop | draws the card out, then opens the hand | 0.0 (void) |
| hold | draws the card out and holds it, never puts it back | 0.75 |
| drop_in | lets go of the card 40 mm above the pocket floor | 0.0 (the card bounces out onto the shelf: void) |

The tiers are milestones of the act, as a real reader logs them: card in, key, eject, taken. They are kept once reached, because the act ends where it began (the card in the wallet); 1.0 reads the end state now (in the pocket, let go within 10 mm of its floor, untouched, still, 2 s). The key tier is edge-triggered and first-press-only: a key held down from before the card went in does not count, and any other key first (or several at once) leaves the card in the reader. "Held" and "let go" are PhysX contact forces between the card and every link of both arms (fingers, pads, gripper housing, wrist), not nearness; a key counts only when its sprung joint has gone in 1.5 mm of its 3 mm. `check_score.py` runs 20 routes through the scorer with no simulator, including the wrong key, every key at once, a key held from before, a drop into the wallet, and do-then-undo (lifted out again after the 2 s).

## Why it matters

Most adults use a cash machine: 54% of US adults used an ATM in the past year (YouGov Profiles, November 2024 to November 2025, https://yougov.com/profiles). US ATMs dispensed 3.4 billion cash withdrawals in 2024, each begun by inserting a card (Federal Reserve Payments Study, National Payment Volumes, Top-Line Data CY 2015-24, https://www.federalreserve.gov/paymentsystems/frps_cy2015_24_topline.htm).

## Sizes

| What | In the scene | Published or estimated | Source |
|---|---|---|---|
| Card | 85.60 x 53.98 x 0.76 mm, 5 g, chip contacts 11.5 x 8.5 mm centred 15 mm from the chip end, stripe on the back | ID-1 85.60 x 53.98 x 0.76 mm | ISO/IEC 7810; contact position ISO/IEC 7816-2 |
| Card reader slot height | 249 mm above the shelf | 1039 mm above the ATM base | NCR SS32 Site Preparation B006-6585-B000, Access for All, item L |
| Shelf top | 790 mm above the ATM base | not published in that table | estimate, between the No. 5 key (786 mm) and fingerprint reader (805 mm) heights |
| Slot depth from the shelf front | 152 mm | 152 mm | NCR SS32, item L |
| Reader body | 89 x 167 x 43 mm | 89 x 167 x 43 mm | Nidec Sankyo ICT3K5-3R6940 drawing |
| Throat | 58 x 3 mm, 15 degree lead-in 4 mm deep, 18.5 mm throat block | guides 54 mm, throat block 18.5 mm | Sankyo drawing; opening and lead-in estimated |
| Trigger depth | 20 mm | | estimate |
| Ejected stub | 30 mm out of the bezel | | estimate |
| Capture timeout | 20 s after the eject | | estimate (capture after a timeout: US patents 11222514, 11704982) |
| Side keys | 8 round caps 20 mm across, 3 mm travel, sprung, about 3 N at full travel; 178 mm either side of the screen centre, rows 50 mm apart, the lowest 120 mm above the reader | positions as the photo | NCR SelfServ 22e photo; force and travel estimated |
| Screen labels | SELECT A SERVICE; right keys CASH, BALANCE, STATEMENT, RETURN CARD; left keys FAST CASH, DEPOSIT, PIN CHANGE, OTHER; block capitals 14 mm high | | as an ATM's service menu |
| Roller speed | 0.2 m/s | | estimate |
| Fascia layout | screen surround 410 x 290 mm, reader 136 mm right of the screen centre and 68 mm below the surround, keypad and receipt slot on the reader's row, dispenser below | as the photo | NCR SelfServ 22e photo (commons.wikimedia.org/wiki/File:Promsvyazbank_ATM_by_NCR.jpg); sizes scaled from the 15 in display |
| Wallet | vertical card wallet open flat, two leaves 72 x 85 mm, 3 mm leather; card pocket 6 mm between its panels and 58 mm wide, 45.6 mm deep, so 40 mm of the card stands out | | like a portrait (vertical) card holder (commons.wikimedia.org/wiki/File:Black_vertical_credit_card_holder_with_10_pockets_-_A.jpg) |

## Colliders

| Body | Colliders | Free or fixed |
|---|---|---|
| ATM | boxes for the shelf, the cabinet sides and top, the privacy hood, the screen surround, the keypad, the receipt slot and dispenser frames, the fascia round the throat, the bezel, the throat's floor, roof and sides, the card path's floor, roof and guides and its back stop; two hulls for the lead-in | fixed |
| Side keys | one hull per key cap on its own sprung slide joint (an articulation with a fixed frame) | fixed frame, keys move |
| Wallet | seven boxes: front and back panels, pocket floor and sides, spine, left leaf | fixed (held in the left hand) |
| Card | one 85.60 x 53.98 x 0.76 mm box | free, 5 g, friction 0.9 |
| Fingers | the harness's YAM finger colliders | the robot |

## What is modelled, and what is scripted

The robot's part of the act is plain contact: the card is a free body held only by friction in the fingers (about 3 N of contact on the 0.76 mm card), it has to be square and level to enter the 3 mm throat, and it has to be pushed 20 mm in. The reader's rollers are a scripted drive, as a reader's motor is to the customer: once the card is 20 mm in, chip end leading and chip up within 20 degrees, and let go, the reader sets the card's pose along its card path each step (draw in, eject after the RETURN CARD key, or capture). The ejected card is then released and lies in the throat; drawing it out is again plain contact and friction. A held card is not drawn in: on the real reader a customer lets go when the rollers take it. The side keys are sprung slide joints the fingertips push in. The wallet is fixed in place: the left gripper closes on its left leaf but does not carry it. The demo reads the card's pose to line it up with the throat and the wallet, as a person does by eye.

## Robot figures the demo keeps within

The YAM's jaws close fully on the 0.76 mm card (about 3 N of contact, about 1.5 N a finger at that width, well above the card's 0.05 N weight and the throat's drag); a key needs about 3 N. Each arm pushes about 20 N and reaches about 60 cm; the card's work point is about 0.42 m ahead of the right arm's base at 0.27 m above the bench. Loads under 0.01 N m.

## Real trial

Build the ATM front from 18 mm plywood or MDF to the sizes above (the screen, keypad, receipt slot and dispenser can be a printed sheet glued on; only the reader works), with the shelf top 249 mm below the slot's centre and its front edge 152 mm in front of the bezel; mount a motorized card reader behind a 58 x 3 mm throat. A Nidec Sankyo ICT3K5 is the reader the sizes come from; used units are sold by ATM parts resellers. Drive it from a laptop over its serial port with its host command set (accept card, eject, capture after 20 s), as an ATM's controller does. The eight side keys are panel-mount momentary push buttons (about $1 each) wired to the laptop through a USB keypad encoder board, with a printed screen sheet (or a tablet showing the menu) behind them; the laptop ejects the card when the RETURN CARD key is the first key pressed after card-in. Any expired or blank PVC ID-1 card serves as the card. The wallet is any vertical card wallet (about $10), held open in the left gripper. A person reads the laptop's log for the chip read (ICC answer-to-reset) and card-in (a 0.5 needs both, and the video must show the chip end going in first and chip up), key and eject; measures with a ruler that the card stands 36 to 44 mm out of the pocket mouth when seated, and checks from a side camera that its top edge is no more than 10 mm above rest when the fingers open; photographs the throat and the wallet; and reads capture from the bin.

| Real-trial kit | Use | Price (about) |
|---|---|---|
| Motorized card reader (Nidec Sankyo ICT3K5, used) | the reader | $100 to $300 (estimate) |
| Plywood or MDF, 18 mm, about 0.5 m^2 | shelf and fascia | $15 |
| Blank PVC ID-1 card | the card | $1 |
| Vertical card wallet | the wallet | $10 |
| 8 momentary push buttons and a USB keypad encoder | the side keys | $15 |
| Printed fascia sheet | screen, keypad, slots | $5 |
