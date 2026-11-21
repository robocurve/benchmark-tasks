"""Take a bank card out of a wallet, insert it chip-first into an ATM's card reader, press the on-screen key beside RETURN CARD, and put the card back.

A true-size NCR SelfServ 22e front (screen with four labelled side keys a side, receipt slot, keypad, green-lit card reader, cash dispenser) stands on its
shelf in front of the right arm. Its card reader slot is a closed throat 58 x 3 mm, 249 mm above the shelf and 152 mm back from the shelf's front edge.
The left hand holds a leather card-sleeve wallet upright over the shelf; an ISO/IEC 7810 ID-1 card (85.60 x 53.98 x 0.76 mm, 5 g) stands in its pocket,
chip end down, chip facing the ATM, 40 mm out of the pocket. The right arm slides it out, turns it chip end first and chip up, and pushes it into the
throat. Once the card is 20 mm in, the right way round and let go, the reader draws it in to its stop and the screen offers its services. The arm presses
the side key beside RETURN CARD; the reader ejects the card with 30 mm out of the bezel. The arm pinches that stub, draws the card out and slides it back
into the wallet.

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --mode full,none,wrong_key --render 320x176
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = ("You are at a cash machine, holding your wallet in your left hand. Slide your bank card out of the wallet and insert it into the ATM's card slot, "
               "chip end first and chip facing up, until the reader takes it. Then press the key beside RETURN CARD on the screen (the lowest key on the right). "
               "When the reader pushes the card back out, take it and slide it back into the wallet.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (STAND_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
HZ = d.CONTROL_HZ


def _e(k, v):
    return float(os.environ.get(k, v))


# -- sizes, in metres ---------------------------------------------------------------------------------------------------------------------
CL, CW, CT = 0.08560, 0.05398, 0.00076  # ISO/IEC 7810 ID-1
CARD_MASS = 0.005
CHIP = (0.0115, 0.0085)  # gold contact plate, ISO/IEC 7816-2 position: centre about 15 mm in from the chip end
CHIP_X = CL / 2 - 0.0150

YS = _e("ATM_Y", -0.20)  # the slot's centre line, in front of the right arm
XB = _e("ATM_X", 0.42)  # the throat's face (bezel front)
BEZEL_D = 0.006  # the bezel stands this far proud of the fascia
XF = XB + BEZEL_D  # fascia face
SHELF_T = 0.020
ZSH = TOP + SHELF_T  # shelf top
SHELF_FRONT = XF - 0.152  # NCR SS32: card reader 152 mm back from the shelf front
ZS = ZSH + _e("ATM_DZ", 0.249)  # NCR SS32: card reader 1039 mm above the ATM base; shelf top estimated at 790 mm
TH, TW = 0.0030, 0.058  # throat opening, high and wide (estimate)
LEAD = 0.004  # 15 degree lead-in this deep at the bezel face, opening to TH + 2 * LEAD * tan 15
THROAT_D = 0.0185  # throat block depth (Sankyo ICT3K5 drawing)
PATH_L = 0.135  # card path from the throat face to the reader's back stop
TRIGGER = 0.020  # leading edge this far past the throat face starts the rollers (estimate)
IN_DEPTH = 0.012  # drawn in: trailing edge this far inside the throat face
STUB = 0.030  # ejected: this much of the card out of the bezel (estimate)
CAPTURE_S = 20.0
DRIVE_V = 0.20  # roller speed, m/s (estimate)
ALIGN_DEG = 20.0  # the reader takes the card only chip end leading and chip up within this

W_X, W_Y = _e("WALLET_X", 0.25), _e("WALLET_Y", -0.15)  # the card pocket's centre: the wallet held open, upright, in the left hand, just in front of the shelf
W_Z0 = TOP + _e("WALLET_DZ", 0.005)  # the wallet's bottom edge, this high over the bench
W_W, W_H, W_FRONT_H = 0.072, 0.085, 0.085  # vertical bifold card wallet, open flat: each leaf 72 mm wide and 85 mm high, the pocket as high as its leaf
W_PANEL = 0.003  # leather panel thickness
ST_GAP = 0.006  # the card pocket: 6 mm between the panels (a sleeve that holds a few cards)
EXPOSED = 0.040  # the card stands this far out of the pocket mouth
W_FLOOR = W_Z0 + W_FRONT_H - (CL - EXPOSED)  # the pocket's floor
CARD0 = (W_X, W_Y, W_FLOOR + CL / 2 + 0.0005)
Q_STAND = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # card x (chip end) down, chip face towards the ATM (+x)

LOOKS = {"fascia": m.Look(m.rgb(0.93, 0.42, 0.10), roughness=0.45), "frame": m.Look(m.rgb(0.13, 0.18, 0.48), roughness=0.5),
         "bezel": m.Look(m.rgb(0.10, 0.55, 0.30), roughness=0.3, emissive=m.rgb(0.02, 0.22, 0.08)), "dark": m.Look(m.rgb(0.16, 0.16, 0.18), roughness=0.5),
         "green": m.Look(m.rgb(0.15, 0.75, 0.30), emissive=m.rgb(0.05, 0.4, 0.12)), "shelf": m.Look(m.rgb(0.60, 0.61, 0.63), metallic=0.6, roughness=0.35),
         "reader": m.Look(m.rgb(0.06, 0.06, 0.07), roughness=0.5), "card": m.Look(m.rgb(0.12, 0.30, 0.72), roughness=0.35),
         "gold": m.Look(m.rgb(0.85, 0.68, 0.30), metallic=0.9, roughness=0.3), "stripe": m.Look(m.rgb(0.05, 0.05, 0.06), roughness=0.3),
         "white": m.Look(m.rgb(0.93, 0.93, 0.93)), "screen": m.Look(m.rgb(0.10, 0.25, 0.55), roughness=0.15, emissive=m.rgb(0.04, 0.12, 0.30)),
         "ui": m.Look(m.rgb(0.90, 0.92, 0.95), emissive=m.rgb(0.30, 0.32, 0.36)), "key": m.Look(m.rgb(0.72, 0.73, 0.75), metallic=0.7, roughness=0.3),
         "steel": m.Look(m.rgb(0.70, 0.71, 0.73), metallic=0.8, roughness=0.3), "plate": m.Look(m.rgb(0.15, 0.27, 0.62), roughness=0.4),
         "red": m.Look(m.rgb(0.80, 0.12, 0.10)), "yellow": m.Look(m.rgb(0.95, 0.80, 0.15)),
         "leather": m.Look(m.rgb(0.32, 0.18, 0.09), roughness=0.7), "seam": m.Look(m.rgb(0.55, 0.40, 0.22), roughness=0.7)}

# NCR SelfServ 22e fascia layout (reference photo on the card), in the throat-face frame: y is the customer's left, z up.
SCR_Y, SCR_Z = 0.136, 0.213  # screen surround centre: the card reader sits under its right part, 136 mm right of its centre
FAS_Y0, FAS_Y1, FAS_Z1 = -0.102, 0.374, 0.450  # orange fascia: 476 mm wide, top 450 mm above the reader
ORANGE_Z0 = -0.120  # below this the blue cabinet front, with the cash dispenser



# -- the screen's side keys and their labels -------------------------------------------------------------------------------------------------
KEYS_R = ("CASH", "BALANCE", "STATEMENT", "RETURN CARD")  # beside the right-hand keys, top to bottom
KEYS_L = ("FAST CASH", "DEPOSIT", "PIN CHANGE", "OTHER")  # beside the left-hand keys
TARGET = os.environ.get("ATM_KEY", "RETURN CARD")  # the key the instruction names
KEY_Z0, KEY_DZ = SCR_Z + 0.057, 0.050  # key rows: the lowest 120 mm above the reader, as on the 22e
KEY_DY = 0.178  # keys this far either side of the screen centre
KEY_R, KEY_H, KEY_TRAVEL = 0.010, 0.005, 0.003  # round key caps 20 mm across, 5 mm proud, 3 mm travel
KEY_N = 3.0  # force at full travel (estimate for a function key)
KEY_ON = 0.0015  # pressed: the key in by this much
KEY_GAP = 0.0035  # cap base this far in front of the screen surround, so a pressed key never meets it

_GLYPHS = {
    "A": (" ### ", "#   #", "#   #", "#####", "#   #", "#   #", "#   #"), "B": ("#### ", "#   #", "#   #", "#### ", "#   #", "#   #", "#### "),
    "C": (" ####", "#    ", "#    ", "#    ", "#    ", "#    ", " ####"), "D": ("#### ", "#   #", "#   #", "#   #", "#   #", "#   #", "#### "),
    "E": ("#####", "#    ", "#    ", "#### ", "#    ", "#    ", "#####"), "F": ("#####", "#    ", "#    ", "#### ", "#    ", "#    ", "#    "),
    "G": (" ####", "#    ", "#    ", "#  ##", "#   #", "#   #", " ### "), "H": ("#   #", "#   #", "#   #", "#####", "#   #", "#   #", "#   #"),
    "I": ("#####", "  #  ", "  #  ", "  #  ", "  #  ", "  #  ", "#####"), "L": ("#    ", "#    ", "#    ", "#    ", "#    ", "#    ", "#####"),
    "M": ("#   #", "## ##", "# # #", "# # #", "#   #", "#   #", "#   #"), "N": ("#   #", "##  #", "# # #", "#  ##", "#   #", "#   #", "#   #"),
    "O": (" ### ", "#   #", "#   #", "#   #", "#   #", "#   #", " ### "), "P": ("#### ", "#   #", "#   #", "#### ", "#    ", "#    ", "#    "),
    "R": ("#### ", "#   #", "#   #", "#### ", "# #  ", "#  # ", "#   #"), "S": (" ####", "#    ", "#    ", " ### ", "    #", "    #", "#### "),
    "T": ("#####", "  #  ", "  #  ", "  #  ", "  #  ", "  #  ", "  #  "), "U": ("#   #", "#   #", "#   #", "#   #", "#   #", "#   #", " ### "),
    "V": ("#   #", "#   #", "#   #", "#   #", "#   #", " # # ", "  #  "), " ": ("     ",) * 7,
}


def _text(s, x, y, z, align="left", px=0.0020):
    """Block capitals on the screen, 7 px high, centred on z, read by a customer facing +x (their left is +y). `y` is the text's left or right end."""
    w = len(s) * 6 * px - px
    y0 = y if align == "left" else y + w  # the left end, in +y
    boxes = []
    for i, ch in enumerate(s):
        for r, row in enumerate(_GLYPHS[ch]):
            c = 0
            while c < 5:
                if row[c] == "#":
                    e = c
                    while e < 5 and row[e] == "#":
                        e += 1
                    yl = y0 - (i * 6 + c) * px
                    boxes.append(m.box((0.0006, (e - c) * px, px), (x, yl - (e - c) * px / 2, z + (3 - r) * px)))
                    c = e
                else:
                    c += 1
    return m.join(*boxes)


def _keys():
    """The eight screen side keys, each sprung on its own slide joint (a press is the key going in, against its spring)."""
    fx = BEZEL_D - 0.008  # screen surround face, in the throat-face frame
    links = []
    for side, names in ((-1, KEYS_R), (1, KEYS_L)):
        for k, _ in enumerate(names):
            at = (fx - KEY_GAP, SCR_Y + side * KEY_DY, KEY_Z0 - k * KEY_DZ)
            links.append(m.button(f"k{'r' if side < 0 else 'l'}{k}", at, outward="-X", look="white", radius=KEY_R, height=KEY_H, travel=KEY_TRAVEL,
                                  spring=KEY_N / KEY_TRAVEL, damping=2.0, mass=0.01))
    frame = []
    for side in (-1, 1):  # the dark key bezels behind the caps
        for k in range(4):
            frame.append((f"kb{side + 1}{k}", m.cylinder(KEY_R + 0.003, KEY_GAP, (fx - KEY_GAP / 2, SCR_Y + side * KEY_DY, KEY_Z0 - k * KEY_DZ), axis="x", segments=24), "dark"))
    return m.mechanism("keys", (XB, YS, ZS), links, LOOKS, frame_parts=frame, friction=(0.6, 0.5))


def _key_name(name):
    side, k = name[1], int(name[2])
    return (KEYS_R if side == "r" else KEYS_L)[k]


KEY_LINKS = [f"k{s}{k}" for s in "rl" for k in range(4)]
TARGET_LINK = next(n for n in KEY_LINKS if _key_name(n) == TARGET)


def _bx(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return hi - lo, (lo + hi) / 2


def _wedge(x0, x1, y, z_at_x0, z_at_x1, z_far):
    """A hull in x and z (extruded over +-y): face from (x0, z_at_x0) to (x1, z_at_x1), closed out to z_far."""
    pts = [(x0, z_at_x0), (x1, z_at_x1), (x1, z_far), (x0, z_far)]
    v = np.array([(x, s * y, z) for x, z in pts for s in (-1, 1)])
    return m.Mesh(v, np.array([[0, 1, 2]]))


def _atm():
    """An NCR SelfServ 22e front at true size, in frame at the card reader throat face's centre (XB, YS, ZS): screen with four keys a side,
    receipt slot, EPP keypad, green-lit card reader, cash dispenser shutter, on its shelf. Every part a hand can reach collides."""
    h, w = TH / 2, TW / 2
    zsh = ZSH - ZS
    F0, F1 = BEZEL_D, BEZEL_D + 0.020
    BY, BZ = 0.040, 0.030  # the reader bezel: 80 x 60 mm, 6 mm proud
    boxes = []  # (name, lo, hi, look, collides)
    boxes.append(("shelf", (SHELF_FRONT - XB, FAS_Y0 - 0.02, -SHELF_T + zsh), (BEZEL_D + 0.02, FAS_Y1 + 0.02, zsh), "shelf", True))
    # fascia panel, 20 mm, with a hole round the throat; orange above ORANGE_Z0, blue below
    boxes += [("fas_lo", (F0, FAS_Y0, ORANGE_Z0), (F1, FAS_Y1, -h), "fascia", True), ("fas_hi", (F0, FAS_Y0, h), (F1, FAS_Y1, FAS_Z1), "fascia", True),
              ("fas_l", (F0, w, -h), (F1, FAS_Y1, h), "fascia", True), ("fas_r", (F0, FAS_Y0, -h), (F1, -w, h), "fascia", True),
              ("cab_lo", (F0 - 0.004, FAS_Y0, zsh), (F1, FAS_Y1, ORANGE_Z0), "frame", True)]
    # the blue cabinet round and behind the fascia
    CB = F1 + 0.20  # cabinet back
    boxes += [("cab", (PATH_L + 0.011, FAS_Y0, zsh), (CB, FAS_Y1, FAS_Z1), "frame", False),
              ("cab_r", (F0 - 0.01, FAS_Y0 - 0.03, zsh), (CB, FAS_Y0, FAS_Z1 + 0.06), "frame", True),
              ("cab_l", (F0 - 0.01, FAS_Y1, zsh), (CB, FAS_Y1 + 0.03, FAS_Z1 + 0.06), "frame", True),
              ("cab_top", (F0 - 0.01, FAS_Y0 - 0.03, FAS_Z1), (CB, FAS_Y1 + 0.03, FAS_Z1 + 0.06), "frame", True)]
    # privacy hood: orange cheeks either side of the screen and a roof over it, set back from the reader row
    boxes += [("hood_r", (-0.035, FAS_Y0, 0.075), (F0, FAS_Y0 + 0.012, FAS_Z1), "fascia", True),
              ("hood_l", (-0.035, FAS_Y1 - 0.012, 0.075), (F0, FAS_Y1, FAS_Z1), "fascia", True),
              ("hood_top", (-0.035, FAS_Y0, FAS_Z1 - 0.012), (F0, FAS_Y1, FAS_Z1), "fascia", True)]
    # screen: dark surround 410 x 290 mm, 15 in display 305 x 229 mm
    boxes += [("scr_sur", (F0 - 0.008, SCR_Y - 0.205, SCR_Z - 0.145), (F0, SCR_Y + 0.205, SCR_Z + 0.145), "dark", True)]
    # receipt printer exit (left of the keypad) and EPP keypad (under the screen's centre)
    boxes += [("rcpt", (F0 - 0.004, SCR_Y + 0.128 - 0.055, -0.021), (F0, SCR_Y + 0.128 + 0.055, 0.021), "dark", True),
              ("epp", (F0 - 0.012, SCR_Y - 0.048, -0.112), (F0, SCR_Y + 0.048, -0.028), "steel", True),
              ("info", (F0 - 0.002, SCR_Y - 0.060, -0.020), (F0, SCR_Y + 0.060, 0.050), "plate", False)]
    # cash dispenser: a dark recess with a steel shutter, 255 x 45 mm, under the keypad
    boxes += [("disp", (F0 - 0.010, SCR_Y - 0.150, -0.215), (F0 - 0.004, SCR_Y + 0.150, -0.150), "dark", True),
              ("shutter", (F0 - 0.0105, SCR_Y - 0.128, -0.205), (F0 - 0.009, SCR_Y + 0.128, -0.160), "steel", False)]
    # card reader bezel, green-lit, 6 mm proud, with the lead-in at its face
    boxes += [("bz_lo", (LEAD, -BY, -BZ), (F0, BY, -h), "bezel", True), ("bz_hi", (LEAD, -BY, h), (F0, BY, BZ), "bezel", True),
              ("bz_l", (0.0, w, -h), (F0, BY, h), "bezel", True), ("bz_r", (0.0, -BY, -h), (F0, -w, h), "bezel", True),
              ("bz_flo", (0.0, -BY, -BZ), (LEAD, BY, -h - LEAD * math.tan(math.radians(15)) - 0.0005), "bezel", True),
              ("bz_fhi", (0.0, -BY, h + LEAD * math.tan(math.radians(15)) + 0.0005), (LEAD, BY, BZ), "bezel", True)]
    # the reader behind the fascia: floor, roof, guides and back stop of the card path
    boxes += [("rd_lo", (F1, -0.045, -h - 0.012), (PATH_L, 0.045, -h), "reader", True), ("rd_hi", (F1, -0.045, h), (PATH_L, 0.045, h + 0.012), "reader", True),
              ("rd_l", (F1, w, -h), (PATH_L, 0.045, h), "reader", True), ("rd_r", (F1, -0.045, -h), (PATH_L, -w, h), "reader", True),
              ("rd_back", (PATH_L, -0.045, -h - 0.012), (PATH_L + 0.01, 0.045, h + 0.012), "reader", True)]
    parts, cols = [], []
    for name, lo, hi, look, c in boxes:
        size, cen = _bx(lo, hi)
        parts.append((name, m.box(size, cen), look))
        if c:
            cols.append(m.col_box(f"c_{name}", size, cen))
    t = math.tan(math.radians(15))
    lo_w = _wedge(0.0, LEAD, BY, -h - LEAD * t, -h, -h - LEAD * t - 0.0005)
    hi_w = _wedge(0.0, LEAD, BY, h + LEAD * t, h, h + LEAD * t + 0.0005)
    cols += [m.col_hull("c_lead_lo", lo_w), m.col_hull("c_lead_hi", hi_w)]
    # flat face details (under 2 mm): display and its text lines, side keys, keypad keys, receipt slot, reader boss and card sign
    fx = F0 - 0.008  # screen surround face
    parts.append(("display", m.box((0.001, 0.305, 0.229), (fx - 0.0005, SCR_Y, SCR_Z)), "screen"))
    parts.append(("title", _text("SELECT A SERVICE", fx - 0.0010, SCR_Y + 0.072, SCR_Z + 0.092, align="left", px=0.0016), "ui"))
    for side, names in ((-1, KEYS_R), (1, KEYS_L)):  # the on-screen labels beside the side keys, each with a tab pointing at its key
        for k, label in enumerate(names):
            z = KEY_Z0 - k * KEY_DZ
            edge = SCR_Y + side * 0.1525
            parts.append((f"tab{side + 1}{k}", m.box((0.0012, 0.010, 0.004), (fx - 0.0009, edge - side * 0.006, z)), "ui"))
            parts.append((f"lbl{side + 1}{k}", _text(label, fx - 0.0010, edge - side * 0.014, z, align="right" if side < 0 else "left"), "ui"))
    for r in range(4):
        for c in range(4):  # 3 x 4 digits, and Cancel (red), Clear (yellow), blank, Enter (green) down the right
            look = "key" if c < 3 else ["red", "yellow", "key", "green"][r]
            parts.append((f"key{r}{c}", m.box((0.004, 0.017, 0.013), (F0 - 0.014, SCR_Y + 0.033 - c * 0.022, -0.045 - r * 0.018)), look))
    parts.append(("rcpt_slot", m.box((0.0012, 0.080, 0.004), (F0 - 0.0045, SCR_Y + 0.128, -0.004)), "reader"))
    parts.append(("rcpt_led", m.box((0.0012, 0.040, 0.003), (F0 - 0.0045, SCR_Y + 0.128, 0.010)), "green"))
    parts.append(("boss", m.cylinder(0.052, 0.0015, (F0 - 0.0007, 0.0, 0.0), axis="x", segments=40), "fascia"))
    parts.append(("eye", m.cylinder(0.008, 0.003, (F0 - 0.0015, SCR_Y, FAS_Z1 - 0.035), axis="x"), "dark"))
    for k in range(4):
        parts.append((f"grille{k}", m.box((0.0012, 0.10, 0.003), (F0 - 0.0006, SCR_Y + 0.13, SCR_Z + 0.19 - k * 0.008)), "dark"))
    return m.fixed("atm", (XB, YS, ZS), parts, cols, LOOKS, friction=(0.4, 0.3))


def _wallet():
    """A brown leather vertical card wallet held open flat and upright (like a portrait card holder): the right leaf (-y) has the card pocket, a front
    panel over the card with stitched sides and bottom; the left leaf (+y) is where the left hand holds it. Frame at the pocket's bottom centre."""
    g, hw, t = ST_GAP / 2, W_W / 2, W_PANEL
    fz = W_FLOOR - W_Z0
    side = CW / 2 + 0.002
    boxes = [("front", (-g - t, -hw, 0), (-g, hw, W_FRONT_H), "leather"), ("back", (g, -hw, 0), (g + t, hw, W_H), "leather"),
             ("floor", (-g, -hw, 0), (g, hw, fz), "leather"), ("side_l", (-g, side, fz), (g, hw, W_FRONT_H), "leather"),
             ("side_r", (-g, -hw, fz), (g, -side, W_FRONT_H), "leather"),
             ("spine", (-g - t, hw, 0), (g + t, hw + 0.004, W_H), "leather"), ("leaf_l", (-g, hw + 0.004, 0), (g + t, hw + 0.004 + W_W, W_H), "leather")]
    parts, cols = [], []
    for name, lo, hi, look in boxes:
        size, cen = _bx(lo, hi)
        parts.append((name, m.box(size, cen), look))
        cols.append(m.col_box(f"c_{name}", size, cen))
    stitches = [((-hw + 0.003, 0.004), (-hw + 0.004, W_FRONT_H - 0.003)), ((hw - 0.004, 0.004), (hw - 0.003, W_FRONT_H - 0.003)),
                ((-hw + 0.003, 0.004), (hw - 0.003, 0.005)), ((-hw + 0.003, W_FRONT_H - 0.005), (hw - 0.003, W_FRONT_H - 0.004))]
    for k, ((y0, z0), (y1, z1)) in enumerate(stitches):
        size, cen = _bx((-g - t - 0.0004, y0, z0), (-g - t, y1, z1))
        parts.append((f"stitch{k}", m.box(size, cen), "seam"))
    for k, (y0, y1) in enumerate([(hw + 0.008, hw + 0.009), (hw + W_W - 0.001, hw + W_W)]):
        size, cen = _bx((-g - 0.0004, y0, 0.004), (-g, y1, W_H - 0.004))
        parts.append((f"lstitch{k}", m.box(size, cen), "seam"))
    return m.fixed("wallet", (W_X, W_Y, W_Z0), parts, cols, LOOKS, friction=(0.5, 0.4))


def _card():
    parts = [("body", m.box((CL, CW, CT)), "card"),
             ("chip", m.box((CHIP[0], CHIP[1], 0.0002), (CHIP_X, 0.002, CT / 2 + 0.0001)), "gold"),
             ("stripe", m.box((CL, 0.0127, 0.0002), (0.0, CW / 2 - 0.0056 - 0.0064, -CT / 2 - 0.0001)), "stripe")]
    return m.free("card", CARD0, parts, [m.col_box("c", (CL, CW, CT))], LOOKS, mass=CARD_MASS, friction=(0.9, 0.8), torsional_patch=None, start_quat=Q_STAND)


OBJECTS = [_atm(), _keys(), _wallet(), _card()]


# -- contact: the robot on the card ------------------------------------------------------------------------------------------------------

_VIEW: dict = {}


def robot_on_card(env) -> float:
    """Force (N) all robot links (both arms: fingers, pads, gripper housing, wrist) press on the card, summed over links. PhysX contact report."""
    base = env.unwrapped
    key = id(base)
    if key not in _VIEW:
        links = []
        for side in ("left", "right"):
            links += [str(p) for p in base.scene[f"{side}_robot"].root_view.link_paths[0]]
        path = str(base.scene["card"].root_view.prim_paths[0])
        view = base.sim.physics_sim_view.create_rigid_contact_view([path], filter_patterns=[links], max_contact_data_count=1024)
        _VIEW[key] = (view, len(links))
    view, n = _VIEW[key]
    mat = view.get_contact_force_matrix(dt=float(base.sim.get_physics_dt()))
    mat = np.asarray(mat.numpy() if hasattr(mat, "numpy") else mat.detach().cpu().numpy(), float).reshape(n, 3)
    return float(np.linalg.norm(mat, axis=1).sum())


def _drive(env, x_centre):
    """The reader's rollers and guides: the card on the card path, centre at x_centre, chip end in, chip up, still."""
    import torch

    base = env.unwrapped
    a = base.scene["card"]
    o = m._np(base.scene.env_origins[0])
    pose = torch.tensor([[x_centre + o[0], YS + o[1], ZS + o[2], 0.0, 0.0, 0.0, 1.0]], device=a.device, dtype=torch.float32)
    a.write_root_pose_to_sim(pose)
    a.write_root_velocity_to_sim(torch.zeros((1, 6), device=a.device, dtype=torch.float32))


# -- the reader and the scorer ------------------------------------------------------------------------------------------------------------

CORNERS = np.array([(sx * CL / 2, sy * CW / 2, sz * CT / 2) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
X_IN = XB + IN_DEPTH + CL / 2  # card centre when drawn in
X_OUT = XB - STUB + CL / 2  # card centre when ejected
TOUCH_N = 0.05  # robot contact above this counts as touching
STAND_STEPS = int(2.0 * HZ)  # 2 s in the wallet, let go
SPINUP_S = 0.4  # the rollers start this long after a card is detected and let go (estimate)
RELEASE_MAX = 0.010  # the card let go within this of the pocket floor (slid in, not dropped in)


def in_wallet(c) -> bool:
    """In the wallet's pocket: long axis within 15 degrees of vertical, its lowest point on the pocket floor, its centre over the pocket."""
    R = c.rotation()
    low = float(c.to_world(CORNERS)[:, 2].min())
    return bool(abs(R[2, 0]) >= math.cos(math.radians(15.0)) and abs(c.pos[0] - W_X) <= ST_GAP / 2 + 0.002 and abs(c.pos[1] - W_Y) <= 0.008
                and abs(low - W_FLOOR) <= 0.004)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.n = 0
        self.reader = "idle"  # idle, spinup, drawing, inside, ejecting, presented, removed, capturing
        self.t = 0
        self.x = 0.0
        self.lifted = self.took = self.keyed = False
        self.key = None  # the first side key pressed with the card inside
        self.down: set = set()
        self.stand_n = 0
        self.held_low = 1.0  # the card's height over the pocket floor when the robot last touched it
        self.void, self.why = False, ""
        self.score, self.full = 0.0, False

    def _say(self, what):
        print(f"[atm] step {self.n}: {what}", flush=True)

    def step(self, env):
        self.n += 1
        c = m.state(env, "card")
        R = c.rotation()
        pts = c.to_world(CORNERS)
        depth = float(pts[:, 0].max() - XB)  # how far the card's innermost point is past the throat face
        touch = robot_on_card(env)
        in_throat = bool(depth > 0 and np.all(np.abs(pts[:, 1] - YS) <= TW / 2 + 0.002) and np.all(np.abs(pts[:, 2] - ZS) <= TH / 2 + LEAD + 0.002))
        right_way = bool(R[0, 0] >= math.cos(math.radians(ALIGN_DEG)) and R[2, 2] >= math.cos(math.radians(ALIGN_DEG)))
        keys = m.state(env, "keys")
        down = {k for k in KEY_LINKS if abs(keys.joint(k)) >= KEY_ON}
        new = sorted(down - self.down)  # keys that went in this step
        self.down = down
        st = self.reader
        if st == "idle" and in_throat and depth >= TRIGGER and right_way and touch < TOUCH_N:
            self.reader, self.t = "spinup", 0
            self._say(f"reader: card detected, {depth * 1000:.1f} mm in, chip end first and chip up; let go")
        elif st == "spinup":
            self.t += 1
            if touch >= TOUCH_N or depth < TRIGGER - 0.003:
                self.reader = "idle"
            elif self.t >= SPINUP_S * HZ:
                self.reader, self.x = "drawing", float(c.pos[0])
        elif st == "drawing":
            self.x = min(X_IN, self.x + DRIVE_V / HZ)
            _drive(env, self.x)
            if self.x >= X_IN:
                self.reader, self.took = "inside", True
                self._say("reader: card drawn in to its stop (card inside); screen shows the service keys")
        elif st == "inside":
            _drive(env, X_IN)
            if new and self.key is None:
                self.key = new
                if new == [TARGET_LINK]:
                    self.keyed, self.reader, self.x = True, "ejecting", X_IN
                    self._say(f"key beside {TARGET} pressed: card returned")
                else:
                    self._say(f"key beside {[_key_name(k) for k in new]} pressed, not {TARGET}: the reader keeps the card")
        elif st == "ejecting":
            self.x = max(X_OUT, self.x - DRIVE_V / HZ)
            _drive(env, self.x)
            if self.x <= X_OUT:
                self.reader, self.t = "presented", 0
                self._say(f"reader: card ejected, {STUB * 1000:.0f} mm out of the bezel")
        elif st == "presented":
            self.t += 1
            if depth < 0:
                self.reader = "removed"
                self._say(f"reader: card taken out after {self.t / HZ:.1f} s")
            elif self.t >= CAPTURE_S * HZ:
                self.reader, self.x = "capturing", float(c.pos[0])
                self._say("reader: card not taken within 20 s: captured")
        elif st == "capturing":
            self.x = min(X_IN, self.x + DRIVE_V / HZ)
            _drive(env, self.x)
        # tiers
        c = m.state(env, "card")
        low = float(c.to_world(CORNERS)[:, 2].min())
        if not self.lifted and low >= W_Z0 + W_FRONT_H + 0.005:
            self.lifted = True
            self._say("card out of the wallet")
        if touch >= TOUCH_N:
            self.held_low = low - W_FLOOR
        home = in_wallet(c)
        on_shelf = low <= ZSH + 0.002 and not home
        if (on_shelf or low < ZSH - 0.01) and not self.void:
            self.void, self.why = True, "dropped"
            self._say("VOID: card dropped")
        back = self.keyed and self.reader == "removed" and home and self.held_low <= RELEASE_MAX
        self.stand_n = self.stand_n + 1 if (back and touch < TOUCH_N and float(np.linalg.norm(c.lin_vel)) < 0.02) else 0
        tier = 0.25 if self.lifted else 0.0
        if self.took:
            tier = 0.5
        if self.keyed:
            tier = 0.75
        if self.stand_n >= STAND_STEPS:
            tier = 1.0
        self.score = 0.0 if self.void else tier
        self.full = self.score >= 1.0
        if os.environ.get("ATM_TRACE") and self.n % int(os.environ["ATM_TRACE"]) == 0:
            self._say(f"reader {self.reader} depth {depth * 1000:.1f} mm touch {touch:.2f} N low {(low - W_FLOOR) * 1000:.1f} keys {sorted(down)} "
                      f"xx {R[0, 0]:.2f} zz {R[2, 2]:.2f} score {self.score} stand_n {self.stand_n}")
            if low - W_FLOOR < 0.08:
                p = c.to_world(CORNERS) - np.array([W_X, W_Y, W_FLOOR])
                self._say(f"  in pocket frame (mm): x {p[:, 0].min() * 1000:.1f}..{p[:, 0].max() * 1000:.1f} (panels +-{ST_GAP * 500:.0f}) "
                          f"y {p[:, 1].min() * 1000:.1f}..{p[:, 1].max() * 1000:.1f} (sides +-{(CW / 2 + 0.002) * 1000:.0f}) z {p[:, 2].min() * 1000:.1f}")


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the scripted demo ------------------------------------------------------------------------------------------------------------------------
# It reads the card's pose to steer it (the card is in the wrist camera's view): the same correction a person makes by eye.

DOWN = d.down(90)  # pointing down, fingers closing along x: across the upright card's thickness
INSERT = d._as_grip(d.axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -math.pi / 2) @ DOWN.rot)  # pointing +x, closing along z
PINCH_IN = _e("ATM_PINCH", 0.017)  # the TCP this far below the card's top edge (pads 13 mm past it): at 10 mm the card squirts out of the pinch as it leaves the pocket
PUSH_TO = _e("ATM_PUSH", 0.028)  # leading edge this far into the throat
TIP_GAP = _e("ATM_TIPGAP", 0.004)  # pad tips this far short of the bezel face when taking the stub
TIP = 0.013  # pad tips this far past the TCP


def _card_point(ctx, local):
    """A point on the card, in the world, from the card's pose (what the wrist camera sees)."""
    st = ctx.state("card")
    return np.asarray(st.pos, float) + m.quat_matrix(st.quat_xyzw) @ np.asarray(local, float)


def _square(ctx, r, want, label):
    """Turn the hand until the card sits in the pose `want` (a rotation matrix; its face may point either way): a thin card pivots in a pinch,
    so it turns in the fingers; a person squares it by eye before a slot or a pocket."""
    for _ in range(3):
        R = m.quat_matrix(ctx.state("card").quat_xyzw)
        W = np.asarray(want, float)
        if np.dot(R[:, 2], W[:, 2]) < 0:  # either face out: turn it half about its long axis
            W = W @ np.diag([1.0, -1.0, -1.0])
        E = W @ R.T
        ang = math.acos(max(-1.0, min(1.0, (np.trace(E) - 1.0) / 2.0)))
        print(f"[demo] {label}: card {math.degrees(ang):.1f} deg off", flush=True)
        if ang < math.radians(0.7):
            return
        yield from r.move(r.at(), grip=d._as_grip(E @ r.grip.rot), t=0.6, label=label)
        yield from r.wait(0.2)


LEAD_PT = (CL / 2, 0.0, 0.0)  # the chip end's centre, in the card's frame


def _key_at(name):
    side = -1 if name[1] == "r" else 1
    return np.array([XB + BEZEL_D - 0.008 - KEY_GAP - KEY_H, YS + SCR_Y + side * KEY_DY, ZS + KEY_Z0 - int(name[2]) * KEY_DZ])


def demo_script(ctx):
    r, l = ctx.right, ctx.left
    mode = ctx.mode
    if mode == "none":
        yield from r.wait(3.0)
        return
    top = CARD0[2] + CL / 2
    grab = np.array([W_X, W_Y, top - PINCH_IN])
    hold = np.array([W_X, W_Y + W_W / 2 + 0.004 + 0.062, W_Z0 + W_H - 0.010])  # the left leaf's top edge near its outer corner, 100 mm from the card
    yield from l.move(hold + [0, 0, 0.05], grip=d.down(-90), finger=d.FINGER_OPEN, t=2.0, label="left hand to the wallet")
    yield from l.move(hold, t=0.8, label="fingers round the left leaf's top edge")
    yield from l.close(width=2 * W_PANEL + ST_GAP, force=10.0, label="hold the wallet")
    yield from r.move(grab + [0, 0, 0.05], grip=DOWN, finger=-0.012, t=2.0, label="over the card")
    yield from r.servo(grab + [0, 0, 0.05], timeout=1.0)
    yield from r.move(grab, t=1.0, label="fingers round the card's top")
    yield from r.servo(grab, timeout=1.0)
    print(f"[demo] at the card: TCP off {np.round((r.tcp() - grab) * 1000, 1).tolist()} mm", flush=True)
    yield from r.close(force=30.0)
    yield from r.move(grab + [0, 0, _e("ATM_LIFT", 0.065)], t=1.5, label="slide the card out of the wallet")
    if mode == "lift_only":
        yield from _seat(ctx, r)
        return
    grip = INSERT
    if mode == "chip_down":  # roll the card over: chip face down
        grip = d._as_grip(INSERT.rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.pi))
    pre = np.array([XB - 0.030, YS, ZS])  # the chip end's centre 30 mm short of the throat
    lead = lambda: _card_point(ctx, LEAD_PT)  # noqa: E731
    yield from r.move(np.array([XB - 0.12, YS, ZS]), grip=grip, t=2.5, label="turn the card chip end first")
    yield from r.servo(lambda s: pre, read=lead, tol=0.001, timeout=2.0, label="card in front of the throat")
    if mode != "chip_down":
        yield from _square(ctx, r, np.eye(3), "square the card to the slot")
    yield from r.servo(lambda s: pre, read=lead, tol=0.0007, timeout=2.0, label="line the card up with the throat")
    print(f"[demo] card at the throat: chip end off {np.round((lead() - pre) * 1000, 1).tolist()} mm", flush=True)
    depth_goal = PUSH_TO if mode != "short" else 0.012
    goal = np.array([XB + depth_goal, YS, ZS])
    yield from r.servo(lambda s: pre + (goal - pre) * min(1.0, s / 2.0), read=lead, tol=0.0015, timeout=3.5, label="push into the throat")
    print(f"[demo] pushed: chip end {(lead()[0] - XB) * 1000:.1f} mm into the throat", flush=True)
    yield from r.open(t=0.3, label="let go")
    yield from r.move(r.at() - [_e("ATM_BACK", 0.10), 0, 0], t=1.0, label="back off")
    if mode in ("short", "chip_down"):
        yield from r.wait(4.0)
        return
    yield from ctx.until(lambda: ctx.pos("card")[0] >= X_IN - 0.001, timeout=6.0, label="watch the reader draw the card in")
    if mode == "no_key":
        yield from r.wait(8.0)
        return
    key = TARGET_LINK if mode != "wrong_key" else next(n for n in KEY_LINKS if _key_name(n) == "STATEMENT")
    at = _key_at(key)
    yield from r.close(t=0.3, label="close the fingers")
    yield from r.move(at - [TIP + 0.03, 0, 0], grip=INSERT, t=2.0, label=f"in front of the key beside {_key_name(key)}")
    yield from r.servo(at - [TIP + 0.03, 0, 0], timeout=1.0)
    yield from r.move(at - [TIP - 0.004, 0, 0], t=1.0, label=f"press the key beside {_key_name(key)}")  # fingertips 4 mm past the cap face: the key goes in its 3 mm
    yield from r.wait(0.3)
    yield from r.move(r.at() - [0.05, 0, 0], t=0.8, label="off the key")
    if mode == "wrong_key":
        yield from r.wait(5.0)
        return
    yield from ctx.until(lambda: ctx.pos("card")[0] <= X_OUT + 0.001, timeout=8.0, label="wait for the eject")  # a person watches the card come back out
    yield from r.wait(0.5)
    if mode == "leave":
        yield from r.wait(22.0)
        return
    c = ctx.pos("card")
    tcp = np.array([XB - TIP_GAP - TIP, YS, c[2]])
    yield from r.move(tcp - [0.06, 0, 0], grip=INSERT, finger=-0.010, t=1.5, label="in front of the stub")
    yield from r.servo(tcp - [0.06, 0, 0], timeout=1.0)
    yield from r.move(tcp, t=1.0, label="fingers over and under the stub")
    yield from r.servo(tcp, timeout=1.0)
    yield from r.close(force=30.0)
    yield from r.move(r.at() - [0.10, 0, 0], t=1.5, label="draw the card straight out")
    if mode == "drop":
        yield from r.open()
        yield from r.wait(3.0)
        return
    if mode == "hold":
        yield from r.wait(4.0)
        return
    yield from _seat(ctx, r, drop_from=0.040 if mode == "drop_in" else 0.0)


def _seat(ctx, r, drop_from=0.0):
    """Slide the held card back into the wallet's pocket: upright over the pocket, lined up by its own pose, lowered in, let go."""
    over = np.array([W_X, W_Y, W_Z0 + W_FRONT_H + 0.012])  # the chip end's centre 12 mm over the pocket mouth
    low = lambda: _card_point(ctx, LEAD_PT)  # noqa: E731
    yield from r.move(np.array([W_X, W_Y, W_FLOOR + CL / 2 + 0.08]), grip=DOWN, t=2.5, label="card upright over the wallet")
    yield from r.wait(0.5)
    # The card hangs from a light pinch and pivots in it when the hand moves: a straight shift after squaring swung it 7 deg, and a card
    # that leans enters the 6 mm pocket at a slant and jams between the panels 30 mm in. So turn and shift the hand together, so that the
    # card ends square and its chip end over the pocket, let it settle, read it, and repeat until both hold.
    W = m.quat_matrix(Q_STAND)
    for k in range(8):
        R = m.quat_matrix(ctx.state("card").quat_xyzw)
        Wk = W if np.dot(R[:, 2], W[:, 2]) >= 0 else W @ np.diag([1.0, -1.0, -1.0])
        E = Wk @ R.T
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(E) - 1.0) / 2.0))))
        tcp = r.tcp()
        err = low() - over
        print(f"[demo] over the pocket, pass {k}: card {ang:.2f} deg off, chip end off {np.round(err * 1000, 1).tolist()} mm", flush=True)
        if ang < 1.0 and np.linalg.norm(err[:2]) < 0.0007:
            break
        want = over - E @ (low() - tcp)  # the TCP that puts the turned card's chip end over the pocket
        yield from r.move(r.at() + (want - tcp), grip=d._as_grip(E @ r.grip.rot), t=1.0, label="square the card over the pocket")
        yield from r.wait(0.6)
    # Lower it along the pocket's centre line, steering by the chip end as it goes: the arm's pose drifts a few degrees over a 5 cm descent,
    # and a chip end 3 mm off centre rubs the back panel and stalls the card in the light pinch.
    seat = np.array([W_X, W_Y, W_FLOOR + 0.001 + drop_from])
    start, T = low(), _e("ATM_SEAT_T", 3.0)
    yield from r.servo(lambda s: start + (seat - start) * min(1.0, s / T), read=low, tol=0.0015, timeout=T + 1.5, label="slide it into the pocket")
    yield from r.wait(0.3)
    print(f"[demo] seated: chip end off {np.round((low() - seat) * 1000, 1).tolist()} mm", flush=True)
    yield from r.open(t=0.4, label="let go")
    yield from r.move(r.at() + [0, 0, 0.08], t=1.0, label="clear")
    yield from r.wait(3.0)
