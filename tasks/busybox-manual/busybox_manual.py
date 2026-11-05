"""Busybox task file: set a machine up from its setup sheet and start it with a two-hand control.

The busybox (``busybox/``: design, USD, state and per-step mechanics) is spawned through ``yampatch.py`` as an
articulation with its own joint actuators; ``on_step`` runs its mechanics (switch and dial detents, button springs,
lamps), tracks its state and watches for a two-hand start, and ``on_reset`` puts every control back as configured.
The setup sheet is a printed card on the table (a fixed, textured box). The docstring an agent reads is set at the
end of this file.

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/busybox-manual/busybox_manual.py -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
    --instruction "Set the machine up for program P2 as the setup sheet says, then start it." \\
    --scorer task_score --max-steps 2400
"""

from __future__ import annotations

import math
import os
import sys
import tempfile
from pathlib import Path

from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

# Task files are imported by path, so this folder is not on sys.path: add it for yampatch.py and the busybox package.
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import yampatch  # noqa: E402
from busybox import design as d  # noqa: E402  (pure Python; usd and sim are imported after Isaac Sim starts)

SCENE = "station"
TOP = TABLE_TOP[SCENE]
# START (big button) near left and the green button near right, 26.5 cm apart: one gripper cannot press both.
CFG = d.BusyboxConfig(
    layout=("switches", "dial", "sliders", "big_button", "keypad", "buttons"),
    big_button_color="white",
    big_button_label="START",
)
BUSYBOX_XY = (0.30, 0.0)  # the case's bottom centre, between the arms
REST_GAP = 0.001  # it spawns this far above the table and settles onto it
_DEBUG = bool(os.environ.get("BUSYBOX_DEBUG"))  # print button events, control changes and starts

# -- the setup sheet -------------------------------------------------------------------------------------------------
PROGRAMS = {  # program: (dial, left slider, right slider, switches on)
    "P1": (2, 1, 5, (2, 3)),
    "P2": (5, 4, 2, (1, 4)),
    "P3": (7, 3, 3, (1, 2, 3)),
}
PROGRAM = "P2"
SYNC_S = 0.5  # two-hand start: both buttons pressed within this of each other ...
START_HOLD_S = 2.0  # ... and both held down together this long
CARD_XY = (0.065, 0.0)  # the sheet's centre on the table, between the arm bases and the box: 1 cm short of it
CARD_SIZE = (0.20, 0.32, 0.001)  # x (depth), y (width), thickness [m]
CARD_PX = (1000, 625)  # image width (along -y) and height (along +x): 31 px per cm


def target(program: str = PROGRAM) -> tuple[int, tuple[int, int], tuple[bool, ...]]:
    dial, left, right, on = PROGRAMS[program]
    return dial, (left, right), tuple(i + 1 in on for i in range(d.NUM_SWITCHES))


def sheet_image():
    """The setup sheet: a table of programs and the start procedure, black on white, large print."""
    from PIL import Image, ImageDraw, ImageFont

    fonts = _HERE / "assets" / "fonts"
    big, small = (ImageFont.truetype(str(fonts / "DejaVuSans-Bold.ttf"), s) for s in (56, 32))
    img = Image.new("RGB", CARD_PX, (248, 248, 244))
    draw = ImageDraw.Draw(img)
    draw.rectangle((0, 0, CARD_PX[0] - 1, CARD_PX[1] - 1), outline=(20, 20, 20), width=8)
    draw.text((CARD_PX[0] / 2, 60), "SETUP SHEET", font=big, fill=(10, 10, 10), anchor="mm")
    cols = (100, 270, 450, 630, 840)
    head = (("PROG", ""), ("DIAL", ""), ("LEFT", "SLIDER"), ("RIGHT", "SLIDER"), ("SWITCHES", "ON"))
    for x, (h1, h2) in zip(cols, head, strict=True):
        draw.text((x, 125 if h2 else 140), h1, font=small, fill=(10, 10, 10), anchor="mm")
        if h2:
            draw.text((x, 158), h2, font=small, fill=(10, 10, 10), anchor="mm")
    draw.line((30, 182, CARD_PX[0] - 30, 182), fill=(10, 10, 10), width=4)
    for k, (prog, (dial, left, right, on)) in enumerate(PROGRAMS.items()):
        y = 232 + 75 * k
        for x, v in zip(cols, (prog, dial, left, right, " ".join(map(str, on))), strict=True):
            draw.text((x, y), str(v), font=big, fill=(10, 10, 10), anchor="mm")
    draw.line((30, 450, CARD_PX[0] - 30, 450), fill=(10, 10, 10), width=4)
    draw.text((CARD_PX[0] / 2, 500), "Other switches off.", font=small, fill=(10, 10, 10), anchor="mm")
    draw.text((CARD_PX[0] / 2, 560), "To start: press START and GREEN together, hold 2 s", font=small, fill=(10, 10, 10),
              anchor="mm")  # fmt: skip
    return img


def build_card() -> str:
    """A fixed card on the table, the sheet printed on its top face (after Isaac Sim starts: imports pxr)."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    out = Path(tempfile.gettempdir()) / "yam_tasks" / "setup_sheet"
    (out / "textures").mkdir(parents=True, exist_ok=True)
    sheet_image().save(out / "textures" / "sheet.png")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Card")
    stage.SetDefaultPrim(root.GetPrim())
    sx, sy, sz = CARD_SIZE
    cube = UsdGeom.Cube.Define(stage, "/Card/paper")
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, sz / 2))
    cube.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.95, 0.93)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    mat = UsdShade.Material.Define(stage, "/Card/Looks/sheet")
    reader = UsdShade.Shader.Define(stage, "/Card/Looks/sheet/st")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, "/Card/Looks/sheet/image")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("./textures/sheet.png")
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    surf = UsdShade.Shader.Define(stage, "/Card/Looks/sheet/surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))  # fmt: skip
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    # The print, just above the paper: image right along -y, image up along +x (read from the arms' side).
    z = sz + 0.0002
    mesh = UsdGeom.Mesh.Define(stage, "/Card/print")
    mesh.CreatePointsAttr([(-sx / 2, sy / 2, z), (-sx / 2, -sy / 2, z), (sx / 2, -sy / 2, z), (sx / 2, sy / 2, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    stage.GetRootLayer().Export(str(out / "card.usda"))
    return str(out / "card.usda")


def _make_busybox():
    from busybox.sim import BusyboxObject
    from busybox.usd import cached_build

    return BusyboxObject(cached_build(CFG), CFG, (*BUSYBOX_XY, TOP + REST_GAP))


OBJECTS = [
    yampatch.usd_object("busybox", (*BUSYBOX_XY, TOP + REST_GAP), kind="articulation", make=_make_busybox,
                        extent=d.CASE_SIZE[1]),
    yampatch.usd_object("setup_sheet", (*CARD_XY, TOP), kind="fixed", build=build_card, extent=CARD_SIZE[1]),
]  # fmt: skip


# -- state -----------------------------------------------------------------------------------------------------------

_SIM = None  # busybox.sim.BusyboxSim, once the simulator runs
_STEPS = 0
_STARTS: list[tuple[float, bool]] = []  # two-hand starts this episode: (time, settings were right)


def _sim(env):
    global _SIM
    if _SIM is None:
        from busybox.sim import BusyboxSim

        _SIM = BusyboxSim(env, CFG)
    return _SIM


def state():
    """The busybox's tracked state (``busybox.state.BusyboxState``), or None before the simulator runs."""
    return None if _SIM is None else _SIM.state


def settings_right(s) -> list[bool]:
    """Dial, left slider, right slider and the switches, each as the program says."""
    dial, sliders, switches = target()
    return [s.dial == dial, s.slider_marks[0] == sliders[0], s.slider_marks[1] == sliders[1], s.switches == switches]


def _two_hand_start(s) -> bool:
    """START and green both down, pressed within SYNC_S of each other, held together START_HOLD_S."""
    a, b = "big_button", "button_green"
    if not (s.pressed[a] and s.pressed[b]):
        return False
    return abs(s.press_start[a] - s.press_start[b]) <= SYNC_S and min(s.hold_time(a), s.hold_time(b)) >= START_HOLD_S


def _record_start(s) -> None:
    """Count a two-hand start once per pair of presses, on the step its hold completes."""
    since = max(s.press_start["big_button"], s.press_start["button_green"])
    if _two_hand_start(s) and not any(t >= since for t, _ in _STARTS):
        _STARTS.append((s.time, all(settings_right(s))))
        if _DEBUG:
            print(f"[busybox] t={s.time:.2f} two-hand start, settings {settings_right(s)}", flush=True)


def on_reset(env) -> None:
    """Every control back as configured, the clock at 0, no events and no starts."""
    global _STEPS
    sim = _sim(env)
    sim.reset()
    sim.restart_clock()
    _STEPS = 0
    _STARTS.clear()


def on_step(env, states) -> None:
    global _STEPS
    sim = _sim(env)
    _STEPS += 1
    s = sim.state
    before = (s.switches, s.slider_marks, s.dial)
    events = sim.after_step(_STEPS * env.unwrapped.step_dt)
    _record_start(s)
    if _DEBUG:
        for e in events:
            print(f"[busybox] t={e.time:.2f} {e.button} {e.kind}" + (f" held {e.held:.2f}s" if e.held else ""), flush=True)
        after = (s.switches, s.slider_marks, s.dial)
        if after != before:
            print(f"[busybox] t={s.time:.2f} switches={after[0]} sliders={after[1]} dial={after[2]}", flush=True)


yampatch.install(on_reset=on_reset, on_step=on_step)


# -- scoring ---------------------------------------------------------------------------------------------------------

NOT_FOR_USE = tuple(b for b in d.BUTTONS if b not in ("big_button", "button_green"))  # red, yellow, blue, keypad


def _forbidden(s) -> bool:
    return any(s.press_count[b] for b in NOT_FOR_USE)


def success(objects):
    """Started with the two-hand control while set up for the program, and still set up."""
    s = state()
    return bool(s is not None and not _forbidden(s) and any(ok for _, ok in _STARTS) and all(settings_right(s)))


def score(objects):
    """0.15 for each of the dial, the left slider, the right slider and the switches set as the program says (at the
    end), plus 0.4 for a two-hand start made while all four were right. 0 if the red, yellow or blue button or a
    keypad key was pressed."""
    s = state()
    if s is None or _forbidden(s):
        return 0.0
    return round(0.15 * sum(settings_right(s)) + (0.4 if any(ok for _, ok in _STARTS) else 0.0), 6)


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Left arm: read the setup sheet, switch 3 off, switch 4 on, then START. Right arm: wait for the reading, left slider 2 -> 4, right slider 4 -> 2, dial 3 -> 5,
# then green. Both press their start button at the same moment.

YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # pads 8 mm along x, 15.5 mm across
CASE_TOP = TOP + d.CASE_SIZE[2]
HOVER = CASE_TOP + 0.04 + PAD_TIP_PAST_TCP
READ_HEIGHT = 0.20  # TCP above the table while reading the sheet (higher, a straight-down TCP is out of reach)
DIAL_OPEN, DIAL_GRIP = -0.015, -0.003  # finger targets: pads 30 mm apart, then squeezing the 12 mm bar


def _xy(module: str, local_xy=(0.0, 0.0)) -> tuple[float, float]:
    c = CFG.module_center(module)
    return (BUSYBOX_XY[0] + c[0] + local_xy[0], BUSYBOX_XY[1] + c[1] + local_xy[1])


def _tip(z_above_case: float) -> float:
    return CASE_TOP + z_above_case + PAD_TIP_PAST_TCP


def _wp(t, xy, z, label, yaw, finger=FINGER_CLOSED):
    return Waypoint(t, (*xy, z), finger, label, yaw)


def _press(arm, label, xy, z_down, hold=0.5):
    y = YAW[arm]
    return [_wp(1.2, xy, HOVER, f"above {label}", y), _wp(0.4, xy, HOVER, "settle", y),
            _wp(0.8, xy, z_down, f"press {label}", y), _wp(hold, xy, z_down, "hold", y),
            _wp(0.6, xy, HOVER, "release", y)]  # fmt: skip


def _push(arm, label, start_xy, end_xy, z):
    y = YAW[arm]
    return [_wp(1.2, start_xy, HOVER, f"above {label}", y), _wp(0.4, start_xy, HOVER, "settle", y),
            _wp(0.8, start_xy, z, "down", y), _wp(2.0, end_xy, z, f"push {label}", y),
            _wp(0.6, end_xy, HOVER, "up", y)]  # fmt: skip


def _twist_dial(arm, turn: float):
    """Grip the dial's bar across it, turn it ``turn`` rad about the vertical (negative: clockwise from above)."""
    xy, y0 = _xy("dial"), YAW[arm]
    grip_z = _tip(0.016)  # pad tips 10 mm down the bar's sides, above its disk
    return [_wp(1.2, xy, HOVER, "above dial", y0, DIAL_OPEN), _wp(0.4, xy, HOVER, "settle", y0, DIAL_OPEN),
            _wp(0.8, xy, grip_z, "down", y0, DIAL_OPEN), _wp(0.6, xy, grip_z, "grip", y0, DIAL_GRIP),
            _wp(2.0, xy, grip_z, "turn", y0 + turn, DIAL_GRIP), _wp(0.6, xy, grip_z, "let go", y0 + turn, DIAL_OPEN),
            _wp(0.6, xy, HOVER, "up", y0 + turn, DIAL_OPEN), _wp(0.6, xy, HOVER, "turn back", y0)]  # fmt: skip


def _duration(plan) -> float:
    return sum(w.duration for w in plan)


def demo():
    dial, (left, right), _ = target()
    # Switches: 3 (index 2) is on, press its near end (12 mm before the pivot); 4 (index 3) is off, press its far end
    # 15 mm past the pivot, down to the case top: at this reach the rocker's slope pushes the fingers back and up.
    ys = d.switch_ys()
    # First the left arm reads the setup sheet: its wrist camera looks down at it from 20 cm above the table (the TCP
    # 3 cm beyond the sheet's centre centres the camera on it).
    read = (CARD_XY[0] + 0.03, CARD_XY[1], TOP + READ_HEIGHT)
    left_plan = [Waypoint(1.5, read, FINGER_CLOSED, "over the setup sheet", 0.0),
                 Waypoint(1.5, read, FINGER_CLOSED, "read the setup sheet", 0.0),
                 *_press("left", "switch 3", _xy("switches", (-0.012, ys[2])), _tip(0.002)),
                 *_press("left", "switch 4", _xy("switches", (0.015, ys[3])), _tip(0.0))]  # fmt: skip
    # The right arm waits out to its side, gripper down, until the left arm has read the sheet.
    right_plan = [_wp(_duration(left_plan[:2]), (0.20, -0.20), HOVER + 0.03, "wait: the left arm reads the sheet",
                      YAW["right"])]
    # Sliders: push each knob (12 mm along x) from the side it moves away from, pads 2 mm off its face.
    for i, goal in enumerate((left, right)):
        start, sy = CFG.sliders[i], d.slider_ys()[i]
        sign = 1.0 if goal > start else -1.0
        behind = 0.006 + 0.004 + 0.002  # half knob + half pad + gap
        a = _xy("sliders", (d.slider_x(start - 1) - sign * behind, sy))
        b = _xy("sliders", (d.slider_x(goal - 1) - sign * (behind - 0.002 - 0.003), sy))  # 3 mm past: friction lag
        right_plan += _push("right", f"slider {i + 1}", a, b, _tip(0.008))
    right_plan += _twist_dial("right", -d.DIAL_STEP * (dial - CFG.dial))
    # Two-hand start: both arms wait above their button, then press together and hold 2.8 s.
    start_xy = {"left": _xy("big_button"), "right": _xy("buttons", d.button_xy()["green"])}
    start_down = {"left": _tip(0.018 - 0.010), "right": _tip(0.014 - 0.008)}  # cap tops; commanded past the travel
    plans = {"left": left_plan, "right": right_plan}
    for arm in plans:
        plans[arm].append(_wp(1.5, start_xy[arm], HOVER, "above start button", YAW[arm]))
    t = max(_duration(p) for p in plans.values()) + 0.5
    for arm, p in plans.items():
        p.append(_wp(t - _duration(p), start_xy[arm], HOVER, "wait for the other arm", YAW[arm]))
        p += [_wp(0.8, start_xy[arm], start_down[arm], "press", YAW[arm]),
              _wp(2.8, start_xy[arm], start_down[arm], "hold", YAW[arm]),
              _wp(0.6, start_xy[arm], HOVER, "release", YAW[arm])]  # fmt: skip
    return plans


def docs(cfg: d.BusyboxConfig, xy=BUSYBOX_XY) -> str:
    """What an agent reads about the table, after the robot's and the room's docs."""
    sx, sy, sz = d.CASE_SIZE
    names = {"switches": "four rocker switches", "sliders": "two sliders", "buttons": "four coloured buttons",
             "dial": "a dial", "big_button": "a big button", "keypad": "a keypad with a display"}  # fmt: skip
    far = ", ".join(names[m] for m in cfg.layout[:3])
    near = ", ".join(names[m] for m in cfg.layout[3:])
    return (
        f"On the table, between the arms with its centre {xy[0]:.2f} m in front of their bases: a busybox, a loose "
        f"dark grey case {sx * 100:.0f} cm deep, {sy * 100:.0f} cm wide and {sz * 100:.0f} cm tall "
        f"({d.CASE_MASS:g} kg), its controls on top in six 11 cm modules. Seen from behind the arms, the far row "
        f"holds, left to right, {far}; the near row {near}. A serial-number sticker is on its "
        f"{cfg.serial_side} side face."
    )


_RULES = f"""

A printed setup sheet ({CARD_SIZE[0] * 100:.0f} x {CARD_SIZE[1] * 100:.0f} cm) lies flat on the table between the arm
bases and the box, readable from the arms' side. It lists programs P1, P2 and P3, each with a dial number, the
left and right slider marks and which switches are on (all others off), and the start procedure: press the big
START button and the green button together (within {SYNC_S:g} s) and hold both for {START_HOLD_S:g} s."""

__doc__ = docs(CFG, BUSYBOX_XY) + _RULES
