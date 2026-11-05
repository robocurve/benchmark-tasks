"""Reusable lab props for task scenes: labelled blocks, trays/mats, push buttons, lamps,
7-segment displays, a spring scale, and state readers for monitors and scorers.

Builders take an ``mujoco.MjSpec`` (at ``build`` time). Readers are small classes that
resolve ids in ``bind(model)`` (call from the task's ``reset``) and read ``MjData``.

Conventions
- World frame: arms face +x, left arm base at y=0, right at y=-0.61, table top z=0.
- The top camera looks forward-down: image up ~ world +x, image right ~ world -y.
  Text on flat props is drawn so it reads upright in that camera.
- Visual-only geoms use contype=conaffinity=0. Gripper pads are the only finger colliders.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from robosci import scene

G = mujoco.mjtGeom
GEN = scene.ASSETS / "generated"
FONT = str(Path(__file__).parent / "fonts" / "DejaVuSans-Bold.ttf")  # bundled, DejaVu licence

RED = (0.80, 0.10, 0.10, 1)
BLUE = (0.15, 0.35, 0.80, 1)
GREEN = (0.15, 0.60, 0.25, 1)
GREY = (0.55, 0.55, 0.58, 1)
DARK = (0.15, 0.15, 0.17, 1)
CARDBOARD = (196, 160, 112)


# ---------------------------------------------------------------- materials / textures
def material(spec: mujoco.MjSpec, name: str, rgba, **kw) -> str:  # noqa: ANN001, ANN003
    """Add a material once (idempotent by name) and return its name."""
    if spec.material(name) is None:
        spec.add_material(name=name, rgba=list(rgba), **kw)
    return name


def label_png(text: str, bg=CARDBOARD, fg=(30, 30, 30), size: int = 256) -> Path:  # noqa: ANN001
    """Square label image with centred text (cached under assets/generated)."""
    from PIL import Image, ImageDraw, ImageFont

    import hashlib
    import os

    tag = "".join(c if c.isalnum() else "_" for c in text)
    tag += "_" + hashlib.md5(text.encode()).hexdigest()[:6]  # "+" and "-" must not share a file
    out = GEN / f"lbl_{tag}_{'-'.join(map(str, bg))}_{'-'.join(map(str, fg))}.png"
    if out.exists() and out.stat().st_size > 0:
        return out
    GEN.mkdir(parents=True, exist_ok=True)
    img = Image.new("RGB", (size, size), tuple(bg))
    dr = ImageDraw.Draw(img)
    edge = tuple(max(0, int(c * 0.75)) for c in bg)
    dr.rectangle([6, 6, size - 7, size - 7], outline=edge, width=6)
    pts = int(size * (0.66 if len(text) <= 1 else 0.9 / max(2, len(text)) * 1.6))
    font = ImageFont.truetype(FONT, pts)
    while pts > 8 and dr.textlength(text, font=font) > 0.84 * size:  # fit the label inside the border
        pts -= 4
        font = ImageFont.truetype(FONT, pts)
    dr.text((size // 2, size // 2 + 4), text, fill=tuple(fg), font=font, anchor="mm")
    tmp = out.with_suffix(f".{os.getpid()}.tmp.png")
    img.save(tmp)
    os.replace(tmp, out)  # atomic: parallel workers never see a half-written file
    return out


def labelled_material(spec: mujoco.MjSpec, name: str, text: str, bg=CARDBOARD, fg=(30, 30, 30)) -> str:  # noqa: ANN001
    """Material with ``text`` printed on every face of a box geom."""
    if spec.material(name) is None:
        spec.add_texture(name=f"tex_{name}", type=mujoco.mjtTexture.mjTEXTURE_CUBE,
                         file=str(label_png(text, bg, fg)))
        spec.add_material(name=name, textures=["", f"tex_{name}"])
    return name


# ---------------------------------------------------------------- objects
def block(spec: mujoco.MjSpec, name: str, pos, half, mass: float, *, label: str | None = None,  # noqa: ANN001
          rgba=None, bg=CARDBOARD, fg=(30, 30, 30), friction=(1.0, 0.01, 0.001),  # noqa: ANN001
          free: bool = True, parent=None) -> mujoco.MjsBody:  # noqa: ANN001
    """Box body named ``name`` with one collision geom also named ``name``.
    ``half`` = (hx, hy, hz) or a scalar. Sits on the table at ``pos`` (x, y[, z])."""
    half = np.broadcast_to(np.asarray(half, float), (3,))
    p = list(pos) + ([] if len(pos) == 3 else [float(half[2]) + 0.001])
    b = (parent or spec.worldbody).add_body(name=name, pos=p)
    if free:
        b.add_freejoint(name=f"{name}_free")
    kw = {}
    if label is not None:
        kw["material"] = labelled_material(spec, f"mat_{name}", label, bg, fg)
    elif rgba is not None:
        kw["rgba"] = list(rgba)
    b.add_geom(name=name, type=G.mjGEOM_BOX, size=list(half), mass=mass, friction=list(friction),
               condim=4, **kw)
    return b


def flat(spec: mujoco.MjSpec, name: str, xy, half_xy, rgba, z: float = 0.0) -> None:  # noqa: ANN001
    """Visual-only flat patch on the table (mat, marker, lane line)."""
    spec.worldbody.add_geom(name=name, type=G.mjGEOM_BOX, size=[half_xy[0], half_xy[1], 0.001],
                            pos=[xy[0], xy[1], z + 0.001], rgba=list(rgba), contype=0, conaffinity=0)


def tray(spec: mujoco.MjSpec, name: str, xy, half_xy, wall_h: float = 0.03, rgba=RED,  # noqa: ANN001
         wall: float = 0.004) -> None:
    """Static open bin: thin floor + four walls. Floor geom is ``name``."""
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], 0])
    hx, hy = half_xy
    b.add_geom(name=name, type=G.mjGEOM_BOX, size=[hx, hy, 0.002], pos=[0, 0, 0.002], rgba=list(rgba))
    for sx, sy, size in ((1, 0, [wall, hy, wall_h / 2]), (-1, 0, [wall, hy, wall_h / 2]),
                         (0, 1, [hx, wall, wall_h / 2]), (0, -1, [hx, wall, wall_h / 2])):
        b.add_geom(type=G.mjGEOM_BOX, size=size, pos=[sx * (hx - wall), sy * (hy - wall), wall_h / 2],
                   rgba=list(rgba))


def text_plate(spec: mujoco.MjSpec, name: str, xy, text: str, half=(0.03, 0.03), z: float = 0.0,  # noqa: ANN001
               bg=(240, 240, 235), fg=(20, 20, 20)) -> None:  # noqa: ANN001
    """Visual-only printed sign lying on the table (labels for bins, lanes, slots)."""
    mat = labelled_material(spec, f"mat_{name}", text, bg, fg)
    spec.worldbody.add_geom(name=name, type=G.mjGEOM_BOX, size=[half[0], half[1], 0.0015],
                            pos=[xy[0], xy[1], z + 0.0015], material=mat, contype=0, conaffinity=0,
                            quat=_TEXT_QUAT)


# Rotate a flat label so its texture "up" points to world +x (top-camera up).
_TEXT_QUAT = [0.7071068, 0, 0, -0.7071068]


# ---------------------------------------------------------------- push button
@dataclass
class ButtonSpec:
    name: str
    pos: tuple[float, float]
    cap_half: float = 0.018
    travel: float = 0.008
    height: float = 0.03  # cap top above the table when released
    rgba: tuple = RED


def push_button(spec: mujoco.MjSpec, b: ButtonSpec) -> None:
    """Spring-return push button on a dark housing. Press with the closed gripper
    (``Motion.press(side, top, depth=travel + 0.004)``). Cap top is at ``b.height``."""
    base_h = b.height - 0.012
    body = spec.worldbody.add_body(name=f"{b.name}_housing", pos=[b.pos[0], b.pos[1], 0])
    body.add_geom(type=G.mjGEOM_BOX, size=[b.cap_half + 0.008] * 2 + [base_h / 2], pos=[0, 0, base_h / 2],
                  rgba=list(DARK))
    cap = body.add_body(name=b.name, pos=[0, 0, b.height - 0.006])
    cap.add_joint(name=f"{b.name}_slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1],
                  range=[-b.travel, 0], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                  stiffness=300.0, damping=[3.0, 0, 0], armature=0.01)
    cap.add_geom(name=b.name, type=G.mjGEOM_BOX, size=[b.cap_half, b.cap_half, 0.006], mass=0.01,
                 rgba=list(b.rgba), friction=[1.0, 0.01, 0.001])


class Button:
    """Reader: ``pressed`` while travel > 50 %; ``presses`` counts rising edges."""

    def __init__(self, b: ButtonSpec) -> None:
        self.spec = b
        self.presses = 0
        self.pressed = False

    def bind(self, model: mujoco.MjModel) -> None:
        j = model.joint(f"{self.spec.name}_slide")
        self._q = j.qposadr[0]
        self.presses, self.pressed = 0, False

    def top(self) -> np.ndarray:
        return np.array([self.spec.pos[0], self.spec.pos[1], self.spec.height])

    def update(self, data: mujoco.MjData) -> bool:
        """Call from ``monitor``. Returns True on the step a new press starts."""
        down = -float(data.qpos[self._q]) > 0.5 * self.spec.travel
        edge = down and not self.pressed
        self.pressed = down
        self.presses += int(edge)
        return edge


# ---------------------------------------------------------------- lamp
def lamp(spec: mujoco.MjSpec, name: str, pos, radius: float = 0.018, rgba_on=(1.0, 0.85, 0.2, 1),  # noqa: ANN001
         post: float = 0.03) -> None:
    """Bulb on a short post. Toggle with ``Lamp.set``. Own material so it can glow."""
    material(spec, f"{name}_mat", (0.35, 0.33, 0.30, 1), emission=0.0, specular=0.5)
    b = spec.worldbody.add_body(name=name, pos=[pos[0], pos[1], pos[2] if len(pos) == 3 else 0])
    b.add_geom(type=G.mjGEOM_CYLINDER, size=[0.008, post / 2], pos=[0, 0, post / 2], rgba=list(DARK),
               contype=0, conaffinity=0)
    b.add_geom(name=name, type=G.mjGEOM_SPHERE, size=[radius], pos=[0, 0, post + radius],
               material=f"{name}_mat", contype=0, conaffinity=0)


class Lamp:
    OFF = np.array([0.35, 0.33, 0.30, 1.0])

    def __init__(self, name: str, rgba_on=(1.0, 0.85, 0.2, 1)) -> None:  # noqa: ANN001
        self.name = name
        self.on_rgba = np.array(rgba_on, float)
        self.on = False

    def bind(self, model: mujoco.MjModel) -> None:
        self._mat = model.material(f"{self.name}_mat").id
        self.set(model, False)

    def set(self, model: mujoco.MjModel, on: bool) -> None:
        self.on = bool(on)
        model.mat_rgba[self._mat] = self.on_rgba if on else self.OFF
        model.mat_emission[self._mat] = 0.9 if on else 0.0


# ---------------------------------------------------------------- 7-segment display
#   a
#  f b
#   g
#  e c
#   d
_SEGS = {"0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg", "5": "acdfg", "6": "acdefg",
         "7": "abc", "8": "abcdefg", "9": "abcdfg", "-": "g", " ": "", "E": "adefg", "r": "eg",
         "o": "cdeg", "t": "defg", "A": "abcefg", "b": "cdefg", "C": "adef", "d": "bcdeg", "F": "aefg",
         "H": "bcefg", "L": "def", "P": "abefg", "U": "bcdef", "n": "ceg"}


def seg7(spec: mujoco.MjSpec, name: str, pos, n_digits: int = 4, h: float = 0.024,  # noqa: ANN001
         dp_after: int | None = None) -> None:
    """Flat display panel facing up, readable in the top camera (text up = +x).
    ``pos`` = panel centre on the table (z = panel top height). Digit height ``h``.
    ``dp_after``: draw a fixed, always-lit decimal point after digit k (0-based from the left)."""
    w = 0.55 * h
    t = 0.12 * h
    pitch = w + 0.5 * h
    hx = h / 2 + 0.012
    hy = n_digits * pitch / 2 + 0.008
    z = pos[2]
    body = spec.worldbody.add_body(name=name, pos=[pos[0], pos[1], 0])
    body.add_geom(type=G.mjGEOM_BOX, size=[hx, hy, z / 2], pos=[0, 0, z / 2], rgba=[0.05, 0.05, 0.06, 1])
    # Segment centres in (up, right) digit coords; up = +x, right = -y.
    seg_xy = {"a": (h / 2, 0), "g": (0, 0), "d": (-h / 2, 0), "f": (h / 4, -w / 2), "b": (h / 4, w / 2),
              "e": (-h / 4, -w / 2), "c": (-h / 4, w / 2)}
    for k in range(n_digits):
        right0 = (k - (n_digits - 1) / 2) * pitch
        for s, (u, r) in seg_xy.items():
            horiz = s in "agd"
            size = [t / 2, w / 2, 0.0008] if horiz else [h / 4 - t / 4, t / 2, 0.0008]
            body.add_geom(name=f"{name}_{k}{s}", type=G.mjGEOM_BOX, size=size,
                          pos=[u, -(right0 + r), z + 0.0008], rgba=[0.15, 0.03, 0.03, 1],
                          contype=0, conaffinity=0)
    if dp_after is not None:
        right = (dp_after - (n_digits - 1) / 2) * pitch + pitch / 2
        body.add_geom(name=f"{name}_dp", type=G.mjGEOM_BOX, size=[t, t, 0.0008],
                      pos=[-h / 2, -right, z + 0.0008], rgba=[1.0, 0.15, 0.1, 1], contype=0, conaffinity=0)


class Seg7:
    ON = np.array([1.0, 0.15, 0.1, 1.0])
    OFF = np.array([0.15, 0.03, 0.03, 1.0])

    def __init__(self, name: str, n_digits: int = 4) -> None:
        self.name, self.n = name, n_digits
        self.text = ""

    def bind(self, model: mujoco.MjModel) -> None:
        self._g = {(k, s): model.geom(f"{self.name}_{k}{s}").id for k in range(self.n) for s in "abcdefg"}
        self.show(model, "")

    def show(self, model: mujoco.MjModel, text: str) -> None:
        """Right-aligned text (digits, '-', a few letters). Truncates to n digits."""
        text = str(text)[-self.n:].rjust(self.n)
        self.text = text
        for k, ch in enumerate(text):
            on = _SEGS.get(ch, "")
            for s in "abcdefg":
                model.geom_rgba[self._g[(k, s)]] = self.ON if s in on else self.OFF

    def show_number(self, model: mujoco.MjModel, value: float) -> None:
        lim = 10 ** self.n - 1
        v = int(round(np.clip(value, -(10 ** (self.n - 1) - 1), lim)))
        self.show(model, str(v))


# ---------------------------------------------------------------- spring scale
@dataclass
class ScaleSpec:
    name: str
    pos: tuple[float, float]
    pan_half: float = 0.06
    height: float = 0.03  # pan top above the table
    stiffness: float = 3000.0  # N/m: 100 g sags 0.33 mm


def scale_platform(spec: mujoco.MjSpec, s: ScaleSpec) -> None:
    """Digital-scale body: dark base, pan on a stiff vertical spring. Pair with ``seg7``
    for the readout (placed in front of it) and read with ``Scale``."""
    base_h = s.height - 0.01
    body = spec.worldbody.add_body(name=f"{s.name}_base", pos=[s.pos[0], s.pos[1], 0])
    body.add_geom(type=G.mjGEOM_BOX, size=[s.pan_half + 0.01, s.pan_half + 0.01, base_h / 2],
                  pos=[0, 0, base_h / 2], rgba=[0.85, 0.85, 0.87, 1])
    pan = body.add_body(name=s.name, pos=[0, 0, s.height - 0.004])
    pan.add_joint(name=f"{s.name}_slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1],
                  range=[-0.006, 0.002], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                  stiffness=s.stiffness, damping=[60.0, 0, 0], armature=0.05)
    pan.add_geom(name=s.name, type=G.mjGEOM_BOX, size=[s.pan_half, s.pan_half, 0.004], mass=0.05,
                 rgba=[0.75, 0.76, 0.80, 1], friction=[1.0, 0.01, 0.001], condim=4)


class Scale:
    """Reads the load on the pan in grams from the spring deflection (pan weight removed)."""

    def __init__(self, s: ScaleSpec) -> None:
        self.spec = s

    def bind(self, model: mujoco.MjModel) -> None:
        j = model.joint(f"{self.spec.name}_slide")
        self._q, self._v = j.qposadr[0], j.dofadr[0]
        self._k = float(model.jnt_stiffness[j.id])
        self._ref = float(model.qpos_spring[j.qposadr[0]])
        self._pan = 0.05

    def grams(self, data: mujoco.MjData) -> float:
        defl = self._ref - float(data.qpos[self._q])
        return 1000.0 * (self._k * defl / 9.81 - self._pan)

    def still(self, data: mujoco.MjData) -> bool:
        return abs(float(data.qvel[self._v])) < 2e-4


# ---------------------------------------------------------------- state readers
def gripper_geoms(model: mujoco.MjModel) -> set[int]:
    return {i for i in range(model.ngeom) if "tip" in model.body(model.geom_bodyid[i]).name}


def held_geoms(model: mujoco.MjModel, data: mujoco.MjData, tips: set[int]) -> set[int]:
    """Geoms currently touched by a fingertip pad."""
    out = set()
    for c in data.contact[: data.ncon]:
        g1, g2 = int(c.geom1), int(c.geom2)
        if g1 in tips:
            out.add(g2)
        if g2 in tips:
            out.add(g1)
    return out


def contacts_between(data: mujoco.MjData, a: set[int], b: set[int]) -> bool:
    for c in data.contact[: data.ncon]:
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 in a and g2 in b) or (g1 in b and g2 in a):
            return True
    return False


def in_box(p, center_xy, half_xy, margin: float = 0.0, z_max: float | None = None) -> bool:  # noqa: ANN001
    ok = bool(np.all(np.abs(np.asarray(p[:2]) - np.asarray(center_xy)) <= np.asarray(half_xy) - margin))
    return ok and (z_max is None or p[2] <= z_max)


@dataclass
class TouchTracker:
    """Stage-1 detector: a gripper touched one of ``bodies`` or one moved > 1 cm."""

    bodies: tuple[str, ...]
    touched: bool = False
    _start: dict[str, np.ndarray] = field(default_factory=dict)

    def bind(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self._tips = gripper_geoms(model)
        self._ids = {model.body(b).id for b in self.bodies}
        self._geoms = {g for g in range(model.ngeom) if model.geom_bodyid[g] in self._ids}
        self._start = {b: data.xpos[model.body(b).id].copy() for b in self.bodies}
        self.touched = False

    def update(self, data: mujoco.MjData) -> None:
        if self.touched:
            return
        if held_geoms(self._m, data, self._tips) & self._geoms:
            self.touched = True
            return
        for b, p0 in self._start.items():
            if np.linalg.norm(data.xpos[self._m.body(b).id] - p0) > 0.01:
                self.touched = True
                return
