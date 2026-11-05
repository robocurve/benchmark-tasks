"""The busybox as an articulated USD asset, written with plain ``pxr`` (no Isaac Sim needed).

Layout of the stage (default prim ``/Busybox``, which carries the ArticulationRootAPI; a floating base):

- ``/Busybox/case``: the root rigid body (the case, the switch frames as colliders, and every printed label,
  plate, rim and lamp as visuals). Its origin is the bottom centre of the case (the box frame, see
  :mod:`busybox.design`).
- ``/Busybox/<joint>_link``: one rigid body per moving part, a sibling of the case, posed at its joint.
- ``/Busybox/joints/<joint>``: the joints, named as in :data:`busybox.design.JOINTS`. Each has a drive (its
  gains are set by the actuator config in :mod:`busybox.sim`).
- ``/Busybox/Looks``: materials; textures are PNGs in ``textures/`` next to the USD file.

Self-collisions are turned off when the asset is spawned, so moving parts never collide with the case.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from busybox import design as d
from busybox.textures import write_textures

BUILD_VERSION = 2  # bump when the geometry changes, so cached builds are redone
GRIP_FRICTION = (1.2, 1.0)  # static, dynamic: as yam_arena's graspable objects
ROOT = "/Busybox"
CASE = f"{ROOT}/case"
LOOKS = f"{ROOT}/Looks"


def linear_rgb(srgb) -> tuple[float, float, float]:
    """sRGB (0..1) to the linear colour USD shaders take."""
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb)


def link_name(joint: str) -> str:
    return f"{joint}_link"


def lamp_path(i: int, on: bool) -> str:
    """The prim of switch ``i``'s lamp, lit (``on``) or dark, relative to the asset root."""
    return f"case/lamp_{i}_{'on' if on else 'off'}"


class _Builder:
    def __init__(self, stage: Usd.Stage, cfg: d.BusyboxConfig):
        self.stage, self.cfg = stage, cfg
        self.materials: dict[str, UsdShade.Material] = {}
        self._n = 0
        grip = UsdShade.Material.Define(stage, f"{LOOKS}/grip_physics")
        api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
        api.CreateStaticFrictionAttr(GRIP_FRICTION[0])
        api.CreateDynamicFrictionAttr(GRIP_FRICTION[1])
        api.CreateRestitutionAttr(0.0)
        self.grip = grip

    # -- materials -----------------------------------------------------------------------

    def color(self, rgb, roughness: float = 0.5) -> UsdShade.Material:
        key = f"color_{'_'.join(f'{round(c * 255):02x}' for c in rgb)}"
        if key not in self.materials:
            mat = UsdShade.Material.Define(self.stage, f"{LOOKS}/{key}")
            shader = UsdShade.Shader.Define(self.stage, f"{LOOKS}/{key}/surface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(rgb)))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            self.materials[key] = mat
        return self.materials[key]

    def texture(self, name: str, emissive: bool = False, roughness: float = 0.6) -> UsdShade.Material:
        """A material showing ``textures/<name>.png`` through the mesh's ``st`` coordinates."""
        key = f"tex_{name}"
        if key not in self.materials:
            base = f"{LOOKS}/{key}"
            mat = UsdShade.Material.Define(self.stage, base)
            reader = UsdShade.Shader.Define(self.stage, f"{base}/st")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            tex = UsdShade.Shader.Define(self.stage, f"{base}/image")
            tex.CreateIdAttr("UsdUVTexture")
            tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./textures/{name}.png")
            tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            shader = UsdShade.Shader.Define(self.stage, f"{base}/surface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
            if emissive:
                shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            self.materials[key] = mat
        return self.materials[key]

    @staticmethod
    def bind(prim: Usd.Prim, mat: UsdShade.Material) -> None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    # -- geometry ------------------------------------------------------------------------

    def body(self, path: str, pos, mass: float) -> UsdGeom.Xform:
        xf = UsdGeom.Xform.Define(self.stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(mass)
        return xf

    def _collide(self, prim: Usd.Prim) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.grip, UsdShade.Tokens.weakerThanDescendants, "physics")

    def box(self, parent: str, name: str, size, pos, rgb, collide: bool = True) -> Usd.Prim:
        cube = UsdGeom.Cube.Define(self.stage, f"{parent}/{name}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*pos))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
        self.bind(cube.GetPrim(), self.color(rgb))
        if collide:
            self._collide(cube.GetPrim())
        return cube.GetPrim()

    def cylinder(self, parent: str, name: str, radius: float, height: float, pos, mat, collide: bool = True):
        cyl = UsdGeom.Cylinder.Define(self.stage, f"{parent}/{name}")
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(height)
        cyl.CreateAxisAttr("Z")
        cyl.CreateExtentAttr([(-radius, -radius, -height / 2), (radius, radius, height / 2)])
        cyl.AddTranslateOp().Set(Gf.Vec3d(*pos))
        self.bind(cyl.GetPrim(), mat if isinstance(mat, UsdShade.Material) else self.color(mat))
        if collide:
            self._collide(cyl.GetPrim())
        return cyl.GetPrim()

    def quad(self, parent: str, name: str, center, right, up, w: float, h: float, mat) -> Usd.Prim:
        """A flat label ``w`` x ``h`` showing a whole texture: image right along ``right``, image up along
        ``up``; it faces ``right x up``. Visual only."""
        c, r, u = (np.asarray(v, dtype=float) for v in (center, right, up))
        corners = [
            c - r * w / 2 - u * h / 2,
            c + r * w / 2 - u * h / 2,
            c + r * w / 2 + u * h / 2,
            c - r * w / 2 + u * h / 2,
        ]
        return self._mesh(parent, name, corners, [(0, 0), (1, 0), (1, 1), (0, 1)], [[0, 1, 2, 3]], mat)

    def dome(self, parent: str, name: str, center, radius: float, height: float, mat, rings=6, seg=24):
        """A dome whose texture is projected from above (image up = +x)."""
        pts, st = [(0.0, 0.0, height)], [(0.5, 0.5)]
        for i in range(1, rings + 1):
            a = i / rings * math.pi / 2
            for j in range(seg):
                b = 2 * math.pi * j / seg
                x, y = radius * math.sin(a) * math.cos(b), radius * math.sin(a) * math.sin(b)
                pts.append((x, y, height * math.cos(a)))
                st.append((0.5 - y / (2 * radius), 0.5 + x / (2 * radius)))

        def ring(i, j):
            return 1 + (i - 1) * seg + j % seg

        faces = []
        for j in range(seg):
            faces.append([0, ring(1, j), ring(1, j + 1)])
            for i in range(1, rings):
                faces.append([ring(i, j), ring(i + 1, j), ring(i + 1, j + 1), ring(i, j + 1)])
        pts = [np.asarray(center) + np.asarray(p) for p in pts]
        return self._mesh(parent, name, pts, st, faces, mat)

    def _mesh(self, parent, name, points, st, faces, mat) -> Usd.Prim:
        mesh = UsdGeom.Mesh.Define(self.stage, f"{parent}/{name}")
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in points])
        mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
        mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateSubdivisionSchemeAttr("none")
        pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set([Gf.Vec2f(*map(float, s)) for s in st])
        lo, hi = np.min(points, axis=0), np.max(points, axis=0)
        mesh.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
        self.bind(mesh.GetPrim(), mat)
        return mesh.GetPrim()

    # -- joints --------------------------------------------------------------------------

    def joint(self, name: str, kind: str, axis: str, pos, limits: tuple[float, float] | None) -> None:
        """A joint from the case to ``<name>_link``, at ``pos`` (box frame, where the link's origin is)."""
        path = f"{ROOT}/joints/{name}"
        if kind == "revolute":
            j = UsdPhysics.RevoluteJoint.Define(self.stage, path)
            drive = "angular"
            if limits is not None:  # USD takes revolute limits in degrees
                j.CreateLowerLimitAttr(math.degrees(limits[0]))
                j.CreateUpperLimitAttr(math.degrees(limits[1]))
        else:
            j = UsdPhysics.PrismaticJoint.Define(self.stage, path)
            drive = "linear"
            j.CreateLowerLimitAttr(limits[0])
            j.CreateUpperLimitAttr(limits[1])
        j.CreateAxisAttr(axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(CASE)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{ROOT}/{link_name(name)}")])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos)))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0))
        api = UsdPhysics.DriveAPI.Apply(j.GetPrim(), drive)
        api.CreateTypeAttr("force")
        api.CreateStiffnessAttr(0.0)
        api.CreateDampingAttr(0.0)

    def part(self, name: str, pos, kind: str, axis: str, limits, mass: float) -> str:
        """A moving part's body at ``pos`` and its joint; returns the body's path for its geometry."""
        path = f"{ROOT}/{link_name(name)}"
        self.body(path, pos, mass)
        self.joint(name, kind, axis, pos, limits)
        return path

    # -- the box -------------------------------------------------------------------------

    def build(self) -> None:
        cfg, (_sx, sy, sz) = self.cfg, d.CASE_SIZE
        self.body(CASE, (0.0, 0.0, 0.0), d.CASE_MASS)
        self.box(CASE, "shell", d.CASE_SIZE, (0, 0, sz / 2), d.CASE_COLOR)
        # The serial sticker on one long side face, upright, read from that side.
        if cfg.serial_side == "left":
            center, right = (0.0, sy / 2 + d.LABEL_T, sz / 2), (-1, 0, 0)
        else:
            center, right = (0.0, -sy / 2 - d.LABEL_T, sz / 2), (1, 0, 0)
        self.quad(CASE, "serial", center, right, (0, 0, 1), 0.10, 0.03, self.texture("serial"))
        plates = {"switches": "plate_switches", "sliders": "plate_sliders", "dial": "plate_dial"}
        for module in d.MODULES:
            c = cfg.module_center(module)
            self.quad(CASE, f"{module}_plate", c, (0, -1, 0), (1, 0, 0), d.MODULE_SIZE, d.MODULE_SIZE,
                      self.texture(plates.get(module, "plate_plain")))  # fmt: skip
            getattr(self, f"_{module}")(np.asarray(c))

    def _switches(self, c) -> None:
        hx, hy, h, wall = 0.0175, 0.0085, 0.005, 0.002  # opening half sizes, frame height, wall
        pivot = 0.0075  # rocker pivot height: its ends stand above the low frame, so a fingertip can rock it over
        for i, y in enumerate(d.switch_ys()):
            p = c + (0, y, 0)
            walls = [
                ((wall, 2 * hy + 2 * wall, h), (hx + wall / 2, 0)),
                ((wall, 2 * hy + 2 * wall, h), (-hx - wall / 2, 0)),
                ((2 * hx, wall, h), (0, hy + wall / 2)),
                ((2 * hx, wall, h), (0, -hy - wall / 2)),
            ]
            for k, (size, (ox, oy)) in enumerate(walls):
                self.box(CASE, f"switch_{i}_frame{k}", size, p + (ox, oy, h / 2), d.FRAME_COLOR)
            # The rocker pivots just below the frame's rim.
            link = self.part(f"switch_{i}", p + (0, 0, pivot), "revolute", "Y",
                             (-d.SWITCH_ANGLE, d.SWITCH_ANGLE), d.PART_MASS["switch"])  # fmt: skip
            self.box(link, "rocker", (0.033, 0.0156, 0.005), (0, 0, 0), d.ROCKER_COLOR)
            self.box(link, "mark", (0.012, 0.0158, 0.0052), (0.0095, 0, 0), d.ROCKER_MARK_COLOR, collide=False)
            lamp = np.asarray((c[0] + d.LAMP_X, p[1], c[2]))
            self.cylinder(CASE, f"lamp_{i}_bezel", d.LAMP_RADIUS + 0.0015, 0.002, lamp + (0, 0, 0.001),
                          d.FRAME_COLOR, collide=False)  # fmt: skip
            self.dome(CASE, f"lamp_{i}_off", lamp + (0, 0, 0.002), d.LAMP_RADIUS, 0.004, self.texture("lamp_off"))
            on = self.dome(CASE, f"lamp_{i}_on", lamp + (0, 0, 0.0021), d.LAMP_RADIUS * 1.03, 0.0042,
                           self.texture("lamp_on", emissive=True))  # fmt: skip
            if not self.cfg.switches[i]:
                UsdGeom.Imageable(on).MakeInvisible()

    def _sliders(self, c) -> None:
        for i, y in enumerate(d.slider_ys()):
            link = self.part(f"slider_{i}", c + (0, y, 0.009), "prismatic", "X", (d.slider_x(0),
                             d.slider_x(d.SLIDER_POSITIONS - 1)), d.PART_MASS["slider"])  # fmt: skip
            self.box(link, "knob", (0.012, 0.020, 0.016), (0, 0, 0), d.KNOB_COLOR)
            self.box(link, "mark", (0.0024, 0.0202, 0.0162), (0, 0, 0), (0.95, 0.95, 0.95), collide=False)

    def _button(self, name: str, pos, radius: float, height: float, mat, mass: float) -> str:
        link = self.part(name, pos, "prismatic", "Z", (-d.travel(name), 0.0), mass)
        self.cylinder(link, "cap", radius, height, (0, 0, 0), mat)
        return link

    def _buttons(self, c) -> None:
        for color, (x, y) in d.button_xy().items():
            p = c + (x, y, 0)
            self.cylinder(CASE, f"button_{color}_rim", 0.016, 0.003, p + (0, 0, 0.0015), d.FRAME_COLOR, collide=False)
            self._button(f"button_{color}", p + (0, 0, 0.009), 0.013, 0.010, self.color(d.BUTTON_COLORS[color], 0.4),
                         d.PART_MASS["button"])  # fmt: skip

    def _dial(self, c) -> None:
        self.cylinder(CASE, "dial_base", 0.024, 0.002, c + (0, 0, 0.001), (0.15, 0.15, 0.15), collide=False)
        link = self.part("dial", c + (0, 0, 0.002), "revolute", "Z", None, d.PART_MASS["dial"])
        self.cylinder(link, "disk", 0.02, 0.006, (0, 0, 0.003), d.KNOB_COLOR)
        self.box(link, "bar", (0.042, 0.012, 0.018), (0, 0, 0.015), d.KNOB_COLOR)
        self.box(link, "pointer", (0.014, 0.003, 0.0004), (0.012, 0, 0.0242), (1, 1, 1), collide=False)

    def _big_button(self, c) -> None:
        cfg = self.cfg
        self.cylinder(CASE, "big_button_rim", 0.034, 0.004, c + (0, 0, 0.002), d.FRAME_COLOR, collide=False)
        color = self.color(d.BIG_BUTTON_COLORS[cfg.big_button_color], 0.4)
        link = self._button("big_button", c + (0, 0, 0.011), 0.028, 0.014, color, d.PART_MASS["big_button"])
        self.quad(link, "label", (0, 0, 0.007 + d.LABEL_T), (0, -1, 0), (1, 0, 0), 0.036, 0.036,
                  self.texture("big_label"))  # fmt: skip

    def _keypad(self, c) -> None:
        self.box(CASE, "display_bezel", (0.036, 0.094, 0.006), c + (0.032, 0, 0.003), (0.05, 0.05, 0.05), collide=False)
        self.quad(CASE, "display", c + (0.032, 0, 0.006 + d.LABEL_T), (0, -1, 0), (1, 0, 0), 0.084, 0.0252,
                  self.texture("display", emissive=True))  # fmt: skip
        for i, (x, y) in enumerate(d.key_xy()):
            link = self.part(f"key_{i}", c + (x, y, 0.009), "prismatic", "Z", (-d.BUTTON_TRAVEL, 0.0),
                             d.PART_MASS["key"])  # fmt: skip
            self.box(link, "cap", (0.023, 0.023, 0.010), (0, 0, 0), d.KEY_COLOR)
            self.quad(link, "label", (0, 0, 0.005 + d.LABEL_T), (0, -1, 0), (1, 0, 0), 0.018, 0.018,
                      self.texture(f"greek_{self.cfg.letters[i]}"))  # fmt: skip


def build_stage(cfg: d.BusyboxConfig, stage: Usd.Stage) -> None:
    """Author the busybox for ``cfg`` into ``stage`` (textures are referenced, not written)."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, ROOT)
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    UsdGeom.Scope.Define(stage, LOOKS)
    UsdGeom.Scope.Define(stage, f"{ROOT}/joints")
    _Builder(stage, cfg).build()


def build(cfg: d.BusyboxConfig, out_dir: str | Path) -> Path:
    """Write ``busybox.usda`` and its ``textures/`` for ``cfg`` into ``out_dir``; returns the USD path."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_textures(cfg, out_dir / "textures")
    path = out_dir / "busybox.usda"
    stage = Usd.Stage.CreateNew(str(path)) if not path.exists() else Usd.Stage.Open(str(path))
    stage.GetRootLayer().Clear()
    build_stage(cfg, stage)
    stage.GetRootLayer().Save()
    return path


def default_cache_dir() -> Path:
    import os

    return Path(os.environ.get("BUSYBOX_ASSET_DIR", Path.home() / ".cache" / "busybox"))


def cached_build(cfg: d.BusyboxConfig, cache_dir: str | Path | None = None) -> Path:
    """The USD for ``cfg``, built once per configuration and :data:`BUILD_VERSION` under ``cache_dir``."""
    out = Path(cache_dir or default_cache_dir()) / f"v{BUILD_VERSION}-{cfg.key()}"
    path = out / "busybox.usda"
    if not (out / "done").exists():
        build(cfg, out)
        (out / "done").touch()
    return path
