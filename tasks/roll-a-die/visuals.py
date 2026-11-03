"""Procedural die markings and a fixed recording camera; no extra physics.

The simulator's single 16 mm Box remains the only die collision shape. These
meshes are paint-like, non-colliding children that move with that rigid body.
USD/Isaac imports are delayed until after the simulator has launched.
"""

from __future__ import annotations

import math


FACE_MARKINGS = (
    {"axis": "+Z", "normal": (0, 0, 1), "pips": 1, "colour": "white", "srgb": (0.95, 0.95, 0.92)},
    {"axis": "-Z", "normal": (0, 0, -1), "pips": 6, "colour": "yellow", "srgb": (0.98, 0.82, 0.17)},
    {"axis": "+X", "normal": (1, 0, 0), "pips": 2, "colour": "red", "srgb": (0.88, 0.24, 0.20)},
    {"axis": "-X", "normal": (-1, 0, 0), "pips": 5, "colour": "cyan", "srgb": (0.15, 0.78, 0.88)},
    {"axis": "+Y", "normal": (0, 1, 0), "pips": 3, "colour": "green", "srgb": (0.27, 0.77, 0.32)},
    {"axis": "-Y", "normal": (0, -1, 0), "pips": 4, "colour": "purple", "srgb": (0.72, 0.43, 0.86)},
)
DIE_EDGE_M = 0.016
PAINT_OFFSET_M = 0.000004
PIP_RADIUS_M = 0.00115
REVIEW_CAMERA = {
    "name": "review_cam", "width": 1600, "height": 1280,
    "eye_m": (0.84, -0.25, 1.55), "target_m": (0.25, 0.17, 0.90),
    "focal_length_mm": 40.0, "horizontal_aperture_mm": 20.955,
    "policy_input": False,
}


def pip_centres(count):
    """Standard pip layouts, coordinates in the face's own unit square."""
    corners = ((-1, -1), (1, -1), (-1, 1), (1, 1))
    return {
        1: ((0, 0),),
        2: ((-1, -1), (1, 1)),
        3: ((-1, -1), (0, 0), (1, 1)),
        4: corners,
        5: (*corners, (0, 0)),
        6: (*corners, (-1, 0), (1, 0)),
    }[count]


def add_review_camera(embodiment):
    """Extend a newly built embodiment, before ``yam_arena.scene.make_env``.

    The three calibrated policy cameras stay unchanged. The runner records
    ``base.scene['review_cam']`` separately, never exposing it to Dream.
    """
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import CameraCfg
    from isaaclab_arena.utils.cameras import ArenaCameraCfg
    from isaaclab_arena.utils.configclass import make_configclass
    from pxr import Gf
    from yam_arena.embodiment import RENDERER

    c = REVIEW_CAMERA
    rotation = Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(*c["eye_m"]), Gf.Vec3d(*c["target_m"]), Gf.Vec3d(0, 0, 1),
    ).GetInverse().ExtractRotationQuat()
    q = (*rotation.GetImaginary(), rotation.GetReal())
    config = CameraCfg(
        prim_path="{ENV_REGEX_NS}/DieReviewCam",
        height=c["height"], width=c["width"], data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=c["focal_length_mm"],
            horizontal_aperture=c["horizontal_aperture_mm"],
            clipping_range=(0.01, 20.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=c["eye_m"], rot=q, convention="opengl"),
        renderer_cfg=RENDERER,
    )
    # The default config is generated using the same helper. Rebuild its fields
    # so Arena's config traversal includes this sensor as an actual field.
    fields = [(name, CameraCfg, value) for name, value in vars(embodiment.camera_config).items()
              if isinstance(value, CameraCfg) and name != c["name"]]
    fields.append((c["name"], CameraCfg, config))
    embodiment.camera_config = make_configclass(
        "DieReviewCameraCfg", fields, bases=(ArenaCameraCfg,),
    )()


def install_scene_visuals(base):
    """Add markings once after environment construction, before trial reset.

    Returns a serializable description for the run provenance. A failure to
    locate the die is fatal: unmarked runs must never masquerade as marked runs.
    """
    import omni.usd
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    roots = [p for p in stage.Traverse() if p.GetName() == "die" and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(roots) != 1:
        raise RuntimeError(f"expected one rigid-body die prim, found {len(roots)}")
    die_path = str(roots[0].GetPath())
    paint_path = die_path + "/DiePaint"
    if stage.GetPrimAtPath(paint_path).IsValid():
        return visual_metadata(die_path)
    UsdGeom.Xform.Define(stage, paint_path)
    material_root = "/World/DieReviewMaterials"
    UsdGeom.Scope.Define(stage, material_root)

    def material(name, srgb):
        path = f"{material_root}/{name}"
        mat = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        linear = tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in srgb)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat

    black = material("black", (0.025, 0.025, 0.025))
    white = material("ruler_white", (0.93, 0.93, 0.93))
    red = material("reference_red", (0.9, 0.08, 0.07))

    def mesh(path, vertices, mat):
        item = UsdGeom.Mesh.Define(stage, path)
        item.CreatePointsAttr([Gf.Vec3f(*p) for p in vertices])
        item.CreateFaceVertexCountsAttr([len(vertices)])
        item.CreateFaceVertexIndicesAttr(list(range(len(vertices))))
        item.CreateSubdivisionSchemeAttr("none")
        item.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(item.GetPrim()).Bind(mat)
        # Deliberately no CollisionAPI, RigidBodyAPI or mass/inertia attributes.
        return item

    for face in FACE_MARKINGS:
        n = face["normal"]
        a = next(i for i, v in enumerate(n) if v)
        uv = [i for i in range(3) if i != a]
        def point(u, v, offset=0.0):
            p = [0.0, 0.0, 0.0]
            p[a] = n[a] * (DIE_EDGE_M / 2 + PAINT_OFFSET_M + offset)
            p[uv[0]], p[uv[1]] = u, v
            return p
        root = f"{paint_path}/Face{face['pips']}"
        half = 0.0078  # narrow white edge keeps adjacent face boundaries clear
        mesh(root, [point(u, v) for u, v in ((-half, -half), (half, -half), (half, half), (-half, half))],
             material(face["colour"], face["srgb"]))
        for j, (u, v) in enumerate(pip_centres(face["pips"])):
            vertices = [point(u * 0.004 + PIP_RADIUS_M * math.cos(t * math.tau / 24),
                              v * 0.004 + PIP_RADIUS_M * math.sin(t * math.tau / 24),
                              PAINT_OFFSET_M) for t in range(24)]
            mesh(f"{paint_path}/Pips{face['pips']}_{j}", vertices, black)

    # Render-only calibration graphics, outside the normal pickup/release area.
    # The tabletop scale has 5 mm ticks and 1 cm major ticks. The upright guide
    # shows 0–20 cm, with a red 3 cm reference. They are visual annotations, not
    # physical obstacles; the run metadata records this simplification.
    from yam_arena.task_file import TABLE_TOP
    top = TABLE_TOP["station"]
    ruler_path = "/World/DieReviewRulers"
    UsdGeom.Xform.Define(stage, ruler_path)

    def horizontal_rect(name, x0, x1, y0, y1, mat):
        mesh(f"{ruler_path}/{name}", [(x0, y0, top + .00002), (x1, y0, top + .00002),
                                      (x1, y1, top + .00002), (x0, y1, top + .00002)], mat)

    horizontal_rect("TableScale", .14, .42, .275, .292, white)
    for i in range(57):
        x = .14 + .005 * i
        length = .012 if i % 2 == 0 else .007
        # Separate surfaces slightly to prevent depth flicker without collision.
        mesh(f"{ruler_path}/TableTick{i}", [(x-.00025,.276,top+.00004), (x+.00025,.276,top+.00004),
                                          (x+.00025,.276+length,top+.00004), (x-.00025,.276+length,top+.00004)], black)

    def vertical_rect(name, x0, x1, z0, z1, mat, y=.295):
        mesh(f"{ruler_path}/{name}", [(x0,y,z0), (x1,y,z0), (x1,y,z1), (x0,y,z1)], mat)

    vertical_rect("HeightScale", .102, .138, top, top+.207, white)
    for i in range(41):
        z = top + .005 * i
        width = .013 if i % 2 == 0 else .007
        vertical_rect(f"HeightTick{i}", .120, .120+width, z-.0003, z+.0003,
                      red if i == 6 else black, y=.2949)
    vertical_rect("ThirtyMillimetres", .120, .157, top+.0295, top+.0305, red, y=.2948)

    # Plain seven-segment numbers give the height guide a readable scale without
    # requiring a font, texture, external file or renderer-specific text API.
    segments = {
        "a": ((0, 1), (1, 1)), "b": ((1, 1), (1, .5)),
        "c": ((1, .5), (1, 0)), "d": ((1, 0), (0, 0)),
        "e": ((0, 0), (0, .5)), "f": ((0, .5), (0, 1)),
        "g": ((0, .5), (1, .5)),
    }
    digits = {"0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg"}

    def glyph_stroke(name, start, end, x, z, mat):
        a = (x + start[0] * .0045, z + start[1] * .008)
        b = (x + end[0] * .0045, z + end[1] * .008)
        dx, dz = b[0] - a[0], b[1] - a[1]
        norm = math.hypot(dx, dz)
        ox, oz = -dz / norm * .0003, dx / norm * .0003
        mesh(f"{ruler_path}/{name}",
             [(a[0]+ox,.2947,a[1]+oz), (b[0]+ox,.2947,b[1]+oz),
              (b[0]-ox,.2947,b[1]-oz), (a[0]-ox,.2947,a[1]-oz)], mat)

    for label, height in (("0", .006), ("3", .03), ("10", .10), ("20", .20)):
        for j, char in enumerate(label):
            for segment in digits[char]:
                glyph_stroke(f"Label{label}_{j}_{segment}", *segments[segment],
                             .105 + .0065 * j, top + height - .004,
                             red if label == "3" else black)
    # Lower-case 'cm' just below the 20 cm label.
    for j, (start, end) in enumerate((((1,1),(0,1)), ((0,1),(0,0)), ((0,0),(1,0)))):
        glyph_stroke(f"UnitC{j}", start, end, .105, top+.181, black)
    for j, (start, end) in enumerate((((0,0),(0,1)), ((0,1),(.5,.5)),
                                     ((.5,.5),(1,1)), ((1,1),(1,0)))):
        glyph_stroke(f"UnitM{j}", start, end, .1115, top+.181, black)

    added = list(Usd.PrimRange(stage.GetPrimAtPath(paint_path))) + list(Usd.PrimRange(stage.GetPrimAtPath(ruler_path)))
    if any(p.HasAPI(UsdPhysics.CollisionAPI) or p.HasAPI(UsdPhysics.RigidBodyAPI) for p in added):
        raise RuntimeError("review markings unexpectedly acquired physics properties")
    return visual_metadata(die_path)


def visual_metadata(die_path=None):
    return {
        "version": "die-markings-v1", "die_prim": die_path,
        "collision_edge_m": DIE_EDGE_M, "die_mass_kg": 0.005,
        "face_markings": FACE_MARKINGS, "pip_radius_m": PIP_RADIUS_M,
        "maximum_visual_surface_offset_m": 2 * PAINT_OFFSET_M,
        "review_camera": REVIEW_CAMERA,
        "references": {
            "physical": False, "tick_spacing_m": 0.005,
            "major_tick_spacing_m": 0.01, "red_height_above_table_m": 0.03,
            "note": "Render-only rulers; perspective is not an exact height measurement away from the ruler plane.",
        },
    }
