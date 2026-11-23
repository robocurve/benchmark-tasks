"""Test the same native fixture in standalone CPU PhysX. Independent of Kit, RTX and Dream."""

import json, math, tempfile, argparse, warnings
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from ovphysx import PhysX
from ovphysx.types import TensorType as T
from ovphysx.samples.python_samples.tensor_bindings import attach_scene
from pxr import Gf, Usd, Sdf, UsdGeom as G, UsdPhysics as P
from yam_arena import YAM_USD, station
from yam_arena.constants import READY, FINGER_OPEN, SHOULDER, WRIST, GRIPPER
from inspect_robots.guard import ClampGuard, DeltaLimitGuard
from inspect_robots.types import Action
from yam_arena.scripted import YamScriptedPolicy
from isaac.readout import Readout, ORIGIN, BODIES, LINKS
from isaac.camera import configure, floor_stand
from isaac.build import build
from task.policy import Push
from task.run import lift_moment
from task.station import make_model
from task.scoring import score
from tools.suite import CASES

SIDES = ("left", "right")


def attr(prim, name, typ, v):
    prim.CreateAttribute(name, typ).Set(v)


def contact_reporting(p):
    p.AddAppliedSchema("PhysxContactReportAPI")
    attr(p, "physxContactReport:threshold", Sdf.ValueTypeNames.Float, 0.0)


def make_scene(path, fixture):
    s = Usd.Stage.CreateInMemory()
    G.SetStageMetersPerUnit(s, 1)
    G.SetStageUpAxis(s, "Z")
    scene = P.Scene.Define(s, "/scene")
    scene.CreateGravityDirectionAttr((0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)

    def ref(name, usd, pos):
        a = G.Xform.Define(s, "/" + name)
        a.GetPrim().GetReferences().AddReference(str(usd))
        a.AddTranslateOp().Set(pos)

    ref("Bedside", fixture, tuple(ORIGIN))

    def fixed(name, kind, size, pos, q=(0, 0, 0, 1)):
        if kind == "box":
            g = G.Cube.Define(s, "/" + name)
            g.CreateSizeAttr(1)
        else:
            g = G.Capsule.Define(s, "/" + name)
            g.CreateAxisAttr("Z")
            g.CreateRadiusAttr(size[0])
            g.CreateHeightAttr(size[1])
        g.AddTranslateOp().Set(pos)
        g.AddOrientOp().Set(Gf.Quatf(q[3], Gf.Vec3f(*q[:3])))
        if kind == "box":
            g.AddScaleOp().Set(size)
        P.CollisionAPI.Apply(g.GetPrim())

    fixed("floor", "box", (5, 5, 0.02), (0, 0, station.FLOOR_Z - 0.01))
    for n, part in floor_stand(
        configure(station.rig()).top_cam, station.camera_stand
    ).items():
        fixed(n, *part)
    arms = {}
    shapes = []
    for side in SIDES:
        ref(side, YAM_USD, station.BASES[side])
        for prim in s.Traverse():
            if str(prim.GetPath()).startswith("/" + side + "/") and prim.HasAPI(
                P.RigidBodyAPI
            ):
                arms[side, prim.GetName()] = str(prim.GetPath())
                prim.AddAppliedSchema("PhysxRigidBodyAPI")
                attr(
                    prim, "physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool, True
                )
                contact_reporting(prim)
        ap = s.GetPrimAtPath("/" + side)
        ap.AddAppliedSchema("PhysxArticulationAPI")
        for prop, val in [
            ("solverPositionIterationCount", 16),
            ("solverVelocityIterationCount", 4),
        ]:
            attr(ap, "physxArticulation:" + prop, Sdf.ValueTypeNames.Int, val)
        for i in range(1, 9):
            prim = s.GetPrimAtPath(f"/{side}/Physics/joint{i}")
            angular = i < 7
            drive = P.DriveAPI.Get(prim, "angular" if angular else "linear")
            gains = SHOULDER if i <= 3 else WRIST if angular else GRIPPER
            scale = math.pi / 180 if angular else 1
            drive.CreateStiffnessAttr(gains["stiffness"] * scale)
            drive.CreateDampingAttr(gains["damping"] * scale)
            drive.CreateMaxForceAttr(gains["effort"])
    for p in s.Traverse():
        if str(p.GetPath()).startswith("/Bedside/") and p.HasAPI(P.CollisionAPI):
            shapes.append((str(p.GetPath()), p.GetParent().GetName(), p.GetName()))
            contact_reporting(p)
    s.GetRootLayer().Export(str(path))
    return arms, shapes


class ContactMatrix:
    def __init__(self, eng, sensors, filters):
        self.rows = sensors
        self.cols = filters
        self.capacity = 4096
        self.b = eng.create_contact_binding(
            sensor_patterns=[p[0] for p in sensors],
            filter_patterns=[p[0] for p in filters] * len(sensors),
            filters_per_sensor=len(filters),
            max_contact_data_count=self.capacity,
        )
        if (self.b.sensor_count, self.b.filter_count) != (len(sensors), len(filters)):
            raise RuntimeError("Missing contact channels")
        self.data = [np.zeros((self.capacity, n), np.float32) for n in (1, 3, 3, 1)]
        self.count = np.zeros((len(sensors), len(filters)), np.int32)
        self.start = self.count.copy()

    def read(self):
        n = self.b.read_normal_contact_data(*self.data, self.count, self.start)
        if n > self.capacity:
            raise RuntimeError("Contact buffer overflow")
        rows = []
        for i, j in zip(*np.nonzero(self.count)):
            _, a, sa = self.rows[i]
            _, b, sb = self.cols[j]
            if a == b:
                continue
            k = self.start[i, j]
            m = self.count[i, j]
            f = float(np.abs(self.data[0][k : k + m]).sum())
            if f > 1e-6:
                rows.append((a, b, sa, sb, f))
        return rows


def simulate(name, out):
    mode, options, target = CASES[name]
    if name == "fixed_late_lift":
        target = 1.0
    PhysX.set_cpu_mode(True)
    eng = PhysX()
    cache = []

    def tensor(p, kind):
        b = eng.create_tensor_binding(pattern=p, tensor_type=kind)
        if b.count != 1:
            raise ValueError("unresolved " + p)
        cache.append(b)
        return b, np.zeros(b.shape, np.float32)

    def put(pair, x):
        b, a = pair
        a.ravel()[:] = x
        b.write(a)

    def get(pair):
        b, a = pair
        b.read(a)
        return a.ravel().copy()

    with tempfile.TemporaryDirectory(dir=out) as tmp:
        p = Path(tmp)
        import hashlib

        geo = {k: v for k, v in options.items() if k != "delay"}
        key = hashlib.sha256(json.dumps(geo, sort_keys=True).encode()).hexdigest()[:12]
        fixture = (
            Path(__file__).resolve().parent / "assets" / ("fixture-" + key + ".usda")
        )
        arms, shapes = make_scene(p / "test.usda", fixture)
        stage = attach_scene(eng, p / "test.usda", "bedside")
        poses = {n: tensor("/Bedside/" + n, T.RIGID_BODY_POSE) for n in BODIES}
        poses.update(
            {
                s + "/" + n: tensor(arms[s, n], T.RIGID_BODY_POSE)
                for s in SIDES
                for n in LINKS
            }
        )
        drives = {}
        for s in SIDES:
            drives[s] = tensor("/" + s, T.ARTICULATION_DOF_POSITION_TARGET)
            put(
                tensor("/" + s, T.ARTICULATION_DOF_POSITION),
                [*READY, FINGER_OPEN, FINGER_OPEN],
            )
            put(drives[s], [*READY, FINGER_OPEN, FINGER_OPEN])
        sensors = [s for s in shapes if s[2] == "heel"]
        filters = [
            (k, k, "")
            for k in [
                "/Bedside/frame",
                "/Bedside/cushion",
                *("/Bedside/foam" + str(i) for i in range(6)),
                *arms.values(),
            ]
        ]
        bank = ContactMatrix(eng, sensors, filters)
        wrench = {
            n: tensor("/Bedside/" + n, T.RIGID_BODY_WRENCH)
            for n in ("upper_leg", "shin")
        }
        twist = {n: tensor("/Bedside/" + n, T.RIGID_BODY_VELOCITY) for n in wrench}
        read = Readout(options)
        ctrl = Push(make_model(), mode)
        space = YamScriptedPolicy().info.action_space
        clamp = ClampGuard(space)
        limit = DeltaLimitGuard(space)
        guard = {}
        rows = []
        heel = 0.0
        torque = 0.0
        rate = 0.0
        commands = []
        native_frames = []

        def capture():
            return dict(
                poses={n: get(v).tolist() for n, v in poses.items()},
                heel_n=heel / 12,
                assist_nm=torque,
                knee_rate=float(rate),
            )

        for k in range(-120, 3360):
            if k == 0:
                initial = capture()
                _, obs = read.sample(initial)
                heel = 0.0
            if k >= 0 and k % 12 == 0:
                q = ctrl.command(k // 12, obs)
                action = Action(data=np.r_[READY, 1.0, q[:6], q[6] / FINGER_OPEN])
                for g in (clamp, limit):
                    action = g.review(action, guard)
                commands.append(np.asarray(action.data, dtype=float).tolist())
                for i, s in enumerate(SIDES):
                    a = action.data[i * 7 : i * 7 + 7]
                    put(drives[s], [*a[:6], a[6] * FINGER_OPEN, a[6] * FINGER_OPEN])
            if k >= 0:
                a, b = (get(poses[n]) for n in ("upper_leg", "shin"))
                ang = (R.from_quat(a[3:]).inv() * R.from_quat(b[3:])).as_rotvec()[0]
                rate = get(twist["shin"])[3] - get(twist["upper_leg"])[3]
                torque = lift_moment(ang, rate, k / 240, options.get("delay", 0.0))
                if torque:
                    for n, sign in (("upper_leg", -1), ("shin", 1)):
                        put(wrench[n], [0, 0, 0, sign * torque, 0, 0, 0, 0, 0])
            eng.step_sync(1 / 240)
            if k >= 0:
                heel += sum(r[-1] for r in bank.read())
                if (k + 1) % 12 == 0:
                    raw = capture()
                    native_frames.append(raw)
                    w, obs = read.sample(raw)
                    rows.append(
                        dict(tick=k // 12, witness=w, heel_bed_mm=obs["heel_gap_mm"])
                    )
                    heel = 0.0
        try:
            value, reason = score(rows)
        except ValueError as e:
            value, reason = None, str(e)
        import hashlib

        import gzip

        trace = out / (name + ".jsonl.gz")
        with gzip.open(trace, "wt", compresslevel=4) as f:
            f.write(json.dumps(dict(initial=initial, settings=options)) + "\n")
            for i, (raw, action) in enumerate(
                zip(native_frames, commands, strict=True)
            ):
                f.write(
                    json.dumps(
                        dict(tick=i, raw=raw, action=action),
                        allow_nan=False,
                        separators=(",", ":"),
                    )
                    + "\n"
                )
        record = dict(
            trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
            fixture_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),
            terminal_closed=abs(commands[-1][-1]) < 0.02,
            bed_touch_after_10_5s=any(r["witness"]["bed_touch"] for r in rows[210:240]),
            idle_height_mm=[r["heel_bed_mm"] for r in rows] if name == "idle" else None,
            mode=name,
            engine="standalone CPU PhysX (gravity compensation approximation)",
            score=value,
            reason=reason,
            expected=target,
            matched=value == target,
            initial=rows[0],
            final=rows[-1],
            actions_sha256=hashlib.sha256(
                np.array(commands, dtype="<f8").tobytes()
            ).hexdigest(),
        )
        (out / (name + ".json")).write_text(json.dumps(record, indent=2) + "\n")
        print(name, value, reason, flush=True)
        bank.b.destroy()
        for b in cache:
            b.destroy()
        eng.wait_task(eng.detach_ovstage())
        stage.destroy()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=CASES)
    ap.add_argument("out", type=Path)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    simulate(args.mode, args.out.resolve())
