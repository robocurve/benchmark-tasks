"""Check fitted STEP surfaces against recorded native YAM link poses at 20 Hz."""
import argparse
import gzip
from hashlib import sha256
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import cadquery
import fcl
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

LINKS = ("base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ANCHORS = {"carriage": (360, 0, 0), "brake": (360, -65, 45), "tap": (345, -85, -50), "mast": (0, 0, 0)}
BODY = {"rack": "mast", "brake_tab": "brake", "locking_yoke": "brake", "tap_crank": "tap", "tap_rotor": "tap"}


class Surface:
    def __init__(self, points, triangles):
        self.model = fcl.BVHModel()
        self.model.beginModel(len(points), len(triangles))
        self.model.addSubModel(np.ascontiguousarray(points), np.ascontiguousarray(triangles, dtype=np.int32))
        self.model.endModel()
        self.object = fcl.CollisionObject(self.model)
        low, high = np.min(points, axis=0), np.max(points, axis=0)
        self.half = (high-low)*0.5
        self.center = (high+low)*0.5

    def pose(self, pose):
        rotation = Rotation.from_quat(pose[3:]).as_matrix()
        translation = np.asarray(pose[:3])*1000
        self.object.setTransform(fcl.Transform(rotation, translation))
        half = abs(rotation) @ self.half
        center = rotation @ self.center + translation
        self.low, self.high = center-half, center+half


def load_robot(urdf):
    model = ET.parse(urdf)
    surfaces = {}
    for name in LINKS:
        points = []
        faces = []
        offset = 0
        for shape in model.findall(f".//link[@name='{name}']/collision"):
            mesh = shape.find("geometry/mesh")
            data = trimesh.load_mesh(urdf.parent/mesh.get("filename"), process=False)
            pose = shape.find("origin")
            rot = Rotation.from_euler("xyz", np.fromstring(pose.get("rpy"), sep=" "))
            xyz = np.fromstring(pose.get("xyz"), sep=" ")
            scale = np.fromstring(mesh.get("scale","1 1 1"), sep=" ")
            points.append(1000 * (rot.apply(data.vertices*scale) + xyz))
            faces.append(data.faces+offset)
            offset += len(data.vertices)
        surfaces[name] = Surface(np.concatenate(points), np.concatenate(faces))
    return surfaces


def scan(cad, urdf, poses):
    robot = load_robot(urdf)
    parts = {}
    hashes = {}
    for file in sorted(cad.glob("*.step")):
        obj = cadquery.importers.importStep(str(file)).val()
        vs, fs = obj.tessellate(0.05, 0.12)
        parts[file.stem] = Surface(np.asarray([v.toTuple() for v in vs])-ANCHORS[BODY.get(file.stem, "carriage")],
                                   np.array(fs))
        hashes[file.name] = sha256(file.read_bytes()).hexdigest()
    collisions = {}
    request = fcl.CollisionRequest()
    raw_digest = sha256()
    stream = gzip.open(poses, "rb") if poses.suffix == ".gz" else poses.open("rb")
    with stream:
        return scan_frames(stream, robot, parts, hashes, urdf, raw_digest, collisions, request)


def scan_frames(stream, robot, parts, hashes, urdf, raw_digest, collisions, request):
    count = 0
    for row in stream:
        raw_digest.update(row)
        tick, fixture, arms, _ = json.loads(row)
        count += 1
        for name,p in parts.items():
            p.pose(fixture[BODY.get(name, "carriage")])
        for side, values in arms.items():
            for link, pose in zip(LINKS, values, strict=True):
                r = robot[link]
                r.pose(pose)
                for name, p in parts.items():
                    if np.any(r.low > p.high) or np.any(p.low > r.high):
                        continue
                    if fcl.collide(r.object, p.object, request, fcl.CollisionResult()):
                        key=f"{side}/{link}/{name}"
                        rec=collisions.setdefault(key, {"first_tick":tick,"last_tick":tick,"sample_count":0})
                        rec["sample_count"]+=1
                        rec["last_tick"]=tick
    allowed = {(side,link,part) for side,part in (("left","lift_neck"),("right","brake_tab"),("right","tap_rotor"))
               for link in ("tip_left","tip_right")}
    unexpected=[k for k in collisions if tuple(k.split("/")) not in allowed]
    return {"unexpected_robot_step_pairs":unexpected,"method":"20 Hz mesh-surface overlap screen on recorded poses; 0.05 mm linear and 0.12 rad angular CAD tessellation. Inter-sample motion, hoses, fasteners and physical compliance are unobserved.",
            "urdf_sha256":sha256(urdf.read_bytes()).hexdigest(),"poses_sha256":raw_digest.hexdigest(),
            "step_sha256":hashes,"frames":count,"contact_pairs":collisions}

if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("cad", type=Path)
    p.add_argument("urdf", type=Path)
    p.add_argument("poses", type=Path)
    p.add_argument("report", type=Path)
    args=p.parse_args()
    result = scan(args.cad,args.urdf,args.poses)
    args.report.write_text(json.dumps(result,indent=2)+"\n")
    raise SystemExit(1 if result["unexpected_robot_step_pairs"] else 0)
