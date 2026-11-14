#!/usr/bin/env python3
"""CPU-only sampled IK and scene-clearance check. Does not prove Isaac physics."""
import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np

def aabb_gap(a_lo, a_hi, b_lo, b_hi):
    return float(np.max(np.maximum(b_lo-a_hi, a_lo-b_hi)))

def run(mode):
    from yam_arena import analysis, ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    from yam_arena.constants import READY
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import Box, Cylinder
    from yam_arena.urdf import quat_xyzw_to_matrix
    import task
    plans = task.demo_plan.plans(mode)
    arms = {a: ArmPlan(p, station.BASES[a]) for a, p in plans.items()}
    duration = max(p.duration for p in arms.values())
    objects = []
    for obj in task.OBJECTS:
        if getattr(obj,"boxes",()):
            for local,size,_ in obj.boxes:
                objects.append((obj.name,np.asarray(obj.pos),np.asarray(local),np.asarray(size)))
            for local,radius,height,_,axis in obj.cylinders:
                if axis[-1]=="Z":size=(radius*2,radius*2,height)
                elif axis[-1]=="X":size=(height,radius*2,radius*2)
                else:size=(radius*2,height,radius*2)
                objects.append((obj.name,np.asarray(obj.pos),np.asarray(local),np.asarray(size)))
        elif isinstance(obj, Box):
            objects.append((obj.name,np.asarray(obj.pos),np.zeros(3),np.asarray(obj.size)))
        elif isinstance(obj, Cylinder):
            objects.append((obj.name,np.asarray(obj.pos),np.zeros(3),
                            np.asarray((obj.radius*2,obj.radius*2,obj.height))))
    room = [("table", np.array((.3, 0., task.g.TOP-.03)), np.array((1.1, 1.3, .06)))]
    for name, (kind, size, pos, quat) in station.camera_stand(station.top_camera()).items():
        if kind == "capsule": size = (2*size[0], 2*size[0], size[1]+2*size[0])
        rotated_size=2*np.abs(quat_xyzw_to_matrix(np.asarray(quat)))@(np.asarray(size)/2)
        room.append((name, np.asarray(pos), rotated_size))
    failures = {}
    stats = {"max_position_error_mm": 0., "max_rotation_error_deg": 0., "min_joint_margin_rad": 9.,
             "min_arm_clearance_mm": 1e6}
    release_times={}
    for points in plans.values():
        elapsed=0.
        for point in points:
            elapsed+=point.duration
            if point.label == "release clamp":
                release_times["motherboard"]=elapsed
    # Every waypoint endpoint and the midpoint of every transition. The
    # trajectory between samples is not certified by this offline gate.
    times={0.,round(duration,4)}
    for pts in plans.values():
        endpoints=np.r_[0.,np.cumsum([p.duration for p in pts])]
        times.update(np.round(endpoints,4))
        times.update(np.round((endpoints[:-1]+endpoints[1:])/2,4))
    for t in sorted(times):
        rows = {}
        for arm, plan in arms.items():
            p, rot, finger, label = plan.target(float(t))
            q_arm, _, _ = ik.solve(p, ik.matrix_to_quat_xyzw(rot), READY,
                                   base_pos=station.BASES[arm], iters=60)
            actual, actual_rot = ik.tcp_pose(q_arm, station.BASES[arm])
            err = np.linalg.norm(p-actual)*1000
            re = math.degrees(math.acos(np.clip((np.trace(rot.T@actual_rot)-1)/2, -1, 1)))
            margin = float(min((q_arm-ik.LOWER).min(), (ik.UPPER-q_arm).min()))
            stats["max_position_error_mm"] = max(stats["max_position_error_mm"], float(err))
            stats["max_rotation_error_deg"] = max(stats["max_rotation_error_deg"], float(re))
            stats["min_joint_margin_rad"] = min(stats["min_joint_margin_rad"], margin)
            if err > 8 or re > 5 or margin < .01:
                failures.setdefault("reach:"+arm+":"+label, {"t": t, "mm": round(err, 2), "deg": round(re, 2), "margin": round(margin, 3)})
            q = dict(zip(ARM_JOINTS, q_arm)); q.update({n: finger for n in FINGER_JOINTS})
            rows[arm] = (label, analysis.world_hulls(analysis.fk_link_poses_world(q, station.BASES[arm])))
        current_objects=[(name,(np.asarray(task.g.GOALS[name]) if name in release_times and t>=release_times[name]
                          else pos)+local, size) for name,pos,local,size in objects]
        for arm, (label, hulls) in rows.items():
            for link, hull in hulls:
                lo, hi = hull.vertices.min(0), hull.vertices.max(0)
                for name, pos, size in room+current_objects:
                    if name == "table" and link == "base": continue
                    # Finger tips must contact the moving board's edge clamps
                    # during grasp, carry, and release. Other contacts remain checked.
                    if (name == "motherboard" and link.startswith("tip_")
                            and label in ("grip edge clamp", "board lift", "board carry",
                                          "align above standoffs", "lower onto standoffs",
                                          "hold seated board", "release clamp")):
                        continue
                    gap = aabb_gap(lo, hi, pos-size/2, pos+size/2)
                    if gap < -.002:
                        failures.setdefault("collision:"+arm+":"+link+":"+name, {"t": t, "label": label, "overlap_mm": round(-gap*1000, 2)})
        for lname, lh in rows["left"][1]:
            for rname, rh in rows["right"][1]:
                gap = aabb_gap(lh.vertices.min(0), lh.vertices.max(0), rh.vertices.min(0), rh.vertices.max(0))
                stats["min_arm_clearance_mm"] = min(stats["min_arm_clearance_mm"], gap*1000)
                if gap < .015:
                    failures.setdefault("arms:"+lname+":"+rname, {"t": t, "gap_mm": round(gap*1000, 2)})
    return {"mode": mode, "duration_s": round(duration, 2), "samples": len(times),
            "metrics": stats, "failures": failures, "passed": not failures}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yam-arena", required=True)
    parser.add_argument("--modes", nargs="+", default=["demo", "settle"])
    parser.add_argument("--report", default="local_checks.json")
    args = parser.parse_args()
    sys.path.insert(0, str(Path(args.yam_arena).resolve()))
    result = {"scope": "independent sampled CPU IK and AABB clearances, not Isaac physics or the demo controller",
              "results": [run(mode) for mode in args.modes]}
    Path(args.report).write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))
    return int(not all(r["passed"] for r in result["results"]))

if __name__ == "__main__": raise SystemExit(main())
