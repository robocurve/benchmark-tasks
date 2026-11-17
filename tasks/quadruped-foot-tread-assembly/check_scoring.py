"""Offline adversarial rubric checks; synthetic states are NOT physics evidence."""
import importlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

TASK = Path(__file__).resolve().parent
results = []


def check_task(name):
    for key in list(sys.modules):
        if key == "assembly" or key.startswith("assembly."):
            del sys.modules[key]
    sys.path.insert(0, str(TASK))
    g = importlib.import_module("assembly.geometry")
    score = importlib.import_module("assembly.scoring")
    paths = [g.ROOT + "/" + g.PREFIX + n for n in g.BODY_NAMES]
    n = len(paths)
    poses = np.zeros((n, 7)); poses[:, 6] = 1
    contacts = np.zeros((n, n + 2), int)
    robot = np.zeros((18, n), int)
    environment = np.zeros((18, 20), int)
    cup = name.startswith("suction")
    if cup:
        poses[:5, :3] = g.TARGET
        poses[5, :3] = g.CORE_TARGET
        contacts[0, n] = contacts[5, 0] = 1
    else:
        contacts[0, n] = 1
        poses[:5, :3] = g.ORIGIN
        poses[:, 3:] = Rotation.from_matrix(g.R).as_quat()
        for side, i in [("left", 5), ("right", 8)]:
            poses[i:i+3, :3] = g.CLAMP_TARGET[side]
            contacts[i, n] = 1
    geometry = g.contract(); geometry["physics_hz"] = 20

    def evaluate(label, modify=None, expected_full=False):
        p, r, c, e = [a.copy() for a in (poses, robot, contacts, environment)]
        if modify: modify(p, r, c, e)
        scorer = score.Outcome(paths, geometry, {})
        for _ in range(61): scorer.update(p, r, c, e)
        assert (scorer.value == 1) == expected_full, (name, label, scorer.detail)
        results.append(dict(task=name, case=label, score=scorer.value, passed=True))

    evaluate("complete released supported geometry", expected_full=True)
    evaluate("still held by a robot finger", lambda p,r,c,e: r.__setitem__((7, 0), 1))
    evaluate("unsupported pieces", lambda p,r,c,e: c.fill(0))
    evaluate("interarm contact", lambda p,r,c,e: e.__setitem__((7, 16), 1))
    evaluate("robot table contact", lambda p,r,c,e: e.__setitem__((7, -1), 1))
    evaluate("main part displaced 3mm", lambda p,r,c,e: p.__setitem__((slice(0,5), 0), p[:5, 0]+.003))
    if cup:
        evaluate('rib has not passed capture face',lambda p,r,c,e:p.__setitem__((5,1),p[5,1]+.0004))
        evaluate('head still expanded around retention rib',lambda p,r,c,e:p.__setitem__((1,0),p[1,0]+.0008))
        def tilted_seated(p,r,c,e):
            rot=Rotation.from_euler('x',2.8,degrees=True);origin=np.array(g.HOLE_ORIGIN)-rot.apply([0,.024,0])+[0,0,-.0003]
            p[:5,:3]=origin;p[:,3:]=rot.as_quat();p[5,:3]=origin+rot.apply([0,.0315,0])
        evaluate('axis centred but retaining head crosses plate rear face',tilted_seated)
        def legal_tilt(p,r,c,e):
            rot=Rotation.from_euler('x',1.,degrees=True);origin=np.array(g.HOLE_ORIGIN)-rot.apply([0,.0238,0])+[0,0,-.0003]
            p[:5,:3]=origin;p[:,3:]=rot.as_quat();p[5,:3]=origin+rot.apply([0,.0315,0])
        evaluate('captured tilted cup with complete bore clearance',legal_tilt,expected_full=True)
        evaluate("core absent", lambda p,r,c,e: p.__setitem__((5, 1), p[5, 1]+.02))
        evaluate("one head sector still compressed", lambda p,r,c,e: p.__setitem__((1, 0), p[1, 0]-.002))
        # Broken native joints invalidate simulation evidence in validate_run.py;
        # they are not a human measurement condition in the primary rubric.
        evaluate("core tilted", lambda p,r,c,e: p.__setitem__((5, slice(3,None)), Rotation.from_euler('x',10,degrees=True).as_quat()))
        evaluate('offset within old0.8mm bound but intersects neck',lambda p,r,c,e:p.__setitem__((5,0),p[5,0]+.0007))
        evaluate('tilt within old3deg bound but shaft crosses neck',lambda p,r,c,e:p.__setitem__((5,slice(3,None)),Rotation.from_euler('x',2.8,degrees=True).as_quat()))
    else:
        def support_on_pad(p,r,c,e):
            c[5,n]=c[8,n]=0;c[5,0]=c[8,0]=1
        evaluate('clamp support transmitted through seated cover',support_on_pad,expected_full=True)
        evaluate('cover lacks actual foot support',lambda p,r,c,e:c.__setitem__((0,n),0))
        evaluate("one clamp missing", lambda p,r,c,e: p.__setitem__((slice(5,8), 0), p[5:8, 0]+.02))
        evaluate("clamp end still spread", lambda p,r,c,e: p.__setitem__((6, 0), p[6, 0]+.0015))
        evaluate("one corner not wrapped", lambda p,r,c,e: p.__setitem__((1, 2), p[1, 2]+.004))
        def shifted_bar(p,r,c,e):
            p[5:8,:3] += g.R @ np.array([0.,.0011,0.])
        evaluate('bar within old pose tolerance but one nose outside ear',shifted_bar)
        def shifted_outward(p,r,c,e):
            p[5:8,:3] += g.R @ np.array([.0007,0.,0.])
        evaluate('recovered clip beyond inner retaining face',shifted_outward)
        def half_turn_pad(p,r,c,e):
            p[:5,3:]=Rotation.from_matrix(g.R@Rotation.from_euler('z',180,degrees=True).as_matrix()).as_quat()
        evaluate('equivalent half-turn cover orientation',half_turn_pad,expected_full=True)
        def swapped_bars(p,r,c,e):
            rotation=Rotation.from_matrix(g.R@Rotation.from_euler('z',180,degrees=True).as_matrix()).as_quat()
            p[5:8,:3]=g.CLAMP_TARGET['right'];p[8:11,:3]=g.CLAMP_TARGET['left'];p[5:,3:]=rotation
        evaluate('equivalent exchanged and half-turned bars',swapped_bars,expected_full=True)
    if cup:
        spin=score.Outcome(paths,geometry,{})
        for i in range(61):
            state=poses.copy();state[:,3:]=Rotation.from_euler('y',i*3,degrees=True).as_quat()
            spin.update(state,robot,contacts,environment)
        assert spin.value==1,'Axial spin of circular retained parts is not misalignment'
        results.append(dict(task=name,case='free axial rotation preserves functional seating',passed=True))
    # The installed fit may move without losing its geometric engagement.
    # This is a rubric counterexample, not a fabricated physical trajectory.
    flexible = score.Outcome(paths, geometry, {})
    for i in range(61):
        state = poses.copy()
        if cup:
            rr = Rotation.from_euler('x', .3*np.cos(i*np.pi), degrees=True)
            state[:5,:3] = np.asarray(g.TARGET)+[0,.0001,0]
            state[:,3:] = rr.as_quat()
            state[5,:3] = state[0,:3]+rr.apply([0,.0315,0])
        else:
            state[:5,:3] += g.R@np.array([0,.0003*np.cos(i*np.pi),0])
        flexible.update(state,robot,contacts,environment)
    assert flexible.value == 1, flexible.detail
    results.append(dict(task=name,case='motion within installed fit does not require microscopic stillness',passed=True))
    # A single unsupported sample resets the entire release/retention interval.
    interrupted = score.Outcome(paths, geometry, {})
    for _ in range(61): interrupted.update(poses,robot,contacts,environment)
    interrupted.update(poses,robot,np.zeros_like(contacts),environment)
    for _ in range(60): interrupted.update(poses,robot,contacts,environment)
    assert interrupted.value < 1
    interrupted.update(poses,robot,contacts,environment)
    assert interrupted.value == 1
    results.append(dict(task=name,case='support interruption restarts full three-second interval',passed=True))
    scorer = score.Outcome(paths, geometry, {})
    for _ in range(60): scorer.update(poses,robot,contacts,environment)
    assert scorer.value < 1, "60 samples at20Hz cover only2.95s"
    scorer.update(poses,robot,contacts,environment)
    assert scorer.value == 1
    # A release violation at the very end must immediately revoke full score.
    bad_robot = robot.copy(); bad_robot[7, 0] = 1
    scorer.update(poses,bad_robot,contacts,environment)
    assert scorer.value < 1
    results.append(dict(task=name,case="3s duration and final-state revocation",passed=True))
    sys.path.pop(0)


check_task(TASK.name)
print(json.dumps(dict(passed=len(results), synthetic_only=True, cases=results), indent=2))
