"""Replayable geometric public score and separately labeled physical gate diagnostics.

No controller stages, action intent, history-best partial credit, or attachment flags.
"""
from collections import deque
import itertools
import math
import numpy as np
from config import S, F, P, T, HZ, FLOOR_Z, FOOT_CENTERS, PAD_H

# Version 2 separates visible transport retention from the stationary contact
# assessment. Defaults let the current scorer rescore original recordings whose
# saved specification predates these fields; they do not restore the old rule.
SCORING_VERSION = 2
TRANSPORT = {
    'foot_center_offset_m': T.get('transport_foot_center_offset_m', .020),
    'height_tolerance_m': T.get('transport_height_tolerance_m', .010),
    'upright_tolerance_deg': T.get('transport_upright_tolerance_deg', 15.),
}


def matching(candidates):
    """Maximum one-to-one foot/pad matching, in deterministic input order."""
    for n in range(min(4,len(candidates)),0,-1):
        for c in itertools.combinations(candidates,n):
            if len({a for a,b in c})==n and len({b for a,b in c})==n:
                return list(c)
    return []


def footprints_on_panel(foot_corners):
    """Horizontal footprint bounds only; no hidden vertical contact test."""
    floor = S['floor']
    lo = np.array(floor['center_m'][:2]) - np.array(floor['size_m'][:2])/2
    hi = np.array(floor['center_m'][:2]) + np.array(floor['size_m'][:2])/2
    return bool(all(np.all(c[:,:2] >= lo) and np.all(c[:,:2] <= hi)
                    and not np.any((c[:,0]<.10) & (np.abs(c[:,1])>.195) & (np.abs(c[:,1])<.415))
                    for c in foot_corners))


def transport_snapshot(m, foot_corners):
    """Centimetre-scale, video-measurable retention; no forces or contact gaps.

    A foot-base centre stays within 20 mm horizontally of its pad centre.
    Foot-base centres stay within 10 mm of the resting height (floor + pad
    thickness), and pad underside centres within 10 mm of floor height. These
    absolute height bands reject visible carrying, including slow cumulative
    lifting. Sub-resolution unloading is deliberately not inferred from video.
    """
    candidates = []
    eps = 1e-9
    for i,corners in enumerate(foot_corners):
        center = corners.mean(axis=0)
        if not abs(center[2] - (FLOOR_Z + PAD_H)) <= TRANSPORT['height_tolerance_m'] + eps:
            continue
        for name,pad in m['pads'].items():
            if (np.linalg.norm(center[:2] - np.asarray(pad[:2])) <= TRANSPORT['foot_center_offset_m'] + eps
                    and abs(pad[2] - FLOOR_Z) <= TRANSPORT['height_tolerance_m'] + eps
                    and tilt(rotation(pad[3:])) <= TRANSPORT['upright_tolerance_deg'] + eps):
                candidates.append((i,name))
    pairs = matching(candidates)
    retained = (len(pairs)==4 and footprints_on_panel(foot_corners)
                and tilt(rotation(m['furniture'][3:])) <= TRANSPORT['upright_tolerance_deg'] + eps)
    return {'transport_matches':pairs, 'transport_retained':bool(retained)}


def rotation(q):
    x,y,z,w = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def tilt(r):
    return math.degrees(math.acos(float(np.clip(r[2,2],-1,1))))


def points(pose, local):
    return np.asarray(local) @ rotation(pose[3:]).T + np.asarray(pose[:3])


def angular_distance(a,b):
    return math.degrees(math.acos(float(np.clip((np.trace(rotation(a).T @ rotation(b))-1)/2,-1,1))))


def snapshot(m):
    pose = m['furniture']
    r = rotation(pose[3:])
    s = F['foot_width_m']/2
    foot_corners = [points(pose, [(x+dx,y+dy,z) for dx in (-s,s) for dy in (-s,s)])
                    for x,y,z in FOOT_CENTERS]
    candidates = []
    on_panel = footprints_on_panel(foot_corners) and all(
        c[:,2].min() >= FLOOR_Z - T['max_penetration_m'] for c in foot_corners)
    for i,corners in enumerate(foot_corners):
        for name, p in m['pads'].items():
            rp = rotation(p[3:])
            local = (corners - np.asarray(p[:3])) @ rp
            radius = np.linalg.norm(local[:,:2],axis=1).max()
            gap = float(local[:,2].min()-PAD_H)
            # At a tilt, edge contact can support a rigid foot. Requiring all corners
            # coplanar would incorrectly reject the one-pad intermediate state.
            supported = -T['max_penetration_m'] <= gap <= T['contact_gap_m']
            # Use the lowest point of the actual felt disk, not just its origin.
            # At 1 degree, the rim can penetrate 0.78 mm while the origin is
            # exactly on the floor. A physically resting tilted disk instead
            # raises its origin so that its lowest rim remains on the floor.
            ground_gap = p[2] - P['diameter_m']/2*np.linalg.norm(rp[2,:2]) - FLOOR_Z
            pad_on_floor = -T['max_penetration_m'] <= ground_gap <= T['contact_gap_m']
            if (supported and radius <= P['usable_foam_radius_m']-T['containment_margin_m']
                    and pad_on_floor and tilt(rp) <= T['max_pad_tilt_deg']):
                candidates.append((i,name))
    # Maximum bipartite matching: each separate pad supports at most one foot.
    matches = matching(candidates)
    top = points(pose, [(x,y,F['size_m'][2]) for x in (-F['size_m'][0]/2,F['size_m'][0]/2)
                       for y in (-F['size_m'][1]/2,F['size_m'][1]/2)])
    goal = np.asarray(S['layout']['goal_xy_m'])
    goal_half = np.asarray(S['layout']['goal_size_m'])/2
    in_goal = bool(np.all(np.abs(top[:,:2]-goal) <= goal_half+1e-9))
    clear = m['hand_clearance_m'] >= T['release_clearance_m'] and not m.get('robot_contact',False)
    return {'matches':matches, 'on_panel':bool(on_panel), 'in_goal':in_goal,
            'tilt_deg':tilt(r), 'hands_clear':bool(clear), 'foot_corners':[c.tolist() for c in foot_corners],
            **transport_snapshot(m, foot_corners)}


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.window = deque(maxlen=round(T['hold_s']*HZ)+1)
        self.previous = None
        self.transport_start = None
        self.transport_max = 0.
        self.transport_pairs = None
        self.transport_start_step = None
        self.score = 0.
        self.success = False
        self.detail = {}
        self.gate_start = None
        self.gate_pair = None
        self.gate_local = None
        self.gate_max = 0.
        self.gate_drift = 0.
        self.gate_pass = False

    def update(self,m):
        if self.previous is not None and m['step'] != self.previous['step']+1:
            raise ValueError('Scoring requires contiguous control-step observations')
        d = snapshot(m)
        pos = np.asarray(m['furniture'][:3])
        pairs = tuple(sorted(tuple(v) for v in d['matches']))
        transport_pairs = tuple(sorted(tuple(v) for v in d['transport_matches']))
        if d['transport_retained']:
            if self.transport_start is None or transport_pairs != self.transport_pairs:
                self.transport_start = pos[:2].copy(); self.transport_max=0.; self.transport_pairs=transport_pairs
                self.transport_start_step = m['step']
            self.transport_max=max(self.transport_max,float(np.linalg.norm(pos[:2]-self.transport_start)))
        else:
            self.transport_start=None; self.transport_max=0.; self.transport_pairs=None
            self.transport_start_step=None
        self.window.append(m)
        stable = len(self.window)==self.window.maxlen
        if stable:
            anchor=self.window[0]
            stable=all(np.linalg.norm(np.array(v['furniture'][:3])-anchor['furniture'][:3])<=T['stable_translation_m']
                       and angular_distance(v['furniture'][3:],anchor['furniture'][3:])<=T['stable_rotation_deg']
                       and v['hand_clearance_m']>=T['release_clearance_m'] and not v.get('robot_contact',False)
                       for v in self.window)
            stable=stable and all(tuple(sorted(tuple(p) for p in snapshot(v)['matches']))==pairs for v in self.window)
        # Credit a released standing table. Upending remains a legal installation
        # strategy, but propping a table on its side is not a scored final state.
        self.score=round(.1*len(pairs),3) if stable and d['on_panel'] and d['tilt_deg']<=T['max_final_tilt_deg'] else 0.
        self.success=bool(self.score==.4 and d['in_goal'] and d['transport_retained']
                          and pairs==self.transport_pairs and self.transport_max+1e-6>=T['transport_distance_m'])
        if self.success: self.score=1.
        # One-pad gate: first a released settled loaded support, then a continuous
        # 100 mm movement with this pad unheld and retained, then released again.
        if self.gate_pair is None and stable and pairs:
            pair=next((p for p in pairs if m.get('pad_loads_N',{}).get(p[1],0)>=T['one_pad_min_load_N']),None)
            if pair:
                self.gate_pair=pair; self.gate_start=pos[:2].copy()
                self.gate_local=self._relative(m,pair)
                self.gate_max=0.; self.gate_drift=0.
        if self.gate_pair is not None:
            retained=(self.gate_pair in pairs and d['on_panel'] and not m.get('pad_held',{}).get(self.gate_pair[1],False))
            if retained:
                self.gate_max=max(self.gate_max,float(np.linalg.norm(pos[:2]-self.gate_start)))
                self.gate_drift=max(self.gate_drift,float(np.linalg.norm(self._relative(m,self.gate_pair)-self.gate_local)))
                retained=self.gate_drift<=T['one_pad_max_drift_m']
            if not retained:
                self.gate_pair=None; self.gate_start=None; self.gate_max=0.; self.gate_drift=0.; self.gate_pass=False
            else:
                self.gate_pass=bool(stable and self.gate_max+1e-6>=T['one_pad_distance_m']
                                    and m.get('pad_loads_N',{}).get(self.gate_pair[1],0)>=T['one_pad_min_load_N'])
        self.detail={**d, 'scoring_version':SCORING_VERSION, 'transport_rule':dict(TRANSPORT),
                     'stable_released':bool(stable), 'score':self.score, 'success':self.success,
                     'transport_m':self.transport_max, 'transport_start_step':self.transport_start_step,
                     'one_pad_gate':self.gate_pass,
                     'one_pad_travel_m':self.gate_max, 'one_pad_drift_m':self.gate_drift}
        self.previous=m
        return self.detail

    @staticmethod
    def _relative(m,pair):
        foot=points(m['furniture'],[FOOT_CENTERS[pair[0]]])[0]
        pad=m['pads'][pair[1]]
        return ((foot-np.asarray(pad[:3]))@rotation(pad[3:]))[:2]
