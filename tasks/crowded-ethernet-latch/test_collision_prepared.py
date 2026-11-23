"""Parity with the frozen original face-plane certificate; CPU only."""
import json
from pathlib import Path
import time
import unittest
from unittest.mock import patch

import numpy as np

import collision_geometry as collision
from geometry import box_triangles, object_obstacles
from yam_arena import ik
from yam_arena.urdf import rpy_matrix


def original_separating_gap(first, second):
    av,bv = first.reshape(-1,3),second.reshape(-1,3)
    gap = float(np.maximum(av.min(0)-bv.max(0),bv.min(0)-av.max(0)).max())
    if gap > .003:
        return gap
    triangles = np.concatenate((first,second))
    axes = np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    lengths = np.linalg.norm(axes,axis=1)
    axes = np.unique(np.round(axes[lengths>1e-12]/lengths[lengths>1e-12,None],8),axis=0)
    for start in range(0,len(axes),128):
        a,b = av@axes[start:start+128].T,bv@axes[start:start+128].T
        gap = max(gap,float(np.maximum(a.min(0)-b.max(0),b.min(0)-a.max(0)).max()))
    return max(0.,gap)


def original_minimum_separation(first_pieces, second_pieces):
    first_min = np.array([p.min((0,1)) for p in first_pieces])
    first_max = np.array([p.max((0,1)) for p in first_pieces])
    second_min = np.array([p.min((0,1)) for p in second_pieces])
    second_max = np.array([p.max((0,1)) for p in second_pieces])
    gaps = np.maximum(first_min[:,None]-second_max[None,:],second_min[None,:]-first_max[:,None]).max(2)
    for i,j in np.argwhere(gaps <= .003):
        gaps[i,j] = original_separating_gap(first_pieces[i],second_pieces[j])
    return float(gaps.min())


TRACE = Path('/private/tmp/ethernet-tracking025.jsonl')


def recorded_geometry(row, geometry):
    hand = row['native_hands'][1]
    arm = row['robot_control']['arms'][1]
    start = np.asarray(hand['tcp']),np.asarray(hand['tcp_rotation'])
    end = np.asarray(arm['bounded_command_fk_tcp']),np.asarray(arm['bounded_command_fk_rotation'])
    pieces = geometry.pieces(hand['finger_joints'])
    sweep = [np.concatenate([a@ik.slerp_matrix(start[1],end[1],s).T+start[0]+s*(end[0]-start[0])
                             for s in np.linspace(0.,1.,5)]) for _,a in pieces]
    plugs = {int(i):np.asarray(p) for i,p in row['plug_poses'].items()}
    clips = {int(i):np.asarray(p) for i,p in row['clip_poses'].items()}
    return sweep,[a for _,_,a in object_obstacles(plugs,clips,include_wires=False)]


def trace_rows():
    rows = [json.loads(line) for line in TRACE.read_text().splitlines()]
    rows = [row for row in rows if row['phase']=='prepress_endpoint']
    return [rows[0],rows[len(rows)//2],rows[-1]]


class PreparedCollisionTests(unittest.TestCase):
    def assert_parity(self, first, second):
        expected = original_minimum_separation(first,second)
        actual = collision.minimum_separation(first,second)
        self.assertAlmostEqual(actual,expected,delta=2e-15)
        # A zero original certificate must never become a false clear.
        if expected == 0:self.assertEqual(actual,0)
        for a in first:
            for b in second:
                self.assertAlmostEqual(collision.separating_gap(a,b),original_separating_gap(a,b),delta=2e-15)

    def test_rotated_overlapping_crossing_and_degenerate_meshes(self):
        thin = box_triangles((0,0,0),(.02,.002,.002))
        cross = thin@rpy_matrix((0,0,np.pi/2)).T
        self.assert_parity([thin],[cross])
        self.assertEqual(collision.separating_gap(thin,cross),0)
        for angle in (0,.13,.71,1.5):
            rotated = thin@rpy_matrix((angle,.27,-.39)).T
            for offset in (0,.001,.003,.02):
                self.assert_parity([thin],[rotated+[0,offset,0]])
        point = np.zeros((1,3,3))
        self.assert_parity([point],[point+.001])

    def test_sweeps_and_duplicated_vertices_preserve_support(self):
        a = box_triangles((0,0,0),(.01,.003,.004))
        sweep = np.concatenate([a@rpy_matrix((0,0,s*.2)).T+[s*.002,0,0] for s in np.linspace(0,1,5)])
        b = box_triangles((0,.004,0),(.02,.002,.002))
        self.assert_parity([sweep,np.tile(sweep,(3,1,1))],[b,b+[0,.001,0]])

    def test_preparation_is_lazy_shared_within_call_and_never_stale(self):
        a = box_triangles((0,0,0),(.01,.01,.01))
        b = a+[0,.011,0]
        with patch.object(collision,'_prepare_piece',wraps=collision._prepare_piece) as prepare:
            collision.minimum_separation([a],[b,b.copy(),b+[0,.1,0]])
            self.assertEqual(prepare.call_count,3)
        self.assertGreater(collision.minimum_separation([a],[b]),0)
        b[:]-=[0,.011,0]
        self.assertEqual(collision.minimum_separation([a],[b]),0)
        with patch.object(collision,'_prepare_piece',side_effect=AssertionError('broad phase should skip')):
            self.assertEqual(collision.separating_gap(a,a+[1,0,0]),.99)

    @unittest.skipUnless(TRACE.exists(),'native tracking025 trace is a local validation artifact')
    def test_recorded_tracking025_five_pose_sweeps(self):
        geometry = collision.FingerGeometry()
        for row in trace_rows():
            first,second = recorded_geometry(row,geometry)
            # Full scene parity; individual touched pairs are also checked by
            # the frozen original function while reducing this certificate.
            expected = original_minimum_separation(first,second)
            actual = collision.minimum_separation(first,second)
            self.assertAlmostEqual(actual,expected,delta=2e-15)
            if expected == 0:self.assertEqual(actual,0)


def benchmark(repeats=5):
    geometry = collision.FingerGeometry()
    first,second = recorded_geometry(trace_rows()[-1],geometry)
    results = {}
    for name,function in (('original',original_minimum_separation),('prepared',collision.minimum_separation)):
        function(first,second)
        durations=[]
        for _ in range(repeats):
            started=time.perf_counter();gap=function(first,second);durations.append(time.perf_counter()-started)
        results[name]={'gap_m':gap,'median_s':float(np.median(durations)),'samples_s':durations}
    results['piece_counts']=[len(first),len(second)]
    results['speedup']=results['original']['median_s']/results['prepared']['median_s']
    print(json.dumps(results,indent=2))


if __name__ == '__main__':
    unittest.main()
