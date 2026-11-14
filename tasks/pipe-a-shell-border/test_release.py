"""A held or moving dispenser must not receive released-finish credit."""
import unittest
import numpy as np
from audit_run import release_evidence,distance


class ReleaseTests(unittest.TestCase):
    def record(self):
        bounds={'barrel':[[.165,-.20,.7625],[.195,-.12,.7925]],
                'plunger':[[.1625,-.1025,.760],[.1975,-.0975,.795]]}
        bounds.update({f'robot{i}':[[-.5,-.5,1.2],[-.4,-.4,1.3]] for i in range(18)})
        return [{'step':i,'time_s':i/20,'barrel_pose_xyzw':[.18,-.16,.7775,0,0,1,0],
                 'rigid_bounds_m':{k:[list(lo),list(hi)] for k,(lo,hi) in bounds.items()}}
                for i in range(21)]

    def test_released_supported_tool_and_held_tool_differ(self):
        rows=self.record();paths={'barrel':'barrel','plunger':'plunger'}
        self.assertTrue(release_evidence(rows,paths)['released_clear'])
        for row in rows:row['rigid_bounds_m']['robot0']=[[.17,-.18,.77],[.19,-.14,.79]]
        self.assertFalse(release_evidence(rows,paths)['released_clear'])

    def test_motion_or_incomplete_window_cannot_certify_release(self):
        rows=self.record();paths={'barrel':'barrel','plunger':'plunger'}
        self.assertFalse(release_evidence(rows[:-1],paths)['released_clear'])
        rows[-1]['barrel_pose_xyzw'][2]+=.01
        self.assertFalse(release_evidence(rows,paths)['released_clear'])

    def test_invalid_robot_bounds_fail_closed(self):
        with self.assertRaises(ValueError):distance([[np.nan,0,0],[1,1,1]],[[0,0,0],[1,1,1]])


if __name__=='__main__':unittest.main()
