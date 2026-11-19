"""Exercise actual task.score() / success() through mock YAM package.

No physics connection is forged. Failure to read physics must return 0.
"""
import importlib.util,sys,types,unittest
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
T=HERE if (HERE/'operator_access_safe_transfer.py').exists() else HERE.parent/'benchmark_source'/'tasks'/'operator-access-safe-transfer'
sys.path.insert(0,str(T))
import arm_safety as arm,transfer_scoring as transfer

class Box:
    def __init__(self,name,**kwargs):
        self.name=name;self.__dict__.update(kwargs)

class Waypoint:
    def __init__(self,*a):self.args=a

class YamMock:
    def __enter__(self):
        self.keys=[n for n in sys.modules if n=='yam_arena' or n.startswith('yam_arena.')]
        self.backup={n:sys.modules[n] for n in self.keys}
        m=types.ModuleType('yam_arena');tf=types.ModuleType('yam_arena.task_file')
        tf.TABLE_TOP={'station':.7475};tf.Box=Box
        const=types.ModuleType('yam_arena.constants');const.FINGER_OPEN=-.04695;const.FINGER_CLOSED=0.
        demo=types.ModuleType('yam_arena.demo');demo.Waypoint=Waypoint
        sys.modules.update({'yam_arena':m,'yam_arena.task_file':tf,'yam_arena.constants':const,'yam_arena.demo':demo})
    def __exit__(self,*a):
        for n in ('yam_arena','yam_arena.task_file','yam_arena.constants','yam_arena.demo'):
            sys.modules.pop(n,None)
        sys.modules.update(self.backup)

def load_task():
    with YamMock():
        s=importlib.util.spec_from_file_location('operator_test_module',T/'operator_access_safe_transfer.py')
        m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m

class TaskContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.task=load_task()
    def setUp(self):transfer.reset_for_new_trial();arm.reset_for_new_trial()

    def test_has_correct_scene_and_objects(self):
        m=self.task
        self.assertEqual(m.SCENE,'station')
        self.assertEqual(len(m.OBJECTS),5)
        self.assertEqual({o.name for o in m.OBJECTS},{'component','service_column','handoff_pad','target_pad','operator_hand_dummy'})
        self.assertTrue(any(o.name=='operator_hand_dummy' and o.pos[1]>.04 for o in m.OBJECTS))

    def test_demo_has_independent_tapes(self):
        tapes=self.task.scripted_waypoints()
        self.assertIn('left',tapes);self.assertIn('right',tapes)
        self.assertGreater(len(tapes['left']),10)
        self.assertGreater(len(tapes['right']),10)

    def test_success_must_not_advance_counters(self):
        before=transfer.state()
        for _ in range(100):self.assertIs(self.task.success({}),False)
        self.assertEqual(transfer.state(),before)

    def test_physics_unavailable_fails_closed(self):
        with patch.object(arm,'current_poses',side_effect=RuntimeError('not ready')):
            v=self.task.score({})
        self.assertEqual(v,0.)
        self.assertTrue(arm.state()['latched'])

    def test_missing_objects_never_get_full(self):
        with patch.object(arm,'current_poses',return_value={side:{} for side in arm.SIDES}):
            v=self.task.score({})
        self.assertEqual(v,0.)
        self.assertTrue(arm.state()['latched'])

if __name__=='__main__':unittest.main()
