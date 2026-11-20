import unittest
from angle_tracking import ContinuousBottomAngles,BODIES,PHYSICS_DT
from angle_step_observer import AngleStepObserver
from test_angle_tracking import state


class StepObserverTests(unittest.TestCase):
    def build(self):
        self.counter=0;self.events=[]
        def step(*args,**kwargs):
            self.events.append(('step',args,kwargs));self.counter+=1
            return 'unchanged-result'
        def read():
            self.events.append(('read',self.counter))
            return state([self.counter]*4)
        t=ContinuousBottomAngles({n:5729.58 for n in BODIES})
        observer=AngleStepObserver(t,step,read,lambda:self.counter,lambda:self.counter*PHYSICS_DT)
        self.assertTrue(observer.initialize());self.events=[]
        return observer

    def test_calls_original_once_and_reads_after_return(self):
        o=self.build();o.begin_control()
        for _ in range(12):self.assertEqual(o.step(render=False),'unchanged-result')
        e=o.end_control();self.assertTrue(e['valid']);self.assertEqual(e['counter'],12)
        self.assertEqual(len(o.samples),12)
        self.assertEqual([x[0] for x in self.events],['step','read']*12+['read'])
        self.assertEqual(o.samples[-1]['angles_deg'],[12.]*4)

    def test_short_interval_invalidates(self):
        o=self.build();o.begin_control();o.step()
        self.assertFalse(o.end_control()['valid'])
        with self.assertRaises(RuntimeError):o.begin_control()

    def test_unobserved_step_invalidates(self):
        o=self.build();self.counter=1
        with self.assertRaises(RuntimeError):o.begin_control()

    def test_extra_step_aborts_before_physics(self):
        o=self.build();o.begin_control()
        for _ in range(12):o.step()
        with self.assertRaises(RuntimeError):o.step()
        self.assertEqual(self.counter,12)

    def test_outside_interval_aborts_before_physics(self):
        o=self.build()
        with self.assertRaises(RuntimeError):o.step()
        self.assertEqual(self.counter,0)

    def test_native_step_failure_is_sticky(self):
        o=self.build();o.begin_control()
        def failed():raise RuntimeError('test integration failure')
        o.original=failed
        with self.assertRaises(RuntimeError):o.step()
        self.assertIn('continuity unknown',o.tracker.failure)

    def test_post_step_endpoint_change_is_rejected(self):
        o=self.build();o.begin_control()
        for _ in range(12):o.step()
        o.read_state=lambda:state([99.]*4)
        self.assertFalse(o.end_control()['valid'])
        self.assertIn('endpoint changed',o.tracker.failure)

if __name__=='__main__':unittest.main()
