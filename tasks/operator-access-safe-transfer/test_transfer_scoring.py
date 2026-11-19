"""Primary score red-team, with deterministic synthetic states and real model geometries.

These tests do not simulate motion or contact. The full physical reference is
still a required separate native Isaac qualification.
"""
import json,sys,unittest
from dataclasses import dataclass
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
T=HERE if (HERE/'personnel_guard.py').exists() else HERE.parent/'benchmark_source'/'tasks'/'operator-access-safe-transfer'
sys.path.insert(0,str(T))
import transfer_scoring as t
import personnel_guard as person
import arm_safety as arm

@dataclass
class DummyState:
    pos:tuple
    low:tuple|None=None
    high:tuple|None=None
    stopped:bool=True
    def resting(self):return self.stopped


def cube(xy,z,stopped=True):
    pos=(xy[0],xy[1],z);half=.020
    return DummyState(pos,tuple(p-half for p in pos),tuple(p+half for p in pos),stopped)


def objects(c):
    return {'component':c,
            'operator_hand_dummy':DummyState(person.PERSON_CENTER),
            'service_column':DummyState((.082,0.,t.TABLE_TOP+.330))}


def pose():
    return {side:{name:(2.+(0 if side=='left' else 1),2.,1.,0.,0.,0.,1.)
                  for name in arm.BODY_NAMES} for side in arm.SIDES}


def step(c,gap=.12):
    with patch.object(person,'finger_body_gap',return_value=(gap,'left','tip_left')):
        return t.update(objects(c),pose(),arm._MODEL,arm._capsule,arm._segment_box)



class ScorerRedTeam(unittest.TestCase):
    def setUp(self):t.reset_for_new_trial()

    def test_zero_for_do_nothing(self):
        for _ in range(50):self.assertEqual(step(cube(t.PICK,t.SOURCE_Z)),0.)

    def test_safe_supported_stage_credit(self):
        # Only stage achieved; no fabricated success flag.
        self._reach_stage()
        self.assertEqual(t.state()['level'],.5)

    def _reach_stage(self):
        step(cube(t.PICK,t.SOURCE_Z))
        step(cube(t.PICK,t.SOURCE_Z+.040))
        for xy,z in [((.19,.13),t.SOURCE_Z+.040),((.22,.11),t.SOURCE_Z+.040),
                     ((.25,.08),t.SOURCE_Z+.040),((.28,.05),t.SOURCE_Z+.040),
                     (t.STAGE,t.STAGE_CUBE_Z+.045),
                     (t.STAGE,t.STAGE_CUBE_Z+.005),
                     (t.STAGE,t.STAGE_CUBE_Z)]:
            step(cube(xy,z))
        for _ in range(t.STAGE_DWELL):step(cube(t.STAGE,t.STAGE_CUBE_Z))
        self.assertEqual(t.STATE['fault'],None,t.state())
        self.assertTrue(t.STATE['stage_qualified'])

    def test_held_at_stage_never_qualifies(self):
        step(cube(t.PICK,t.SOURCE_Z))
        step(cube(t.PICK,t.SOURCE_Z+.040))
        for xy,z in [((.19,.13),t.SOURCE_Z+.040),((.22,.11),t.SOURCE_Z+.040),
                     ((.25,.08),t.SOURCE_Z+.040),((.28,.05),t.SOURCE_Z+.040),
                     (t.STAGE,t.STAGE_CUBE_Z+.045),
                     (t.STAGE,t.STAGE_CUBE_Z+.005),
                     (t.STAGE,t.STAGE_CUBE_Z)]:
            step(cube(xy,z))
        for _ in range(24):self.assertEqual(step(cube(t.STAGE,t.STAGE_CUBE_Z),gap=0.0),0.)
        self.assertFalse(t.STATE['stage_qualified'])
        self.assertIsNone(t.STATE['fault'])
        for _ in range(t.STAGE_DWELL):step(cube(t.STAGE,t.STAGE_CUBE_Z),gap=.05)
        self.assertTrue(t.STATE['stage_qualified'])

    def test_hover_at_stage_does_not_qualify(self):
        self._reach_stage()
        t.reset_for_new_trial()
        step(cube(t.PICK,t.SOURCE_Z));step(cube(t.PICK,t.SOURCE_Z+.040))
        for xy,z in [((.19,.13),t.SOURCE_Z+.040),((.22,.11),t.SOURCE_Z+.040),
                     ((.25,.08),t.SOURCE_Z+.040),((.28,.05),t.SOURCE_Z+.040),
                     (t.STAGE,t.STAGE_CUBE_Z+.045),
                     (t.STAGE,t.STAGE_CUBE_Z+.009)]:step(cube(xy,z))
        for _ in range(12):self.assertEqual(step(cube(t.STAGE,t.STAGE_CUBE_Z+.009)),0.)

    def test_full_safe_flow_scores_one(self):
        self._reach_stage()
        self._reach_goal()
        self.assertEqual(t.STATE['level'],1.)

    def _reach_goal(self, held=False):
        step(cube(t.STAGE,t.STAGE_CUBE_Z+.040))
        step(cube(t.STAGE,t.STAGE_CUBE_Z+.075))
        # XY each step <=45mm; z stays elevated before landing.
        for xy in [(.29,0),(.28,-.04),(.26,-.08),(.24,-.12),(.22,-.16),t.GOAL]:
            step(cube(xy,t.GOAL_CUBE_Z+.075))
        step(cube(t.GOAL,t.GOAL_CUBE_Z+.035))
        step(cube(t.GOAL,t.GOAL_CUBE_Z))
        for _ in range(t.GOAL_DWELL):step(cube(t.GOAL,t.GOAL_CUBE_Z),gap=.001 if held else .12)

    def test_held_against_goal_cannot_earn_full(self):
        self._reach_stage();self._reach_goal(held=True)
        self.assertEqual(t.STATE['level'],.5)
        self.assertEqual(t.STATE['goal_release_streak'],0)
        for _ in range(t.GOAL_DWELL):step(cube(t.GOAL,t.GOAL_CUBE_Z),gap=.12)
        self.assertEqual(t.STATE['level'],1.)

    def test_shortcut_source_to_goal_is_zero(self):
        # No staging / relift even if workpiece visibly reaches the goal.
        step(cube(t.PICK,t.SOURCE_Z))
        # Reposition with valid <=45mm increments, no stage dwell.
        for xy in [(.18,.12),(.2,.09),(.2,.05),(.2,.01),(.2,-.03),
                   (.2,-.07),(.2,-.11),(.2,-.15),t.GOAL]:
            step(cube(xy,t.GOAL_CUBE_Z))
        for _ in range(12):step(cube(t.GOAL,t.GOAL_CUBE_Z))
        self.assertEqual(t.STATE['level'],0.)

    def test_payload_near_person_latches_zero(self):
        step(cube(t.PICK,t.SOURCE_Z))
        # Isolated test creates an instantaneous intrusion, which is also a jump.
        step(cube(person.PERSON_CENTER[:2],person.PERSON_CENTER[2]))
        self.assertIsNotNone(t.STATE['fault'])
        self.assertEqual(step(cube(t.PICK,t.SOURCE_Z)),0.)

    def test_tampered_operator_dummy_latches(self):
        o=objects(cube(t.PICK,t.SOURCE_Z))
        o['operator_hand_dummy'].pos=(person.PERSON_CENTER[0]+.02,*person.PERSON_CENTER[1:])
        with patch.object(person,'finger_body_gap',return_value=(.12,'left','tip_left')):
            self.assertEqual(t.update(o,pose(),arm._MODEL,arm._capsule,arm._segment_box),0.)
        self.assertEqual(t.STATE['fault'],'OPERATOR_SURROGATE_DISPLACED_OR_MISSING')

    def test_fixture_touch_fails(self):
        o=objects(cube(t.PICK,t.SOURCE_Z))
        o['component']=cube((.082,.0),t.SOURCE_Z)
        with patch.object(person,'finger_body_gap',return_value=(.12,'left','tip_left')):
            self.assertEqual(t.update(o,pose(),arm._MODEL,arm._capsule,arm._segment_box),0.)
        self.assertIsNotNone(t.STATE['fault'])

    def test_return_to_start_does_not_clear_fault(self):
        o=objects(cube(t.PICK,t.SOURCE_Z));o['operator_hand_dummy'].pos=(.5,.5,1.)
        with patch.object(person,'finger_body_gap',return_value=(.12,'left','tip_left')):
            t.update(o,pose(),arm._MODEL,arm._capsule,arm._segment_box)
        for _ in range(12):self.assertEqual(step(cube(t.PICK,t.SOURCE_Z)),0.)
        self.assertIsNotNone(t.STATE['fault'])

    def test_discontinuous_teleport_latches(self):
        step(cube(t.PICK,t.SOURCE_Z))
        self.assertEqual(step(cube(t.GOAL,t.GOAL_CUBE_Z)),0.)
        self.assertEqual(t.STATE['fault'],'PAYLOAD_JUMP_EXCEEDS_STEP_LIMIT')

    def test_still_cube_cannot_substitute_release(self):
        self._reach_stage()
        self._reach_goal(held=True)
        for _ in range(120):step(cube(t.GOAL,t.GOAL_CUBE_Z),gap=.002)
        self.assertEqual(t.STATE['level'],.5)

    def test_one_observation_per_step(self):
        step(cube(t.PICK,t.SOURCE_Z))
        self.assertEqual(t.STATE['calls'],1)
        self.assertEqual(t.STATE['stage_release_streak'],0)

if __name__=='__main__':unittest.main()
