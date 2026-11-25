"""Official Inspect interfaces, with separate policy and evaluator channels."""
from dataclasses import replace
from pathlib import Path
import hashlib
import json
import traceback
import numpy as np
from inspect_robots import Action, ActionChunk, Observation, StepResult
from inspect_robots.embodiment import EmbodimentBase, EmbodimentInfo, RESETTABLE, SEEDABLE, PRIVILEGED_SUCCESS
from inspect_robots.policy import PolicyBase, PolicyConfig, PolicyInfo
from inspect_robots.scorer import Score
from inspect_robots_yam.config import action_box, observation_space, DEFAULT_CAMERAS
from inspect_robots_yam.packing import STATE_KEY, pack
import spec

ROOT = Path(__file__).resolve().parent
INSTRUCTION = json.loads((ROOT/'task.json').read_text())['instruction']
NAMES = {'top_cam':'overhead_cam','left_cam':'left_wrist_cam','right_cam':'right_wrist_cam'}


class Cards(EmbodimentBase):
    def __init__(self, output, *, robot_state=True):
        from yam_arena import YAM_URDF
        from yam_arena.constants import ARM_JOINTS, FINGER_OPEN
        from yam_arena.urdf import parse_joints
        self.output = Path(output)
        self.robot_state = robot_state
        self.world = None
        self.episode = 0
        joints = parse_joints(YAM_URDF)
        low = [*[joints[n].lower for n in ARM_JOINTS], 0.]
        high = [*[joints[n].upper for n in ARM_JOINTS], spec.finger_joint_position(.075)/FINGER_OPEN]
        observations = observation_space(176, 320, DEFAULT_CAMERAS)
        if not robot_state:
            observations = replace(observations, state_keys=frozenset(), state=None)
        revision = hashlib.sha256(b''.join(p.read_bytes() for p in sorted(ROOT.glob('*.py')))).hexdigest()
        self.info = EmbodimentInfo(name='card_house', action_space=action_box(pack(low,low),pack(high,high)),
                    observation_space=observations, control_hz=20., is_simulated=True,
                    capabilities=frozenset({RESETTABLE,SEEDABLE,PRIVILEGED_SUCCESS}),
                    environment_id='isaaclab-arena-card-house', environment_revision=revision,
                    docs='Two YAM arms; 14 absolute targets: six joint angles and normalized gripper opening per arm. '
                         'Three calibrated RGB views; Dream also receives robot joint readings. No object poses or evaluator state. '
                         'The maximum opening is 75 mm and each finger is capped at 40 N.')

    def reset(self, scene, *, seed=None):
        from arena_runtime import CardWorld
        if self.world is not None:
            self.world.close()
        self.instruction = scene.instruction
        try:
            self.world = CardWorld(self.output/f'episode-{self.episode:02d}', seed=seed or 0)
        except Exception:
            traceback.print_exc()
            raise
        self.episode += 1
        return self.observation()

    def observation(self):
        images = self.world.images()
        images = {sdk:images[physical] for sdk,physical in NAMES.items()}
        t = self.world.steps/spec.CONTROL_HZ
        state = {STATE_KEY:self.world.joints()} if self.robot_state else {}
        return Observation(images=images, state=state, instruction=self.instruction,
                           image_times=dict.fromkeys(images,t), state_time=t)

    def step(self, action):
        target = np.asarray(action.data, dtype=np.float64)
        if target.shape != (14,) or not np.isfinite(target).all():
            raise ValueError('Invalid 14-D action')
        target = np.clip(target,self.info.action_space.low,self.info.action_space.high)
        row = self.world.step(target)
        return StepResult(observation=self.observation(), reward=row['score'],
                          terminated=row['score']==1. or row['invalid'],
                          termination_reason='success' if row['score']==1. else 'failure' if row['invalid'] else None,
                          info={'score':row['score'],'success':row['score']==1.,'invalid':row['invalid']})

    def close(self):
        if self.world is not None:
            self.world.close()
            self.world = None


class CardsScore:
    name = 'cards_score'
    def __call__(self, record, target):
        if getattr(record,'status',None)=='error' or not record.steps:
            return Score(0., 'No completed trial evidence')
        info = record.steps[-1].result.info
        return Score(float(info['score']), 'Terminal physical score with latched partial credit and invalid-event gating.')


class RGBDemo(PolicyBase):
    config = PolicyConfig(action_horizon=1)
    def bind(self, info):
        if info.observation_space.state_keys:
            raise ValueError('The demonstration must have RGB-only observations')
        self.info = PolicyInfo(name='cards_rgb_demo',action_space=info.action_space,
                              observation_space=info.observation_space,control_hz=20.)

    def reset(self, scene):
        from yam_arena import ik, constants
        import task_station as station
        from demo import Controller
        self.controller = Controller(ik,constants,station)
        self.finger_open = constants.FINGER_OPEN
        self.step_index = 0

    def act(self, observation):
        if observation.state:
            raise ValueError('Unexpected state channel in RGB-only demo')
        rgb = {physical:observation.images[sdk] for sdk,physical in NAMES.items()}
        command = self.controller.action(self.step_index,rgb)
        self.step_index += 1
        target = np.concatenate([np.r_[command[s][:6],command[s][6]/self.finger_open] for s in ['left','right']])
        return ActionChunk(actions=(Action(target),),control_hz=20.)

    def transcript(self):
        return {'observation_inputs':'three RGB cameras only',
                'rgb_observations':self.controller.observations,
                'offline_ik_errors':self.controller.errors}


class Hold(PolicyBase):
    config = PolicyConfig(action_horizon=1)
    def bind(self, info):
        self.info = PolicyInfo(name='cards_hold',action_space=info.action_space,
                              observation_space=info.observation_space,control_hz=20.)
    def reset(self, scene):
        self.target = None
    def act(self, observation):
        if self.target is None:
            self.target = observation.state[STATE_KEY].copy()
        return ActionChunk(actions=(Action(self.target.copy()),),control_hz=20.)
