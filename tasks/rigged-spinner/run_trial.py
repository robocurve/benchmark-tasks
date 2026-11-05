"""Run one trial of this task and print its score. Usage: python run_trial.py --seed 0 [--policy oracle|guess|noop]"""
import argparse, json
import numpy as np
from robosci.env import SimEnv
from robosci.tasks import POLICIES, TASKS

KEY = "biased_spinner"
ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--policy", default="oracle", choices=sorted(POLICIES[KEY]))
ap.add_argument("--difficulty", default="hard")
a = ap.parse_args()
env = SimEnv(TASKS[KEY](difficulty=a.difficulty), seed=a.seed)
env.settle(0.5)
for act in POLICIES[KEY][a.policy](env, np.random.default_rng(a.seed)):
    env.step(act)
s = env.score()
print(json.dumps({"stage": s["stage"], "score": s["stage"] / 4, **{k: v for k, v in s.items() if k != "stage"}}, default=float, indent=1))
