"""Milestones and hazard events of the strut spring-nut task, from simulator state. Pure torch.

Milestones (each needs the ones before it at the same step; once reached a milestone is kept):
- 20: the bolt started by the robot at least STARTED_TURNS turns into the spring nut (the thread caught square).
- 40: also the spring nut seated under the channel's lips: turned across the channel (within NUT_ACROSS_DEG), its
  grooves on the lips (its top within SEAT_TOL of the seated height) and centred in the channel.
- 60: also the bolt's axis within MARK_TOL of the red mark, along the channel.
- 80: also the bolt engaged in the nut by at least ENGAGED_TURNS turns.
- 100: the after-release check passed: it starts once every finger pad has stayed at least RELEASE_CLEAR from the
  fitting and the bolt for TEST_WAIT_S without a break (the count restarts whenever a pad comes closer); the 80
  conditions, the fitting seated flat on the channel and square to it, and the bolt head down on it must hold when
  it starts; then a pull of TEST_PULL_N along the channel on the fitting for TEST_S must move it less than
  TEST_MAX_MOVE (a fitting that is not held to the channel by a seated nut and a hand-snug bolt slides).

Hazards (each sets the score to 0 for good, whatever was reached before or after): a part dropped (it fell to the
bench); the nut loose in the channel (in it with the bolt not started in it, or run off the end of the bolt: in a
real channel it falls in and is lost); the bolt cross-threaded (it caught tilted more than ALIGN_DEG to the nut).
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

MILESTONES = (20, 40, 60, 80, 100)


@dataclass
class Step:
    started: torch.Tensor
    seated: torch.Tensor
    at_mark: torch.Tensor
    engaged: torch.Tensor
    test_passed: torch.Tensor
    dropped: torch.Tensor
    nut_off: torch.Tensor
    cross_threaded: torch.Tensor


class Scorer:
    HAZARDS = {"dropped": "part_dropped", "nut_off": "nut_loose_in_channel", "cross_threaded": "cross_threaded"}

    def __init__(self, num_envs: int, device="cpu"):
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.reset()

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        n = self.num_envs
        fields = {"milestone": torch.zeros(n, dtype=torch.long, device=self.device),
                  "step": torch.zeros(n, dtype=torch.long, device=self.device),
                  "hazard_step": torch.full((n,), -1, dtype=torch.long, device=self.device)}
        for name in self.HAZARDS:
            fields[name] = torch.zeros(n, dtype=torch.bool, device=self.device)
        for name, value in fields.items():
            if env_ids is None or not hasattr(self, name):
                setattr(self, name, value)
            else:
                getattr(self, name)[env_ids] = value[env_ids]

    def update(self, obs: Step) -> None:
        m = self.milestone
        reached = torch.zeros_like(m)
        s1 = obs.started
        s2 = s1 & obs.seated
        s3 = s2 & obs.at_mark
        s4 = s3 & obs.engaged
        reached = torch.where(s1, 20, reached)
        reached = torch.where(s2, 40, reached)
        reached = torch.where(s3, 60, reached)
        reached = torch.where(s4, 80, reached)
        reached = torch.where(obs.test_passed & (m >= 80), 100, reached)
        before = self.hazard.clone()
        for name in self.HAZARDS:
            getattr(self, name).__ior__(getattr(obs, name))
        new = self.hazard & ~before
        self.hazard_step = torch.where(new, self.step, self.hazard_step)
        self.milestone = torch.maximum(m, reached)
        self.step += 1

    @property
    def hazard(self) -> torch.Tensor:
        return self.dropped | self.nut_off | self.cross_threaded

    @property
    def score(self) -> torch.Tensor:
        return torch.where(self.hazard, torch.zeros_like(self.milestone), self.milestone)

    @property
    def success(self) -> torch.Tensor:
        return (self.milestone >= 100) & ~self.hazard

    def report(self, env_id: int = 0) -> dict:
        return {
            "progress_milestone": int(self.milestone[env_id]),
            "milestone": int(self.score[env_id]),
            "score": int(self.score[env_id]) / 100.0,
            "success": bool(self.success[env_id]),
            "hazard_step": int(self.hazard_step[env_id]),
            "semantic_events": {label: bool(getattr(self, name)[env_id]) for name, label in self.HAZARDS.items()},
        }
