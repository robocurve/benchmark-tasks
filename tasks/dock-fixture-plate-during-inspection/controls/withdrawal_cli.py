"""Register the two comparison policies for this single Inspect process."""

from dataclasses import replace
from inspect_robots.cli import main
from inspect_robots.registry import policy
from yam_arena.scripted import YamScriptedPolicy
from withdrawal_controls import PAIR, TASK, plan


class WithdrawalPolicy(YamScriptedPolicy):
    def __init__(self, mode, hold_s=31):
        super().__init__(hold_s=hold_s)
        self.mode = mode
        self.info = replace(self.info, name=mode)

    def plan_for(self, instruction, target=None):
        if (
            self._task is None
            or self._task.path.resolve() != (TASK / "entry.py").resolve()
        ):
            raise ValueError("Comparison policy requires the cover task")
        return plan(self.mode)


def register(mode):
    @policy(mode)
    def factory(hold_s=31):
        return WithdrawalPolicy(mode, hold_s)


for mode in PAIR:
    register(mode)

if __name__ == "__main__":
    raise SystemExit(main())
