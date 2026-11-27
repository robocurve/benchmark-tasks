from inspect_robots.registry import register
from rally_task.embodiment import make_embodiment
from rally_task.policy import make_policy
from inspect_robots.cli import main
register('embodiment','yam_tabletop_rally')(make_embodiment)
register('policy','rally_scripted')(make_policy)
if __name__=='__main__':raise SystemExit(main())
