"""A 2 ft length of green Unistrut P1000 strut channel is bolted open side up to the bench in front of the arms,
left to right; a strip of red tape across its near wall and lip, just right of the middle, marks where a fitting
goes. In front of it stand the installer's parts, not assembled, on two wooden parts blocks: a Unistrut P1010 1/2-13
channel spring nut standing on a post (its spring in the post's pocket), and a zinc Unistrut P1064 square plate
fitting (1-5/8 in, 1/4 in thick) lying on a block with a 1/2-13 x 15/16 in hex head bolt dropped through its hole (not
in any nut). Goal: the fitting fastened to the strut at the red mark with the spring nut and bolt, the bolt left
hand-snug (not torqued). Constraints: do not drop any part, and let go of them when done. The usual way (not given to
the robot): hold the nut, lay the fitting on it, start the bolt through the fitting into the nut by hand square to it
(two or more turns; crooked, it cross-threads), push the nut edge-on through the slot, turn it a quarter turn
clockwise so its grooves catch the lips, slide it to the mark, run the bolt down hand-snug.

Task file for yam-isaac-lab-arena (commit 0059d02); see README.md. ``SNC_CONTROL=<seconds>`` forces the after-release
check that many seconds into the run whatever the robot has done (control runs).
"""

from __future__ import annotations

import os

from strutnut_sim import station as hb

CONTROL_AT_S = float(os.environ.get("SNC_CONTROL", "0") or 0)


def _props():
    from strutnut_sim.task import all_props

    return all_props()


def _task():
    from strutnut_sim.task import StrutNutTask

    return StrutNutTask()


def _test_phase(view, task):
    st = task.state(view)
    phase = int(st.phase[0])
    if phase >= 2 and not getattr(task, "_printed", False):
        task._printed = True
        print("[strut-nut] REPORT", task.episode_report(view, 0), flush=True)
    return "idle" if phase == 0 else ("done" if phase >= 2 else "running")


def _control(view, task):
    if CONTROL_AT_S > 0:
        st = task.state(view)
        if int(st.scorer.step[0]) == round(CONTROL_AT_S / view.step_dt):
            task.force_test(view)


STATION = hb.install(hb.StationSpec(
    props=_props, task=_task, objects=("stand", "kit", "fitting"),
    pose_modules=("strutnut_sim.task",), on_step=_control, test_phase=_test_phase,
    physics_hz=float(os.environ.get("SNC_PHYSICS_HZ", "480")),
))
SCENE, OBJECTS, SUCCESS_HOLD_S = STATION.SCENE, STATION.OBJECTS, STATION.SUCCESS_HOLD_S
success = STATION.success


def score(objects):
    """The reported score (strutnut_sim.scorer.Scorer): milestone / 100 (0.2 the bolt started 2 turns or more into the
    nut by the robot, square; 0.4 also the spring nut seated under the lips; 0.6 also the bolt within 3 mm of the red
    mark; 0.8 also the bolt engaged 5 turns or more; 1.0 the after-release pull check passed: 20 N along the channel
    moves the fitting less than 1 mm); 0 for the whole trial once a part has been dropped, the nut is loose in the
    channel (not on the bolt) or the bolt has cross-threaded, whatever was reached before or after."""
    report = STATION.report or {}
    return float(report.get("score", 0.0))
