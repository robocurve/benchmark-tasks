"""The score without the simulator, from made-up positions. It needs NumPy and yam_arena's package on the path. Run: python test_alarm_score.py"""

from __future__ import annotations

import numpy as np

import alarm_score as s
import alarm_usd as u

DT = 0.05
FAR = np.array([[0.2, 0.3, 1.2], [0.2, 0.3, 1.2], [0.2, -0.3, 1.2], [0.2, -0.3, 1.2]])
PANE = u.RIM + u.PANE_THICK
IN_WINDOW = -0.03


def tip(along: float, across: float, depth: float) -> np.ndarray:
    p = u.on_table(along, across, PANE - depth) + np.array([0.0, 0.0, s.TOP])
    return np.array([p, p, *FAR[2:]])


def expect(name: str, got, want) -> None:
    assert got == want, f"{name}: got {got}, want {want}"
    print(f"ok   {name}: {got}")


def run() -> None:
    c = s.Scorer(DT)
    expect("doing nothing", c.update(FAR, 0.0)["score"], 0.0)
    c = s.Scorer(DT)
    expect("a fingertip 4 mm through the glass", c.update(tip(IN_WINDOW, 0.0, 0.004), 0.0)["score"], s.SCORE_GLASS)
    c = s.Scorer(DT)
    expect("3 mm through, the limit", c.update(tip(IN_WINDOW, 0.0, 0.0031), 0.0)["score"], s.SCORE_GLASS)
    c = s.Scorer(DT)
    expect("a touch, 2 mm through", c.update(tip(IN_WINDOW, 0.0, 0.002), 0.0)["score"], 0.0)
    c = s.Scorer(DT)
    expect("resting on the glass", c.update(tip(IN_WINDOW, 0.0, 0.0), 0.0)["score"], 0.0)
    c = s.Scorer(DT)
    for k in range(60):  # 2 mm/s: slow still breaks it
        r = c.update(tip(IN_WINDOW, 0.0, -0.001 + k * 0.002 * DT), 0.0)
    expect("pushed through slowly, still broken", r["score"], s.SCORE_GLASS)
    c = s.Scorer(DT)
    expect("as deep, beside the window", c.update(tip(IN_WINDOW, 0.05, 0.01), 0.0)["score"], 0.0)
    c = s.Scorer(DT)
    expect("as deep, on the wording above the window", c.update(tip(0.06, 0.0, 0.01), 0.0)["score"], 0.0)

    c = s.Scorer(DT)
    expect("the handle moved with the glass whole earns nothing", c.update(FAR, 0.03)["score"], 0.0)
    c = s.Scorer(DT)
    c.update(tip(IN_WINDOW, 0.0, 0.01), 0.0)
    expect("the glass stays broken", c.update(FAR, 0.0)["score"], s.SCORE_GLASS)
    expect("handle part way", c.update(FAR, 0.0275)["score"], s.SCORE_GLASS)
    r = c.update(FAR, 0.0281)
    expect("pulled gets 1.0 at once", r["score"], s.SCORE_PULLED)
    expect("a pull counts as success", r["success_now"], True)
    expect("it stays pulled", c.update(FAR, 0.0)["score"], s.SCORE_PULLED)

    c = s.Scorer(DT)
    c.update(tip(IN_WINDOW, 0.0, 0.01), 0.03)
    c.reset()
    assert c.broken_at is None and c.pulled_at is None and c.deepest == 0.0
    print("ok   a new trial clears the kept state")


if __name__ == "__main__":
    run()
    print("all passed")
