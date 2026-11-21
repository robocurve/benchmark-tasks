"""Offline checks for the mandoline task: score ladder, cheat strategies, cut rule, safety distances, demo IK replay."""

from __future__ import annotations

import math

import numpy as np


def main(T) -> int:
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan

    n = 0

    def check(cond, msg):
        nonlocal n
        n += 1
        assert cond, msg

    # score ladder (pure)
    p = T.points
    check(p("", 0, False, 0) == 0.0, "nothing")
    check(p("", 1, False, 0) == 0.25, "one slice")
    check(p("", 0, True, 0) == 0.25, "holder seated")
    check(p("", 2, False, 0) == 0.5 and p("", 4, True, 0) == 0.5, "2 to 4")
    check(p("", 5, False, 0) == 0.5, "5 slices without clear hold")
    check(p("", 5, False, T.HOLD_STEPS - 1) == 0.5, "19 clear steps")
    check(p("", 5, False, T.HOLD_STEPS) == 1.0 and p("", 6, True, 99) == 1.0, "full")
    check(p("tipped", 6, True, 99) == 0.0 and p("finger", 1, True, 0) == 0.0, "latched violation")

    # keep-out box and contact with the food (the gripper's whole volume)
    I = np.eye(3)
    m = np.array([T.MX, T.MY, T.M_Z0])
    down = T.grip_rotation(0.0)
    c_pos0 = m + [0.0, T.CARROT_Y0, T.DECK_Z + T.N_DISCS * T.DISC_H]  # the carrot core's origin (its underside)
    top_z = c_pos0[2] + T.CORE_H

    def tcp_at(x, y, z):
        return np.array([T.MX + x, T.MY + y, z])

    def touches(tcp, half):
        return T.cyl_distance(T.gripper_points(tcp, down, half), c_pos0, I, T.CARROT_R, T.CORE_H) < T.TOUCH

    over_blade_low = tcp_at(0.0, T.YB, T.M_Z0 + T.DECK_Z + 0.020 + 0.013)  # pad tips 20 mm over the deck, at the blade
    check(T.in_keepout(T.gripper_points(over_blade_low, down, 0.02), m, I), "a hand low over the blade is in the box")
    over_blade_high = tcp_at(0.0, T.YB, T.M_Z0 + T.DECK_Z + 0.045 + 0.013)  # pad tips 45 mm up: the knob grip
    check(not T.in_keepout(T.gripper_points(over_blade_high, down, 0.02), m, I), "a hand 45 mm up over the blade is clear")
    beside = tcp_at(0.0, T.YB + T.KO + 0.040, T.M_Z0 + T.DECK_Z + 0.020 + 0.013)
    check(not T.in_keepout(T.gripper_points(beside, down, 0.02), m, I), "a hand 40 mm beyond the line is clear")
    # bare gripper near the blade voids; safe handling away from the blade does not
    pinch = tcp_at(0.0, T.CARROT_Y0, top_z - 0.004)
    check(touches(pinch, T.CARROT_R), "a bare pinch on the carrot touches it")
    check(T.food_near_blade(c_pos0, m, I), "start stub is still near the blade")
    check(not T.in_keepout(T.gripper_points(pinch, down, T.CARROT_R), m, I), "...outside the keep-out box, but near-blade contact still voids")
    side = c_pos0 + down[:, 0] * (T.CARROT_R + 0.0005) + [0.0, 0.0, 0.02]
    check(touches(side, 0.002), "a gripper pressing the carrot with the side of a finger touches it")
    check(not T.in_keepout(T.gripper_points(side, down, 0.002), m, I), "...and is outside the box too")
    away = c_pos0 + np.array([0.0, 0.12, 0.0])  # relocated clear of the blade approach
    check(not T.food_near_blade(away, m, I), "carrot moved away from the blade is not near-blade")
    over = tcp_at(0.0, T.CARROT_Y0, top_z + 0.06)
    check(not touches(over, 0.02), "a hand well above the carrot does not touch it")
    holder_grip = tcp_at(0.0, T.CARROT_Y0, top_z + T.KNOB_GRIP - T.COLLAR_H)  # the knob grip with the holder seated
    check(not touches(holder_grip, 0.011), "the knob grip with the holder seated does not touch the carrot")
    check(not T.in_keepout(T.gripper_points(holder_grip, down, 0.011), m, I), "...nor is it in the box")

    # mandoline kept in place
    check(T.mandoline_violation(m, I) == "", "at rest")
    check("moved" in T.mandoline_violation(m + [0.06, 0, 0], I), "skidded 6 cm")
    tilt = np.array([[1, 0, 0], [0, math.cos(0.5), -math.sin(0.5)], [0, math.sin(0.5), math.cos(0.5)]])
    check("tilted" in T.mandoline_violation(m, tilt), "tipped 29 deg")

    # cut rule: a disc starting behind the blade, pushed across it pressed, then freed; unpressed or unarmed: nothing
    def run(path, fn):
        cs, freed, drag = T.fresh_cut(), False, 0.0
        for y, f in zip(path, fn):
            freed, sp = T.cut_update(cs, (0.0, y, T.DECK_Z), f)
            drag = max(drag, sp)
            if freed:
                break
        return freed, cs, drag

    y0 = T.YB + T.CARROT_R + 0.002
    ys = list(np.arange(y0, T.YB - T.CARROT_R - 0.004, -0.0025))
    freed, cs, drag = run(ys, [3.0] * len(ys))
    check(freed, "pressed stroke frees the slice")
    check(0 < drag <= 0.25, "stroke speed sane")
    freed, cs, _ = run(ys, [0.5] * len(ys))
    check(not freed and sum(cs["bins"]) == 0, "light touch cuts nothing")
    start_in_cut = list(np.arange(T.YB, T.YB - 0.03, -0.0025))
    freed, cs, _ = run(start_in_cut, [3.0] * len(start_in_cut))
    check(not freed, "a disc never armed behind the blade is not cut")
    both = ys + ys[::-1] + ys
    freed, cs, _ = run(ys[: len(ys) // 2], [3.0] * (len(ys) // 2))
    check(not freed and 0 < sum(cs["bins"]) < T.FREE_BINS, "half a stroke is a partial cut")
    check(T.drag_force(0.0) == 0.0 and T.drag_force(1e-4) > 0.0 and T.drag_force(5.0) == T.CUT_FORCE_N, "cutting force ramps in at tiny speeds, full above V0")
    # the mandoline's friction capacity (static 0.475 x (0.35 kg x 9.81 + a 3 N press) = 3.0 N) is under the cutting force
    # a slow stroke cannot cut for free: every step that fills a bin carries the cutting force, whatever the speed
    for step in (0.0001, 0.0005, 0.002):
        cs, freed, bare = T.fresh_cut(), False, 0
        y = T.YB + T.CARROT_R + 0.002
        iters = 0
        while y > T.YB - T.CARROT_R - 0.01 and iters < 5000:
            before = sum(cs["bins"])
            freed, sp = T.cut_update(cs, (0.0, y, T.DECK_Z), 3.0)
            if sum(cs["bins"]) > before and T.drag_force(sp) == 0.0:
                bare += 1
            y -= step
            iters += 1
            if freed:
                break
        check(freed and bare == 0, f"a stroke of {step * 1000:.1f} mm per step frees the slice only under the cutting force")
    # the unbraced mandoline cannot take that force: one hand's push moves it forward
    check(0.475 * (T.M_MASS * 9.81 + 3.0) < T.CUT_FORCE_N, "unbraced mandoline skids even with a 3 N press")

    # holder seated
    c_pos = np.array([0.33, -0.03, 0.85])
    h_pos = c_pos + [0.001, 0, T.CORE_H - T.COLLAR_H]
    check(T.holder_seated(h_pos, I, c_pos, I), "seated")
    check(not T.holder_seated(h_pos + [0.02, 0, 0], I, c_pos, I), "beside the carrot")
    check(not T.holder_seated(h_pos + [0, 0, 0.02], I, c_pos, I), "lifted off")

    # slices in the bowl
    check(T.in_bowl((T.BOWL_X, T.BOWL_Y, T.BOARD_TOP + 0.01)), "in the bowl")
    check(not T.in_bowl((T.BOWL_X + 0.2, T.BOWL_Y, T.BOARD_TOP + 0.01)), "on the board")
    check(not T.in_bowl((T.BOWL_X, T.BOWL_Y, T.BOARD_TOP + 0.2)), "held above the bowl")

    # clear of parts: gripper open far away vs closed / near
    arms_far = {"left": (m + [0.0, 0.40, 0.2], I @ np.diag([1, -1, -1]), 0.04),
                "right": (m + [0.0, -0.40, 0.2], I @ np.diag([1, -1, -1]), 0.04)}
    poses = {"mandoline": (m, I)}
    check(T.clear_of_parts(arms_far, poses), "clear")
    near = dict(arms_far)
    near["left"] = (m + [0.0, 0.17, 0.08], I @ np.diag([1, -1, -1]), 0.04)
    check(not T.clear_of_parts(near, poses), "hand on the handle is not clear")
    closed = dict(arms_far)
    closed["left"] = (m + [0.0, 0.40, 0.2], I @ np.diag([1, -1, -1]), 0.0)
    check(not T.clear_of_parts(closed, poses), "closed gripper is not open")

    # demo: IK replay within limits, no jumps
    for arm, wps in T.demo().items():
        plan = ArmPlan(wps, station.BASES[arm])
        t, room = 0.0, 9.0
        while t < plan.duration:
            plan.action7(t)
            room = min(room, float(np.minimum(plan.q - ik.LOWER, ik.UPPER - plan.q).min()))
            t += 0.05
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg, joint room {room:.3f} rad")
        check(plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 1.0, f"{arm} IK replay")
        check(room > 0.02, f"{arm} joint limits")
        check(plan.duration * 20 < 2400, "plan within the run limit")
    print(f"selftest: {n} checks pass")
    return 0
