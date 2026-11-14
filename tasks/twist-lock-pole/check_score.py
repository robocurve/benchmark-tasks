"""The scorer's table on hand-built records, with no simulator: python check_score.py (needs yam_arena importable, or stubs)."""

import task as T

cases = [  # (to_mark_mm, rot_deg, ext_mm, push measured, push passed, off table) -> score
    ((2.0, -0.4, 242.0, True, True, False), 1.0),
    ((9.9, 15.0, 249.9, True, True, False), 1.0),
    ((2.0, -0.4, 242.0, True, False, False), 0.5),
    ((2.0, -80.0, 242.0, False, False, False), 0.5),
    ((-25.0, 0.0, 215.0, False, False, False), 0.25),
    ((-40.0, 0.0, 200.0, False, False, False), 0.0),
    ((-240.0, 0.0, 0.0, False, False, False), 0.0),
    ((2.0, -0.4, 242.0, True, True, True), 0.0),
    ((12.0, -0.4, 252.0, True, True, False), 0.25),
]
bad = 0
for (to_mark, rot, ext, measured, passed, off), want in cases:
    T.CAM.last = {"to_mark_mm": to_mark, "rot_deg": rot, "ext_mm": ext}
    T.CAM.short_mm = 1.0 if measured else None
    T.CAM.passed, T.CAM.off_table = passed, off
    got = T.score(None)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), dict(to_mark=to_mark, rot=rot, ext=ext, measured=measured, passed=passed, off=off), got, "want", want)
print("all pass" if not bad else f"{bad} failed")
