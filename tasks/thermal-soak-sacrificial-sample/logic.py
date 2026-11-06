"""Hidden state machine for the thermal soak task. Pure Python.

Tracks:
- Plate temperature Tp: rises 5 deg C/s toward 150 when HEAT is on; falls 1.5 deg C/s toward 20 when OFF.
- Two buttons: HEAT (red cap) and OFF (white cap), 8 mm stroke, trigger <= -6.4 mm, re-arm >= -3.5 mm.
- Sample 1 (blue, expendable) & Sample 2 (orange, target):
  - When on heating pad (rest within 30 mm of pad centre): Ts relaxes toward Tp with tau = 6 s.
  - When not on plate: Ts relaxes toward 20 C with tau = 60 s.
  - Core Tc always relaxes toward Ts with tau = 15 s (lag).
  - Fire condition: any sample core > 90 deg C at any time trips FIRE (latched failure).
- Indicators:
  - 9-disc temperature bar shows surface T of the sample currently on the plate (lamp k lit if Ts >= 20 + 10*k C).
  - Status lamp: amber while a sample is on plate and heating; green when success conditions are met; off otherwise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import geometry as g


@dataclass
class ThermalState:
    tp: float = g.PLATE_TEMP_MIN
    heater_on: bool = False
    ts1: float = g.PLATE_TEMP_MIN
    tc1: float = g.PLATE_TEMP_MIN
    ts2: float = g.PLATE_TEMP_MIN
    tc2: float = g.PLATE_TEMP_MIN
    prev_tc2: float = g.PLATE_TEMP_MIN
    dtc2_dt: float = 0.0
    s1_was_on_plate: bool = False
    s1_was_lifted: bool = False
    s1_on_cool_pad: bool = False
    s1_complete: bool = False
    fire_tripped: bool = False
    heat_pressed: bool = False
    off_pressed: bool = False
    heat_count: int = 0
    off_count: int = 0
    bar_lit: tuple[bool, ...] = (False,) * 9
    bar_count: int = 0
    status_lamp: str = "off"  # "off", "amber", "green"


class ThermalLogic:
    """Pure-Python simulation of plate heater, heat transfer, and button detection."""

    def __init__(self) -> None:
        self.state = ThermalState()

    def reset(self) -> None:
        self.state = ThermalState()

    def step(
        self,
        dt: float,
        heat_slider_z: float,
        off_slider_z: float,
        s1_on_plate: bool,
        s1_on_cool: bool,
        s2_on_plate: bool,
        s2_on_cool: bool,
        is_success: bool = False,
    ) -> ThermalState:
        s = self.state

        # 1. Button detection with hysteresis
        # HEAT button
        if heat_slider_z <= g.BUTTON_TRIGGER_Z:
            if not s.heat_pressed:
                s.heat_pressed = True
                s.heat_count += 1
                s.heater_on = True
        elif heat_slider_z >= g.BUTTON_REARM_Z:
            s.heat_pressed = False

        # OFF button
        if off_slider_z <= g.BUTTON_TRIGGER_Z:
            if not s.off_pressed:
                s.off_pressed = True
                s.off_count += 1
                s.heater_on = False
        elif off_slider_z >= g.BUTTON_REARM_Z:
            s.off_pressed = False

        # 2. Plate temperature update
        if s.heater_on:
            s.tp = min(g.PLATE_TEMP_MAX, s.tp + g.PLATE_HEAT_RATE * dt)
        else:
            s.tp = max(g.PLATE_TEMP_MIN, s.tp - g.PLATE_COOL_RATE * dt)

        # 3. Sample 1 thermal update
        if s1_on_plate:
            s.ts1 += (s.tp - s.ts1) / g.TAU_SURF_HEAT * dt
            s.s1_was_on_plate = True
        else:
            s.ts1 += (g.PLATE_TEMP_MIN - s.ts1) / g.TAU_COOL * dt
            if s.s1_was_on_plate:
                s.s1_was_lifted = True

        s.tc1 += (s.ts1 - s.tc1) / g.TAU_CORE * dt

        s.s1_on_cool_pad = s1_on_cool
        if s1_on_cool and s.s1_was_lifted:
            s.s1_complete = True

        # 4. Sample 2 thermal update
        if s2_on_plate:
            s.ts2 += (s.tp - s.ts2) / g.TAU_SURF_HEAT * dt
        else:
            s.ts2 += (g.PLATE_TEMP_MIN - s.ts2) / g.TAU_COOL * dt

        s.prev_tc2 = s.tc2
        s.tc2 += (s.ts2 - s.tc2) / g.TAU_CORE * dt
        s.dtc2_dt = (s.tc2 - s.prev_tc2) / dt if dt > 0 else 0.0

        # 5. Check FIRE zero-gate
        if s.tc1 > g.CORE_FIRE_LIMIT or s.tc2 > g.CORE_FIRE_LIMIT:
            s.fire_tripped = True

        # 6. Temperature bar display
        # Shows surface T of the sample currently on the plate.
        # If no sample is on the plate, but a sample rests on the cool pad, shows that sample's surface T.
        # Otherwise ambient 20 C (0 discs lit).
        cool_sample = None
        if s2_on_cool:
            cool_sample = (s.ts2, s.tc2)
        elif s1_on_cool:
            cool_sample = (s.ts1, s.tc1)

        if s2_on_plate:
            t_disp = s.ts2
        elif s1_on_plate:
            t_disp = s.ts1
        elif cool_sample is not None:
            t_disp = cool_sample[0]
        else:
            t_disp = g.PLATE_TEMP_MIN

        s.bar_lit = tuple(bool(t_disp >= temp_thresh) for temp_thresh in g.BAR_DISC_TEMP)
        s.bar_count = sum(s.bar_lit)

        # 7. Status lamp state
        # Core-dependent observable while resting on cool pad:
        #   amber: Tc < 60 C
        #   green: 60 C <= Tc <= 70 C (target core band)
        #   red:   Tc > 70 C (overheated)
        # When no sample is on cool pad:
        #   amber while sample is on plate and heating
        #   green if final success holds
        #   off otherwise
        if is_success:
            s.status_lamp = "green"
        elif cool_sample is not None:
            tc_val = cool_sample[1]
            if tc_val < g.CORE_BAND_MIN:
                s.status_lamp = "amber"
            elif tc_val <= g.CORE_BAND_MAX:
                s.status_lamp = "green"
            else:
                s.status_lamp = "red"
        elif (s1_on_plate or s2_on_plate) and s.heater_on:
            s.status_lamp = "amber"
        else:
            s.status_lamp = "off"

        return s
