"""Sampled video-equivalent gates for a core's hopper-to-bin transfer."""
from dataclasses import dataclass
from math import isfinite, radians, sin

# 15 degrees of downward pitch at the actual lip. At 20 Hz allow 1 s
# from mouth crossing to rim crossing, well beyond a short gravity fall.
TIP_SLOPE = sin(radians(15.0))
MAX_FLIGHT_STEPS = 20
BOUNDARY_TOL = 0.002


@dataclass(frozen=True)
class TransferSample:
    pan: tuple[float, float, float]
    bin: tuple[float, float, float]
    lip_in_bin: tuple[float, float, float]
    mouth_down_slope: float


class CoreTransfer:
    def __init__(self, pan_half_inner, pan_rear_inner, pan_front, pan_height,
                 bin_half_inner_x, bin_half_inner_y, bin_height):
        self.pan_half_inner = pan_half_inner
        self.pan_rear_inner = pan_rear_inner
        self.pan_front = pan_front
        self.pan_height = pan_height
        self.bin_half_inner_x = bin_half_inner_x
        self.bin_half_inner_y = bin_half_inner_y
        self.bin_height = bin_height
        self.previous = None
        self.departure_step = None
        self.entered_step = None

    @staticmethod
    def _crossing(previous, current, index, plane):
        fraction = (plane - previous[index]) / (current[index] - previous[index])
        return tuple(a + fraction * (b - a) for a, b in zip(previous, current))

    def _inside_pan_section(self, point):
        x, _, z = point
        return abs(x) <= self.pan_half_inner + BOUNDARY_TOL and 0.0 <= z <= self.pan_height

    def _inside_opening(self, point):
        x, y, _ = point
        return abs(x) <= self.bin_half_inner_x and abs(y) <= self.bin_half_inner_y

    def _crossed_tipped_mouth(self, previous, sample):
        if not (
            self.pan_rear_inner <= previous.pan[1] <= self.pan_front < sample.pan[1]
            and self._inside_pan_section(previous.pan)
            and sample.mouth_down_slope >= TIP_SLOPE
            and self._inside_opening(sample.lip_in_bin)
            and sample.lip_in_bin[2] >= self.bin_height
        ):
            return False
        mouth_point = self._crossing(previous.pan, sample.pan, 1, self.pan_front)
        return self._inside_pan_section(mouth_point)

    def update(self, step, sample):
        if not all(isfinite(value) for value in (
            *sample.pan, *sample.bin, *sample.lip_in_bin, sample.mouth_down_slope
        )):
            self.previous = None
            self.departure_step = self.entered_step = None
            return

        previous, self.previous = self.previous, sample
        if previous is None:
            return
        if self.entered_step is not None:
            if not self._inside_opening(sample.bin) or sample.bin[2] > self.bin_height + BOUNDARY_TOL:
                self.entered_step = self.departure_step = None

        if self.entered_step is not None:
            return
        if self._crossed_tipped_mouth(previous, sample):
            self.departure_step = step
        elif self.departure_step is not None and (
            step - self.departure_step > MAX_FLIGHT_STEPS
            or (self.pan_rear_inner <= sample.pan[1] <= self.pan_front
                and self._inside_pan_section(sample.pan))
        ):
            self.departure_step = None

        if self.departure_step is not None and previous.bin[2] >= self.bin_height > sample.bin[2]:
            crossing = self._crossing(previous.bin, sample.bin, 2, self.bin_height)
            if self._inside_opening(crossing) and self._inside_opening(sample.bin):
                self.entered_step = step
            else:
                self.departure_step = None
