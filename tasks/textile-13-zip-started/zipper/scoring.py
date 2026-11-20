"""Award insertion and zipper progress only when the pin was physically seated."""

from dataclasses import dataclass
from .mechanism import END_TOLERANCE, LENGTH, PIN_SEATED_TOLERANCE


@dataclass(frozen=True)
class Reading:
    slider_m: float
    pin_x_m: float
    pin_y_m: float
    pin_z_m: float
    engaged: bool
    released: bool

    @property
    def pin_seated(self):
        return max(abs(self.pin_x_m), abs(self.pin_y_m), abs(self.pin_z_m)) <= PIN_SEATED_TOLERANCE


def zip_complete(reading):
    return bool(reading.engaged and reading.pin_seated and reading.slider_m >= LENGTH - END_TOLERANCE
                and reading.released)


def zip_score(readings):
    if not readings or not readings[-1].engaged:
        return 0.0
    final = readings[-1]
    if not final.pin_seated:
        return 0.0
    if zip_complete(final):
        return 1.0
    return min(0.95, 0.25 + 0.70 * max(0.0, min(final.slider_m / LENGTH, 1.0)))
