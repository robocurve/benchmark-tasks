"""Score an actual seated floor and proof of both sliding hinge locks."""

from dataclasses import dataclass


@dataclass(frozen=True)
class CartState:
    floor_seated: bool
    both_rails_engaged: bool
    both_walls_straight: bool
    supported: bool
    inward_load: tuple[bool, bool]
    hands_clear: bool


class CartCheck:
    def __init__(self):
        self.last_tick = -1
        self._samples = 0
        self.proven = False
        self.value = 0.

    def sample(self, tick: int, state: CartState) -> float:
        if tick == self.last_tick:
            return self.value
        self.last_tick = tick
        floor = state.floor_seated and state.supported
        open_cart = floor and state.both_walls_straight
        locked = open_cart and state.both_rails_engaged
        self._samples = self._samples + 1 if locked and all(state.inward_load) else 0
        self.proven = locked and (self.proven or self._samples >= 15)
        if not floor:
            self.value = 0.
        elif not open_cart:
            self.value = .25
        elif not locked:
            self.value = .50
        else:
            self.value = 1. if self.proven and state.hands_clear else .75
        return self.value
