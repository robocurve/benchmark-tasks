"""Contact-order independent rally rules; usable without a simulator."""
from dataclasses import dataclass

@dataclass
class RallyState:
    phase: str = 'table'
    hits: int = 0
    cycles: int = 0
    invalid: bool = False
    done: bool = False


def apply_contacts(state, kinds, position, *, table, board, robot_contact=False,
                   blade=True, held=True):
    """Consume newly entered contacts. Invalid contact wins; a miss is absorbing.

    Several surfaces first touched in one tick are an ambiguous/double contact,
    so that rally ends rather than manufacturing an order between them.
    """
    if state.done:
        return
    kinds = set(kinds)
    if robot_contact or ('paddle' in kinds and (not blade or not held)):
        state.invalid = state.done = True
        return
    if not kinds:
        return
    if len(kinds) != 1:
        state.done = True
        return
    kind = next(iter(kinds))
    x, y, z = position
    if kind == 'paddle':
        if state.phase != 'paddle':
            state.done = True
            return
        if state.hits:
            state.cycles += 1
        state.hits += 1
        state.phase = 'wall'
        state.done = state.cycles >= 3
    elif kind == 'wall':
        front = x < board - .005 and -.2 <= y <= .4
        height = table + .015 <= z <= table + .285
        if state.phase != 'wall' or not front or not height:
            state.done = True
        else:
            state.phase = 'table'
    elif kind == 'table':
        if state.phase != 'table' or abs(z - (table + .02)) > .008:
            state.done = True
        else:
            state.phase = 'paddle'
    else:
        state.done = True


def score(state):
    return 0. if state.invalid else min(1., state.cycles / 3)
