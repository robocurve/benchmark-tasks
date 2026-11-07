"""Carrier, display and starter-tab geometry, in metres.

The carrier is a benchmark-designed nest, not a manufacturer-standard fixture.
The tab's two side bridges leave a central entry notch behind its grip surface.
"""
from dataclasses import dataclass
import numpy as np

TABLE_Z=.7475
CARRIER_CENTER=np.array([.28,-.035,TABLE_Z+.0085])
CARRIER_SIZE=np.array([.110,.350,.017])
GLASS_Z=TABLE_Z+.015
FILM_Z=GLASS_Z+.0005
FILM_SIZE=np.array([.066,.005,.001])  # one 5 mm film strip
CREASE_Y=-.055
SLOT_X=(-.006,.006)  # relative to carrier centre x
SLOT_Y=(-.055,-.049)  # world
TAB_ROOT=np.array([.28,-.05325,FILM_Z-.00475])
TAB_BOUNDS=np.array([.008,.0035,.0105])
TAB_SEAM=np.array([0,-.00175,.00475])


@dataclass(frozen=True)
class Part:
    name: str
    size: tuple
    center: tuple
    color: tuple


def carrier_parts():
    """Each box's local transform under one dynamic carrier root."""
    def part(name,lo,hi,color):
        lo,hi=np.array(lo),np.array(hi)
        return Part(name,tuple(hi-lo),tuple((hi+lo)/2-CARRIER_CENTER),color)
    plastic=(.12,.15,.18);glass=(.12,.20,.25)
    return [part('base',(.225,-.21,TABLE_Z),(.335,.14,TABLE_Z+.003),plastic),
            part('display',(.244,-.190,TABLE_Z+.003),(.316,-.055,GLASS_Z),glass),
            part('left_guard',(.225,-.21,TABLE_Z+.003),(.246,.14,TABLE_Z+.017),plastic),
            part('right_guard',(.314,-.21,TABLE_Z+.003),(.335,.14,TABLE_Z+.017),plastic),
            part('support_flange',(.246,-.049,TABLE_Z+.003),(.314,.14,TABLE_Z+.017),plastic),
            part('slot_left',(.246,-.055,TABLE_Z+.003),(.274,-.049,TABLE_Z+.017),plastic),
            part('slot_right',(.286,-.055,TABLE_Z+.003),(.314,-.049,TABLE_Z+.017),plastic),
            part('back_guard',(.246,-.21,TABLE_Z+.003),(.314,-.191,TABLE_Z+.017),plastic)]


def tab_parts():
    return [Part('stem',(.008,.001,.010),(0,.00125,-.00025),(.03,.2,.85)),
            Part('bridge_left',(.0012,.003,.001),(-.0034,-.00025,.00475),(.03,.2,.85)),
            Part('bridge_right',(.0012,.003,.001),(.0034,-.00025,.00475),(.03,.2,.85))]


def strip_pos(i):
    return np.array([.28,CREASE_Y-(i+.5)*FILM_SIZE[1],FILM_Z])
