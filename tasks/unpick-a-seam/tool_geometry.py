"""Fixed seam-ripper geometry in the handle frame, in metres.

The tool lies along +X with its fork in the XZ plane. The square handle can
rest in this orientation without a fixture; the long point is below the edge.
Only the blade body supplies cutting-force measurements.
"""
import numpy as np

HANDLE_SIZE=(.070,.012,.012)
BLADE_OFFSET=(.050,0.,0.)
BLADE_SIZE=(.001,.0005,.004)
METAL_PARTS=(
    ('Shank',(.0175,.0005,.001),(.04075,0.,0.)),
    ('ForkRoot',(.0025,.0005,.0062),(.04825,0.,0.)),
)
# Axial start, length, vertical offset, radius. Capsule height is the length
# between hemisphere centres, so its physical tip extends another radius.
GUARDS=((.048,.014,-.0013,.00025),(.048,.009,.0026,.0005))
# A fine long point fits between the thread bridge and the fabric. Its
# 0.5 mm diameter is explicit geometry, not a collision exemption.


def blade_edge_samples(count=9):
    """The +X face's centre cutting line, matching the physical blade."""
    if count<2:raise ValueError('At least two samples are required')
    return np.column_stack((np.full(count,BLADE_SIZE[0]/2),np.zeros(count),
                            np.linspace(-BLADE_SIZE[2]/2,BLADE_SIZE[2]/2,count)))
