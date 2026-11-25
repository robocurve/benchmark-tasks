"""Validate and sum the active patches of a native filtered friction buffer."""
import numpy as np


def filtered_friction_sum(forces, counts, starts, filter_count):
    """Sum only selected contact patches, excluding unused native buffer slots.

    PhysX supplies a packed force buffer plus count/start pairs per sensor and
    filter. This adapter declares one sensor body and concrete, unique filters.
    Empty filters may have a negative start sentinel. Their buffer is not read.
    """
    forces=np.asarray(forces);counts=np.asarray(counts);starts=np.asarray(starts)
    if (forces.ndim!=2 or forces.shape[1]!=3 or filter_count<1
            or counts.shape!=(1,filter_count) or starts.shape!=counts.shape
            or not np.issubdtype(counts.dtype,np.integer)
            or not np.issubdtype(starts.dtype,np.integer) or np.any(counts<0)):
        raise ValueError('invalid native filtered friction buffer shape or indices')
    result=np.zeros(3);used=set()
    for count,start in zip(counts[0],starts[0]):
        if count==0:continue
        start=int(start);end=start+int(count)
        if start<0 or end>len(forces) or used.intersection(range(start,end)):
            raise ValueError('native friction patches are out of bounds or counted twice')
        values=forces[start:end]
        if not np.isfinite(values).all():raise ValueError('nonfinite active native friction force')
        result+=values.sum(axis=0);used.update(range(start,end))
    return result
