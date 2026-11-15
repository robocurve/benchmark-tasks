"""Lossless float32 native cloth telemetry beside lightweight JSON records.

This changes recording only. Every control-step native node and velocity is
preserved, including failures. It supplies no feedback to a policy.
"""
from dataclasses import fields
from pathlib import Path
import numpy as np


class DenseClothTrace:
    def __init__(self,folder,specs,frames):
        self.folder=Path(folder);self.arrays={};self.count=0
        for name,spec in specs.items():
            for field in ('nodes','velocities'):
                filename=f'{name}-{field}.npy'
                self.arrays[name,field]=np.lib.format.open_memmap(self.folder/filename,mode='w+',dtype=np.float32,shape=(frames,len(spec.mesh.rest),3))

    def record(self,state,step,encode):
        result={}
        for name,item in state.items():
            if (name,'nodes') not in self.arrays:
                result[name]=encode(item);continue
            result[name]={f.name:encode(getattr(item,f.name)) for f in fields(item) if f.name not in ('nodes','velocities')}
            result[name]['dense_cloth_arrays']={}
            for field in ('nodes','velocities'):
                value=np.asarray(getattr(item,field))
                if value.dtype!=np.float32:raise ValueError('Dense recording requires native float32 telemetry; refusing precision loss')
                self.arrays[name,field][step]=value
                result[name]['dense_cloth_arrays'][field]=dict(file=f'{name}-{field}.npy',frame=step)
        self.count=max(self.count,step+1)
        return result

    def flush(self):
        for array in self.arrays.values():array.flush()


_CACHE={}


def cloth_array(folder,row,name,field='nodes'):
    item=row['objects'][name]
    if field in item:return np.asarray(item[field])
    reference=item['dense_cloth_arrays'][field]
    path=(Path(folder)/reference['file']).resolve()
    if path.parent!=Path(folder).resolve():raise ValueError('Native array reference must remain inside the trial')
    if path not in _CACHE:_CACHE[path]=np.load(path,allow_pickle=False,mmap_mode='r')
    if reference['frame']!=row['step']:raise ValueError('Native array frame does not match the trace')
    return _CACHE[path][reference['frame']]
