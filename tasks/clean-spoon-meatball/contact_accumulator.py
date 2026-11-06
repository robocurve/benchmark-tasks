"""Accumulate all substep contacts on device; copy them once per control step."""
import warp as wp


@wp.kernel
def accumulate(counts: wp.array2d(dtype=wp.int32),
               starts: wp.array2d(dtype=wp.int32),
               distances: wp.array(dtype=wp.float32),
               totals: wp.array2d(dtype=wp.int32)):
    i,j=wp.tid()
    count=counts[i,j]
    first=starts[i,j]
    if count>0:
        if first<0 or first+count>distances.shape[0]:
            wp.atomic_max(totals,i,counts.shape[1],1)
        else:
            for k in range(count):
                if distances[first+k]<=0.0005:
                    totals[i,j]+=1
                    break
