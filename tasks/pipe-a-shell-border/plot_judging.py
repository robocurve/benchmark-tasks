"""Plot measured final geometry; these figures are not simulator photographs."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from scoring import _peaks


def plot(directory, output):
    with np.load(directory/'material.npz', allow_pickle=False) as state:
        x, y = state['x'], state['y']
        dx = float(state['dx'])
        height = state['volume']/dx**2
    audit = json.loads((directory/'audit.json').read_text())['assessment']
    shells = [s for s in audit['shells'] if s['valid']]
    if audit.get('piping_direction') != '+Y':
        raise ValueError('This explanatory plot currently supports the recorded +Y demo only')
    cross = abs(x-.235) <= .009
    active = abs(y) <= .047
    xx, yy = (x[cross]-.235)*1000, y[active]*1000
    h = height[cross][:,active]*1000
    fig = plt.figure(figsize=(12,9), layout='constrained')
    grid = fig.add_gridspec(3,3,height_ratios=(1.3,1,1))
    ax = fig.add_subplot(grid[0,:])
    image = ax.pcolormesh(yy,xx,h,shading='nearest',cmap='magma',vmin=0,vmax=8)
    ax.contour(yy,xx,h,levels=[.5],colors=['cyan'],linewidths=.6)
    ax.axvline(-45,color='white',linestyle='--',linewidth=1)
    ax.axvline(45,color='white',linestyle='--',linewidth=1)
    ax.set(xlabel='Along the marked strip (mm)',ylabel='Across the strip (mm)',
           title='Measured deposited surface — cyan outline is 0.5 mm high')
    fig.colorbar(image,ax=ax,label='Height above cake (mm)',shrink=.8)
    evidence=[]
    for index,shell in enumerate(shells[:6]):
        start=shell['head_y_mm']
        stop=shells[index+1]['head_y_mm'] if index+1<len(shells) else 47.
        candidates=np.flatnonzero((yy>=start-2)&(yy<stop))
        choices=[]
        for column in candidates:
            profile=h[:,column]
            peaks=_peaks(profile,minimum=1.,spacing=max(1,round(.0015/dx)),prominence=.3)
            for i,left in enumerate(peaks):
                for right in peaks[i+1:]:
                    dip=min(profile[left],profile[right])-float(profile[left:right+1].min())
                    if dip>=.3:choices.append((dip,column,left,right))
        ax=fig.add_subplot(grid[1+index//3,index%3])
        if not choices:
            ax.text(.5,.5,'No qualifying ridge pair',ha='center',transform=ax.transAxes)
            continue
        dip,column,left,right=max(choices)
        profile=h[:,column]
        ax.plot(xx,profile,color='#a32865',linewidth=2)
        ax.scatter(xx[[left,right]],profile[[left,right]],color='#153b50',s=20,zorder=3)
        ax.set(xlim=(-7,7),ylim=(0,8),xlabel='Across strip (mm)',ylabel='Height (mm)',
               title=f'Shell {index+1}: section at y={yy[column]:.2f} mm')
        ax.grid(alpha=.2)
        ax.text(.03,.96,f'Peak separation {xx[right]-xx[left]:.1f} mm\nDip {dip:.2f} mm',
                va='top',transform=ax.transAxes,fontsize=9)
        evidence.append({'shell':index+1,'section_y_mm':float(yy[column]),
                         'peak_separation_mm':float(xx[right]-xx[left]),'dip_mm':float(dip)})
    fig.suptitle('Final icing geometry from native trial data\n'
                 'Illustrative sections; continuous ridge length is checked separately by the scorer',fontsize=13)
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=180)
    plt.close(fig)
    output.with_suffix('.json').write_text(json.dumps({
        'kind':'measured_geometry_plot_not_photograph','sections':evidence},indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    plot(args.directory,args.out)
