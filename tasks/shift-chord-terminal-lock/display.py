"""Move pre-created pixels for cap legends, card glyphs, screen buffer and state lamp."""
import warnings
import traceback
import numpy as np
import geometry as g

class TerminalDisplay:
    def __init__(self):
        self.markers=None
        self.disabled=False
    def reset(self,environment):
        self.update(environment,dict(buffer='',locked=False),None)
    def update(self,environment,ledger,deck):
        if environment is None or self.disabled: return
        try:
            import parts
            if self.markers is None:
                import isaaclab.sim as sim
                from isaaclab.markers import VisualizationMarkers,VisualizationMarkersCfg
                colours=(g.INK,g.CREAM,g.GREEN,g.RED)
                self.markers=VisualizationMarkers(VisualizationMarkersCfg(prim_path='/Visuals/terminal_pixels',markers={'px'+str(i):sim.CuboidCfg(size=(1,1,1),visual_material=sim.PreviewSurfaceCfg(diffuse_color=c)) for i,c in enumerate(colours)}))
            xyz=np.tile([0.,0.,-10.],(352,1))
            scale=np.tile([.001,.001,.001],(352,1))
            indices=np.zeros(352,dtype=np.int64)
            origin=parts._to_numpy(environment.unwrapped.scene.env_origins[0])
            def glyph(char,base,pitch,start,vertical=False,colour=0):
                for row,bits in enumerate(g.GLYPHS[char]):
                    for col,value in enumerate(bits):
                        i=start+row*5+col
                        if value=='1':
                            delta=np.array([0,(2-col)*pitch,(2-row)*pitch]) if vertical else np.array([(2-row)*pitch,(2-col)*pitch,0])
                            xyz[i]=base+delta+origin
                        scale[i]=(.0005,pitch*.9,pitch*.9) if vertical else (pitch*.9,pitch*.9,.0005)
                        indices[i]=colour
            for i,(key,xy) in enumerate(g.KEYS.items()):
                char='A' if key=='a' else 'L' if key=='l' else key
                q=0 if deck is None else deck.sliders.get('digit7' if key=='7' else key,0)
                glyph(char,np.array([*xy,g.TABLE_TOP+g.CAP_TOP_Z+q+.00035]),.0038,i*25)
            for i,char in enumerate(g.CODE):
                glyph(char,np.array([g.CARD_XY[0],g.CARD_XY[1]+(.045-i*.045),g.TABLE_TOP+g.CARD_SIZE[2]+.00035]),.007,200+i*25)
            for i,char in enumerate(ledger['buffer'][:3]):
                glyph(char,np.array([.3545,-.095+i*.045,g.TABLE_TOP+.110]),.0045,275+i*25,True,1)
            for i in (350,351):
                scale[i]=(.001,.020,.020)
                indices[i]=2 if i==350 else 3
            xyz[351 if ledger['locked'] else 350]=np.array([.3545,.065,g.TABLE_TOP+.110])+origin
            self.markers.visualize(translations=xyz.astype(np.float32),scales=scale.astype(np.float32),marker_indices=indices)
        except ImportError:
            self.disabled=True
        except Exception as exc:
            self.disabled=True
            warnings.warn('Terminal display unavailable: '+repr(exc)+' '+traceback.format_exc().replace(chr(10),' | '))

DISPLAY=TerminalDisplay()
