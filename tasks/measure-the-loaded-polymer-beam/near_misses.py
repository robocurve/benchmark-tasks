"""Unknown-history poses cannot earn credit. Stateful protocol cases run in fixture/check_protocol.py."""
import task
CASES=[('do nothing',{},0.,False),
 ('beam centred but no instrument or hand state',{'polymer_beam':task.BEAM.state(pos=(task.X,0,task.BEAM_Z))},0.,False),
 ('beam bent2mm without a first reading',{'polymer_beam':task.BEAM.state(pos=(task.X,0,task.BEAM_Z),joints={'bend':.002})},0.,False),
 ('beam returned without records',{'polymer_beam':task.BEAM.state(pos=task.RACK_A)},0.,False),
 ('beam tilted upside down',{'polymer_beam':((task.X,0,task.BEAM_Z),None,None,(1.,0.,0.,0.))},0.,False)]
