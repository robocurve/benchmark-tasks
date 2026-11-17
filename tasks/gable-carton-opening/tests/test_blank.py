"""Manufacturing geometry consistency, not evidence of a fabricated sample."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from make_blank import design,validate

def test_full_net_matches_existing_folded_mechanism():
    p,t,s,_=design();_,r=validate(p,t,s)
    assert r['checked_hinges']==12 and r['fold_sweep']['configurations']==302
    assert r['total_simulated_mass_g']==pytest.approx(522.7010321946)
    assert not r['physical_fabrication_verified']

def test_short_lip_or_wrong_slant_is_rejected():
    for name in ('carton_inner_fin_minus','carton_spout'):
        p,t,s,_=design();p[name][-1,1]-=2
        with pytest.raises(AssertionError):validate(p,t,s)

def test_missing_base_attachment_breaks_connected_net():
    p,t,s,_=design();p['floor'][:,1]-=1
    with pytest.raises(AssertionError):validate(p,t,s)
