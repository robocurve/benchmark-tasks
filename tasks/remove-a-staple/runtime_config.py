"""Explicit development reset/contact choices, frozen before scene creation."""
RESET_PROFILE='floating-reference'
MESH_PROFILE='baseline'
REFINED_CONTACT=False
MAX_DEPENETRATION_M_S=None
PAPER_PROFILE='office-80gsm'
PANEL_SPACING_M=.012


def configure(reset_profile,mesh_profile,refined_contact,max_depenetration_m_s):
    global RESET_PROFILE,MESH_PROFILE,REFINED_CONTACT,MAX_DEPENETRATION_M_S
    if reset_profile not in ('floating-reference','supported-edge-candidate','pocket-support-candidate'):raise ValueError('Unknown reset profile')
    if mesh_profile not in ('baseline','coarse','clearance-candidate'):raise ValueError('Unknown mesh profile')
    if max_depenetration_m_s is not None and not .01<=max_depenetration_m_s<=1.:raise ValueError('Invalid contact correction cap')
    RESET_PROFILE=reset_profile;MESH_PROFILE=mesh_profile
    REFINED_CONTACT=bool(refined_contact);MAX_DEPENETRATION_M_S=max_depenetration_m_s
