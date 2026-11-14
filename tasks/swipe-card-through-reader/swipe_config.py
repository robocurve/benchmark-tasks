"""Lazy Isaac config factory in a normally imported, registered module."""


def swipe_config():
    import isaaclab.sim as sim
    from isaaclab.utils import configclass

    # task_file.load() does not register its task module in sys.modules.
    # configclass merges SpawnerCfg's string annotations before dataclass looks
    # up the subclass module, so define the subclass in this imported helper.
    @configclass
    class CardCfg(sim.SpawnerCfg):
        card_name: str = ""

    return CardCfg
