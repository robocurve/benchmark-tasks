"""Record a demo or direct Dream trial in the same native physical scene.

Saved execution is not automatically a success score. The final material,
contact audit, complete trajectory and photographs must be judged together.
"""
from preview_yam import main


if __name__=='__main__':
    main(default_policy='demo',default_steps=2400,default_review=True)
