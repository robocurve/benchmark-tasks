"""Fixed task scene and geometry; metres, kilograms, seconds."""
CONFIG = {'body_height': 0.0192,
 'instruction': 'Press the red block onto the blue block, connecting one stud. '
                'Lift the pair by the red block alone so the blue block is at '
                'least 2 cm above the table for one second. Set the joined '
                'pair down upright and let go for one second. Any of the blue '
                "block's four studs is allowed.",
 'lower_color': (0.16, 0.4, 0.9),
 'lower_mass': 0.01211,
 'lower_shape': (2, 2),
 'lower_xy': (0.26, 0.1),
 'name': 'join-1x1-to-2x2',
 'pitch': 0.016,
 'required_studs': 1,
 'stud_height': 0.0048,
 'table_top': 0.7475,
 'target_offset': (0.008, 0.008),
 'title': 'Join a 1×1 block to a 2×2 block',
 'upper_color': (0.94, 0.27, 0.15),
 'upper_mass': 0.003907,
 'upper_shape': (1, 1),
 'upper_xy': (0.2, 0.2)}
