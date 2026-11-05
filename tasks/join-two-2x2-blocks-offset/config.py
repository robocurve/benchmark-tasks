"""Fixed task scene and geometry; metres, kilograms, seconds."""
CONFIG = {'body_height': 0.0192,
 'instruction': 'Press the red block onto the blue block, connecting exactly '
                'two neighbouring studs. Lift the pair by the red block alone '
                'so the blue block is at least 2 cm above the table for one '
                'second. Set the joined pair down upright and let go for one '
                'second. Either row or column is allowed; leave the other two '
                'blue studs exposed.',
 'lower_color': (0.16, 0.4, 0.9),
 'lower_mass': 0.01211,
 'lower_shape': (2, 2),
 'lower_xy': (0.26, 0.1),
 'name': 'join-two-2x2-blocks-offset',
 'pitch': 0.016,
 'required_studs': 2,
 'stud_height': 0.0048,
 'table_top': 0.7475,
 'target_offset': (0.016, 0.0),
 'title': 'Join two 2×2 blocks with two studs overlapping',
 'upper_color': (0.94, 0.27, 0.15),
 'upper_mass': 0.01211,
 'upper_shape': (2, 2),
 'upper_xy': (0.2, 0.2)}
