PATIENT_LINE = (
    (0.56, 0.19, 0.072),
    (0.54, 0.17, 0.075),
    (0.50, 0.13, 0.07),
    (0.46, 0.09, 0.052),
    (0.43, 0.07, -0.04),
    (0.427, 0.023, -0.051),
    (0.43, -0.018, 0.001),
)


def loop_points(head_z, carriage_z):
    # Endpoints move with their native mounted connectors. The low bend represents slack.
    end = (0.401, -0.085, carriage_z - 0.050)
    return [
        (0.430, -0.018, head_z + 0.001),
        (0.448, -0.029, head_z - 0.018),
        (0.457, -0.039, min(head_z, carriage_z) - 0.062),
        (0.452, -0.057, min(head_z, carriage_z) - 0.082),
        (0.436, -0.073, min(head_z, carriage_z) - 0.082),
        (0.417, -0.082, carriage_z - 0.061),
        end,
    ]

# A separate 100 mL receiver. A 2.5 mm rise per mL corresponds to a 22.57 mm bore.
GAUGE_X, GAUGE_Y, GAUGE_BASE = 0.420, -0.209, 0.842
METRES_PER_ML = 0.0025
BORE = (4e-6 / (3.141592653589793 * METRES_PER_ML)) ** 0.5


def column_geometry(volume_ml):
    height = min(100.0, max(0.0, volume_ml)) * METRES_PER_ML
    return ((GAUGE_X, GAUGE_Y, GAUGE_BASE + height / 2),
            (BORE, BORE, max(0.00001, height)),
            (GAUGE_X, GAUGE_Y, GAUGE_BASE + height))
