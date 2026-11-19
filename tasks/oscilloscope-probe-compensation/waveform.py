"""A compensated passive probe model and a scorer for the acquired trace."""

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from math import exp, isfinite
from statistics import median


@dataclass(frozen=True)
class WaveformResult:
    score: float
    error_fraction: float | None
    frequency_hz: float | None
    vpp: float | None


def waveform_samples(
    angle_rad,
    *,
    samples_per_cycle=256,
    frequency_hz=1000.0,
    source_vpp=5.0,
    nominal_angle_rad=0.0,
    trim_fraction_per_rad=0.5,
):
    """Return 2.5 ms at 1 kHz as tuples of seconds and input-referred volts.

    The 10× divider has a compensation capacitor across its 9 MΩ series
    resistor and a lumped scope and cable capacitance across the 1 MΩ
    termination. The 150 pF load gives 15 pF at the compensated input,
    within the P2220's specified 13–17 pF range. The partition and angle
    dependence are modeling estimates. The source has ideal edges at this
    trace resolution.
    """
    if not all(
        isfinite(value)
        for value in (angle_rad, frequency_hz, source_vpp, nominal_angle_rad, trim_fraction_per_rad)
    ):
        raise ValueError("Probe model parameters must be finite.")
    if frequency_hz <= 0 or source_vpp <= 0 or samples_per_cycle < 32:
        raise ValueError("Use a positive source and at least 32 samples per period.")

    period_s = 1.0 / frequency_hz
    series_ohm, load_ohm = 9e6, 1e6
    load_f = 150e-12
    matched_f = load_f * load_ohm / series_ohm
    trim_multiplier = 1.0 + trim_fraction_per_rad * (angle_rad - nominal_angle_rad)
    series_f = matched_f * max(0.05, trim_multiplier)
    total_f = series_f + load_f
    dc_gain = load_ohm / (series_ohm + load_ohm)
    edge_gain = series_f / total_f
    decay_s = series_ohm * dc_gain * total_f

    # The exponential starts at the steady periodic value left by the last edge.
    transient_v = (edge_gain / dc_gain - 1.0) * source_vpp
    transient_v /= 1.0 + exp(-0.5 * period_s / decay_s)

    sample_count = 5 * samples_per_cycle // 2 + 1
    time_s = tuple(i * period_s / samples_per_cycle for i in range(sample_count))
    volts = []
    for i in range(sample_count):
        phase = (i / samples_per_cycle - 0.125) % 1.0
        high = phase < 0.5
        since_edge_s = (phase if high else phase - 0.5) * period_s
        deviation_v = transient_v * exp(-since_edge_s / decay_s)
        volts.append(source_vpp + deviation_v if high else -deviation_v)
    return time_s, tuple(volts)


def evaluate_waveform(time_s, voltage_v, *, frequency_hz=1000.0, expected_vpp=5.0):
    """Grade one trace using the same data that appears on the scope.

    Use at least two rising and two falling edges with the scale fixed.
    Normalize the largest early-plateau error by the measured plateau span.
    """
    invalid = WaveformResult(0.0, None, None, None)
    if not (isfinite(frequency_hz) and isfinite(expected_vpp) and frequency_hz > 0 and expected_vpp > 0):
        raise ValueError("Expected frequency and voltage must be finite and positive.")
    if len(time_s) != len(voltage_v) or len(time_s) < 48:
        return invalid
    if not all(map(isfinite, time_s)) or not all(map(isfinite, voltage_v)):
        return invalid
    if any(after <= before for before, after in zip(time_s, time_s[1:])):
        return invalid

    period_s = 1.0 / frequency_hz
    if any(after - before > 0.026 * period_s for before, after in zip(time_s, time_s[1:])):
        return invalid

    ordered_volts = sorted(voltage_v)
    lower_quartile = ordered_volts[len(ordered_volts) // 4]
    upper_quartile = ordered_volts[3 * len(ordered_volts) // 4]
    if upper_quartile - lower_quartile < 0.5 * expected_vpp:
        return invalid

    midpoint_v = (lower_quartile + upper_quartile) / 2.0
    edges = []
    for i, (before, after) in enumerate(zip(voltage_v, voltage_v[1:])):
        if (before >= midpoint_v) != (after >= midpoint_v):
            fraction = (midpoint_v - before) / (after - before)
            crossing_s = time_s[i] + fraction * (time_s[i + 1] - time_s[i])
            edges.append((crossing_s, after > before))

    if len(edges) < 4:
        return invalid
    if not all(
        0.4 * period_s <= end - start <= 0.6 * period_s
        for (start, _), (end, _) in zip(edges, edges[1:])
    ):
        return invalid
    periods = [edges[i + 2][0] - edges[i][0] for i in range(len(edges) - 2)]
    if not all(0.95 * period_s <= period <= 1.05 * period_s for period in periods):
        return invalid
    measured_hz = 1.0 / median(periods)

    plateau = {False: [], True: []}
    early = {False: [], True: []}
    for index, (edge_s, high) in enumerate(edges):
        next_s = edges[index + 1][0] if index + 1 < len(edges) else time_s[-1]
        settling_start = edge_s + 0.34 * period_s
        settling_stop = min(edge_s + 0.46 * period_s, next_s - 0.025 * period_s)
        sample_start = bisect_left(time_s, edge_s + 0.02 * period_s)
        sample_stop = bisect_right(time_s, edge_s + 0.34 * period_s)
        if time_s[-1] >= edge_s + 0.34 * period_s:
            early[high].extend(voltage_v[sample_start:sample_stop])
        if settling_stop > settling_start:
            settle_start = bisect_left(time_s, settling_start)
            settle_stop = bisect_right(time_s, settling_stop)
            plateau[high].extend(voltage_v[settle_start:settle_stop])

    if any(not plateau[high] or not early[high] for high in (False, True)):
        return invalid
    low_v, high_v = median(plateau[False]), median(plateau[True])
    vpp = high_v - low_v
    if not 0.8 * expected_vpp <= vpp <= 1.2 * expected_vpp:
        return WaveformResult(0.0, None, measured_hz, vpp)

    error = max(
        max(abs(sample - low_v) for sample in early[False]),
        max(abs(sample - high_v) for sample in early[True]),
    ) / vpp
    score = 0.0
    for cutoff, credit in ((0.03, 1.0), (0.06, 0.75), (0.12, 0.5), (0.20, 0.25)):
        if error <= cutoff:
            score = credit
            break
    return WaveformResult(score, error, measured_hz, vpp)
