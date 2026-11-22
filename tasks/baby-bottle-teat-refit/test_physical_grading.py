"""Regression examples for the intermediate outcome and an inconclusive bottle inspection."""
import pytest
from physical_grading import grade


@pytest.mark.parametrize('fit,bottle,seams,inspection,expected', [
    ('retained', 'failed', 'unclear', 'incomplete', 0.5),
    ('failed', 'failed', 'unclear', 'incomplete', 0),
    ('unclear', 'failed', 'unclear', 'incomplete', None),
    ('retained', 'held', 'dry', 'complete', 1),
    ('unclear', 'held', 'dry', 'complete', None),
    (None, 'held', 'dry', 'complete', None),
    ('retained', 'cross-threaded', 'dry', 'complete', 0.5),
    ('retained', 'held', 'both', 'complete', 0.5),
    ('retained', 'held', 'outlet-only', 'complete', 1),
    ('retained', 'held', 'bottle-collar', 'complete', 0.5),
    ('retained', 'held', 'teat-base', 'complete', 0.5),
    ('failed', 'held', 'teat-base', 'complete', 0),
    ('retained', 'held', 'unclear', 'complete', None),
    ('retained', 'held', 'dry', 'incomplete', None),
    ('retained', 'unclear', 'dry', 'complete', None),
])
def test_physical_rubric(fit, bottle, seams, inspection, expected):
    assert grade(bottle, seams, inspection, teat_fit=fit)['score'] == expected


def test_spilled_prefill_invalidates_trial():
    assert grade('held', 'dry', 'complete', setup='invalid', teat_fit='retained')['score'] is None
