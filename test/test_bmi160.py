from types import SimpleNamespace

import pytest

from klippy.extras.bmi160 import BMI160, SCALE


@pytest.mark.parametrize("raw_samples", [[], [(1.23456789, 2048, -1024, 512)]])
def test_batch_converts_reader_samples_and_reports_total_overflows(raw_samples):
    sensor = BMI160.__new__(BMI160)
    sensor.axes_map = [(1, -SCALE), (0, SCALE), (2, SCALE)]
    sensor.last_error_count = 2
    sensor.ffreader = SimpleNamespace(
        pull_samples=lambda: (list(raw_samples), 1),
        get_total_overflows=lambda: 7,
    )

    result = sensor._process_batch(2.0)

    if not raw_samples:
        assert result == {}
    else:
        assert result == {
            "data": [(1.234568, 4903.325, 9806.65, 2451.6625)],
            "errors": 2,
            "overflows": 7,
        }
