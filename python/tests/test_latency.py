"""The profiling module's own contracts. None of this needs a camera.

Importing it at all is one of them: the reduction and the frame source must load
with numpy alone, because `dev` carries no OpenCV and only the flashing half of
`latency` needs it.
"""

import numpy as np
import pytest

from insta360_driver.latency import Flash, Sample, reduce_frame, report


def test_a_frame_reduces_to_the_size_asked_for():
    small = reduce_frame(np.full((720, 720, 3), 200, np.uint8), 240)
    assert small.shape == (240, 240, 3)
    assert small.dtype == np.uint8


def test_a_mono_frame_keeps_its_single_channel():
    assert reduce_frame(np.full((720, 720), 200, np.uint8), 240).shape == (240, 240)


def test_the_reduction_is_a_block_mean_that_rounds_half_up():
    # 2x2 blocks, so a block mean can land exactly on a half and the rounding is
    # the thing under test: two 1s in four pixels is 0.5 and must come back 1,
    # while one 1 in four is 0.25 and must come back 0.
    image = np.zeros((4, 4), np.uint8)
    image[0, 0] = image[0, 1] = 1
    image[2, 2] = 1
    small = reduce_frame(image, 2, radius=1.0)
    assert small[0, 0] == 1
    assert small[1, 1] == 0


def test_outside_the_disc_is_blanked_and_the_centre_is_not():
    small = reduce_frame(np.full((720, 720, 3), 200, np.uint8), 240)
    assert (small[0, 0] == 0).all()
    assert (small[-1, -1] == 0).all()
    assert (small[120, 120] == 200).all()


def test_a_wider_radius_keeps_the_corners():
    small = reduce_frame(np.full((720, 720, 3), 200, np.uint8), 240, radius=1.0)
    assert (small[0, 0] == 200).all()


@pytest.mark.parametrize("size", [0, 7, 250, 1024])
def test_a_size_that_is_not_a_whole_block_is_refused(size):
    # A resample would be silent, slower, and not what the corpus was built with,
    # so a size that does not divide the frame raises instead of resizing.
    with pytest.raises(ValueError, match="whole blocks"):
        reduce_frame(np.zeros((720, 720, 3), np.uint8), size)


def test_a_frame_that_is_not_square_is_refused():
    with pytest.raises(ValueError, match="whole blocks"):
        reduce_frame(np.zeros((480, 720, 3), np.uint8), 240)


def test_reporting_too_few_samples_says_so_rather_than_dividing_by_zero(capsys):
    report([Sample(0.15, 0.12, 0.03, 0.0)], warmup=10, period=0.033, sensor_delay=0.12)
    assert "too few samples" in capsys.readouterr().out


def test_reporting_names_every_stage_and_the_delay_it_checks(capsys):
    samples = [Sample(0.15, 0.12, 0.03, 0.001)] * 8
    report(samples, warmup=2, period=0.033, sensor_delay=0.12)
    printed = capsys.readouterr().out
    for stage in ("glass -> visible", "glass -> arrival", "arrival -> visible"):
        assert stage in printed
    assert "sensor_delay is 120 ms" in printed
    # The biases are the difference between a number and a number worth trusting.
    assert "16.5 ms" in printed


def test_the_flash_defaults_hold_a_marker_for_about_a_frame():
    flash = Flash()
    assert flash.period == pytest.approx(1 / 30, abs=0.002)
    assert flash.marker_px + 2 * flash.border_px <= 720
