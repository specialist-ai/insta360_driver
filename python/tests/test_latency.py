"""The profiling module's own contracts. None of this needs a camera.

Importing it at all is one of them: the reduction and the frame source must load
with numpy alone, because `dev` carries no OpenCV and only the flashing half of
`latency` needs it.
"""

import numpy as np
import pytest

from insta360_driver.latency import Flash, Sample, histogram, print_histogram, reduce_frame, report


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


def test_the_bins_are_ten_milliseconds_wide_and_land_on_round_edges():
    # Edges start at 0 rather than at the smallest sample, so two runs bin the
    # same way and can be read against each other.
    assert histogram([0.152, 0.157, 0.163]) == [(150.0, 2), (160.0, 1)]


def test_an_empty_bin_between_two_clusters_is_kept():
    # The gap is the finding; dropping the empty bin would hide it.
    assert histogram([0.151, 0.181]) == [(150.0, 1), (160.0, 0), (170.0, 0), (180.0, 1)]


def test_a_negative_sample_bins_below_zero():
    # shutter_error is signed, and a stamp ahead of the marker must not fold onto
    # the 0-10 bin.
    assert histogram([-0.004, 0.002]) == [(-10.0, 1), (0.0, 1)]


def test_the_bin_width_is_settable():
    assert histogram([0.151, 0.157], bin_ms=5.0) == [(150.0, 1), (155.0, 1)]


def test_no_samples_gives_no_bins():
    assert histogram([]) == []
    print_histogram("empty", [])


def test_the_histogram_prints_a_bar_per_bin(capsys):
    print_histogram("glass -> visible", [0.152, 0.157, 0.163], width=10)
    printed = capsys.readouterr().out
    assert "glass -> visible, 10 ms bins (n=3)" in printed
    assert "150-160      2  ##########" in printed
    assert "160-170      1  #####" in printed


def test_reporting_shows_the_distribution_of_the_whole_wait(capsys):
    report([Sample(0.15, 0.12, 0.03, 0.001)] * 8, warmup=2, period=0.033, sensor_delay=0.12)
    printed = capsys.readouterr().out
    assert "glass -> visible, 10 ms bins (n=6)" in printed
    assert "150-160      6" in printed
