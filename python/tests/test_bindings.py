"""The bindings expose the C++ reader, and add nothing of their own.

None of this needs a camera. What needs one is marked and skipped.
"""

from pathlib import Path

import numpy as np
import pytest

import insta360_driver as insta360

# Insta360's USB vendor id. The SDK talks libusb directly, so a camera that is
# not enumerated cannot be opened whatever the SDK reports.
INSTA360_VENDOR_ID = '2e1a'


def camera_attached() -> bool:
    try:
        return any(
            path.read_text().strip() == INSTA360_VENDOR_ID
            for path in Path('/sys/bus/usb/devices').glob('*/idVendor'))
    except OSError:
        return False


def test_a_new_reader_is_not_running():
    reader = insta360.Reader()
    assert not reader.running
    assert reader.latest() is None


def test_read_before_start_times_out_rather_than_blocking_forever():
    reader = insta360.Reader()
    with pytest.raises(TimeoutError):
        reader.read(timeout_ms=10)


@pytest.mark.parametrize('mode', [0, 480, 1081, 2160])
def test_only_the_probed_modes_are_accepted(mode):
    # An arbitrary resolution id can put the camera in a junk mode or wedge it
    # until a power cycle, so the binding refuses one before the SDK sees it.
    reader = insta360.Reader()
    with pytest.raises(ValueError, match='720, 1280 or 1920'):
        reader.start(mode=mode)


def test_the_sensor_must_name_a_lens():
    reader = insta360.Reader()
    with pytest.raises(ValueError, match='rear'):
        reader.start(mode=720, sensor='left')


def test_the_pixel_format_must_be_one_the_binding_converts():
    reader = insta360.Reader()
    with pytest.raises(ValueError, match='bgr8'):
        reader.start(mode=720, pixel_format='yuv420')


def test_the_context_manager_hands_back_the_same_reader():
    # __enter__ must return the object itself, not a copy: a Reader owns the
    # SDK session and a decode thread.
    reader = insta360.Reader()
    with reader as entered:
        assert entered is reader
    assert not reader.running


def test_a_frame_carries_an_image_and_the_clock_it_arrived_on():
    assert {'image', 'arrival_mono', 'decoded_mono', 'index'} <= set(dir(insta360.Frame))


@pytest.mark.skipif(not camera_attached(), reason='needs an Insta360 camera on USB')
def test_streams_a_frame():
    with insta360.Reader() as reader:
        reader.start(mode=720, pixel_format='rgb8')
        frame = reader.read(timeout_ms=5000)
        assert frame.image.shape == (720, 720, 3)
        assert frame.image.dtype == np.uint8
        assert frame.arrival_mono <= frame.decoded_mono
        assert frame.index == 1
