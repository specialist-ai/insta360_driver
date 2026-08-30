"""Low-latency Insta360 fisheye reader.

The decode path and the SPS patch live in the C++ core. This package binds
them; it reimplements nothing.

The C++ API pushes frames to a callback on the SDK's stream thread. This one
pulls, because a Python callback there would take the GIL inside the decode
path:

    import insta360_driver as insta360

    with insta360.Reader() as reader:
        reader.start(mode=720, pixel_format='rgb8')
        frame = reader.read(timeout_ms=200)
        print(frame.image.shape, frame.arrival_mono)
"""

from insta360_driver._core import Frame, Reader

__all__ = ['Frame', 'Reader']
