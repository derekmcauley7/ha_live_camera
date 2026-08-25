#!/usr/bin/env python3
"""Generate MJPEG test streams in every framing the parser must survive.

Writes, into test/data/:
  frames_N.jpg        the ground-truth JPEGs
  stream_<name>.bin   a multipart stream carrying exactly those frames

The C++ test replays each stream at many chunk sizes and asserts the parser
recovers the ground-truth frames byte-for-byte.
"""

import io
import os
import random

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
os.makedirs(DATA, exist_ok=True)

random.seed(1234)


def make_jpeg(w, h, seed, quality=70, progressive=False, exif_thumb=False):
    """A real JPEG with noisy content so entropy data contains FF bytes."""
    img = Image.new("RGB", (w, h))
    px = img.load()
    rnd = random.Random(seed)
    for y in range(h):
        for x in range(w):
            px[x, y] = (rnd.randrange(256), rnd.randrange(256), rnd.randrange(256))
    buf = io.BytesIO()
    kwargs = {"quality": quality, "progressive": progressive}
    if exif_thumb:
        # Embed a full JPEG inside an APP1/EXIF segment. Its SOI/EOI must not
        # be mistaken for frame boundaries.
        thumb = Image.new("RGB", (32, 24))
        tpx = thumb.load()
        for y in range(24):
            for x in range(32):
                tpx[x, y] = (rnd.randrange(256), rnd.randrange(256), rnd.randrange(256))
        tbuf = io.BytesIO()
        thumb.save(tbuf, format="JPEG", quality=80)
        exif = Image.Exif()
        exif[0x501B] = tbuf.getvalue()  # ThumbnailData
        kwargs["exif"] = exif
    img.save(buf, format="JPEG", **kwargs)
    return buf.getvalue()


# Frame set: mixed sizes, one progressive, one with an embedded thumbnail.
FRAMES = [
    make_jpeg(64, 48, 1),
    make_jpeg(96, 54, 2, quality=85),
    make_jpeg(64, 48, 3, progressive=True),
    make_jpeg(80, 60, 4, exif_thumb=True),
    make_jpeg(48, 32, 5, quality=50),
]

for i, f in enumerate(FRAMES):
    with open(os.path.join(DATA, f"frame_{i}.jpg"), "wb") as fh:
        fh.write(f)


def frigate(frames):
    """Frigate: boundary 'frame', NO Content-Length, trailing CRLFCRLF."""
    out = b""
    for f in frames:
        out += b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n\r\n"
    return out


def go2rtc(frames):
    """go2rtc: boundary 'frame', Content-Length, leading boundary per part."""
    out = b""
    for f in frames:
        out += (
            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + str(len(f)).encode()
            + b"\r\n\r\n"
            + f
            + b"\r\n"
        )
    return out


def ffmpeg(frames):
    """ffmpeg mpjpeg muxer: boundary written FIRST, lowercase header names,
    boundary trails each frame."""
    out = b"--ffmpeg\r\n"
    for f in frames:
        out += (
            b"Content-type: image/jpeg\r\nContent-length: "
            + str(len(f)).encode()
            + b"\r\n\r\n"
            + f
            + b"\r\n--ffmpeg\r\n"
        )
    return out


def ha_still(frames):
    """HA async_get_still_stream: boundary declared '--frameboundary' but the
    delimiter written is '--frameboundary' (off by two per RFC)."""
    out = b""
    for f in frames:
        out += (
            b"--frameboundary\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + str(len(f)).encode()
            + b"\r\n\r\n"
            + f
            + b"\r\n"
        )
    return out


def with_http_preamble(body):
    """Real responses arrive with status line + headers before any part."""
    return (
        b"HTTP/1.1 200 OK\r\n"
        b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        b"Cache-Control: no-cache\r\n\r\n" + body
    )


STREAMS = {
    "frigate": frigate(FRAMES),
    "go2rtc": go2rtc(FRAMES),
    "ffmpeg": ffmpeg(FRAMES),
    "ha_still": ha_still(FRAMES),
    "frigate_http": with_http_preamble(frigate(FRAMES)),
    # A truncated final frame must not be emitted, and must not corrupt
    # the frames before it.
    "truncated": frigate(FRAMES) + b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    + FRAMES[0][: len(FRAMES[0]) // 2],
    # Garbage between parts (simulates a proxy injecting keepalives).
    "noisy": b"".join(
        b"\r\n\r\nkeepalive\r\n--frame\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n"
        for f in FRAMES
    ),
}

for name, body in STREAMS.items():
    with open(os.path.join(DATA, f"stream_{name}.bin"), "wb") as fh:
        fh.write(body)

with open(os.path.join(DATA, "manifest.txt"), "w") as fh:
    fh.write(f"{len(FRAMES)}\n")
    for name in STREAMS:
        fh.write(f"{name}\n")

print(f"frames: {[len(f) for f in FRAMES]}")
print(f"streams: {sorted(STREAMS)}")
print(f"max frame: {max(len(f) for f in FRAMES)} bytes")
