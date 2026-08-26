# ha_live_camera — live MJPEG on the ESP32-P4, hardware-decoded

An ESPHome external component that streams **continuous live video** from Frigate to an ESP32-P4 touchscreen panel, decoding every frame on the SoC's hardware JPEG engine. Not snapshots.

Written for the Guition JC4880P443 (4.3", 480×800 MIPI-DSI), but the component itself is board-agnostic — it just needs an ESP32-P4.

## Status — read this first

| Piece | State                                                                                              |
|---|----------------------------------------------------------------------------------------------------|
| MJPEG multipart parser | ✅ **Unit-tested.** 435 assertions, 0 failures, on the host.                                       |
| ESPHome codegen (Python) | ✅ **Verified.** `esphome config` passes on 2026.6.5.                                              |
| C++ ↔ ESP-IDF API usage | ⚠️ **Audited** Every symbol and struct field checked against ESP-IDF 5.5.4 headers. |
| Firmware build | ✅ **Compiled.**                                                                                   |
| On-hardware test | ✅ **Runs.**                                                                                       |

**The C++ has not been through a compiler.** It was written in an environment that could not download the ESP-IDF toolchain. The parser is genuinely proven and the ESP-IDF calls genuinely match the headers, but expect to fix build errors on your first `Install`. The design is verified; the transcription is not.

## Why this exists

The ESP32-P4 has a hardware JPEG codec and **no H.264 decoder at all** — `soc_caps.h` lists `SOC_JPEG_CODEC_SUPPORTED`, `SOC_JPEG_DECODE_SUPPORTED`, `SOC_JPEG_ENCODE_SUPPORTED` and nothing else. Browsers play your cameras over H.264; this panel can't. It needs JPEG frames, which means MJPEG.

Home Assistant won't provide that. `/api/camera_proxy_stream/<entity>` falls back to re-fetching a snapshot every 0.5 s for any camera that doesn't override `handle_async_mjpeg_stream()` — which includes every Frigate camera. That's 2 FPS of stills.

Frigate *will* provide it. `/api/<camera>?fps=15&height=270` serves frames straight off the detect stream it has already decoded, so there is **no transcoding anywhere in the chain**.

## How it works

```
FreeRTOS network task                      ESPHome main loop
─────────────────────────────              ────────────────────────────
esp_http_client_read()
  └─> MjpegParser (frame extraction)
        └─> jpeg_decoder_process()  ← P4 hardware JPEG
              └─> RGB565 into fb[n]
                    └─> publish index  ──> loop() swaps data_start_,
                                            fires on_frame → LVGL redraws
```

- **One active stream at a time**, with automatic reconnect and backoff.
- **Triple-buffered** so the decoder never writes the buffer LVGL is drawing from.
- Presented to LVGL as an ordinary `image::Image`, so the camera page is just `src: live_cams`.
- LVGL is single-threaded inside `LvglComponent::loop()` with no mutex anywhere, so the network task never touches an LVGL object. The only cross-thread state is an atomic buffer index and the status word.

## Configuration

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/YOU/ha-live-camera
      ref: main
    components: [ha_live_camera]

ha_live_camera:
  id: live_cams
  frigate_url: "http://frigate.lan:5000"
  fps: 15
  stream_height: 270
  max_width: 480
  max_height: 272       # 270 rounds up to the JPEG engine's 16px MCU boundary
  max_frame_bytes: 128kB
  cameras:
    - entity_id: camera.back_garden
      name: "Back Garden"
    - entity_id: camera.doorbell_2
      name: "Doorbell"
  on_frame:
    - lvgl.widget.redraw: cam_img
  on_status:
    - lvgl.label.update: {id: cam_status, text: !lambda 'return x;'}
```

Actions: `ha_live_camera.show` (0-based `index`, templatable) and `ha_live_camera.stop`.

**You configure Home Assistant entity IDs, not Frigate camera names.** The component subscribes to each entity's `camera_name` attribute over the ESPHome API connection Home Assistant already holds, so `camera.doorbell_2` resolves itself to Frigate's `doorbell` at runtime. Nothing needs to go in `configuration.yaml`, and no HA camera entities need creating.

## Requirement: Frigate's port 5000

Frigate publishes `8971` (authenticated), `8554` (RTSP) and `8555` (WebRTC) by default — but **not 5000**, the unauthenticated API port this component uses. Add it to your compose:

```yaml
    ports:
      - target: 5000
        published: "5000"
        protocol: tcp
```

and recreate the container. Verify:

```bash
curl -s --max-time 10 "http://<frigate>:5000/api/<camera>?fps=15&height=270" \
  | grep -aci "content-type: image/jpeg"
```

Roughly `fps × 10` means it's working.

⚠️ **Port 5000 is unauthenticated.** Anyone on your LAN can view your cameras through it. Frigate's own docs say access "should be limited". If that's unacceptable, the component would need Frigate JWT login against port 8971 instead — not implemented here.

## The parser, and why it's built the way it is

Four MJPEG sources, four framings, and they disagree about everything:

| Source | Boundary | Content-Length |
|---|---|---|
| Frigate `/api/<cam>` | `frame` | none at all |
| go2rtc `stream.mjpeg` | `frame` | `Content-Length:` |
| HA `ffmpeg` camera | `ffmpeg` | `Content-length:` — **lowercase L** |
| HA still-image fallback | `--frameboundary` | `Content-Length:` |

Two traps. FFmpeg's mpjpeg muxer writes `Content-length:` with a lowercase L, so `strstr(buf, "Content-Length:")` silently finds nothing. And HA's still-stream declares `boundary=--frameboundary` while writing `--frameboundary` as the delimiter — off by two per the RFC.

So the parser trusts none of it and walks the **JPEG marker structure** instead: find SOI, skip each segment by its declared length, and after SOS scan the entropy-coded data for a real EOI, handling `FF 00` byte stuffing and `FF D0..D7` restart markers. Thumbnails embedded in APPn segments can't produce a false EOI because those segments are skipped wholesale by length.

Only the Frigate framing matters for this component today. The rest are supported because the parser is the piece most likely to have subtle bugs, and making it framing-agnostic is cheaper than making it framing-specific.

### Running the tests

```bash
make -C test
```

Generates real JPEGs with PIL (including a progressive one and one with an embedded EXIF thumbnail), wraps them in all four framings plus HTTP-preamble, truncated-frame and noisy-keepalive cases, then replays each at ten chunk sizes — **including one byte at a time**, which splits every boundary, header and marker across calls. Also checks capacity-overflow recovery and mid-frame `reset()`.

```
435 checks, 0 failures
ALL TESTS PASSED
```

## Three things that will bite you

**ESPHome excludes `esp_http_client` from the IDF build.** It's in `DEFAULT_EXCLUDED_IDF_COMPONENTS`, so the streaming client won't link unless you opt back in. Handled in `to_code()` via `include_builtin_idf_component("esp_http_client")`.

**LVGL needs image metadata at codegen time**, or you get `AttributeError: 'NoneType' object has no attribute 'image_type'`. Handled via `add_metadata()`.

**Frigate derives width from each camera's aspect ratio.** `height=270` gives 480 on a 16:9 camera but 360 on a 4:3 one — and 360 isn't a multiple of 16. The JPEG engine pads decoded output to 16-pixel MCU boundaries, which widens the row stride, and LVGL computes stride from image width, so the picture would shear diagonally. The component compacts the rows in place instead of rejecting the frame.

## Files

```
components/ha_live_camera/
  __init__.py           ESPHome schema, codegen, actions
  mjpeg_parser.h/.cpp   framing-agnostic frame extractor (portable, tested)
  ha_live_camera.h/.cpp component, network task, hardware decode
panel-live.yaml         example panel config — mixed camera + light/switch tiles
test/                   host unit tests + stream generator
```

`panel-live.yaml` is an example for one specific board and network. Edit the `substitutions:` block at the top before using it.

## Sources

- [ESP-IDF `esp32p4/soc_caps.h`](https://github.com/espressif/esp-idf/blob/v5.5.4/components/soc/esp32p4/include/soc/soc_caps.h) — JPEG codec present, H.264 absent
- [ESP-IDF JPEG decoder](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/jpeg.html)
- [Frigate `api/media.py`](https://github.com/blakeblackshear/frigate) — MJPEG feed, no Content-Length
- [Frigate ports](https://docs.frigate.video/frigate/installation) — 5000 unauthenticated, 8971 authenticated
- [HA `camera/__init__.py`](https://github.com/home-assistant/core/blob/dev/homeassistant/components/camera/__init__.py) — `MIN_STREAM_INTERVAL`, the 2 FPS fallback
- [FFmpeg `mpjpeg.c`](https://github.com/FFmpeg/FFmpeg/blob/master/libavformat/mpjpeg.c) — lowercase header names
- [ESPHome MIPI DSI](https://esphome.io/components/display/mipi_dsi/) — JC4880P443 panel model
- [jtenniswood/espcontrol](https://github.com/jtenniswood/espcontrol) — JC4880P443 pinout

## Licence

MIT. No warranty — see the status table at the top, and mean it.
