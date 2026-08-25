#pragma once
//
// Streaming multipart/x-mixed-replace MJPEG frame extractor.
//
// Deliberately framing-agnostic. Real-world MJPEG sources disagree about
// almost everything:
//
//   Source                     boundary        Content-Length
//   -------------------------- --------------- -------------------------------
//   Frigate /api/<cam>         frame           absent
//   go2rtc /api/stream.mjpeg   frame           "Content-Length:"
//   HA ffmpeg camera           ffmpeg          "Content-length:" (lowercase L)
//   HA still-image fallback    --frameboundary "Content-Length:"  (boundary is
//                                              also self-inconsistent: declared
//                                              as "--frameboundary", so the RFC
//                                              delimiter would be
//                                              "----frameboundary", but only
//                                              "--frameboundary" is written)
//
// Rather than trust any of that, this parser locates frames by walking the
// JPEG marker structure itself: find SOI, walk segments by their declared
// lengths, and after SOS scan the entropy-coded data for a real EOI. Byte
// stuffing (FF 00) and restart markers (FF D0..D7) are handled, and thumbnails
// embedded inside APPn segments cannot produce a false EOI because those
// segments are skipped wholesale by length.
//
// No dynamic allocation: the caller supplies the frame buffer. Fully portable,
// so it builds and is unit-tested on the host.
//

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace ha_live_camera {

class MjpegParser {
 public:
  // Invoked once per complete JPEG. `data` points into the caller-supplied
  // buffer and is valid only for the duration of the call.
  using FrameCallback = void (*)(void *ctx, const uint8_t *data, size_t len);

  void set_buffer(uint8_t *buffer, size_t capacity) {
    this->buffer_ = buffer;
    this->capacity_ = capacity;
    this->reset();
  }

  void set_on_frame(FrameCallback cb, void *ctx) {
    this->on_frame_ = cb;
    this->on_frame_ctx_ = ctx;
  }

  // Drop any partial frame and return to hunting for the next SOI. Call on
  // reconnect or camera switch so a truncated frame can't merge into the next.
  void reset();

  // Push received bytes. May invoke the frame callback zero or more times.
  void feed(const uint8_t *data, size_t len);

  uint32_t frames_emitted() const { return this->frames_emitted_; }
  uint32_t frames_dropped() const { return this->frames_dropped_; }
  bool in_frame() const { return this->state_ != State::HUNT_SOI; }

 private:
  enum class State : uint8_t {
    HUNT_SOI,        // scanning the byte stream for FF D8
    EXPECT_MARKER,   // at a segment boundary, expecting FF <marker>
    READ_LENGTH,     // reading a segment's 2-byte big-endian length
    SKIP_PAYLOAD,    // skipping skip_remaining_ bytes of segment payload
    ENTROPY,         // inside entropy-coded data, hunting a real EOI
  };

  // Append one byte to the frame buffer. Returns false if the frame would
  // exceed capacity, in which case the frame is abandoned.
  bool push_byte_(uint8_t b);

  void emit_frame_();
  void drop_frame_();

  uint8_t *buffer_{nullptr};
  size_t capacity_{0};
  size_t length_{0};  // bytes currently held in buffer_

  FrameCallback on_frame_{nullptr};
  void *on_frame_ctx_{nullptr};

  State state_{State::HUNT_SOI};

  bool saw_ff_{false};        // previous byte was FF (used by HUNT_SOI/ENTROPY)
  uint8_t marker_{0};         // marker currently being processed
  uint8_t length_hi_{0};      // first byte of a 2-byte segment length
  bool have_length_hi_{false};
  uint32_t skip_remaining_{0};

  uint32_t frames_emitted_{0};
  uint32_t frames_dropped_{0};
};

}  // namespace ha_live_camera
}  // namespace esphome
