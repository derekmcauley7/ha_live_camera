#include "mjpeg_parser.h"

#include <cstring>

namespace esphome {
namespace ha_live_camera {

void MjpegParser::reset() {
  this->state_ = State::HUNT_SOI;
  this->length_ = 0;
  this->saw_ff_ = false;
  this->marker_ = 0;
  this->have_length_hi_ = false;
  this->skip_remaining_ = 0;
}

bool MjpegParser::push_byte_(uint8_t b) {
  if (this->length_ >= this->capacity_) {
    this->drop_frame_();
    return false;
  }
  this->buffer_[this->length_++] = b;
  return true;
}

void MjpegParser::emit_frame_() {
  this->frames_emitted_++;
  if (this->on_frame_ != nullptr)
    this->on_frame_(this->on_frame_ctx_, this->buffer_, this->length_);
  this->state_ = State::HUNT_SOI;
  this->length_ = 0;
  this->saw_ff_ = false;
}

void MjpegParser::drop_frame_() {
  this->frames_dropped_++;
  this->state_ = State::HUNT_SOI;
  this->length_ = 0;
  this->saw_ff_ = false;
}

void MjpegParser::feed(const uint8_t *data, size_t len) {
  if (this->buffer_ == nullptr || this->capacity_ == 0)
    return;

  size_t i = 0;
  while (i < len) {
    switch (this->state_) {
      case State::HUNT_SOI: {
        // Bulk-scan for FF; everything before it is inter-frame padding
        // (boundary lines, part headers, CRLFs) and is thrown away.
        if (this->saw_ff_) {
          uint8_t b = data[i++];
          if (b == 0xD8) {
            this->length_ = 0;
            this->saw_ff_ = false;
            // SOI is part of the frame.
            if (!this->push_byte_(0xFF) || !this->push_byte_(0xD8))
              break;
            this->state_ = State::EXPECT_MARKER;
          } else {
            this->saw_ff_ = (b == 0xFF);
          }
          break;
        }
        const uint8_t *hit =
            static_cast<const uint8_t *>(memchr(data + i, 0xFF, len - i));
        if (hit == nullptr) {
          i = len;
        } else {
          i = static_cast<size_t>(hit - data) + 1;
          this->saw_ff_ = true;
        }
        break;
      }

      case State::EXPECT_MARKER: {
        uint8_t b = data[i++];
        if (!this->push_byte_(b))
          break;
        if (!this->saw_ff_) {
          if (b == 0xFF) {
            this->saw_ff_ = true;
          } else {
            // Not a marker where one was required: stream is out of sync.
            this->drop_frame_();
          }
          break;
        }
        // Previous byte was FF.
        if (b == 0xFF) {
          // Fill byte, still expecting the marker code.
          break;
        }
        this->saw_ff_ = false;
        if (b == 0x00) {
          this->drop_frame_();
          break;
        }
        this->marker_ = b;
        if (b == 0xD9) {  // EOI
          this->emit_frame_();
        } else if ((b >= 0xD0 && b <= 0xD7) || b == 0x01 || b == 0xD8) {
          // Standalone markers carry no payload.
        } else {
          this->state_ = State::READ_LENGTH;
          this->have_length_hi_ = false;
        }
        break;
      }

      case State::READ_LENGTH: {
        uint8_t b = data[i++];
        if (!this->push_byte_(b))
          break;
        if (!this->have_length_hi_) {
          this->length_hi_ = b;
          this->have_length_hi_ = true;
          break;
        }
        uint32_t seglen = (static_cast<uint32_t>(this->length_hi_) << 8) | b;
        if (seglen < 2) {
          this->drop_frame_();
          break;
        }
        this->skip_remaining_ = seglen - 2;
        if (this->skip_remaining_ == 0) {
          this->state_ = (this->marker_ == 0xDA) ? State::ENTROPY
                                                 : State::EXPECT_MARKER;
        } else {
          this->state_ = State::SKIP_PAYLOAD;
        }
        break;
      }

      case State::SKIP_PAYLOAD: {
        // Segment payloads are opaque — copy in bulk. This is what makes
        // embedded EXIF thumbnails harmless: their SOI/EOI never get scanned.
        size_t avail = len - i;
        size_t take = avail < this->skip_remaining_ ? avail
                                                    : this->skip_remaining_;
        if (this->length_ + take > this->capacity_) {
          this->drop_frame_();
          break;
        }
        memcpy(this->buffer_ + this->length_, data + i, take);
        this->length_ += take;
        i += take;
        this->skip_remaining_ -= static_cast<uint32_t>(take);
        if (this->skip_remaining_ == 0) {
          this->state_ = (this->marker_ == 0xDA) ? State::ENTROPY
                                                 : State::EXPECT_MARKER;
        }
        break;
      }

      case State::ENTROPY: {
        if (this->saw_ff_) {
          uint8_t b = data[i++];
          if (!this->push_byte_(b))
            break;
          this->saw_ff_ = false;
          if (b == 0x00) {
            // Stuffed FF inside entropy data — not a marker.
          } else if (b >= 0xD0 && b <= 0xD7) {
            // Restart marker.
          } else if (b == 0xD9) {
            this->emit_frame_();
          } else if (b == 0xFF) {
            this->saw_ff_ = true;  // fill byte
          } else {
            // A real marker follows the scan (progressive JPEG, trailing
            // tables, another SOS...). Parse it as a segment.
            this->marker_ = b;
            this->state_ = State::READ_LENGTH;
            this->have_length_hi_ = false;
          }
          break;
        }
        // Bulk-copy entropy bytes up to the next FF.
        const uint8_t *hit =
            static_cast<const uint8_t *>(memchr(data + i, 0xFF, len - i));
        size_t take = (hit == nullptr) ? (len - i)
                                       : static_cast<size_t>(hit - (data + i));
        if (take > 0) {
          if (this->length_ + take > this->capacity_) {
            this->drop_frame_();
            break;
          }
          memcpy(this->buffer_ + this->length_, data + i, take);
          this->length_ += take;
          i += take;
        }
        if (hit != nullptr) {
          if (!this->push_byte_(0xFF))
            break;
          this->saw_ff_ = true;
          i++;  // consume the FF
        }
        break;
      }
    }
  }
}

}  // namespace ha_live_camera
}  // namespace esphome
