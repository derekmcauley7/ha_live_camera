#pragma once
//
// Live MJPEG camera viewer for the ESP32-P4, decoding with the SoC's hardware
// JPEG engine and presenting frames to LVGL as an ordinary esphome image.
//
// Pipeline:
//
//   FreeRTOS network task            ESPHome main loop
//   ------------------------         ---------------------------
//   esp_http_client_read()
//     -> MjpegParser
//       -> jpeg_decoder_process()
//         -> RGB565 into fb_[n]
//           -> publish ready_index_  --> loop() swaps data_start_,
//                                        fires on_frame, LVGL redraws
//
// LVGL is single-threaded inside the ESPHome main loop (LvglComponent::loop()
// calls lv_timer_handler() with no mutex anywhere), so the network task never
// touches an LVGL object. The only cross-thread state is an atomic buffer
// index and the status word.
//
// Double buffering is also what keeps LVGL's image cache honest: Image::
// get_lv_image_dsc() rebuilds its descriptor only when the data pointer
// changes, and alternating buffers changes it on every frame.
//

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/components/image/image.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "mjpeg_parser.h"

#include <atomic>
#include <string>
#include <vector>

#include "driver/jpeg_decode.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome {
namespace ha_live_camera {

enum class StreamStatus : uint8_t {
  IDLE = 0,
  CONNECTING,
  LIVE,
  RECONNECTING,
  UNAVAILABLE,
  AUTH_FAILED,
  STREAM_ERROR,
};

const char *stream_status_to_string(StreamStatus s);

struct CameraEntry {
  std::string entity_id;
  std::string name;
  // Frigate's own name for this camera, learned at runtime from the entity's
  // camera_name attribute. Empty until Home Assistant pushes it.
  std::string camera_name;
};

class HaLiveCamera : public Component, public image::Image {
 public:
  HaLiveCamera() : image::Image(nullptr, 0, 0, image::IMAGE_TYPE_RGB565, image::TRANSPARENCY_OPAQUE) {}

  void setup() override;
  void loop() override;
  void dump_config() override;
  // MUST be higher than LvglComponent (setup_priority::PROCESSOR, 400).
  // LVGL builds its widgets during setup and calls lv_image_set_src() with
  // whatever get_lv_image_dsc() returns AT THAT MOMENT, then sizes the image
  // widget from it once and never re-measures. If our buffers are not
  // allocated yet the widget is created 0x0 and stays invisible forever, no
  // matter how many frames decode behind it.
  float get_setup_priority() const override { return setup_priority::DATA; }

  // --- configuration (called from generated code) ---
  void set_frigate_url(const std::string &url) { this->frigate_url_ = url; }
  void set_stream_params(uint8_t fps, uint16_t height) {
    this->stream_fps_ = fps;
    this->stream_height_ = height;
  }
  void add_camera(const std::string &entity_id, const std::string &name) {
    this->cameras_.push_back(CameraEntry{entity_id, name, ""});
  }
  void set_max_size(uint16_t w, uint16_t h) {
    this->max_width_ = w;
    this->max_height_ = h;
  }
  void set_max_frame_bytes(uint32_t n) { this->max_frame_bytes_ = n; }
  // The JPEG driver documents BGR element order as "small endian", which is
  // what LVGL wants when LV_COLOR_16_SWAP is 0 (ESPHome's little_endian).
  void set_rgb_order_bgr(bool bgr) { this->rgb_order_bgr_ = bgr; }
  void set_task_priority(uint8_t p) { this->task_priority_ = p; }
  // Frigate's authenticated port (8971). Leave empty to talk to the
  // unauthenticated port 5000, which grants admin-equivalent access to anyone
  // who can reach it -- see the README.
  void set_credentials(const std::string &user, const std::string &pass) {
    this->username_ = user;
    this->password_ = pass;
  }

  void add_frame_trigger(Trigger<> *t) { this->frame_triggers_.push_back(t); }
  void add_status_trigger(Trigger<std::string> *t) { this->status_triggers_.push_back(t); }

  // --- runtime control ---
  // Frigate camera name for `index`, from the entity's camera_name attribute.
  void set_camera_name(size_t index, const std::string &name);
  void show(size_t index);
  void show_entity(const std::string &entity_id);
  void stop();

  size_t camera_count() const { return this->cameras_.size(); }
  // True once Home Assistant has pushed a camera_name for every configured
  // entity -- i.e. the panel could actually stream if asked. Used by the boot
  // screen to know when it is safe to hand over to the UI.
  bool cameras_ready() const {
    if (this->cameras_.empty())
      return false;
    for (const auto &c : this->cameras_)
      if (c.camera_name.empty())
        return false;
    return true;
  }
  const std::string &display_name(size_t i) const { return this->cameras_[i].name; }
  int active_index() const { return this->active_index_.load(std::memory_order_relaxed); }
  StreamStatus status() const {
    return static_cast<StreamStatus>(this->status_.load(std::memory_order_relaxed));
  }
  const char *status_string() const { return stream_status_to_string(this->status()); }
  float measured_fps() const { return this->measured_fps_; }
  uint32_t dropped_frames() const { return this->parser_.frames_dropped() + this->decode_errors_; }
  uint32_t skipped_frames() const { return this->skipped_frames_; }

 protected:
  static void task_trampoline_(void *arg);
  static void frame_trampoline_(void *ctx, const uint8_t *data, size_t len);

  void net_task_();
  void run_stream_(size_t index);
  void handle_jpeg_(const uint8_t *data, size_t len);
  void set_status_(StreamStatus s);
  // Publish an all-black frame so a camera switch never shows the
  // previous camera while the new one is connecting.
  void blank_display_();
  std::string base_url_() const;
  std::string build_url_(const CameraEntry &c) const;

  // POST /api/login and keep the JWT out of the Set-Cookie header. Returns
  // false only when Frigate actively rejected the credentials.
  bool login_();
  static esp_err_t login_event_(esp_http_client_event_t *evt);

  std::string frigate_url_;
  uint8_t stream_fps_{15};
  uint16_t stream_height_{270};
  std::vector<CameraEntry> cameras_;

  uint16_t max_width_{480};
  uint16_t max_height_{272};
  uint32_t max_frame_bytes_{128 * 1024};
  uint8_t task_priority_{5};
  bool rgb_order_bgr_{true};

  std::string username_;
  std::string password_;
  // Session token from /api/login. Default lifetime is 24h (auth.session_length);
  // rather than track expiry we just re-login when a stream comes back 401.
  std::string jwt_;

  // Triple-buffered so the decoder never writes the buffer LVGL is drawing
  // from: the task only picks a buffer that is neither the most recently
  // published frame nor the one currently on screen.
  static constexpr uint8_t NUM_FB = 3;
  uint8_t *fb_[NUM_FB]{nullptr, nullptr, nullptr};
  size_t fb_size_{0};
  uint16_t fb_w_[NUM_FB]{0, 0, 0};
  uint16_t fb_h_[NUM_FB]{0, 0, 0};
  std::atomic<int8_t> ready_index_{-1};
  std::atomic<int8_t> displayed_index_{-1};

  uint8_t *jpeg_in_{nullptr};  // parser writes compressed frames straight here
  size_t jpeg_in_size_{0};

  jpeg_decoder_handle_t decoder_{nullptr};
  MjpegParser parser_;

  TaskHandle_t task_{nullptr};
  std::atomic<int> active_index_{-1};
  std::atomic<int> requested_index_{-1};
  std::atomic<uint8_t> status_{static_cast<uint8_t>(StreamStatus::IDLE)};
  std::atomic<uint32_t> decoded_count_{0};

  StreamStatus last_reported_status_{StreamStatus::IDLE};
  uint32_t last_fps_calc_ms_{0};
  uint32_t last_fps_count_{0};
  float measured_fps_{0.0f};
  uint32_t decode_errors_{0};
  uint32_t published_count_{0};
  uint32_t skipped_frames_{0};
  uint32_t last_diag_ms_{0};
  int last_started_index_{-1};

  std::vector<Trigger<> *> frame_triggers_;
  std::vector<Trigger<std::string> *> status_triggers_;
};

// --- actions -----------------------------------------------------------------

template<typename... Ts> class ShowCameraAction : public Action<Ts...> {
 public:
  explicit ShowCameraAction(HaLiveCamera *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(int, index)
  // Action::play is declared `virtual void play(const Ts &...x)` -- taking the
  // pack by value does not override it, which leaves the class abstract.
  void play(const Ts &...x) override {
    int i = this->index_.value(x...);
    if (i >= 0)
      this->parent_->show(static_cast<size_t>(i));
  }

 protected:
  HaLiveCamera *parent_;
};

template<typename... Ts> class StopCameraAction : public Action<Ts...> {
 public:
  explicit StopCameraAction(HaLiveCamera *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->stop(); }

 protected:
  HaLiveCamera *parent_;
};

}  // namespace ha_live_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4