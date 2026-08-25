#include "ha_live_camera.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "esphome/core/log.h"

#include <cstring>

#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "freertos/task.h"

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#include "esphome/core/string_ref.h"
#include <functional>
#endif

namespace esphome {
namespace ha_live_camera {

static const char *const TAG = "ha_live_camera";

// Read chunk handed to the parser. Small on purpose: the parser is streaming,
// so there is nothing to gain from large reads and everything to lose in
// latency.
static constexpr size_t READ_CHUNK = 4096;

// Reconnect backoff.
static constexpr uint32_t RECONNECT_DELAY_MS = 1500;

const char *stream_status_to_string(StreamStatus s) {
  switch (s) {
    case StreamStatus::IDLE:
      return "Idle";
    case StreamStatus::CONNECTING:
      return "Connecting...";
    case StreamStatus::LIVE:
      return "Live";
    case StreamStatus::RECONNECTING:
      return "Reconnecting...";
    case StreamStatus::UNAVAILABLE:
      return "Camera unavailable";
    case StreamStatus::AUTH_FAILED:
      return "Authentication failed";
    case StreamStatus::STREAM_ERROR:
      return "Stream error";
  }
  return "Unknown";
}

// -----------------------------------------------------------------------------
// setup / teardown
// -----------------------------------------------------------------------------

void HaLiveCamera::setup() {
  // Round the output allocation up to the JPEG engine's 16-pixel MCU
  // boundary. A 480x270 stream really decodes into a 480x272 buffer.
  const uint32_t aw = (this->max_width_ + 15u) & ~15u;
  const uint32_t ah = (this->max_height_ + 15u) & ~15u;
  this->fb_size_ = static_cast<size_t>(aw) * ah * 2u;

  jpeg_decode_memory_alloc_cfg_t out_cfg = {};
  out_cfg.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER;
  for (uint8_t i = 0; i < NUM_FB; i++) {
    size_t got = 0;
    this->fb_[i] = static_cast<uint8_t *>(jpeg_alloc_decoder_mem(this->fb_size_, &out_cfg, &got));
    if (this->fb_[i] == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate frame buffer %u (%u bytes)", (unsigned) i, (unsigned) this->fb_size_);
      this->mark_failed();
      return;
    }
    memset(this->fb_[i], 0, this->fb_size_);
  }

  jpeg_decode_memory_alloc_cfg_t in_cfg = {};
  in_cfg.buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER;
  size_t in_got = 0;
  this->jpeg_in_ = static_cast<uint8_t *>(jpeg_alloc_decoder_mem(this->max_frame_bytes_, &in_cfg, &in_got));
  if (this->jpeg_in_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate JPEG input buffer (%u bytes)", (unsigned) this->max_frame_bytes_);
    this->mark_failed();
    return;
  }
  this->jpeg_in_size_ = in_got != 0 ? in_got : this->max_frame_bytes_;

  jpeg_decode_engine_cfg_t eng_cfg = {};
  eng_cfg.intr_priority = 0;
  eng_cfg.timeout_ms = 100;
  esp_err_t err = jpeg_new_decoder_engine(&eng_cfg, &this->decoder_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "jpeg_new_decoder_engine failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  // The parser decodes straight into the driver-aligned input buffer, so no
  // copy sits between the socket and the JPEG engine.
  this->parser_.set_buffer(this->jpeg_in_, this->jpeg_in_size_);
  this->parser_.set_on_frame(&HaLiveCamera::frame_trampoline_, this);

  // Show a black frame until the first real one lands, so LVGL always has a
  // valid descriptor to draw.
  this->width_ = this->max_width_;
  this->height_ = this->max_height_;
  this->data_start_ = this->fb_[0];
  this->displayed_index_.store(0, std::memory_order_relaxed);

#ifdef USE_API
  // Learn Frigate's name for each camera from the entity's camera_name
  // attribute, over the API connection ESPHome already has. This is what lets
  // the YAML stay in plain entity_ids while the stream comes from Frigate.
  for (size_t i = 0; i < this->cameras_.size(); i++) {
    // The callback type must be spelled out: APIServer overloads this on both
    // std::function<void(StringRef)> and std::function<void(const std::string&)>,
    // and a bare lambda is convertible to both, which is ambiguous.
    std::function<void(StringRef)> cb = [this, i](StringRef state) {
      this->set_camera_name(i, std::string(state.c_str(), state.size()));
    };
    api::global_api_server->subscribe_home_assistant_state(
        this->cameras_[i].entity_id, optional<std::string>("camera_name"), std::move(cb));
  }
#endif

  xTaskCreatePinnedToCore(&HaLiveCamera::task_trampoline_, "ha_live_cam", 6144, this, this->task_priority_,
                          &this->task_, 0);
  if (this->task_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create network task");
    this->mark_failed();
  }
}

void HaLiveCamera::dump_config() {
  ESP_LOGCONFIG(TAG, "HA Live Camera:");
  ESP_LOGCONFIG(TAG, "  Frigate:  %s (fps=%u height=%u)", this->frigate_url_.c_str(),
                (unsigned) this->stream_fps_, (unsigned) this->stream_height_);
  ESP_LOGCONFIG(TAG, "  Max frame: %ux%u (buffer %u bytes x%u)", (unsigned) this->max_width_,
                (unsigned) this->max_height_, (unsigned) this->fb_size_, (unsigned) NUM_FB);
  ESP_LOGCONFIG(TAG, "  Max JPEG:  %u bytes", (unsigned) this->jpeg_in_size_);
  for (size_t i = 0; i < this->cameras_.size(); i++) {
    ESP_LOGCONFIG(TAG, "  Camera %u: %s (%s)", (unsigned) i, this->cameras_[i].name.c_str(),
                  this->cameras_[i].entity_id.c_str());
  }
}

// -----------------------------------------------------------------------------
// main loop: publish frames and status to LVGL
// -----------------------------------------------------------------------------

void HaLiveCamera::loop() {
  const int8_t ready = this->ready_index_.load(std::memory_order_acquire);
  if (ready >= 0 && ready != this->displayed_index_.load(std::memory_order_relaxed)) {
    this->width_ = this->fb_w_[ready];
    this->height_ = this->fb_h_[ready];
    this->data_start_ = this->fb_[ready];
    this->displayed_index_.store(ready, std::memory_order_release);
#ifdef USE_LVGL
    // Image::get_lv_image_dsc() is what copies data_start_/width_/height_ into
    // the lv_image_dsc_t. LVGL is handed a pointer to that struct once, when
    // the widget's src is set, and never asks for it again -- so without this
    // call the widget keeps drawing whatever buffer was current at setup(),
    // no matter how many frames decode behind it.
    this->get_lv_image_dsc();
#endif
    for (auto *t : this->frame_triggers_)
      t->trigger();
  }

  const StreamStatus s = this->status();
  if (s != this->last_reported_status_) {
    this->last_reported_status_ = s;
    ESP_LOGD(TAG, "Status: %s", stream_status_to_string(s));
    for (auto *t : this->status_triggers_)
      t->trigger(std::string(stream_status_to_string(s)));
  }

  const uint32_t now = millis();
  if (this->last_fps_calc_ms_ == 0) {
    this->last_fps_calc_ms_ = now;
  } else if (now - this->last_fps_calc_ms_ >= 1000) {
    const uint32_t count = this->decoded_count_.load(std::memory_order_relaxed);
    this->measured_fps_ = static_cast<float>(count - this->last_fps_count_) * 1000.0f /
                 static_cast<float>(now - this->last_fps_calc_ms_);
    this->last_fps_count_ = count;
    this->last_fps_calc_ms_ = now;
  }
}

// -----------------------------------------------------------------------------
// control
// -----------------------------------------------------------------------------

void HaLiveCamera::set_camera_name(size_t index, const std::string &name) {
  if (index >= this->cameras_.size() || name.empty())
    return;
  if (this->cameras_[index].camera_name != name) {
    ESP_LOGI(TAG, "%s -> frigate camera '%s'", this->cameras_[index].entity_id.c_str(), name.c_str());
    this->cameras_[index].camera_name = name;
  }
}

void HaLiveCamera::show(size_t index) {
  if (index >= this->cameras_.size()) {
    ESP_LOGW(TAG, "show(%u): no such camera", (unsigned) index);
    return;
  }
  this->set_status_(StreamStatus::CONNECTING);
  this->requested_index_.store(static_cast<int>(index), std::memory_order_release);
}

void HaLiveCamera::show_entity(const std::string &entity_id) {
  for (size_t i = 0; i < this->cameras_.size(); i++) {
    if (this->cameras_[i].entity_id == entity_id) {
      this->show(i);
      return;
    }
  }
  ESP_LOGW(TAG, "show_entity(%s): not configured", entity_id.c_str());
}

void HaLiveCamera::stop() {
  this->requested_index_.store(-1, std::memory_order_release);
  this->set_status_(StreamStatus::IDLE);
}

void HaLiveCamera::set_status_(StreamStatus s) {
  this->status_.store(static_cast<uint8_t>(s), std::memory_order_relaxed);
}

std::string HaLiveCamera::build_url_(const CameraEntry &c) const {
  // Frigate serves MJPEG straight off its already-decoded detect stream, so
  // there is no transcoding anywhere in the chain. Port 5000 is Frigate's
  // unauthenticated API, so no credentials are involved.
  std::string url = this->frigate_url_;
  if (!url.empty() && url.back() == '/')
    url.pop_back();
  url += "/api/";
  url += c.camera_name;
  url += "?fps=";
  url += to_string(this->stream_fps_);
  url += "&height=";
  url += to_string(this->stream_height_);
  return url;
}

// -----------------------------------------------------------------------------
// network task
// -----------------------------------------------------------------------------

void HaLiveCamera::task_trampoline_(void *arg) { static_cast<HaLiveCamera *>(arg)->net_task_(); }

void HaLiveCamera::net_task_() {
  for (;;) {
    const int want = this->requested_index_.load(std::memory_order_acquire);
    if (want < 0) {
      this->active_index_.store(-1, std::memory_order_release);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    this->active_index_.store(want, std::memory_order_release);
    this->run_stream_(static_cast<size_t>(want));

    // run_stream_ returns on error or when the request changed. If the user
    // is still on this camera, back off briefly and reconnect.
    if (this->requested_index_.load(std::memory_order_acquire) == want) {
      this->set_status_(StreamStatus::RECONNECTING);
      vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
    }
  }
}

void HaLiveCamera::run_stream_(size_t index) {
  CameraEntry entry = this->cameras_[index];  // copy: camera_name may change under us
  if (entry.camera_name.empty()) {
    // camera_name arrives from HA over the API connection shortly after boot.
    ESP_LOGW(TAG, "No frigate camera name yet for %s; waiting", entry.entity_id.c_str());
    this->set_status_(StreamStatus::CONNECTING);
    vTaskDelay(pdMS_TO_TICKS(500));
    return;
  }

  const std::string url = this->build_url_(entry);

  esp_http_client_config_t cfg = {};
  cfg.url = url.c_str();
  cfg.method = HTTP_METHOD_GET;
  cfg.timeout_ms = 5000;
  cfg.buffer_size = 1024;
  cfg.buffer_size_tx = 1024;
  cfg.disable_auto_redirect = false;
  cfg.keep_alive_enable = true;

  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (client == nullptr) {
    this->set_status_(StreamStatus::STREAM_ERROR);
    return;
  }

  esp_err_t err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "connect failed: %s", esp_err_to_name(err));
    this->set_status_(StreamStatus::UNAVAILABLE);
    esp_http_client_cleanup(client);
    return;
  }

  esp_http_client_fetch_headers(client);
  const int code = esp_http_client_get_status_code(client);
  if (code == 401 || code == 403) {
    // Only reachable if pointed at Frigate's authenticated port (8971).
    ESP_LOGW(TAG, "auth rejected (HTTP %d) -- use Frigate's port 5000", code);
    this->set_status_(StreamStatus::AUTH_FAILED);
    esp_http_client_cleanup(client);
    return;
  }
  if (code != 200) {
    ESP_LOGW(TAG, "unexpected HTTP %d", code);
    this->set_status_(StreamStatus::UNAVAILABLE);
    esp_http_client_cleanup(client);
    return;
  }

  ESP_LOGI(TAG, "streaming %s", entry.entity_id.c_str());
  this->parser_.reset();

  auto *chunk = static_cast<uint8_t *>(heap_caps_malloc(READ_CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (chunk == nullptr)
    chunk = static_cast<uint8_t *>(heap_caps_malloc(READ_CHUNK, MALLOC_CAP_8BIT));
  if (chunk == nullptr) {
    this->set_status_(StreamStatus::STREAM_ERROR);
    esp_http_client_cleanup(client);
    return;
  }

  uint8_t idle_reads = 0;
  while (this->requested_index_.load(std::memory_order_acquire) == static_cast<int>(index)) {
    const int r = esp_http_client_read(client, reinterpret_cast<char *>(chunk), READ_CHUNK);
    if (r < 0) {
      ESP_LOGW(TAG, "read error");
      this->set_status_(StreamStatus::STREAM_ERROR);
      break;
    }
    if (r == 0) {
      // Read timeout. A live stream should not go quiet for long.
      if (++idle_reads > 3) {
        ESP_LOGW(TAG, "stream stalled");
        this->set_status_(StreamStatus::RECONNECTING);
        break;
      }
      continue;
    }
    idle_reads = 0;
    this->parser_.feed(chunk, static_cast<size_t>(r));
  }

  heap_caps_free(chunk);
  esp_http_client_close(client);
  esp_http_client_cleanup(client);
}

// -----------------------------------------------------------------------------
// decode
// -----------------------------------------------------------------------------

void HaLiveCamera::frame_trampoline_(void *ctx, const uint8_t *data, size_t len) {
  static_cast<HaLiveCamera *>(ctx)->handle_jpeg_(data, len);
}

void HaLiveCamera::handle_jpeg_(const uint8_t *data, size_t len) {
  jpeg_decode_picture_info_t info = {};
  esp_err_t err = jpeg_decoder_get_info(data, len, &info);
  if (err != ESP_OK) {
    this->decode_errors_++;
    return;
  }

  if (info.width > this->max_width_ || info.height > this->max_height_) {
    ESP_LOGW(TAG, "frame %ux%u exceeds configured max %ux%u", (unsigned) info.width, (unsigned) info.height,
             (unsigned) this->max_width_, (unsigned) this->max_height_);
    this->decode_errors_++;
    return;
  }

  // Pick a buffer that is neither on screen nor queued for it.
  const int8_t shown = this->displayed_index_.load(std::memory_order_acquire);
  const int8_t ready = this->ready_index_.load(std::memory_order_acquire);
  int8_t target = -1;
  for (int8_t i = 0; i < static_cast<int8_t>(NUM_FB); i++) {
    if (i != shown && i != ready) {
      target = i;
      break;
    }
  }
  if (target < 0)
    return;  // cannot happen with three buffers, but never scribble on a live one

  jpeg_decode_cfg_t dec_cfg = {};
  dec_cfg.output_format = JPEG_DECODE_OUT_FORMAT_RGB565;
  dec_cfg.rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_RGB;
  dec_cfg.conv_std = JPEG_YUV_RGB_CONV_STD_BT601;

  uint32_t out_size = 0;
  err = jpeg_decoder_process(this->decoder_, &dec_cfg, data, len, this->fb_[target], this->fb_size_, &out_size);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "decode failed: %s", esp_err_to_name(err));
    this->decode_errors_++;
    return;
  }

  // The JPEG engine pads output to 16-pixel MCU boundaries. Extra *rows* are
  // harmless -- we simply don't draw them. Extra *columns* are not: they widen
  // the row stride, and LVGL computes stride from the image width, so the
  // picture would shear. Frigate derives width from each camera's own aspect
  // ratio, so a 4:3 camera at height=270 yields 360 -- not a multiple of 16.
  // Compact the rows in place rather than rejecting the frame.
  const uint32_t padded_w = (info.width + 15u) & ~15u;
  if (padded_w != info.width) {
    uint8_t *b = this->fb_[target];
    const size_t src_stride = static_cast<size_t>(padded_w) * 2u;
    const size_t dst_stride = static_cast<size_t>(info.width) * 2u;
    for (uint32_t y = 1; y < info.height; y++)
      memmove(b + y * dst_stride, b + y * src_stride, dst_stride);
  }

  this->fb_w_[target] = static_cast<uint16_t>(info.width);
  this->fb_h_[target] = static_cast<uint16_t>(info.height);
  this->ready_index_.store(target, std::memory_order_release);
  this->decoded_count_.fetch_add(1, std::memory_order_relaxed);
  this->set_status_(StreamStatus::LIVE);
}

}  // namespace ha_live_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4