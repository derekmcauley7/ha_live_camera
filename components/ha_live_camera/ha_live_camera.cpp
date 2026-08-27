#include "ha_live_camera.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "esphome/core/log.h"

#include <cstring>
#include <strings.h>

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
  ESP_LOGCONFIG(TAG, "  RGB order: %s", this->rgb_order_bgr_ ? "BGR (little endian)" : "RGB (big endian)");
  ESP_LOGCONFIG(TAG, "  Auth:      %s", this->username_.empty()
                                            ? "none (unauthenticated port 5000)"
                                            : this->username_.c_str());
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
    this->published_count_++;
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

  // --- diagnostics -----------------------------------------------------------
  // Prints once every 5s while a stream is active. Between them these numbers
  // separate every remaining failure mode: a flat buffer means the decoder is
  // at fault, a stale dsc pointer means the descriptor is not reaching LVGL,
  // and correct values for both mean the fault is in the LVGL draw path.
  const uint32_t tnow = millis();
  if (this->active_index_.load(std::memory_order_relaxed) >= 0 &&
      (this->last_diag_ms_ == 0 || tnow - this->last_diag_ms_ >= 5000)) {
    this->last_diag_ms_ = tnow;
#ifdef USE_LVGL
    const bool lvgl_built = true;
#else
    const bool lvgl_built = false;
#endif
    const uint16_t *px = reinterpret_cast<const uint16_t *>(this->data_start_);
    uint16_t a = 0, b = 0, c = 0, d = 0;
    size_t npx = static_cast<size_t>(this->width_) * this->height_;
    if (px != nullptr && npx > 0) {
      a = px[0];
      b = px[npx / 4];
      c = px[npx / 2];
      d = px[npx - 1];
    }
    ESP_LOGD(TAG,
             "diag lvgl=%d fb=%ux%u pub=%u dec=%u drop=%u skip=%u fps=%.1f data=%p px=%04x %04x %04x %04x",
             lvgl_built ? 1 : 0, (unsigned) this->width_, (unsigned) this->height_,
             (unsigned) this->published_count_,
             (unsigned) this->decoded_count_.load(std::memory_order_relaxed),
             (unsigned) this->dropped_frames(), (unsigned) this->skipped_frames_, this->measured_fps_,
             (void *) this->data_start_, a, b, c, d);
#ifdef USE_LVGL
    ESP_LOGD(TAG, "diag dsc data=%p w=%u h=%u stride=%u cf=%u", (void *) this->dsc_.data,
             (unsigned) this->dsc_.header.w, (unsigned) this->dsc_.header.h,
             (unsigned) this->dsc_.header.stride, (unsigned) this->dsc_.header.cf);
#endif
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
  // Blank HERE, not in the network task. show() runs on the main loop, so the
  // screen clears on the tap itself. Doing it in run_stream_() meant waiting
  // for the outgoing stream's read to return and its socket to close first --
  // a few hundred milliseconds of the previous camera still on screen.
  if (static_cast<int>(index) != this->last_started_index_)
    this->blank_display_();
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
  // Next open starts from black rather than a frame that may be minutes old.
  this->last_started_index_ = -1;
  this->blank_display_();
  this->set_status_(StreamStatus::IDLE);
}

void HaLiveCamera::set_status_(StreamStatus s) {
  this->status_.store(static_cast<uint8_t>(s), std::memory_order_relaxed);
}

void HaLiveCamera::blank_display_() {
  const int8_t shown = this->displayed_index_.load(std::memory_order_acquire);
  int8_t target = -1;
  for (int8_t i = 0; i < static_cast<int8_t>(NUM_FB); i++) {
    if (i != shown) {
      target = i;
      break;
    }
  }
  if (target < 0 || this->fb_[target] == nullptr)
    return;
  memset(this->fb_[target], 0, this->fb_size_);
  this->fb_w_[target] = this->max_width_;
  this->fb_h_[target] = this->max_height_;
  this->ready_index_.store(target, std::memory_order_release);
}

std::string HaLiveCamera::base_url_() const {
  std::string url = this->frigate_url_;
  if (!url.empty() && url.back() == '/')
    url.pop_back();
  return url;
}

// Frigate answers /api/login with an empty 200 body -- the JWT is only in the
// Set-Cookie header, so we have to read headers as they arrive.
esp_err_t HaLiveCamera::login_event_(esp_http_client_event_t *evt) {
  if (evt->event_id != HTTP_EVENT_ON_HEADER)
    return ESP_OK;
  if (evt->header_key == nullptr || evt->header_value == nullptr)
    return ESP_OK;
  if (strcasecmp(evt->header_key, "Set-Cookie") != 0)
    return ESP_OK;
  auto *self = static_cast<HaLiveCamera *>(evt->user_data);
  if (self == nullptr)
    return ESP_OK;
  // "frigate_token=eyJhbGci...; Path=/; HttpOnly". Take the value whatever the
  // cookie is called -- auth.cookie_name is configurable.
  const char *eq = strchr(evt->header_value, '=');
  if (eq == nullptr)
    return ESP_OK;
  const char *start = eq + 1;
  const char *end = strchr(start, ';');
  const size_t len = (end != nullptr) ? static_cast<size_t>(end - start) : strlen(start);
  if (len == 0)
    return ESP_OK;
  self->jwt_.assign(start, len);
  return ESP_OK;
}

// Minimal JSON string escaping -- a password may legitimately contain a quote
// or a backslash, and anything else we would rather send verbatim.
static void json_escape_into(const std::string &in, std::string &out) {
  for (char ch : in) {
    if (ch == '"' || ch == '\\')
      out += '\\';
    out += ch;
  }
}

bool HaLiveCamera::login_() {
  if (this->username_.empty())
    return true;  // unauthenticated port -- nothing to do

  std::string url = this->base_url_();
  url += "/api/login";

  std::string body = "{\"user\":\"";
  json_escape_into(this->username_, body);
  body += "\",\"password\":\"";
  json_escape_into(this->password_, body);
  body += "\"}";

  esp_http_client_config_t cfg = {};
  cfg.url = url.c_str();
  cfg.method = HTTP_METHOD_POST;
  cfg.timeout_ms = 5000;
  cfg.buffer_size = 1024;
  cfg.buffer_size_tx = 1024;
  cfg.event_handler = HaLiveCamera::login_event_;
  cfg.user_data = this;

  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (client == nullptr)
    return true;  // transient; let the caller retry rather than latch a failure

  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_post_field(client, body.c_str(), body.size());

  this->jwt_.clear();
  const esp_err_t err = esp_http_client_perform(client);
  const int code = esp_http_client_get_status_code(client);
  esp_http_client_cleanup(client);

  if (err != ESP_OK) {
    ESP_LOGW(TAG, "login transport error: %s", esp_err_to_name(err));
    return true;  // network problem, not a credential problem
  }
  if (code == 401 || code == 403) {
    ESP_LOGE(TAG, "login rejected (HTTP %d) -- check username/password", code);
    return false;
  }
  if (code != 200) {
    ESP_LOGW(TAG, "login returned HTTP %d", code);
    return true;
  }
  if (this->jwt_.empty()) {
    ESP_LOGW(TAG, "login succeeded but no session cookie came back");
    return true;
  }
  ESP_LOGI(TAG, "logged in to frigate as %s", this->username_.c_str());
  return true;
}

std::string HaLiveCamera::build_url_(const CameraEntry &c) const {
  // Frigate serves MJPEG straight off its already-decoded detect stream, so
  // there is no transcoding anywhere in the chain. The same path works on
  // port 8971 (authenticated) and 5000 (not); only the Authorization header
  // added in run_stream_() differs.
  std::string url = this->base_url_();
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

  this->last_started_index_ = static_cast<int>(index);

  // A 401 below clears the token, so this re-logs in on the next pass.
  if (!this->username_.empty() && this->jwt_.empty()) {
    if (!this->login_()) {
      this->set_status_(StreamStatus::AUTH_FAILED);
      vTaskDelay(pdMS_TO_TICKS(5000));
      return;
    }
    if (this->jwt_.empty()) {
      this->set_status_(StreamStatus::CONNECTING);
      vTaskDelay(pdMS_TO_TICKS(1000));
      return;
    }
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

  std::string bearer;
  if (!this->jwt_.empty()) {
    bearer = "Bearer ";
    bearer += this->jwt_;
    esp_http_client_set_header(client, "Authorization", bearer.c_str());
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
    esp_http_client_cleanup(client);
    if (this->username_.empty()) {
      ESP_LOGE(TAG, "auth rejected (HTTP %d) and no username is configured -- "
                    "port 8971 needs username/password", code);
      this->set_status_(StreamStatus::AUTH_FAILED);
      return;
    }
    // Sessions expire (auth.session_length, 24h by default). Drop the token
    // and let the next pass log in again; only a rejected LOGIN is fatal.
    ESP_LOGI(TAG, "session expired, re-authenticating");
    this->jwt_.clear();
    this->set_status_(StreamStatus::RECONNECTING);
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

  // A frame that arrived from the camera we are switching away from would
  // land on screen after the switch. Bin it.
  if (this->active_index_.load(std::memory_order_acquire) !=
      this->requested_index_.load(std::memory_order_acquire)) {
    this->skipped_frames_++;
    return;
  }

  // Latency control. If the previously decoded frame has not been picked up
  // by loop() yet, we are producing faster than the UI can consume, and
  // decoding this one would only add to a backlog we can never work off --
  // every frame would then be displayed later than the last. Skip it.
  // Parsing is far cheaper than decoding, so skipping also lets the read loop
  // drain the socket faster, which is what actually claws back latency when
  // the stream is ahead of us.
  const int8_t shown = this->displayed_index_.load(std::memory_order_acquire);
  const int8_t ready = this->ready_index_.load(std::memory_order_acquire);
  if (ready >= 0 && ready != shown) {
    this->skipped_frames_++;
    return;
  }
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
  dec_cfg.rgb_order =
      this->rgb_order_bgr_ ? JPEG_DEC_RGB_ELEMENT_ORDER_BGR : JPEG_DEC_RGB_ELEMENT_ORDER_RGB;
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