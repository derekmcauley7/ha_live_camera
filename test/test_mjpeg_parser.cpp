// Host unit tests for MjpegParser.
//
// Replays each generated stream through the parser at many chunk sizes --
// including 1 byte at a time, which splits every boundary, header and JPEG
// marker across calls -- and asserts the recovered frames are byte-identical
// to the ground truth.
//
// Build & run:  make -C test

#include "../components/ha_live_camera/mjpeg_parser.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using esphome::ha_live_camera::MjpegParser;

static std::vector<uint8_t> read_file(const std::string &path) {
  FILE *f = fopen(path.c_str(), "rb");
  if (f == nullptr) {
    fprintf(stderr, "FATAL: cannot open %s\n", path.c_str());
    exit(2);
  }
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::vector<uint8_t> out(static_cast<size_t>(n));
  if (n > 0 && fread(out.data(), 1, static_cast<size_t>(n), f) != static_cast<size_t>(n)) {
    fprintf(stderr, "FATAL: short read on %s\n", path.c_str());
    exit(2);
  }
  fclose(f);
  return out;
}

struct Collector {
  std::vector<std::vector<uint8_t>> frames;
};

static void on_frame(void *ctx, const uint8_t *data, size_t len) {
  auto *c = static_cast<Collector *>(ctx);
  c->frames.emplace_back(data, data + len);
}

static int g_failures = 0;
static int g_checks = 0;

static void check(bool cond, const std::string &what) {
  g_checks++;
  if (!cond) {
    g_failures++;
    printf("  FAIL: %s\n", what.c_str());
  }
}

static const char *DATA_DIR = "data/";

int main() {
  // Ground truth
  std::vector<std::vector<uint8_t>> truth;
  for (int i = 0;; i++) {
    std::string p = std::string(DATA_DIR) + "frame_" + std::to_string(i) + ".jpg";
    FILE *f = fopen(p.c_str(), "rb");
    if (f == nullptr)
      break;
    fclose(f);
    truth.push_back(read_file(p));
  }
  printf("ground truth: %zu frames\n", truth.size());
  check(truth.size() == 5, "expected 5 ground-truth frames");

  // Every generated stream carries all 5 frames, except the two special cases.
  struct Case {
    const char *name;
    size_t expect;
  };
  const Case cases[] = {
      {"frigate", 5},       {"go2rtc", 5},   {"ffmpeg", 5},
      {"ha_still", 5},      {"noisy", 5},    {"frigate_http", 5},
      {"truncated", 5},  // the 6th, half-written frame must NOT be emitted
  };

  const size_t chunk_sizes[] = {1, 2, 3, 7, 13, 64, 251, 1024, 4096, 65536};

  const size_t CAP = 512 * 1024;
  std::vector<uint8_t> buf(CAP);

  for (const auto &c : cases) {
    std::string path = std::string(DATA_DIR) + "stream_" + c.name + ".bin";
    std::vector<uint8_t> stream = read_file(path);
    printf("stream %-14s (%zu bytes)\n", c.name, stream.size());

    for (size_t cs : chunk_sizes) {
      Collector col;
      MjpegParser p;
      p.set_buffer(buf.data(), CAP);
      p.set_on_frame(on_frame, &col);

      for (size_t off = 0; off < stream.size(); off += cs) {
        size_t n = std::min(cs, stream.size() - off);
        p.feed(stream.data() + off, n);
      }

      std::string tag = std::string(c.name) + " @chunk=" + std::to_string(cs);
      check(col.frames.size() == c.expect,
            tag + ": got " + std::to_string(col.frames.size()) + " frames, expected " +
                std::to_string(c.expect));

      size_t n = std::min(col.frames.size(), truth.size());
      for (size_t i = 0; i < n; i++) {
        bool same = col.frames[i].size() == truth[i].size() &&
                    memcmp(col.frames[i].data(), truth[i].data(), truth[i].size()) == 0;
        check(same, tag + ": frame " + std::to_string(i) + " mismatch (got " +
                        std::to_string(col.frames[i].size()) + " bytes, want " +
                        std::to_string(truth[i].size()) + ")");
      }
    }
  }

  // --- Capacity overflow: an oversized frame is dropped, and the parser
  // recovers to decode the following frames correctly. ---
  {
    std::vector<uint8_t> stream = read_file(std::string(DATA_DIR) + "stream_frigate.bin");
    Collector col;
    MjpegParser p;
    const size_t small = 3000;  // smaller than frames 1 and 3
    std::vector<uint8_t> sbuf(small);
    p.set_buffer(sbuf.data(), small);
    p.set_on_frame(on_frame, &col);
    p.feed(stream.data(), stream.size());

    check(p.frames_dropped() > 0, "overflow: expected dropped frames");
    check(col.frames.size() > 0, "overflow: expected some frames to still decode");
    for (const auto &f : col.frames)
      check(f.size() <= small, "overflow: emitted frame exceeds capacity");
    // Frames that do fit must still be byte-exact.
    for (const auto &f : col.frames) {
      bool matched = false;
      for (const auto &t : truth)
        if (f.size() == t.size() && memcmp(f.data(), t.data(), t.size()) == 0)
          matched = true;
      check(matched, "overflow: emitted frame does not match any ground truth");
    }
    printf("overflow case: emitted=%zu dropped=%u\n", col.frames.size(), p.frames_dropped());
  }

  // --- reset() mid-frame must not merge a partial frame into the next. ---
  {
    std::vector<uint8_t> stream = read_file(std::string(DATA_DIR) + "stream_frigate.bin");
    Collector col;
    MjpegParser p;
    p.set_buffer(buf.data(), CAP);
    p.set_on_frame(on_frame, &col);
    p.feed(stream.data(), 1500);  // lands mid-frame-0
    check(p.in_frame(), "reset: expected to be mid-frame");
    p.reset();
    p.feed(stream.data() + 1500, stream.size() - 1500);
    // Frame 0 is lost; the remaining 4 must be intact.
    check(col.frames.size() == 4, "reset: expected 4 frames after mid-frame reset, got " +
                                      std::to_string(col.frames.size()));
    for (size_t i = 0; i < col.frames.size() && i + 1 < truth.size(); i++) {
      bool same = col.frames[i].size() == truth[i + 1].size() &&
                  memcmp(col.frames[i].data(), truth[i + 1].data(), truth[i + 1].size()) == 0;
      check(same, "reset: frame " + std::to_string(i) + " mismatch");
    }
  }

  printf("\n%d checks, %d failures\n", g_checks, g_failures);
  if (g_failures == 0)
    printf("ALL TESTS PASSED\n");
  return g_failures == 0 ? 0 : 1;
}
