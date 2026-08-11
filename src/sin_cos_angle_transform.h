#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/transforms/transform.h"

namespace sensesp {

// Two-input transform: emits atan2(sin, cos) as an angle in (-π, +π].
// Uses the "member-consumer" pattern (a LambdaConsumer per input) to take two
// float inputs without multiple inheritance.
//
// Smoothing happens HERE, on the sin/cos vector, and NOT on the emitted angle.
// Averaging the angle is wrong: on either side of the ±π discontinuity the
// samples alternate between roughly +179° and -179°, and their arithmetic mean
// is 0° — running dead downwind would read as wind on the bow. Averaging the
// vector and taking atan2 of the mean is continuous everywhere on the circle.
//
// The mean is re-summed across the ring buffer on every sample instead of being
// carried in a running accumulator. n is small (≤ 32 samples at ~2 Hz) so the
// cost is irrelevant, and a running accumulator has no way back from a single
// NaN — it would stay NaN until the board is rebooted.
//
// Live-calibration parameters (all exposed via ConfigItem):
//   offset_rad    align the vane zero with the boat centerline.
//   sin_sign      +1 normal; set -1 if the wind reads backward (flips handedness
//                 — equivalent to swapping the Blue/Green wires).
//   gain          common scale on both axes (cosmetic; atan2 is scale-invariant).
//   min_magnitude magnitude below which the angle is treated as invalid (NaN).
//   samples       vector smoothing window, in samples; 1 disables smoothing.
//
// Note that min_magnitude does NOT detect an unpowered or disconnected
// transducer — with both channels at 0 V the centered vector is large and
// perfectly steady. That check belongs on the raw terminal voltage, upstream.
class SinCosAngle : public Transform<float, float> {
 public:
  static constexpr int kMaxSamples = 32;

  SinCosAngle(float gain = 1.0f, float offset_rad = 0.0f,
              float min_magnitude = 0.05f, float sin_sign = 1.0f,
              int samples = 5, const String& config_path = "")
      : Transform<float, float>(config_path),
        gain_(gain), offset_rad_(offset_rad), min_magnitude_(min_magnitude),
        sin_sign_(sin_sign), samples_(clamp_samples(samples)),
        sin_consumer_([this](float v){ sin_value_=v; fresh_sin_=true; recompute(); }),
        cos_consumer_([this](float v){ cos_value_=v; fresh_cos_=true; recompute(); }) {
    this->load();
    samples_ = clamp_samples(samples_);
    reset_window();
  }
  void set(const float&) override {}  // unused; inputs go to member consumers

  LambdaConsumer<float>& sin_input() { return sin_consumer_; }
  LambdaConsumer<float>& cos_input() { return cos_consumer_; }

  bool to_json(JsonObject& root) override {
    root["gain"] = gain_;
    root["offset_rad"] = offset_rad_;
    root["min_magnitude"] = min_magnitude_;
    root["sin_sign"] = sin_sign_;
    root["samples"] = samples_;
    return true;
  }
  bool from_json(const JsonObject& root) override {
    if (root["gain"].is<float>())          gain_          = root["gain"];
    if (root["offset_rad"].is<float>())    offset_rad_    = root["offset_rad"];
    if (root["min_magnitude"].is<float>()) min_magnitude_ = root["min_magnitude"];
    if (root["sin_sign"].is<float>())      sin_sign_      = root["sin_sign"];
    // Absent on configs saved before smoothing moved in here; keep the default.
    if (root["samples"].is<int>()) {
      const int n = clamp_samples(root["samples"]);
      if (n != samples_) {
        samples_ = n;
        reset_window();
      }
    }
    return true;
  }

 private:
  static int clamp_samples(int n) {
    if (n < 1) return 1;
    if (n > kMaxSamples) return kMaxSamples;
    return n;
  }

  void reset_window() {
    ptr_ = 0;
    filled_ = 0;
  }

  void recompute() {
    // One emit per sin/cos pair. Recomputing on each input independently would
    // publish an extra angle built from a fresh sin and a stale cos every cycle.
    if (!fresh_sin_ || !fresh_cos_) return;
    fresh_sin_ = fresh_cos_ = false;

    if (!isfinite(sin_value_) || !isfinite(cos_value_)) {
      reset_window();
      this->emit(NAN);
      return;
    }

    sin_buf_[ptr_] = sin_value_;
    cos_buf_[ptr_] = cos_value_;
    ptr_ = (ptr_ + 1) % samples_;
    if (filled_ < samples_) filled_++;

    float sin_sum = 0.0f, cos_sum = 0.0f;
    for (int i = 0; i < filled_; i++) {
      sin_sum += sin_buf_[i];
      cos_sum += cos_buf_[i];
    }
    const float s = sin_sign_ * gain_ * (sin_sum / filled_);
    const float c = gain_ * (cos_sum / filled_);

    const float mag = sqrtf(s*s + c*c);
    if (mag < min_magnitude_) {
      this->emit(NAN); return;
    }

    float theta = atan2f(s, c) + offset_rad_;
    while (theta >  M_PI) theta -= 2.0f * M_PI;
    while (theta <= -M_PI) theta += 2.0f * M_PI;
    this->emit(theta);
  }

  float gain_, offset_rad_, min_magnitude_, sin_sign_;
  int   samples_;
  float sin_value_ = 0.0f, cos_value_ = 0.0f;
  bool  fresh_sin_ = false, fresh_cos_ = false;
  float sin_buf_[kMaxSamples] = {0.0f};
  float cos_buf_[kMaxSamples] = {0.0f};
  int   ptr_ = 0, filled_ = 0;
  LambdaConsumer<float> sin_consumer_, cos_consumer_;
};

// Config schema for the web UI. Property keys must match to_json/from_json.
inline const String ConfigSchema(const SinCosAngle& obj) {
  const char SCHEMA[] = R"###({
      "type": "object",
      "properties": {
          "offset_rad": { "title": "Angle offset (rad)", "type": "number", "description": "Added to the computed angle to align the vane zero with the boat centerline" },
          "sin_sign": { "title": "Angle direction sign", "type": "number", "description": "+1 normal; set -1 if the wind reads backward (clockwise-from-above is correct)" },
          "samples": { "title": "Smoothing (samples)", "type": "integer", "description": "Sin/cos samples averaged before the angle is computed. Samples arrive at about 2 Hz, so 5 is roughly a 2.5 s window. 1 disables smoothing; 32 is the maximum" },
          "gain": { "title": "Common gain", "type": "number", "description": "Scale applied to both sin and cos (does not affect the angle; only the NaN-guard magnitude)" },
          "min_magnitude": { "title": "Min magnitude (NaN guard)", "type": "number", "description": "Below this averaged sin/cos magnitude the angle is reported as no-data. Does not detect a dead transducer; the raw-voltage check upstream does that" }
      }
    })###";
  return SCHEMA;
}

// Calibration applies live on the next recompute(); no restart needed.
inline const bool ConfigRequiresRestart(const SinCosAngle& obj) { return false; }

}  // namespace sensesp
