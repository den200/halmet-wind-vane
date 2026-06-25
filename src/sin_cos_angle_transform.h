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
// float inputs without multiple inheritance. NaN-guards on low magnitude so a
// disconnected masthead reads "no data" rather than a stuck angle.
//
// Live-calibration parameters (all exposed via ConfigItem):
//   offset_rad   align the vane zero with the boat centerline.
//   sin_sign     +1 normal; set -1 if the wind reads backward (flips handedness
//                — equivalent to swapping the Blue/Green wires).
//   gain         common scale on both axes (cosmetic; atan2 is scale-invariant).
//   min_magnitude magnitude below which the angle is treated as invalid (NaN).
class SinCosAngle : public Transform<float, float> {
 public:
  SinCosAngle(float gain = 1.0f, float offset_rad = 0.0f,
              float min_magnitude = 0.05f, float sin_sign = 1.0f,
              const String& config_path = "")
      : Transform<float, float>(config_path),
        gain_(gain), offset_rad_(offset_rad), min_magnitude_(min_magnitude),
        sin_sign_(sin_sign),
        sin_consumer_([this](float v){ sin_value_=v; got_sin_=true; recompute(); }),
        cos_consumer_([this](float v){ cos_value_=v; got_cos_=true; recompute(); }) {
    this->load();
  }
  void set(const float&) override {}  // unused; inputs go to member consumers

  LambdaConsumer<float>& sin_input() { return sin_consumer_; }
  LambdaConsumer<float>& cos_input() { return cos_consumer_; }

  bool to_json(JsonObject& root) override {
    root["gain"] = gain_;
    root["offset_rad"] = offset_rad_;
    root["min_magnitude"] = min_magnitude_;
    root["sin_sign"] = sin_sign_;
    return true;
  }
  bool from_json(const JsonObject& root) override {
    if (root["gain"].is<float>())          gain_          = root["gain"];
    if (root["offset_rad"].is<float>())    offset_rad_    = root["offset_rad"];
    if (root["min_magnitude"].is<float>()) min_magnitude_ = root["min_magnitude"];
    if (root["sin_sign"].is<float>())      sin_sign_      = root["sin_sign"];
    return true;
  }

 private:
  void recompute() {
    if (!got_sin_ || !got_cos_) return;
    const float s = sin_sign_ * gain_ * sin_value_;
    const float c = gain_ * cos_value_;
    const float mag = sqrtf(s*s + c*c);
    if (mag < min_magnitude_ || !isfinite(s) || !isfinite(c)) {
      this->emit(NAN); return;
    }
    float theta = atan2f(s, c) + offset_rad_;
    while (theta >  M_PI) theta -= 2.0f * M_PI;
    while (theta <= -M_PI) theta += 2.0f * M_PI;
    this->emit(theta);
  }

  float gain_, offset_rad_, min_magnitude_, sin_sign_;
  float sin_value_ = 0.0f, cos_value_ = 0.0f;
  bool  got_sin_ = false, got_cos_ = false;
  LambdaConsumer<float> sin_consumer_, cos_consumer_;
};

// Config schema for the web UI. Property keys must match to_json/from_json.
inline const String ConfigSchema(const SinCosAngle& obj) {
  const char SCHEMA[] = R"###({
      "type": "object",
      "properties": {
          "offset_rad": { "title": "Angle offset (rad)", "type": "number", "description": "Added to the computed angle to align the vane zero with the boat centerline" },
          "sin_sign": { "title": "Angle direction sign", "type": "number", "description": "+1 normal; set -1 if the wind reads backward (clockwise-from-above is correct)" },
          "gain": { "title": "Common gain", "type": "number", "description": "Scale applied to both sin and cos (does not affect the angle; only the NaN-guard magnitude)" },
          "min_magnitude": { "title": "Min magnitude (NaN guard)", "type": "number", "description": "Below this sin/cos magnitude the angle is reported as no-data (disconnected masthead)" }
      }
    })###";
  return SCHEMA;
}

// Calibration applies live on the next recompute(); no restart needed.
inline const bool ConfigRequiresRestart(const SinCosAngle& obj) { return false; }

}  // namespace sensesp
