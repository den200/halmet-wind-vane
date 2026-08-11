#ifndef HALMET_ANALOG_H_
#define HALMET_ANALOG_H_

#include <Adafruit_ADS1X15.h>

#include "sensesp/sensors/sensor.h"
#include "sensesp_base_app.h"

namespace halmet {

// HALMET voltage divider scale factor
const float kVoltageDividerScale = 33.3 / 3.3;

sensesp::FloatProducer* ConnectTankSender(Adafruit_ADS1115* ads1115,
                                          int channel, const String& name,
                                          const String& sk_id, int sort_order,
                                          bool enable_signalk_output = true);

class ADS1115VoltageInput : public sensesp::FloatSensor {
 public:
  // A single-ended conversion takes ~7.8 ms at the ADS1115's default 128 SPS.
  // 25 ms is three times that, and bounds how long a broken bus can stall the
  // event loop.
  static constexpr uint32_t kConversionTimeoutMs = 25;

  ADS1115VoltageInput(Adafruit_ADS1115* ads1115, int channel,
                      const String& config_path,
                      unsigned int read_interval = 500,
                      float calibration_factor = 1.0)
      : sensesp::FloatSensor(config_path),
        ads1115_{ads1115},
        channel_{channel < 0 ? 0 : (channel > 3 ? 3 : channel)},
        read_interval_{read_interval},
        calibration_factor_{calibration_factor} {
    load();

    repeat_event_ = set_repeat_event(read_interval_);
  }

  void update() {
    // Deliberately not Adafruit's readADC_SingleEnded(): it busy-waits on
    // conversionComplete() with no timeout, and a failed I2C read makes that
    // condition false forever. readRegister() puts the register pointer (0x01)
    // in buffer[0] before the transfer and leaves it there when the read is not
    // ACKed, so the config word comes back as 0x01xx with the OS bit clear. One
    // loose SDA wire -- or an ADS1115 that was already missing at boot -- would
    // hang the whole board inside this 500 ms task: no N2K, no web UI, no
    // SignalK, until the watchdog rebooted it straight back into the same hang.
    ads1115_->startADCReading(MUX_BY_CHANNEL[channel_], /*continuous=*/false);
    const uint32_t started = millis();
    while (!ads1115_->conversionComplete()) {
      // Unsigned subtraction, so this stays correct across millis() rollover.
      if (millis() - started > kConversionTimeoutMs) {
        // No-data beats a stale or invented voltage. NAN propagates through the
        // centering transform, fails the plausibility window in main.cpp, and
        // is caught by the angle transform's finite check.
        this->emit(NAN);
        return;
      }
    }
    float adc_output_volts =
        ads1115_->computeVolts(ads1115_->getLastConversionResults());
    this->emit(calibration_factor_ * kVoltageDividerScale * adc_output_volts);
  }

  virtual bool to_json(JsonObject& root) override {
    root["calibration_factor"] = calibration_factor_;
    return true;
  };

  virtual bool from_json(const JsonObject& config) override {
    if (config["calibration_factor"].is<float>()) {
      calibration_factor_ = config["calibration_factor"];
      return true;
    }
    return false;
  }

 protected:
  reactesp::RepeatEvent* repeat_event_ = nullptr;

  reactesp::RepeatEvent* set_repeat_event(unsigned int read_interval) {
    if (repeat_event_ != nullptr) {
      repeat_event_->remove(sensesp::event_loop());
    }

    repeat_event_ = sensesp::event_loop()->onRepeat(
        read_interval, [this]() { this->update(); });
    return repeat_event_;
  }

 private:
  Adafruit_ADS1115* ads1115_;
  int channel_;
  unsigned int read_interval_;
  float calibration_factor_;
};

inline const String ConfigSchema(const ADS1115VoltageInput& obj) {
  const char SCHEMA[] = R"###({
      "type": "object",
      "properties": {
          "calibration_factor": { "title": "Calibration factor", "type": "number", "description": "Multiplier to apply to the raw input value" }
      }
    })###";

  return SCHEMA;
}

inline const bool ConfigRequiresRestart(const ADS1115VoltageInput& obj) {
  return true;
}

}  // namespace halmet

#endif  // HALMET_ANALOG_H_
