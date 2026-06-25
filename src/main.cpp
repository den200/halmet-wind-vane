// HALMET wind interface — V1 (NMEA 2000 PGN 130306).
//
// STEP 5: NMEA 2000 transmit (the V1 deliverable). Adds the wind-sensor N2K
// node and transmits PGN 130306 (Wind Data, Apparent) every 100 ms from the
// latched angle/speed. Device identity is a wind sensor (Function 130 /
// Class 85 / unregistered manufacturer 2046), source address 35.
//
// Bench note: a lone CAN node has nothing to ACK its frames, so on the wire the
// frames only complete once another node is present (the boat backbone). On the
// bench we verify the stack opens, the 100 ms loop runs, and SendMsg queues.

#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <Wire.h>
#include <esp_log.h>

#include "sensesp/sensors/digital_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/transforms/throttle.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"

#include "halmet_const.h"             // pin map, ADS address
#include "halmet_analog.h"            // ADS1115VoltageInput, kVoltageDividerScale
#include "halmet_serial.h"            // GetBoardSerialNumber()
#include "help_note.h"                // read-only web-UI help cards
#include "sin_cos_angle_transform.h"  // SinCosAngle

using namespace sensesp;
using namespace halmet;

static const char* kTag = "halmet-wind";

// Bench test signal: drive a square wave on GPIO33 to exercise the D1 pulse
// counter without the transducer. Set false (or remove) for deployment.
static constexpr bool kEnableTestPwm = false;
static constexpr int kTestOutputPin = 33;
static constexpr uint32_t kTestPwmHz = 20;  // ~20 Hz ≈ strong wind for the bench

TwoWire* i2c = nullptr;
Adafruit_ADS1115* ads = nullptr;
tNMEA2000* nmea2000 = nullptr;

// Latest values, latched from the pipeline for N2K send + serial reporting.
static float last_sin_v = NAN;
static float last_cos_v = NAN;
static float last_awa_rad = NAN;
static float last_hz = NAN;
static float last_aws_mps = NAN;

// PGN 130306 sequence id + transmit counters (bench visibility).
static uint8_t sid = 0;
static uint32_t n2k_tx_ok = 0;
static uint32_t n2k_tx_fail = 0;

// Register a read-only help card in the web UI config page.
static void add_help_card(const char* path, const char* title,
                          const char* description, int sort_order) {
  auto* note = new HelpNote(path);
  ConfigItem(note)
      ->set_title(title)
      ->set_description(description)
      ->set_sort_order(sort_order);
}

// Wiring/power/calibration reference, shown as cards in the web UI. The
// descriptions are rendered as HTML by the frontend. Full version: docs/WIRING.md.
// Sort 1000+ places them between the System cards and the calibration cards.
static void add_help_cards() {
  add_help_card(
      "/help/1-power", "⚠ Power & the 8 V buck — read first",
      "<b style='color:#c0392b'>The buck converter ships at ~20 V output. The "
      "masthead needs 8.0 V — 20 V can destroy it.</b><br><br>"
      "With <b>nothing connected to the buck output</b>:"
      "<ul>"
      "<li>Power the buck from 12 V (input side).</li>"
      "<li>Turn the <b>trim pot</b> until a <b>multimeter</b> reads "
      "<b>8.0 V</b> (±0.25). Don't trust the LED meter alone.</li>"
      "<li>If the output won't drop below the input, turn the pot "
      "<b>counter-clockwise 10+ turns</b> first.</li>"
      "<li>The push-button 'calibration' only trims the LED display, not the "
      "output — the output is set by the <b>pot</b>.</li>"
      "</ul>"
      "<b>Power:</b> HALMET runs off the NMEA 2000 backbone. Feed the buck "
      "input from boat 12 V via a ~1 A fuse. Buck OUT+ &rarr; transducer Red.",
      1000);
  add_help_card(
      "/help/2-ground", "Grounding (important)",
      "Sin/cos are measured <b>relative to HALMET's input ground</b>, so the "
      "transducer's ground must be that same node. Tie together at one point:"
      "<ul>"
      "<li>Buck OUT&minus; (= buck IN&minus;; these modules are non-isolated)</li>"
      "<li>Transducer Screen / bare wire</li>"
      "<li>HALMET's <b>input-side GND</b> (ground terminal on the analog-input "
      "side)</li>"
      "</ul>"
      "Without this shared reference the angle reads garbage.<br>"
      "<b>Do not</b> tie the sensor ground to HALMET's N2K-connector ground — "
      "the board isolates the sensor inputs from the bus on purpose.",
      1001);
  add_help_card(
      "/help/3-wiring", "Transducer wires & jumpers",
      "<b>Raymarine ST60+ masthead &rarr; HALMET:</b>"
      "<ul>"
      "<li>Red &rarr; buck +8 V</li>"
      "<li>Screen/bare &rarr; shared sensor ground (see Grounding)</li>"
      "<li>Blue (sine) &rarr; A1</li>"
      "<li>Green (cosine) &rarr; A2</li>"
      "<li>Yellow (speed pulse) &rarr; D1</li>"
      "</ul>"
      "<b>Jumpers:</b>"
      "<ul>"
      "<li>A1 &amp; A2: constant-current-source (CCS) <b>OFF</b> (passive "
      "voltage mode)</li>"
      "<li>D1: no pull-up; engage the 2.3 kHz low-pass only if you see noise</li>"
      "<li>On-board 120 &Omega; CAN termination: <b>OPEN</b> (backbone already "
      "terminated)</li>"
      "</ul>"
      "If the angle reads backward after calibration, set <b>sin_sign = "
      "&minus;1</b> on the Wind angle card.",
      1002);
  add_help_card(
      "/help/4-calibrate", "Calibration",
      "All values below are live — no reflash."
      "<ul>"
      "<li><b>Speed multiplier K:</b> 0.5144 for the egg-cup ST60+ (~1 kn/Hz); "
      "~0.36 for square-cup. Trim against GPS in calm air.</li>"
      "<li><b>Per-channel centering:</b> free-rotate the vane, watch sin/cos "
      "volts, set each offset so the midpoint &asymp; Vmid (~4.0 V).</li>"
      "<li><b>Angle offset:</b> set so 0&deg; = bow.</li>"
      "<li><b>Angle direction (sin_sign):</b> +1 normal, &minus;1 if it reads "
      "backward.</li>"
      "</ul>",
      1003);
}

void setup() {
  SetupLogging();

  // Hostname doubles as the WiFi AP SSID and mDNS name (halmet-wind.local).
  SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("halmet-wind")->get_app();

  // Wiring/power/calibration help, shown as read-only cards in the web UI.
  add_help_cards();

  // ----- I2C + ADS1115 -----
  // GAIN_ONE = ±4.096 V FS at the ADS pin. Masthead sin/cos swing ~2.5-5.5 V
  // at the terminal → ~0.25-0.55 V after the 10:1 divider — well inside range.
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);
  ads = new Adafruit_ADS1115();
  ads->setGain(GAIN_ONE);
  bool ads_ok = ads->begin(kADS1115Address, i2c);
  if (ads_ok) {
    ESP_LOGI(kTag, "ADS1115 found at 0x%02X", kADS1115Address);
  } else {
    ESP_LOGE(kTag, "ADS1115 NOT found at 0x%02X — check I2C wiring/power",
             kADS1115Address);
  }

  // ----- Wind angle pipeline: A1 = sine, A2 = cosine -----
  // Every configurable stage is wrapped in ConfigItem so it is editable live
  // from the web UI (http://halmet-wind.local/) and persisted to flash — no
  // reflash needed to calibrate.
  auto* ch_sin = new ADS1115VoltageInput(ads, 0, "/wind/sin", 500);
  auto* ch_cos = new ADS1115VoltageInput(ads, 1, "/wind/cos", 500);
  ConfigItem(ch_sin)
      ->set_title("Sin channel (A1) voltage trim")
      ->set_sort_order(2000);
  ConfigItem(ch_cos)
      ->set_title("Cos channel (A2) voltage trim")
      ->set_sort_order(2001);
  ch_sin->connect_to(new LambdaConsumer<float>([](float v) { last_sin_v = v; }));
  ch_cos->connect_to(new LambdaConsumer<float>([](float v) { last_cos_v = v; }));

  // Per-channel centering: terminal volts → (V − Vmid)/amplitude. The offset
  // (−Vmid·slope) sets the midpoint; atan2 is scale-invariant so the slope only
  // matters for the NaN-guard magnitude. CALIBRATION PARAM: per-channel Vmid.
  auto* sin_cal = new Linear(1.0f / 1.5f, -4.0f / 1.5f, "/wind/sin/cal");
  auto* cos_cal = new Linear(1.0f / 1.5f, -4.0f / 1.5f, "/wind/cos/cal");
  ConfigItem(sin_cal)
      ->set_title("Sin centering (slope, −Vmid·slope)")
      ->set_description("Default slope 1/1.5, offset −4.0/1.5 (Vmid≈4.0 V). "
                        "Refine offset from logged free-rotation data.")
      ->set_sort_order(2002);
  ConfigItem(cos_cal)
      ->set_title("Cos centering (slope, −Vmid·slope)")
      ->set_description("Default slope 1/1.5, offset −4.0/1.5 (Vmid≈4.0 V).")
      ->set_sort_order(2003);
  ch_sin->connect_to(sin_cal);
  ch_cos->connect_to(cos_cal);

  // atan2(sin, cos). CALIBRATION PARAMS: angle offset (align zero) + sin_sign
  // (flip if wind reads backward). offset/sign/gain live in this transform.
  auto* angle = new SinCosAngle(1.0f, 0.0f, 0.05f, 1.0f, "/wind/angle/cal");
  ConfigItem(angle)
      ->set_title("Wind angle (offset + direction)")
      ->set_description("offset_rad aligns the vane zero to the boat centerline; "
                        "sin_sign = −1 if the wind reads backward.")
      ->set_sort_order(2100);
  sin_cal->connect_to(&angle->sin_input());
  cos_cal->connect_to(&angle->cos_input());

  auto* angle_smooth = new MovingAverage(5, 1.0f, "/wind/angle/smooth");
  ConfigItem(angle_smooth)
      ->set_title("Angle smoothing (samples)")
      ->set_sort_order(2101);
  angle->connect_to(angle_smooth);
  angle_smooth->connect_to(
      new LambdaConsumer<float>([](float v) { last_awa_rad = v; }));

  // V2: SignalK apparent wind angle. SK wants rad in −π..+π (negative to port,
  // 0 = bow) — exactly what angle_smooth already emits; the N2K path does its
  // own 0..2π conversion, so both outputs share one calibration. The SK server
  // connection + auth are handled entirely by SensESP's built-in SignalK page;
  // we only add the producer. Throttle caps the WebSocket rate — a no-op at our
  // ~2 Hz sample rate, but it guards the socket if sampling is ever sped up.
  auto* awa_meta =
      new SKMetadata("rad", "Apparent Wind Angle",
                     "Apparent wind angle, negative to port, 0 = bow", "AWA");
  angle_smooth->connect_to(new Throttle<float>(100))
      ->connect_to(new SKOutputFloat("environment.wind.angleApparent",
                                     "/wind/angle/sk", awa_meta));

  // ----- Wind speed pipeline: D1 = Yellow pulse -----
  auto* tach = new DigitalInputCounter(kDigitalInputPin1, INPUT, RISING, 500);
  auto* freq = new Frequency(1.0f, "/wind/freq");
  // CALIBRATION PARAM: speed multiplier K (m/s per Hz). 0.5144 = 1 kn/Hz
  // (egg-cup ST60+); ~0.36 for square-cup. Set empirically.
  auto* speed_cal = new Linear(0.5144f, 0.0f, "/wind/speed/cal");
  ConfigItem(freq)
      ->set_title("Pulse frequency multiplier")
      ->set_sort_order(2200);
  ConfigItem(speed_cal)
      ->set_title("Speed multiplier K (m/s per Hz)")
      ->set_description("Default 0.5144 ≈ 1 knot/Hz (egg-cup). ~0.36 for "
                        "square-cup. Calibrate against GPS in calm air.")
      ->set_sort_order(2201);
  tach->connect_to(freq)->connect_to(speed_cal);
  freq->connect_to(new LambdaConsumer<float>([](float v) { last_hz = v; }));
  speed_cal->connect_to(
      new LambdaConsumer<float>([](float v) { last_aws_mps = v; }));

  // V2: SignalK apparent wind speed (m/s). No throttle needed — the 500 ms
  // counter window already limits this to ~2 Hz.
  auto* aws_meta = new SKMetadata("m/s", "Apparent Wind Speed", "", "AWS");
  speed_cal->connect_to(new SKOutputFloat("environment.wind.speedApparent",
                                          "/wind/speed/sk", aws_meta));

  // ----- Bench test signal (jumper GPIO33 → D1 to verify counting) -----
  if (kEnableTestPwm) {
    pinMode(kTestOutputPin, OUTPUT);
    event_loop()->onRepeat(1000 / (2 * kTestPwmHz), []() {
      static bool level = false;
      level = !level;
      digitalWrite(kTestOutputPin, level);
    });
    ESP_LOGI(kTag, "Test signal: ~%u Hz square wave on GPIO%d (jumper to D1)",
             (unsigned)kTestPwmHz, kTestOutputPin);
  }

  // ----- NMEA 2000 (V1): PGN 130306 apparent wind sensor -----
  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);
  nmea2000->SetN2kCANSendFrameBufSize(150);
  nmea2000->SetN2kCANReceiveFrameBufSize(150);
  nmea2000->SetProductInformation("20260601", 140, "HALMET Wind", "1.0.0",
                                  "1.0.0");
  nmea2000->SetDeviceInformation(GetBoardSerialNumber(),
                                 130,    // Function: Atmospheric
                                 85,     // Class: External Environment
                                 2046);  // Manufacturer (unregistered)
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 35);
  nmea2000->EnableForward(false);
  static const unsigned long kTxPGNs[] PROGMEM = {130306L, 0};
  nmea2000->ExtendTransmitMessages(kTxPGNs);
  if (nmea2000->Open()) {
    ESP_LOGI(kTag, "NMEA2000 opened — wind sensor src 35, TX PGN 130306");
  } else {
    ESP_LOGE(kTag, "NMEA2000 Open() failed");
  }
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  // PGN 130306 every 100 ms from the latched values. Angle field is 0..2π;
  // convert from the SignalK convention (−π..+π). NaN → N2kDoubleNA.
  event_loop()->onRepeat(100, []() {
    if (!isfinite(last_awa_rad) && !isfinite(last_aws_mps)) return;
    tN2kMsg msg;
    double awa = isfinite(last_awa_rad)
                     ? (last_awa_rad < 0 ? last_awa_rad + 2.0 * M_PI
                                         : last_awa_rad)
                     : N2kDoubleNA;
    double aws = isfinite(last_aws_mps) ? last_aws_mps : N2kDoubleNA;
    SetN2kWindSpeed(msg, sid, aws, awa, N2kWind_Apparent);
    if (nmea2000->SendMsg(msg)) {
      n2k_tx_ok++;
    } else {
      n2k_tx_fail++;
    }
    sid = (sid >= 252) ? 0 : sid + 1;
  });

  ESP_LOGI(kTag, "HALMET wind scaffold booted — angle + speed + N2K up");

  // 1 Hz report: raw sin/cos volts, computed angle, pulse Hz, speed, tx counts.
  event_loop()->onRepeat(1000, []() {
    static uint32_t beat = 0;
    float awa_deg =
        isfinite(last_awa_rad) ? last_awa_rad * 180.0f / (float)M_PI : NAN;
    ESP_LOGI(kTag,
             "tick %lu  sin=%.3fV cos=%.3fV  AWA=%.4f rad (%.1f deg)  "
             "pulse=%.1f Hz  AWS=%.2f m/s  n2k_tx=%lu fail=%lu",
             (unsigned long)beat++, last_sin_v, last_cos_v, last_awa_rad,
             awa_deg, last_hz, last_aws_mps, (unsigned long)n2k_tx_ok,
             (unsigned long)n2k_tx_fail);
  });
}

void loop() { event_loop()->tick(); }
