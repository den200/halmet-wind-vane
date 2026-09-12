// HALMET wind interface — V1 (NMEA 2000 PGN 130306).
//
// Written with Claude Code (Anthropic). Licensed under Apache-2.0.
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
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
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

// Raw-voltage plausibility window for the sin/cos channels, at the terminal.
// Raymarine specs Blue and Green at 2-6 V against the screen; a healthy ST60+
// sits at 2.5-5.5 V. This is the only reliable way to spot an unpowered,
// shorted or disconnected transducer: with both channels sitting at 0 V the
// centered vector is large and perfectly steady, so the magnitude guard inside
// SinCosAngle sees nothing wrong and happily reports a convincing -135°.
static constexpr float kSensorVMin = 2.0f;
static constexpr float kSensorVMax = 6.0f;

// False for NaN too, so an ADS read that timed out fails the window as well.
static inline bool volts_plausible(float v) {
  return v >= kSensorVMin && v <= kSensorVMax;
}
static inline bool angle_volts_ok() {
  return volts_plausible(last_sin_v) && volts_plausible(last_cos_v);
}
// Finite volts outside the window mean the transducer itself has no 8 V, is
// shorted or is unplugged — and then its pulse output is dead too, so the 0 Hz
// on Yellow is a lie, not calm air. NaN means the ADS did not answer, a
// board-side fault that says nothing about the cups, so speed is left alone.
static inline bool transducer_dead() {
  return (isfinite(last_sin_v) && !volts_plausible(last_sin_v)) ||
         (isfinite(last_cos_v) && !volts_plausible(last_cos_v));
}

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

// Shared inline styles for the help cards. The SensESP frontend renders a
// ConfigItem description as HTML but gives us no stylesheet hook, so every rule
// has to travel inline on the element.
#define HN_UL "margin:.45em 0 .2em;padding-left:1.15em"
#define HN_LI "margin:.22em 0"
#define HN_TD "padding:5px 10px 5px 0;vertical-align:top"
#define HN_SW                                                       \
  "display:inline-block;width:11px;height:11px;border-radius:3px;"  \
  "margin-right:8px;border:1px solid rgba(128,128,128,.45)"
#define HN_KEY "font-weight:600;white-space:nowrap"

// Wiring/power/calibration reference, shown as cards in the web UI. Full
// version: docs/WIRING.md. Sort 999+ places them between the System cards and
// the calibration cards; the numbered titles give them a reading order.
static void add_help_cards() {
  add_help_card(
      "/help/0-start", "Wind interface — start here",
      R"HTML(<p style="margin:0 0 .5em">Raymarine ST60+ masthead vane read into
NMEA 2000 as PGN 130306 (apparent wind), plus SignalK over WiFi.</p>
<p style="margin:0 0 .3em">Work through the cards in order:</p>
<ol style=")HTML" HN_UL R"HTML(">
<li style=")HTML" HN_LI R"HTML("><b>Power</b> — set the buck to 8.0 V <i>before</i>
anything is connected to it.</li>
<li style=")HTML" HN_LI R"HTML("><b>Grounding</b> — one shared ground node.</li>
<li style=")HTML" HN_LI R"HTML("><b>Wiring</b> — five wires, three jumpers.</li>
<li style=")HTML" HN_LI R"HTML("><b>Calibration</b> — the cards below this one.
All live; no reflash.</li>
</ol>
<p style="margin:.5em 0 0">Angle 0&deg; is the bow, positive clockwise from
above. Apparent wind only — true wind is left to the plotter.</p>)HTML",
      999);
  add_help_card(
      "/help/1-power", "1 · Power and the 8 V buck — read first",
      R"HTML(<p style="margin:0 0 .5em;padding:8px 11px;border-left:3px solid #c0392b;
background:rgba(192,57,43,.08);border-radius:0 4px 4px 0"><b>The buck converter
ships at about 20 V output. The masthead needs 8.0 V — 20 V can destroy
it.</b></p>
<p style="margin:0 0 .2em">With <b>nothing connected to the buck output</b>:</p>
<ul style=")HTML" HN_UL R"HTML(">
<li style=")HTML" HN_LI R"HTML(">Power the buck from 12 V on the input side.</li>
<li style=")HTML" HN_LI R"HTML(">Turn the <b>trim pot</b> until a <b>multimeter</b>
reads <b>8.0 V</b> (&plusmn;0.25). Don't trust the LED meter alone.</li>
<li style=")HTML" HN_LI R"HTML(">If the output won't drop below the input, turn the
pot <b>counter-clockwise 10+ turns</b> first.</li>
<li style=")HTML" HN_LI R"HTML(">The push-button marked "calibration" trims the LED
display only. The <b>pot</b> sets the output.</li>
</ul>
<p style="margin:.5em 0 0">HALMET itself runs off the NMEA 2000 backbone. Feed
the buck input from boat 12 V through a ~1 A fuse; buck OUT+ goes to the
transducer's Red wire.</p>)HTML",
      1000);
  add_help_card(
      "/help/2-ground", "2 · Grounding",
      R"HTML(<p style="margin:0 0 .2em">Sin and cos are measured <b>relative to
HALMET's input ground</b>, so the transducer's ground has to be that same node.
Tie these together at one point:</p>
<ul style=")HTML" HN_UL R"HTML(">
<li style=")HTML" HN_LI R"HTML(">Buck OUT&minus; (= buck IN&minus;; these modules are
non-isolated)</li>
<li style=")HTML" HN_LI R"HTML(">Transducer Screen / bare wire</li>
<li style=")HTML" HN_LI R"HTML("><b>HALMET input-side GND</b> — the ground terminal
on the analog-input side</li>
</ul>
<p style="margin:.5em 0 0">Without that shared reference the angle reads
garbage. <b>Do not</b> tie the sensor ground to HALMET's N2K-connector ground:
the board isolates the sensor inputs from the bus on purpose.</p>)HTML",
      1001);
  add_help_card(
      "/help/3-wiring", "3 · Transducer wires and jumpers",
      R"HTML(<p style="margin:0 0 .4em">Raymarine ST60+ masthead &rarr; HALMET:</p>
<table style="border-collapse:collapse;margin:0 0 .7em"><tbody>
<tr><td style=")HTML" HN_TD ";" HN_KEY R"HTML("><span style=")HTML" HN_SW
      R"HTML(;background:#cc2b2b"></span>Red</td>
<td style=")HTML" HN_TD R"HTML(">+8.0 V from the buck</td></tr>
<tr><td style=")HTML" HN_TD ";" HN_KEY R"HTML("><span style=")HTML" HN_SW
      R"HTML(;background:#9aa7b0"></span>Screen</td>
<td style=")HTML" HN_TD R"HTML(">shared sensor ground (see Grounding)</td></tr>
<tr><td style=")HTML" HN_TD ";" HN_KEY R"HTML("><span style=")HTML" HN_SW
      R"HTML(;background:#2b6fd6"></span>Blue</td>
<td style=")HTML" HN_TD R"HTML(">sine &rarr; <b>A1</b></td></tr>
<tr><td style=")HTML" HN_TD ";" HN_KEY R"HTML("><span style=")HTML" HN_SW
      R"HTML(;background:#2faa55"></span>Green</td>
<td style=")HTML" HN_TD R"HTML(">cosine &rarr; <b>A2</b></td></tr>
<tr><td style=")HTML" HN_TD ";" HN_KEY R"HTML("><span style=")HTML" HN_SW
      R"HTML(;background:#f2b134"></span>Yellow</td>
<td style=")HTML" HN_TD R"HTML(">speed pulse &rarr; <b>D1</b> (GPIO23, rising
edge)</td></tr>
</tbody></table>
<p style="margin:0 0 .2em">Jumpers:</p>
<ul style=")HTML" HN_UL R"HTML(">
<li style=")HTML" HN_LI R"HTML(">A1 and A2 constant-current source (CCS):
<b>OFF</b> — this is passive voltage sensing</li>
<li style=")HTML" HN_LI R"HTML(">D1: no pull-up. Engage the 2.3 kHz low-pass only
if you see noise</li>
<li style=")HTML" HN_LI R"HTML(">On-board 120 &Omega; CAN termination: <b>OPEN</b>
— the backbone is already terminated at both ends</li>
</ul>
<p style="margin:.5em 0 0">If the angle reads backward once calibrated, set
<b>Angle direction sign</b> to &minus;1 on the Wind angle card. Swapping Blue
and Green does the same thing in hardware.</p>)HTML",
      1002);
  add_help_card(
      "/help/4-calibrate", "4 · Calibration — what the cards below do",
      R"HTML(<p style="margin:0 0 .4em">Every value below is live and persisted to
flash. No reflash, no reboot.</p>
<table style="border-collapse:collapse;margin:0"><tbody>
<tr><td style=")HTML" HN_TD ";" HN_KEY R"HTML(">Speed multiplier K</td>
<td style=")HTML" HN_TD R"HTML(">0.5144 for the egg-cup ST60+ (about 1 kn/Hz);
~0.36 for the older square-cup. Trim against GPS SOG in calm air.</td></tr>
<tr><td style=")HTML" HN_TD ";" HN_KEY R"HTML(">Channel centering</td>
<td style=")HTML" HN_TD R"HTML(">Free-rotate the vane, watch the sin/cos volts on
the serial console, and set each offset so the midpoint lands on Vmid
(&asymp;4.0 V). <code>tools/ellipse_fit.py</code> derives all four numbers from a
logged rotation.</td></tr>
<tr><td style=")HTML" HN_TD ";" HN_KEY R"HTML(">Angle offset</td>
<td style=")HTML" HN_TD R"HTML(">Rotates the whole scale so 0&deg; sits on the
bow.</td></tr>
<tr><td style=")HTML" HN_TD ";" HN_KEY R"HTML(">Angle direction</td>
<td style=")HTML" HN_TD R"HTML(">+1 normal, &minus;1 if the wind reads backward.
Clockwise-from-above is correct.</td></tr>
</tbody></table>)HTML",
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
      ->set_title("Angle · A1 sine — input voltage trim")
      ->set_description("Multiplier on the terminal voltage read from A1. Leave "
                        "at 1.0 unless the reported volts disagree with a "
                        "multimeter at the terminal.")
      ->set_sort_order(2000);
  ConfigItem(ch_cos)
      ->set_title("Angle · A2 cosine — input voltage trim")
      ->set_description("Multiplier on the terminal voltage read from A2. Leave "
                        "at 1.0 unless the reported volts disagree with a "
                        "multimeter at the terminal.")
      ->set_sort_order(2001);
  // Connected before the calibration chain below, and Observable notifies in
  // connection order, so the latched volts are current by the time the speed
  // gate and the 1 Hz report look at them.
  ch_sin->connect_to(new LambdaConsumer<float>([](float v) { last_sin_v = v; }));
  ch_cos->connect_to(new LambdaConsumer<float>([](float v) { last_cos_v = v; }));

  // Validity gate, per channel and BEFORE the centering: an implausible or
  // missing reading becomes NaN here, and NaN is the one input SinCosAngle
  // already handles completely — it clears its smoothing window and emits
  // no-data, so both outputs see NaN and no garbage sample survives inside the
  // average to blend into the first angles after the transducer comes back.
  // Gating the emitted angle instead left the window full of centred 0 V
  // samples, i.e. several cycles of a convincing wrong angle on every recovery.
  auto* sin_gate = new LambdaTransform<float, float>(
      [](float v) { return volts_plausible(v) ? v : NAN; });
  auto* cos_gate = new LambdaTransform<float, float>(
      [](float v) { return volts_plausible(v) ? v : NAN; });

  // Per-channel centering: terminal volts → (V − Vmid)/amplitude. The offset
  // (−Vmid·slope) sets the midpoint; atan2 is scale-invariant so the slope only
  // matters for the NaN-guard magnitude. CALIBRATION PARAM: per-channel Vmid.
  auto* sin_cal = new Linear(1.0f / 1.5f, -4.0f / 1.5f, "/wind/sin/cal");
  auto* cos_cal = new Linear(1.0f / 1.5f, -4.0f / 1.5f, "/wind/cos/cal");
  ConfigItem(sin_cal)
      ->set_title("Angle · A1 sine — centering")
      ->set_description(
          "Maps terminal volts onto a unit sine. Slope = 1/amplitude, offset = "
          "−Vmid/amplitude. Defaults are 1/1.5 and −4.0/1.5, i.e. Vmid 4.0 V "
          "swinging ±1.5 V. Refine from a logged free rotation.")
      ->set_sort_order(2002);
  ConfigItem(cos_cal)
      ->set_title("Angle · A2 cosine — centering")
      ->set_description(
          "Same mapping for the cosine channel. Slope = 1/amplitude, offset = "
          "−Vmid/amplitude. Both channels share one Vmid on a healthy sensor.")
      ->set_sort_order(2003);
  ch_sin->connect_to(sin_gate)->connect_to(sin_cal);
  ch_cos->connect_to(cos_gate)->connect_to(cos_cal);

  // atan2(sin, cos), with the smoothing done on the sin/cos vector inside the
  // transform — averaging the emitted angle instead would break at the ±π wrap
  // and report dead astern as dead ahead. CALIBRATION PARAMS: angle offset
  // (align zero), sin_sign (flip if wind reads backward), smoothing samples.
  auto* angle =
      new SinCosAngle(1.0f, 0.0f, 0.05f, 1.0f, 5, "/wind/angle/cal");
  ConfigItem(angle)
      ->set_title("Angle · zero offset, direction and smoothing")
      ->set_description(
          "Angle offset (rad) rotates the scale so 0° sits on the bow. "
          "Direction sign is +1 normally, −1 if the wind reads backward — "
          "clockwise from above is correct. Smoothing averages the sin/cos "
          "vector, so it stays correct through dead astern.")
      ->set_sort_order(2100);
  sin_cal->connect_to(&angle->sin_input());
  cos_cal->connect_to(&angle->cos_input());

  angle->connect_to(
      new LambdaConsumer<float>([](float v) { last_awa_rad = v; }));

  // V2: SignalK apparent wind angle. SK wants rad in −π..+π (negative to port,
  // 0 = bow) — exactly what the transform emits; the N2K path does its own
  // 0..2π conversion, so both outputs share one calibration. The SK server
  // connection + auth are handled entirely by SensESP's built-in SignalK page;
  // we only add the producer. Throttle caps the WebSocket rate — a no-op at our
  // ~2 Hz sample rate, but it guards the socket if sampling is ever sped up.
  // ArduinoJson is built with ARDUINOJSON_ENABLE_NAN=0, so a gated-out NaN goes
  // on the wire as JSON null — which is what SignalK means by no data.
  auto* awa_meta =
      new SKMetadata("rad", "Apparent Wind Angle",
                     "Apparent wind angle, negative to port, 0 = bow", "AWA");
  angle->connect_to(new Throttle<float>(100))
      ->connect_to(new SKOutputFloat("environment.wind.angleApparent",
                                     "/wind/angle/sk", awa_meta));

  // ----- Wind speed pipeline: D1 = Yellow pulse -----
  auto* tach = new DigitalInputCounter(kDigitalInputPin1, INPUT, RISING, 500);
  auto* freq = new Frequency(1.0f, "/wind/freq");
  // CALIBRATION PARAM: speed multiplier K (m/s per Hz). 0.5144 = 1 kn/Hz
  // (egg-cup ST60+); ~0.36 for square-cup. Set empirically.
  auto* speed_cal = new Linear(0.5144f, 0.0f, "/wind/speed/cal");
  ConfigItem(freq)
      ->set_title("Speed · pulse frequency multiplier")
      ->set_description("Scales the measured pulse rate itself. Leave at 1.0 — "
                        "calibrate wind speed on the card below instead, so "
                        "the reported Hz stays a true pulse rate.")
      ->set_sort_order(2200);
  ConfigItem(speed_cal)
      ->set_title("Speed · multiplier K (m/s per Hz)")
      ->set_description("Default 0.5144 ≈ 1 knot/Hz (egg-cup ST60+); ~0.36 for "
                        "the older square-cup. Calibrate against GPS SOG in "
                        "calm air.")
      ->set_sort_order(2201);
  // Speed gate: no-data while the sin/cos rails say the transducer is dead
  // (see transducer_dead()). The raw pulse rate stays on the 1 Hz report, so
  // the bench PWM test still shows its Hz with nothing on A1/A2.
  auto* aws_gate = new LambdaTransform<float, float>(
      [](float mps) { return transducer_dead() ? NAN : mps; });
  tach->connect_to(freq)->connect_to(speed_cal)->connect_to(aws_gate);
  freq->connect_to(new LambdaConsumer<float>([](float v) { last_hz = v; }));
  aws_gate->connect_to(
      new LambdaConsumer<float>([](float v) { last_aws_mps = v; }));

  // V2: SignalK apparent wind speed (m/s). No throttle needed — the 500 ms
  // counter window already limits this to ~2 Hz.
  auto* aws_meta = new SKMetadata("m/s", "Apparent Wind Speed", "", "AWS");
  aws_gate->connect_to(new SKOutputFloat("environment.wind.speedApparent",
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
             "tick %lu  sin=%.3fV cos=%.3fV [%s]  AWA=%.4f rad (%.1f deg)  "
             "pulse=%.1f Hz  AWS=%.2f m/s  n2k_tx=%lu fail=%lu",
             (unsigned long)beat++, last_sin_v, last_cos_v,
             angle_volts_ok() ? "ok" : "OUT OF RANGE", last_awa_rad, awa_deg,
             last_hz, last_aws_mps, (unsigned long)n2k_tx_ok,
             (unsigned long)n2k_tx_fail);
  });
}

void loop() { event_loop()->tick(); }
