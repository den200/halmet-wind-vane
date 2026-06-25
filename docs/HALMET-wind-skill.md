# HALMET Firmware Skill: Wind Transducer to NMEA2000 on ESP32

This knowledge base equips you to build, calibrate, and validate expert-level firmware on the Hat Labs **HALMET** board, with a Raymarine ST60+ masthead-to-NMEA2000 wind interface as the lead application. **Critical correction up front**: HALMET does **not** use an MCP2518FD CAN-FD controller as the task brief assumed. It uses the **ESP32's built-in TWAI (classical CAN 2.0B) peripheral** wired directly to an on-board CAN transceiver. NMEA 2000 is classical CAN 2.0B at 250 kbit/s — CAN-FD framing is never used on a marine bus. This changes the driver choice (`NMEA2000_esp32`, not `NMEA2000_mcp`) and eliminates SPI/CS/INT wiring concerns. Every other assumption in the task brief held up.

The second non-obvious finding: HALMET has a **per-channel 10 mA constant-current source (CCS)** on its analog inputs, jumper-enabled, so resistive senders (tank, oil pressure, coolant) work natively without external excitation circuitry — but it does **not** expose an 8 V rail, so the Raymarine masthead transducer's supply needs an external regulator. The third: SensESP v3 (current stable line, v3.4.0 mid-2026) is what HALMET targets; it pairs with `ttlappalainen/NMEA2000-library` and runs the entire firmware on a single ReactESP cooperative event loop — no FreeRTOS tasks needed.

The wind application breaks cleanly into three measurement channels on HALMET. Apparent wind angle comes from a sin/cos pair on A1/A2 (Blue = sine, Green = cosine wires) fed through the ADS1115 16-bit ADC, processed through a custom `SinCosAngle` transform doing `atan2(sin, cos)`. Apparent wind speed comes from a pulse train on D1 (Yellow wire, ~8 V/3.2 V active square wave, **two pulses per cup revolution**, ~1 Hz per knot for the egg-cup ST60+) processed by SensESP's `DigitalInputCounter` + `Frequency` transform. Both values feed simultaneously to a SignalK output over WiFi (V2) and an `N2kWindDataSender` that transmits **PGN 130306** at 100 ms intervals with `WindReference = N2kWind_Apparent` (=2) (V1).

---

## HALMET hardware reference

The board's full name is **HALMET — Hat Labs Marine Engine & Tank interface**. It is the successor to the SH-ESP32 + Engine Top Hat combo, sold separately from the SH-RPi / Sailor Hat family (which are Raspberry Pi HATs). Current production revision is **v1.0.1** (silkscreen fixes plus exposed 3V3/GND pads on the isolated area; KiCad source dated 2024-01-18). Hardware design is **CC BY-SA 4.0** in `github.com/hatlabs/HALMET-hardware`; firmware is Apache-2.0.

**Microcontroller**: ESP32-WROOM-32E module (PCB antenna, **16 MB flash**, no PSRAM, dual-core Xtensa LX6 @ 240 MHz, WiFi b/g/n + BT/BLE). USB-Micro-B for programming via a USB-UART bridge (likely CP2102 — not stated explicitly in markdown docs, verify against `USB.kicad_sch`).

**CAN/NMEA 2000 subsystem**: ESP32 built-in TWAI controller driving an on-board CAN transceiver (exact part not stated in public markdown docs; functionally a 3.3 V transceiver in the SN65HVD23x family per board topology — **verify against `canbus.kicad_sch` in v1.0.1**). Speed is fixed at 250 kbit/s. The CAN side shares ground with the ESP32 (the galvanic isolation barrier is on the *input* side, separating D1–D4 and the I²C-to-ADS1115 link from the MCU/N2K domain). A **120 Ω termination jumper** exists on the bottom side — leave it open when connecting to a real backbone. NMEA 2000 power and the bus connect through a single 4-pin Phoenix MC 3.81 pluggable terminal block, protected by a 500 mA PTC fuse, reverse-polarity diode, TVS, and two-stage filter into a switching DC/DC (2 A max) → 3.3 V rail. RX/TX activity LEDs visualize bus traffic.

**Analog inputs A1–A4**: four channels, each on its own 2-pin Phoenix MC 3.81 screw terminal, tolerant of −32 V to +32 V (measurement range 0–32 V). Each input passes through a fixed **33.3 kΩ / 3.3 kΩ (≈10:1)** voltage divider and a ~160 Hz hardware low-pass into an **ADS1115 16-bit ΔΣ ADC** on the I²C bus at address **0x4B** (SDA = GPIO 21, SCL = GPIO 22). Three solder jumpers on the bottom select the I²C address. With `GAIN_ONE` (±4.096 V FS at the ADS pin) you get ~0.125 mV/LSB at the ADC, or ~1.25 mV/LSB referred to the input terminal — covering 0–32 V with ~1 mV practical resolution. An optional per-channel **2.3 kHz LPF jumper** is on the bottom side for noisy tach-style signals. The internal ESP32 ADC is **not** used for these channels.

A unique feature is the per-channel **10 mA constant-current source (CCS)** for resistive-sender measurement: enable the CCS jumper header on top for the desired channel, and 10 mA is sourced into the sender, giving voltage proportional to resistance up to ~300–320 Ω (1 V per 100 Ω). This covers EU 0–180 Ω tank senders fully and the lower half of US 240–33 Ω senders. Without CCS, the channel reads voltage passively — ideal when a panel gauge already provides excitation, or for direct battery-voltage monitoring (a 32 V battery sits well within range).

**Digital inputs D1–D4**: four channels on Phoenix screw terminals, **±32 V tolerant**, Schmitt-triggered (V_TH ≈ 1.55 V, hysteresis ≈ 0.7 V), each on its own galvanically isolated digital-isolator front-end. Per-channel solder jumpers on the bottom enable a **100 kΩ pull-up or pull-down to the input domain rail**, and a **2.3 kHz LPF** (engage for noisy alternator-W tach lines). The pin mapping is **D1 = GPIO 23, D2 = GPIO 25, D3 = GPIO 27, D4 = GPIO 26** — note the D3/D4 numerical swap is intentional, do not assume natural numerical order.

**Power**: input **5–32 V DC** through the NMEA 2000 connector (or via dedicated power if standalone). Typical current ~70–90 mA at 12 V with WiFi active. Exposed rails: 3.3 V main domain (on the GPIO header and shared with I²C/1-Wire/Qwiic peripherals), and — new in v1.0.1 — a small isolated 3.3 V + GND header on the input-side isolated area. **No 5 V, 8 V, or 12 V passthrough rail is exposed.** This matters for the Raymarine application: HALMET cannot supply the masthead's 8 V red-wire excitation natively. You must add an external 8 V regulator (LM7808 or a small DC-DC like the MP1584 buck the user mentioned, fed from boat 12 V). Budget conservatively for current draw from any 3.3 V rail — total switcher is 2 A but most is consumed by the ESP32 and isolation barrier.

**Physical**: designed for a 100 × 68 × 50 mm waterproof plastic enclosure (sold separately by Hat Labs; no DIN-rail mount). The board exposes a 2×10 (20-pin) 2.54 mm GPIO break-out / JTAG header (not soldered by default), the reset/boot buttons, a blue user LED on GPIO 2, and a red 3.3 V power-good LED.

### Definitive GPIO pin map (board label → ESP32 GPIO)

| Board label | ESP32 GPIO | Constant in firmware | Function |
|---|---|---|---|
| **D1** | **GPIO 23** | `kDigitalInputPin1` | Isolated digital input; default RPM/pulse |
| **D2** | **GPIO 25** | `kDigitalInputPin2` | Isolated digital input; default low-oil alarm |
| **D3** | **GPIO 27** | `kDigitalInputPin3` | Isolated digital input; default over-temp alarm (active-low) |
| **D4** | **GPIO 26** | `kDigitalInputPin4` | Isolated digital input; free |
| **A1** | ADS1115 AIN0 | — | Analog via I²C; default fuel tank |
| **A2** | ADS1115 AIN1 | — | Analog via I²C |
| **A3** | ADS1115 AIN2 | — | Analog via I²C |
| **A4** | ADS1115 AIN3 | — | Analog via I²C |
| I²C SDA | **GPIO 21** | `kSDAPin` | Shared with ADS1115, OLED, Qwiic |
| I²C SCL | **GPIO 22** | `kSCLPin` | Same |
| CAN TX → transceiver | **GPIO 19** | `kCANTxPin` | NMEA 2000 |
| CAN RX ← transceiver | **GPIO 18** | `kCANRxPin` | NMEA 2000 |
| 1-Wire DQ | **GPIO 4** | — | Header (unpopulated) |
| Boot button | **GPIO 0** | — | Also user button |
| Blue user LED | **GPIO 2** | — | Active-high |
| Example test PWM | **GPIO 33** | `kTestOutputPin` | 380 Hz reference for D1 |

The 2×10 GPIO header additionally breaks out GPIOs 5, 12–17, 32, 33, 34, 35, 36, 39. GPIOs 12/13/14/15 double as JTAG (TDI/TCK/TMS/TDO). GPIOs 34–39 are input-only.

**Items to verify against your physical board / v1.0.1 schematic PDFs**: the exact CAN transceiver part number, the USB-UART bridge chip, the main switching regulator part, the divider resistor absolute values (the 10:1 ratio is confirmed in firmware comments but absolute impedance is not stated), the digital isolator parts, the isolated DC/DC IC, and whether GPIO 4 has an internal 1-Wire pull-up.

---

## SensESP framework architecture

SensESP is a C++ reactive framework built on ReactESP (event loop) and the Arduino-ESP32 framework. **Current stable line is v3** (3.4.0 mid-2026); v4 is not yet released. The HALMET example pins to `SignalK/SensESP @ ^3.1.0`. Data flows through typed nodes implementing two interfaces: `ValueProducer<T>` (emits values via `emit(value)`, provides `connect_to()`) and `ValueConsumer<T>` (receives via virtual `set(const T& v)`). Specializations include `FloatProducer`, `IntProducer`, `BoolProducer`, `StringProducer`, `FloatSensor`, etc. The three node roles are **sensors** (producers like `DigitalInputCounter`, `AnalogInput`, `RepeatSensor<T>`), **transforms** (e.g. `Linear`, `MovingAverage`, `Median`, `Frequency`, `CurveInterpolator`, `LambdaTransform`, `RepeatExpiring<T>`), and **consumers/outputs** (`SKOutput<T>`, `LambdaConsumer<T>`, `DigitalOutput`). Wiring is chained: `producer->connect_to(transform)->connect_to(output)`.

The `SensESPApp` is constructed via the **`SensESPAppBuilder` pattern**, configuring hostname (default WiFi AP SSID + mDNS name), optional hard-coded WiFi/SK server credentials, OTA password, and system-info sensors. The global `event_loop()` function returns the underlying `reactesp::EventLoop`; schedule periodic work with `event_loop()->onRepeat(ms, lambda)` and one-shots with `onDelay(0, lambda)`. The Arduino `loop()` must call `event_loop()->tick()`.

Services started automatically by `SensESPApp` include a built-in WiFi provisioner / captive portal (replacing the external WiFiManager library in v3, supporting simultaneous AP+STA), the configuration web UI on HTTP port 80 (Preact/Bootstrap SPA shipped inside the firmware), a Signal K WebSocket client (`SKWSClient`, auto-discovering the SK server via mDNS `_signalk-ws._tcp` if not hard-coded), optional ArduinoOTA, LittleFS persistence for JSON config files, and a system status LED.

### Breaking changes you must respect (v2 → v3)

`ConfigItem` is now **mandatory** to expose objects in the web UI — wrap every persistable object: `ConfigItem(obj)->set_title(...)->set_description(...)->set_sort_order(N)`. Logging migrated from RemoteDebug `debugX()` to ESP-IDF `ESP_LOGI/E/W/D/V(tag, fmt, ...)` and requires the build flag `-D USE_ESP_IDF_LOG`. `SetupSerialDebug(115200)` is now `SetupLogging()`. The `Startable` interface was removed (no more `sensesp_app->start()`); use `event_loop()->onDelay(0, ...)` for post-init hooks. Classes moved into `reactesp::` namespace; use the global `event_loop()` accessor rather than instantiating your own `ReactESP app;`. `WSClient` renamed to `SKWSClient`. SKOutput naming settled: `SKOutputFloat`, `SKOutputInt`, `SKOutputBool`, `SKOutputString` (the old `SKOutputNumeric` from v1 is an alias). `connect_from()` is deprecated — always use `producer->connect_to(consumer)`.

### Key transforms for the wind application

`DigitalInputCounter(pin, mode, interrupt_type, read_delay_ms, config_path)` attaches an ISR (rising/falling/change), accumulates counts, and emits the per-window count every `read_delay_ms`. `Frequency(multiplier, config_path)` consumes the count and divides by elapsed time → Hz, then multiplies by the calibration constant. `Linear(m, b, config_path)` does `output = m*input + b`. `MovingAverage(samples, multiplier, config_path)` and `Median(samples, config_path)` smooth noisy data. `CurveInterpolator` is a subclassable piecewise-linear lookup for non-linear senders (tanks, NTCs). `RepeatExpiring<T>(expire_ms, repeat_ms)` re-emits a stale-protected value at a fixed cadence and emits N/A after the expiry window — **critical for N2K bridging** so a frozen sensor reading doesn't keep transmitting indefinitely. `LambdaTransform` takes a single input plus persistable named parameters; for two-input math (sin and cos), you write a custom multi-input class using the "member-consumer" pattern (a `LambdaConsumer<float>` per input as a class member, not multiple inheritance).

---

## HALMET example firmware: complete code walkthrough

The reference repository is `github.com/hatlabs/HALMET-example-firmware` (Apache-2.0, C++ 100%). The default firmware configures **A1 → fuel tank instance 0, D1 → engine 1 RPM, D2 → engine 1 low-oil-pressure alarm, D3 → over-temp alarm (active-low)**. The folder layout is flat — all custom headers live in `src/`, no `include/` or `lib/` subdirectories, no custom partition CSV (uses Espressif's stock `default_8MB.csv`). Files in `src/`: `main.cpp`, `halmet_const.h`, `halmet_analog.h/.cpp`, `halmet_digital.h/.cpp`, `halmet_display.h`, `halmet_serial.h`, `n2k_senders.h`, `expiring_value.h`, `rate_limiter.h`.

### platformio.ini (verbatim)

```ini
[platformio]
default_envs = halmet

[env]
upload_speed = 2000000
monitor_speed = 115200
lib_deps =
    SignalK/SensESP @ ^3.1.0
    adafruit/Adafruit SSD1306 @ ^2.5.1
    ttlappalainen/NMEA2000-library@^4.17.2
    NMEA2000_twai=https://github.com/skarlsson/NMEA2000_twai
    adafruit/Adafruit ADS1X15@^2.3.0
build_flags =
    -D CORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_VERBOSE
    -D USE_ESP_IDF_LOG
board_build.partitions = min_spiffs.csv

build_unflags = -Werror=reorder
monitor_filters = esp32_exception_decoder
test_build_src = true

[pioarduino]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/53.03.13/platform-espressif32.zip
framework = arduino
lib_deps =
    ${env.lib_deps}
    esp_websocket_client=https://components.espressif.com/api/downloads/?object_type=component&object_id=b072eea5-4e65-4ced-94c3-d6f24207062d
build_flags =
    ${env.build_flags}
    -D SENSESP_SSL_SUPPORT=1

[esp32]
board = esp32dev
build_flags =
    ${env.build_flags}
    -D BUTTON_BUILTIN=0
    -D LED_BUILTIN=2

[env:halmet]
extends = pioarduino, esp32
board_build.partitions = default_8MB.csv
build_flags =
    ${pioarduino.build_flags}
    ${esp32.build_flags}
```

The default env is `halmet` extending `pioarduino + esp32`. **Board target is `esp32dev`** (not a custom board JSON) — HALMET is a stock ESP32-WROOM-32E from PlatformIO's perspective. The **pioarduino fork of platform-espressif32 (53.03.13)** is used because it includes ESP-IDF + Arduino-ESP32 in a known-good combination. Partition scheme is `default_8MB.csv` (Espressif stock 8 MB scheme, fits the 16 MB module: two app slots for OTA + LittleFS for config). The `NMEA2000_twai` library is listed but **not used** by `main.cpp`, which uses `tNMEA2000_esp32(kCANTxPin, kCANRxPin)` from the older `ttlappalainen/NMEA2000_esp32` driver. OTA flashing is commented out as a template; uncomment to enable.

### Pin constants (`src/halmet_const.h`, verbatim)

```cpp
#ifndef HALMET_SRC_HALMET_CONST_H_
#define HALMET_SRC_HALMET_CONST_H_

#include <Arduino.h>

namespace sensesp {

const int kSDAPin = 21;
const int kSCLPin = 22;
const int kADS1115Address = 0x4b;

const gpio_num_t kCANRxPin = GPIO_NUM_18;
const gpio_num_t kCANTxPin = GPIO_NUM_19;

const int kDigitalInputPin1 = GPIO_NUM_23;
const int kDigitalInputPin2 = GPIO_NUM_25;
const int kDigitalInputPin3 = GPIO_NUM_27;
const int kDigitalInputPin4 = GPIO_NUM_26;

}
#endif
```

In `halmet_analog.h`: `const float kVoltageDividerScale = 33.3 / 3.3;` (≈10.09). In `halmet_analog.cpp`: `const float kMeasurementCurrent = 0.01;` (10 mA active CCS) and `const float kTankDefaultSize = 120. / 1000;` (120 L default tank).

### Tacho frequency-counter pattern (the wind-speed fork base)

`ConnectTachoSender(pin, name)` in `halmet_digital.cpp` is the canonical pattern. The constructor `new DigitalInputCounter(pin, INPUT, RISING, 500, config_path)` attaches a rising-edge ISR and emits the accumulated count every 500 ms (2 Hz sample rate). The count flows into `new Frequency(kDefaultFrequencyScale=1/100., config_path)` — divides by the 500 ms window → pulses/sec, then multiplies by 1/100 to assume 100 pulses per revolution and produce rev/s (Hz). The author's comment explicitly says "This is rarely, if ever correct" — the user is expected to override the multiplier from the web UI under `/Tacho main/Revolution Multiplier`. The Hz output flows to a Signal K path `propulsion.{name}.revolutions` (SignalK convention is Hz, not RPM) and into the N2K sender. There is **no software debounce** in the digital input counter — HALMET relies on its hardware Schmitt trigger and the optional LPF solder jumper. Hz→RPM conversion (×60) is done downstream by a `LambdaTransform<double,double>` inside `n2k_senders.h`. A test-signal generator (`kTestOutputPin = GPIO_NUM_33`, `ledcAttach(pin, 380 Hz, 13-bit)`) provides a 380 Hz reference output you can jumper to D1 for bench verification — at the default 100 ppr it reads ~228 RPM.

### Analog input pattern

`ConnectTankSender(ads1115, channel, name, sk_id, sort_order, enable_signalk_output)` in `halmet_analog.cpp` implements the three-stage resistive-sender path. Stage 1 is a `RepeatSensor<float>(500ms, lambda)` that calls `ads1115->readADC_SingleEnded(channel)` → `computeVolts(raw)` → applies `kVoltageDividerScale * V_ADS` to get terminal voltage → divides by `kMeasurementCurrent` (10 mA) to get sender resistance in ohms (assumes CCS jumper is engaged for active mode). Stage 2 is a `CurveInterpolator` mapping resistance (Ω) → fill ratio (0–1), pre-populated with the saturating curve `(0Ω, 0), (180Ω, 1), (1000Ω, 1)` so a broken-open sender doesn't read negative. Stage 3 is `Linear(kTankDefaultSize=0.12, 0)` scaling ratio → m³. Each stage gets its own SK output with appropriate `SKMetadata` units ("ohm", "ratio", "m3") and `ConfigItem` web-UI entries with sort orders.

The voltage-only pattern (`ADS1115VoltageInput`) emits `calibration_factor * kVoltageDividerScale * V_ADS` (volts at terminal) and persists `calibration_factor` to JSON via `to_json`/`from_json`. **Do not change `kVoltageDividerScale` — it is hardware-fixed; use the `calibration_factor` for trimming.**

### NMEA 2000 init (verbatim)

```cpp
nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);
nmea2000->SetN2kCANSendFrameBufSize(250);
nmea2000->SetN2kCANReceiveFrameBufSize(250);
nmea2000->SetProductInformation("20231229", 104, "HALMET", "1.0.0", "1.0.0");
nmea2000->SetDeviceInformation(GetBoardSerialNumber(), 140, 50, 2046);
nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 71);
nmea2000->EnableForward(false);
nmea2000->Open();
event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });
```

Device info encodes Function 140 (Engine), Class 50 (Propulsion), manufacturer 2046 (unregistered dev), default source address 71. `GetBoardSerialNumber()` in `halmet_serial.h` builds a 64-bit unique number from `esp_efuse_mac_get_default()`. `ParseMessages()` runs every 1 ms on the same ReactESP loop — **no separate FreeRTOS task is needed or used**.

`n2k_senders.h` defines three classes that wrap N2K message construction in SensESP consumers. `N2kEngineParameterRapidSender` (PGN 127488, 100 ms cadence, 1000 ms input expiry) accepts engine_speed_ in Hz and multiplies by 60 internally via `LambdaTransform<double,double>` to get RPM, then calls `SetN2kEngineParamRapid(N2kMsg, engine_instance, RPM, boost, tilt)` and `nmea2000->SendMsg(N2kMsg)`. `N2kEngineParameterDynamicSender` (PGN 127489, 500 ms / 5000 ms expiry) handles oil pressure, temperatures, alarms. `N2kFluidLevelSender` (PGN 127505, 2500 ms / 10000 ms expiry) takes a ratio and ×100 internally to get percent. Each class is `FileSystemSaveable` for live config of instance numbers and capacities.

---

## NMEA 2000 library, PGN 130306, and device identity

### Library selection

For HALMET use **`NMEA2000_esp32`** (ttlappalainen, header `NMEA2000_esp32.h`, class `tNMEA2000_esp32(txPin, rxPin)`). The umbrella header `NMEA2000_CAN.h` auto-selects this backend on ESP32. `NMEA2000_mcp` is for MCP2515 SPI shields on classic Arduinos and is **not** applicable. There is **no official ttlappalainen wrapper for MCP2518FD** — if you ever build a custom board with MCP2518FD, you would subclass `tNMEA2000` implementing `CANSendFrame/CANOpen/CANGetFrame` against Pierre Molinaro's `acan2517FD` library in classical CAN 2.0B mode (NMEA 2000 never uses CAN-FD framing). For HALMET this is moot.

### PGN 130306 Wind Data — full field structure

**Single-frame, 8 bytes, default priority 2, default period 100 ms.**

| # | Field | Type | Size | Resolution | Unit | Notes |
|---|---|---|---|---|---|---|
| 1 | SID | uint8 | 8 bits | 1 | — | Sequence ID, cycle 0..252; 0xFF = NA |
| 2 | Wind Speed | uint16 | 16 bits | 0.01 | **m/s** | 0..655.32 m/s. NA = 0xFFFF |
| 3 | Wind Angle | uint16 | 16 bits | 0.0001 | **radians** | 0..2π. 0 = bow, positive = clockwise viewed from above. NA = 0xFFFF |
| 4 | Wind Reference | enum | 3 bits @ bit offset 45 | — | — | **2 = Apparent** for masthead vane |
| — | Reserved | — | 21 bits | — | — | fill with 1s |

The `tN2kWindReference` enum from `N2kTypes.h`: `N2kWind_True_North=0`, `N2kWind_Magnetic=1`, **`N2kWind_Apparent=2`**, `N2kWind_True_boat=3`, `N2kWind_True_water=4`, `N2kWind_Error=6`, `N2kWind_Unavailable=7`. PGN 130306 has **no instance field** — multiple wind sensors are distinguished by source address; multiple reference frames from one sensor by the Reference enum. Use the library helpers and you needn't think about the bit offset:

```cpp
void SetN2kPGN130306(tN2kMsg &N2kMsg, unsigned char SID, double WindSpeed,
                     double WindAngle, tN2kWindReference WindReference);
inline void SetN2kWindSpeed(tN2kMsg &N2kMsg, unsigned char SID, double WindSpeed,
                            double WindAngle, tN2kWindReference WindReference) {
    SetN2kPGN130306(N2kMsg, SID, WindSpeed, WindAngle, WindReference);
}
```

Knots → m/s: `KnotsToms(kn)` (×0.51444). Degrees → radians: `DegToRad(deg)` (×π/180). Pass `N2kDoubleNA` for missing values.

### Device identity for a wind sensor

Use **Device Class = 85 (External Environment), Device Function = 130 (Atmospheric), Manufacturer code = 2046 (unregistered dev), Industry Group = 4 (Marine, library default)**. This matches the conventional `WindMonitor.ino` example in the ttlappalainen library and is appropriate for non-registered DIY devices. Classes 75 (Sensor Communication Interface) with Function 130 is an acceptable alternative. **Do not** use Class 50 / Function 140 (Propulsion / Engine) — that's what the HALMET example uses because it's an engine interface; change it for a wind sensor. Avoid registered manufacturer codes (Raymarine 1851, Garmin 137, Airmar 135, Maretron 144, Simrad 1857). Source address 35 or 71 is a reasonable default; the library handles ISO address claim automatically.

### Complete N2K setup for a HALMET wind sensor

```cpp
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include "halmet_const.h"

const unsigned long TransmitMessages[] PROGMEM = { 130306L, 0 };
const unsigned long ReceiveMessages[]  PROGMEM = { 0 };
static uint8_t SID = 0;

void setupN2k() {
    NMEA2000.SetProductInformation("20260601", 140, "HALMET Wind",
                                   "1.0.0", "1.0.0");
    NMEA2000.SetConfigurationInformation(
        "Hat Labs HALMET, ESP32 wind sensor",
        "Mast head", "Apparent wind only; reference=2");
    NMEA2000.SetDeviceInformation(GetBoardSerialNumber(),
                                  130,    // Function = Atmospheric
                                  85,     // Class = External Environment
                                  2046);  // Manufacturer (unregistered)
    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 35);
    NMEA2000.EnableForward(false);
    NMEA2000.SetN2kCANMsgBufSize(2);
    NMEA2000.SetN2kCANSendFrameBufSize(150);
    NMEA2000.SetN2kCANReceiveFrameBufSize(150);
    NMEA2000.ExtendTransmitMessages(TransmitMessages);
    NMEA2000.ExtendReceiveMessages(ReceiveMessages);
    NMEA2000.Open();
}

void sendWind(double windSpeed_mps, double windAngle_rad) {
    tN2kMsg N2kMsg;
    SetN2kWindSpeed(N2kMsg, SID, windSpeed_mps, windAngle_rad, N2kWind_Apparent);
    NMEA2000.SendMsg(N2kMsg);
    SID = (SID >= 252) ? 0 : SID + 1;
}
```

Call `NMEA2000.ParseMessages()` every ~1 ms in the main loop, and `sendWind(...)` every 100 ms (the standard PGN 130306 cadence). The library auto-handles ISO address claim (PGN 60928), product info (126996), config info (126998), heartbeat (126993), and PGN list (126464).

### Addressing, LEN, instancing

NMEA 2000 source addresses are dynamic via ISO 11783 address claim — pick a "preferred" address in `SetMode()`; the library renegotiates on conflict. To persist a changed address across reboots, periodically check `NMEA2000.ReadResetAddressChanged()` and save to NVS. **LEN (Load Equivalency Number)** = bus current in units of 50 mA at 9 V. HALMET + ESP32 drawing ~135 mA at 9 V = **LEN 3** (round up). LEN goes on the physical product label and in configuration information; the library does not transmit it. **SeaTalkNG is electrically and protocol-wise identical to NMEA 2000** at 250 kbit/s; only the Raymarine 5-pin connector differs. A SeaTalkNG-to-DeviceNet adapter (Raymarine A06045 or third-party) bridges HALMET's standard NMEA 2000 port into an STng backbone.

---

## Raymarine ST60+ masthead transducer electrical interface

Verified against the Raymarine Wind Vane Service Manual (doc 83170-1 / D6956-1) and cross-checked with the sailboatinstruments.blogspot reverse-engineering and the YBW thread by nigelmercier. The 5-conductor + screen cable scheme has been **stable from Autohelm Z080/Z087/Z135 through ST60, ST60+, and i60** — the same wire colors plug into any generation's terminal block.

### Wire color map

| Wire | Signal | Notes |
|---|---|---|
| **Red** | **+8.00 V DC ± 0.25 V** supply | Powered by the ST60 display in normal use; must be provided externally by HALMET |
| **Screen / Bare / Black** | **0 V / GND / shield** | Cable shield serves as signal ground |
| **Blue** | **Sine (sin) wind-angle signal** | Analog, DC-coupled, ratiometric to 8 V supply |
| **Green** | **Cosine (cos) wind-angle signal** | Analog, DC-coupled, ratiometric to 8 V supply |
| **Yellow** | **Wind-speed pulse** | Active square wave, ~8 V high / ~3.2 V low, 2 pulses per cup revolution |

### Wind direction: sin/cos detail

The masthead contains active electronics (newer units use a Senson 2SA-10 two-axis Hall sensor; older Autohelm units use a slide-pot + cam mechanism). Both produce buffered, low-impedance sin/cos outputs centered on Vs/2 ≈ 4.0 V, swinging roughly **±1.5 V from midpoint (2.5 V to 5.5 V range)** at the masthead. Over the 100 ft cable at the display end, the actual range observed by Bermerlin (sailboatinstruments) is ~2.9–5.3 V at 8.0 V supply. The official Raymarine spec is "Blue and Green between 2 and 6 V vs shield." The locus of (Blue, Green) over one full vane revolution is a slightly tilted, slightly eccentric ellipse — for serious accuracy you fit an ellipse and de-rotate; for ±2–3° accuracy a simple `atan2(Blue−Vmid, Green−Vmid)` suffices.

Verified angle table (with 8 V supply, vane arrow pointing in the named direction):

| Vane direction | Blue (sin) | Green (cos) |
|---|---|---|
| Forward (AWA = 0°, wind from bow) | 4.0 V (mid) | 5.0–6.0 V (max) |
| Starboard (AWA = 90°) | 5.0–6.0 V (max) | 4.0 V (mid) |
| Aft (AWA = 180°) | 4.0 V (mid) | 2.0–3.0 V (min) |
| Port (AWA = 270°) | 2.0–3.0 V (min) | 4.0 V (mid) |

The angle formula is:
```
x = V_green − Vs/2      // cosine component, positive when wind from bow
y = V_blue  − Vs/2      // sine component, positive when wind from starboard
AWA_radians = atan2(y, x)   // 0 = bow, positive clockwise viewed from above
```
This matches NMEA 2000 PGN 130306 convention (0 = bow, positive clockwise looking down). For SignalK `environment.wind.angleApparent`, normalize to `[−π, +π]` (negative to port). **Important**: Raymarine's "forward" in the service manual refers to where the vane arrow points; wind arrives from the opposite side. Verify mechanically on your specific unit.

### Wind speed: pulse detail

Yellow is an **actively-driven buffered square wave**, **not** a bare reed switch. Voltage levels are ~8 V high / ~3.2 V low (Raymarine spec) or ~7 V observed by independent measurement. **No external pull-up required** — though a 10 kΩ pull-up to 8 V is safe and sharpens edges on long cables. **Two pulses per cup revolution.** The empirical calibration from nigelmercier's bench test: 600 RPM cup-shaft rotation = 20 Hz = ST60 reads 20 kn with factory defaults; road test at GPS 19.98 kn measured 20.42 Hz → ST60 reads 20.1 kn. So **~1.0 Hz per knot** for the newer egg-cup transducer (E22078 / R28170). Convert to SI: `wind_speed_mps ≈ 0.5144 × freq_Hz`. Older square-cup transducers may need a different constant near 0.7–0.8; calibrate empirically. Expected pulse rates: 5 kn → 5 Hz (200 ms period), 10 kn → 10 Hz, 20 kn → 20 Hz, 50 kn → 50 Hz (20 ms period) — all comfortably within ESP32 interrupt capability with no concern about missed edges.

### Recommended HALMET interface circuit

For power, generate a stable 8.0 V from boat 12 V using an LM7808 (sufficient — masthead draws tens of mA) or a small DC-DC like the user's planned MP1584 buck (set output to 8 V via the trim pot). Filter with 100 nF + 10 µF at the regulator output. Feed Red; bond Screen to HALMET signal GND.

For sin/cos, the masthead outputs ~2.5–5.5 V which exceeds the ESP32 ADC range but is **within HALMET's 0–32 V terminal range** because of the 33.3/3.3 = 10:1 onboard divider. **Connect Blue directly to A1 and Green directly to A2 with CCS jumpers OFF** (passive voltage mode). After the divider these become 0.25–0.55 V at the ADS1115 pin — well within the ±4.096 V `GAIN_ONE` range. With 16-bit resolution you get ~0.04° angular accuracy at typical noise floors, easily exceeding the transducer's mechanical accuracy. The ratiometric nature of the transducer is preserved because both channels share the same scaling.

For the pulse signal, **connect Yellow directly to D1** (passive mode, no jumpers engaged unless you observe noise). HALMET's hardware Schmitt trigger at V_TH ≈ 1.55 V cleanly separates the 3.2 V low from the 8 V high. If you observe noise on a long cable, engage the **2.3 kHz LPF jumper** on D1's bottom side. No pull-up jumper needed (Yellow is actively driven). Configure as `RISING` interrupt; software debounce of `period > 10 ms` is plenty even at 50 kn.

### Calibration procedure

For sin/cos, log (V_blue, V_green) pairs while rotating the vane through a full 360° in 5–10° increments (mast on deck during commissioning). Plot the locus — it should be roughly circular around (4.0 V, 4.0 V). Compute Vmid as the mean of all readings (more robust than assuming Vs/2). Compute angle as above and apply a single offset to align the vane's "zero" with your boat's centerline. For sub-1° accuracy, fit an ellipse (sailboatinstruments has C code) and map back to a unit circle. For the wind-speed pulse calibration, the simplest method is a calm-day motoring test in both directions at GPS-measured 5/10/15/20 knots; average upwind and downwind to subtract residual true wind. Fit a single multiplier so that emitted speed matches GPS-derived AWS (at known boat heading and zero true wind).

---

## The wind firmware: complete code skeleton

This brings everything together. The `SinCosAngle` custom transform uses the "member-consumer" pattern to handle two `float` inputs without multiple inheritance.

### Custom sin/cos angle transform (`src/sin_cos_angle_transform.h`)

```cpp
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/transforms/transform.h"

namespace sensesp {

class SinCosAngle : public Transform<float, float> {
 public:
  SinCosAngle(float gain = 1.0f, float offset_rad = 0.0f,
              float min_magnitude = 0.05f, const String& config_path = "")
      : Transform<float, float>(config_path),
        gain_(gain), offset_rad_(offset_rad), min_magnitude_(min_magnitude),
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
    return true;
  }
  bool from_json(const JsonObject& root) override {
    if (root["gain"].is<float>())          gain_          = root["gain"];
    if (root["offset_rad"].is<float>())    offset_rad_    = root["offset_rad"];
    if (root["min_magnitude"].is<float>()) min_magnitude_ = root["min_magnitude"];
    return true;
  }

 private:
  void recompute() {
    if (!got_sin_ || !got_cos_) return;
    const float s = gain_ * sin_value_;
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

  float gain_, offset_rad_, min_magnitude_;
  float sin_value_ = 0.0f, cos_value_ = 0.0f;
  bool  got_sin_ = false, got_cos_ = false;
  LambdaConsumer<float> sin_consumer_, cos_consumer_;
};

}
```

### Wind interface `main.cpp` (complete dual-output: N2K V1 + SignalK V2)

```cpp
#include <Adafruit_ADS1X15.h>
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include <Wire.h>

#include "sensesp/sensors/digital_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/transforms/repeat.h"
#include "sensesp/transforms/throttle.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"

#include "halmet_const.h"        // copied from HALMET-example-firmware/src
#include "halmet_analog.h"       // ADS1115VoltageInput
#include "halmet_serial.h"       // GetBoardSerialNumber()
#include "sin_cos_angle_transform.h"

using namespace sensesp;
using namespace halmet;

tNMEA2000* nmea2000;
TwoWire*   i2c;
Adafruit_ADS1115* ads;

// SID + last-known values for N2K send
static uint8_t sid = 0;
static volatile float last_awa_rad = NAN;
static volatile float last_aws_mps = NAN;

void setup() {
  SetupLogging();
  Serial.begin(115200);

  // ----- App -----
  SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("halmet-wind")
                       .get_app();

  // ----- I2C + ADS1115 -----
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);
  ads = new Adafruit_ADS1115();
  ads->setGain(GAIN_ONE);  // ±4.096 V
  ads->begin(kADS1115Address, i2c);

  // ----- NMEA 2000 init (wind sensor identity) -----
  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);
  nmea2000->SetN2kCANSendFrameBufSize(150);
  nmea2000->SetN2kCANReceiveFrameBufSize(150);
  nmea2000->SetProductInformation("20260601", 140, "HALMET Wind",
                                  "1.0.0", "1.0.0");
  nmea2000->SetDeviceInformation(GetBoardSerialNumber(),
                                 130,   // Function: Atmospheric
                                 85,    // Class: External Environment
                                 2046); // Manufacturer (unregistered)
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 35);
  nmea2000->EnableForward(false);
  static const unsigned long Tx[] PROGMEM = { 130306L, 0 };
  nmea2000->ExtendTransmitMessages(Tx);
  nmea2000->Open();
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  // ----- Wind angle: A1=sine, A2=cosine -----
  auto* ch_sin = new ADS1115VoltageInput(ads, 0, "/wind/sin");
  auto* ch_cos = new ADS1115VoltageInput(ads, 1, "/wind/cos");
  ConfigItem(ch_sin)->set_title("Sin channel (A1)")->set_sort_order(2000);
  ConfigItem(ch_cos)->set_title("Cos channel (A2)")->set_sort_order(2001);

  // Mid-rail centering: subtract Vmid (~4.0V at terminal), scale to ±1
  auto* sin_cal = new Linear(1.0f / 1.5f, -4.0f / 1.5f, "/wind/sin/cal");
  auto* cos_cal = new Linear(1.0f / 1.5f, -4.0f / 1.5f, "/wind/cos/cal");
  ConfigItem(sin_cal)->set_title("Sin centering (gain, -Vmid×gain)")->set_sort_order(2002);
  ConfigItem(cos_cal)->set_title("Cos centering (gain, -Vmid×gain)")->set_sort_order(2003);
  ch_sin->connect_to(sin_cal);
  ch_cos->connect_to(cos_cal);

  auto* angle = new SinCosAngle(1.0f, 0.0f, 0.05f, "/wind/angle/cal");
  ConfigItem(angle)
      ->set_title("Wind angle transform")
      ->set_description("atan2(sin, cos) + offset_rad. Use offset to align "
                        "vane zero with boat centerline.")
      ->set_sort_order(2100);
  sin_cal->connect_to(&angle->sin_input());
  cos_cal->connect_to(&angle->cos_input());

  auto* angle_smooth = new MovingAverage(5, 1.0f, "/wind/angle/smooth");
  ConfigItem(angle_smooth)->set_title("Angle smoothing")->set_sort_order(2101);

  auto* awa_meta = new SKMetadata();
  awa_meta->units_ = "rad";
  awa_meta->display_name_ = "Apparent Wind Angle";
  awa_meta->short_name_ = "AWA";
  awa_meta->description_ = "Apparent wind angle, negative to port, 0 = bow";

  angle->connect_to(angle_smooth);
  // V2: SignalK
  angle_smooth->connect_to(new Throttle<float>(100, "/wind/angle/throttle"))
              ->connect_to(new SKOutput<float>("environment.wind.angleApparent",
                                               "/wind/angle/sk", awa_meta));
  // V1: latch for N2K
  angle_smooth->connect_to(new LambdaConsumer<float>(
      [](float v){ last_awa_rad = v; }));

  // ----- Wind speed: D1 pulse counter -----
  // RISING edges, 500 ms window. Raymarine: 2 pulses per cup rev.
  // Default mult 0.5144 m/s per Hz (1 kn/Hz × 0.5144 m/s/kn) for newer egg cups.
  // After bench/road calibration, edit "Speed multiplier" in web UI.
  auto* tach = new DigitalInputCounter(kDigitalInputPin1, INPUT, RISING, 500);
  auto* freq = new Frequency(1.0f, "/wind/freq");  // raw Hz
  ConfigItem(freq)->set_title("Pulse frequency (Hz)")->set_sort_order(2200);

  auto* speed_cal = new Linear(0.5144f, 0.0f, "/wind/speed/cal");
  ConfigItem(speed_cal)
      ->set_title("Speed multiplier (m/s per Hz)")
      ->set_description("Default 0.5144 = 1.0 knot per Hz. Calibrate empirically.")
      ->set_sort_order(2201);

  auto* aws_meta = new SKMetadata();
  aws_meta->units_ = "m/s";
  aws_meta->display_name_ = "Apparent Wind Speed";
  aws_meta->short_name_ = "AWS";

  tach->connect_to(freq)->connect_to(speed_cal);
  // V2: SignalK
  speed_cal->connect_to(new SKOutput<float>("environment.wind.speedApparent",
                                            "/wind/speed/sk", aws_meta));
  // V1: latch for N2K
  speed_cal->connect_to(new LambdaConsumer<float>(
      [](float v){ last_aws_mps = v; }));

  // ----- V1: PGN 130306 every 100 ms -----
  event_loop()->onRepeat(100, [](){
    if (!isfinite(last_awa_rad) && !isfinite(last_aws_mps)) return;
    tN2kMsg msg;
    double awa = isfinite(last_awa_rad)
                   ? (last_awa_rad < 0 ? last_awa_rad + 2*M_PI : last_awa_rad)
                   : N2kDoubleNA;  // PGN 130306 expects 0..2π
    double aws = isfinite(last_aws_mps) ? last_aws_mps : N2kDoubleNA;
    SetN2kWindSpeed(msg, sid, aws, awa, N2kWind_Apparent);
    nmea2000->SendMsg(msg);
    sid = (sid >= 252) ? 0 : sid + 1;
  });

  while (true) loop();
}

void loop() { event_loop()->tick(); }
```

Key design notes: the N2K send uses a 100 ms `onRepeat` driven by the latest latched values rather than `RepeatExpiring` because we want a deterministic 10 Hz cadence regardless of input update rates. Both V1 (N2K) and V2 (SignalK) outputs share the same calibrated values — calibration changes from the web UI affect both paths simultaneously. The PGN 130306 angle field expects 0..2π; we convert from the SignalK convention (−π..+π) before sending. NaN inputs are converted to `N2kDoubleNA` so the library encodes the bit pattern 0xFFFF. The SignalK path uses a `Throttle<float>(100ms)` on the angle output to match the N2K cadence and avoid flooding the WebSocket.

---

## SignalK output path (V2)

SensESP uses strict SI units everywhere — radians for angles, m/s for speeds, Kelvin for temperatures, Pa for pressure, 0–1 ratios for fill levels. Clients (KIP, Freeboard, WilhelmSK, Instrument Panel, OpenCPN) handle display unit conversion. For wind, publish **only `environment.wind.angleApparent` (rad, −π..+π, negative to port) and `environment.wind.speedApparent` (m/s)** from the firmware. Let the SignalK server's `signalk-derived-data` plugin compute true wind from heading + STW + your apparent values — publishing both from the firmware duplicates work and risks drift.

The boot flow: WiFi captive portal AP named after `set_hostname()` (default password `thisisfine`, IP 192.168.4.1, up to 3 SSIDs cached); mDNS discovery of `_signalk-ws._tcp.local` advertised by the SK server; access-request handshake (POST to `/signalk/v1/access/requests` with a UUID and `permissions: "readwrite"`, then poll until the user clicks Approve in Security → Access Requests on the SK admin); WebSocket connection to `ws://<server>:3000/signalk/v1/stream?subscribe=none` with the bearer token. Tokens persist in NVS and survive reboots.

On the **Cerbo GX (Venus OS Large)**, SignalK is pre-installed — enable via Settings → Venus OS Large features → SignalK ON, then browse to `http://venus.local:3000` for the admin UI. The Cerbo's VE.Can port already feeds canboatjs-decoded N2K data into SignalK at `$source: n2k-on-ve.can-socket.<addr>`. Once your HALMET WiFi delta path is approved you'll see the **same SK path appearing from two sources**: the N2K path and `$source: ws.<ip>:<port>` (or `sensesp.halmet-wind.…`). This is normal and supported — SignalK keeps the per-source `values[]` array and exposes a single arbitrated value per the configured source priority. Set N2K as priority 1 in Server → Settings → Source Priorities; clients then prefer N2K and fail over to WiFi if N2K stops updating.

The honest V1 vs V2 comparison: **V1 (N2K) wins for the boat's critical instrumentation**. Wind drives the autopilot's wind mode, instrument displays at the helm, and chartplotter laylines — all N2K-native consumers (Raymarine Axiom, B&G Zeus, Garmin, Simrad). Losing WiFi must not blind the helm. V1 keeps wind alive on the wired backbone independently of the SignalK server and WiFi router. **V2 adds real value** for redundancy/cross-check (alarm if the two sources diverge), browser/tablet dashboards (KIP) without N2K hardware, easier debugging via the Data Browser during commissioning, and future SK-only consumers (Saillogger, signalk-to-influxdb, Grafana). Critically, **do not install `signalk-to-nmea2000`** on top of this configuration — it would re-emit your SK wind back onto the bus, creating a loop.

---

## Other marine sensor applications on HALMET

### Engine RPM (PGN 127488)

Use any D channel (default D1). Wire the alternator W terminal through an in-line fuse (5–250 mA) to D1; HALMET clamps ±32 V but engage the **LPF jumper** on D1's back for the 2.3 kHz filter, mandatory to count cleanly. Pulses-per-revolution = alternator_pole_pairs × (engine_pulley / alternator_pulley); typical Bosch/Valeo with 6 pole pairs and 2:1 ratio gives 12 ppr at crankshaft. Calibrate with a handheld optical tach. Hall-effect flywheel sensors and ECU tach signals also wire direct; for open-collector Hall sensors, engage D1's pull-up jumper. PGN 127488 carries Engine Instance (8 bit), Engine Speed (16 bit, 0.25 RPM resolution), Boost Pressure (100 Pa), and Tilt/Trim (1%, signed). SignalK path: `propulsion.<id>.revolutions` in **Hz, not RPM** (SK convention). Use the example firmware's `N2kEngineParameterRapidSender` directly.

### Engine coolant temperature & oil pressure (PGN 127489)

A-channel inputs. For VDO-style resistive thermistors (~287 Ω at 40 °C, ~50 Ω at 100 °C), engage the **CCS jumper** to source 10 mA — voltage = 10 mA × R, perfectly inside HALMET's range. Use SensESP's `CurveInterpolator` to map resistance → Kelvin from the sender's R-T table (sample every 10–20 °C). For VDO 0–10 bar resistive oil pressure senders (10 Ω empty → 184 Ω full), same CCS mode. For modern 0.5–4.5 V ratiometric pressure senders, you need an external 5 V supply (HALMET doesn't expose 5 V at terminals) and read in passive mode. PGN 127489 (fast-packet, 500 ms cadence) encodes Oil Pressure at **100 Pa resolution** and Coolant Temperature at **0.01 K resolution**. SignalK paths: `propulsion.<id>.temperature` (K), `propulsion.<id>.oilPressure` (Pa).

### Tank levels (PGN 127505)

A-channel input with CCS engaged is the canonical HALMET approach. EU 0–180 Ω senders sit fully within HALMET's CCS measurement range; US 240–33 Ω senders are covered up to 300 Ω. The example firmware's `ConnectTankSender(ads1115, channel, name, sk_id, sort_order, sk_enable)` does all the work — three-stage pipeline (resistance → ratio via CurveInterpolator → volume via Linear), three SignalK paths (`tanks.fuel.main.senderResistance`, `.currentLevel`, `.currentVolume`), and a `N2kFluidLevelSender(config_path, instance, fluid_type, capacity_L, nmea2000)` for the N2K side. PGN 127505 carries Instance (4 bit), Fluid Type enum (0=Fuel diesel, 1=Water, 2=Gray water, 3=Live well, 4=Oil, 5=Black water, 6=Fuel gasoline), Level (16 bit signed, **0.004% resolution**), and Capacity (32 bit, 0.1 L resolution). Web UI lets you tune the resistance→ratio curve per tank without rebuilding firmware — fill in known increments and add curve samples.

### Bilge alarm & digital switches (PGN 127501)

D-channel input. For a float switch grounded by the bilge pump circuit (low when full), engage D's **pull-up** jumper; for a switch closing to +12 V, use pull-down. Use **PGN 127501 (Binary Status Report)** to publish — single-frame, 8 bytes, 2500 ms cadence, fields Instance (8 bit) + up to 28 channels of 2-bit status (0=Off, 1=On, 2=Error, 3=Unknown). **Not** PGN 127502, which is Switch Bank Control (commands TO a switch bank, not reports FROM one). The example firmware's `ConnectAlarmSender(pin, name)` gives you the Signal K side at `electrical.switches.bilge.1.state`; for N2K emission, use the library's `N2kResetBinaryStatus`, `N2kSetStatusBinaryOnStatus`, and `SetN2kBinaryStatus` helpers (no SensESP wrapper exists in the example — write a small sender class following the `N2kEngineParameterRapidSender` pattern).

### Rudder reference (PGN 127245)

A-channel input. For a resistive potentiometer rudder feedback unit (Maretron RAA100, Lecomble & Schmitt), wire wiper to A and ends to a 5 V rail (from the GPIO header) and GND — passive mode, CCS off. Calibrate by recording raw voltage at hard-port (−45°), center, and hard-stbd (+45°), then apply a `Linear` transform with slope `(π/4) / (V_stbd − V_center)`. PGN 127245 carries Instance (8 bit), Direction Order (3 bit enum), Angle Order (16 bit signed, 0.0001 rad), and Position (16 bit signed, 0.0001 rad). Positive = starboard rudder. SignalK path: `steering.rudderAngle` (rad). Use `SetN2kPGN127245(msg, position_rad, instance, N2kRDO_NoDirectionOrder, N2kDoubleNA)`.

### Battery voltage (PGN 127508)

A-channel direct — HALMET's 10:1 onboard divider already handles 0–32 V batteries with no external scaling. CCS off. Wire B+ through a 100–250 mA fuse to A+; battery GND to A−. The 16-bit ADS1115 gives ~5 mV resolution after divider; calibrate once against a good DMM with a `Linear` transform. PGN 127508 (1500 ms cadence, single-frame) encodes Battery Instance, Voltage (16 bit signed, 0.01 V), Current (16 bit signed, 0.1 A, positive = charging), Temperature (16 bit unsigned, 0.01 K), and SID. SignalK paths: `electrical.batteries.<n>.{voltage,current,temperature,stateOfCharge}`. For SOC and time-remaining, add PGN 127506 (DC Detailed Status, fast-packet, 1500 ms).

---

## Validation and tooling

Three primary validation paths are available given the user's existing infrastructure. The **most authoritative** is reading the actual CAN frames with `candump` on the RPi4 with MacArthur HAT (MCP251xFD on SPI0/CE1, GPIO25 IRQ, 20 MHz crystal — OpenPlotter's CAN Bus app configures this, or in `/boot/firmware/config.txt`: `dtoverlay=mcp251xfd,spi0-1,interrupt=25,oscillator=20000000`). After `sudo ip link set can0 up type can bitrate 250000` and `sudo apt-get install can-utils`, run `candump can0` for raw frames or filter by PGN with `candump can0,09FD0200:1FFFFF00` (matches any priority, any source, PGN 130306). Pipe through `analyzer -json` from canboat for human-readable JSON: `candump -L can0 | candump2analyzer | analyzer -json | jq 'select(.pgn==130306)'`.

The 29-bit CAN ID decode: bits 28..26 = Priority, 25..24 = Reserved+DP, 23..16 = PDU Format (PF), 15..8 = PDU Specific (PS, = destination address if PF<240), 7..0 = Source Address. PGN = `(R<<17 | DP<<16 | PF<<8 | PS)` if PF≥240 (broadcast, PDU2), else `(R<<17 | DP<<16 | PF<<8)`. PGN 130306 = 0x1FD02, PF=0xFD ≥ 240, so PS is part of the PGN and the destination is broadcast. For source 35 priority 2, the full ID is `0x09FD0223`. Example PGN 130306 frame for 10 kn @ 045° apparent: `09FD0223 [8] 00 02 02 AE 1E FA FF FF` (byte 0 = SID, bytes 1-2 = 514 ×0.01 m/s, bytes 3-4 = 7854 ×0.0001 rad, byte 5 = 0xFA with reference enum 2 in low 3 bits, bytes 6-7 = reserved 0xFFFF).

The **most convenient** validation is the **SignalK server's Data Browser** on the Cerbo GX (`http://venus.local:3000/admin/#/dataBrowser`). Filter on `environment.wind.angleApparent`. You'll see two rows once both paths are running: `$source: n2k-on-ve.can-socket.<addr>` (the N2K path decoded by canboatjs) and `$source: ws.<ip>:<port>` or `sensesp.halmet-wind.…` (the WiFi delta path). Both should agree within tens of milliseconds. If they differ by 180° or sign, you have an angle-convention bug (negative-to-port vs 0-to-2π) — fix in firmware. The same Data Browser shows engine, tank, rudder, battery, and bilge paths once those are deployed.

The **most exhaustive** validation is Actisense NMEA Reader on a Windows laptop with an NGT-1 gateway — the gold standard for confirming PGN encoding correctness, showing decoded fields in a live grid. For Cerbo GX direct display: native Venus VRM shows tank levels (127505), battery (127508/127506), engine (127488/127489), and temperature (130312) on its device list. Wind from PGN 130306 is **not** natively displayed on the GX page; it surfaces only via SignalK. To display wind on a chartplotter screen at the helm, you need a Raymarine/B&G/Garmin MFD downstream on the N2K bus, which is exactly the V1 path's purpose. The **Orca Core 2** acts as a passive listener for everything HALMET produces (it accepts PGNs 127245, 127488/89, 127505/06/08, 130306, 130312, and more per Orca docs); open the Orca iOS app → Instruments / Engine / Tanks pages to confirm.

For **bench-testing sin/cos**, use a dual-gang 10 kΩ linear potentiometer (or two phase-locked function generator channels) wired to produce `V_cos = 2.5 V + 2 V·cos θ` and `V_sin = 2.5 V + 2 V·sin θ`. Apply at θ ∈ {0°, 45°, 90°, …, 315°}, log raw ADC counts, verify computed angle within ±2° at each. Larger errors indicate channel offset/gain mismatch — correct with per-channel `Linear` transforms. For **bench-testing the pulse counter**, jumper the example firmware's `kTestOutputPin = GPIO 33` (configured for 380 Hz, 50% duty) to D1 — you should read 3.8 Hz raw (or 228 RPM with the default 100 ppr multiplier).

---

## Action checklist for V1 firmware delivery

The path to a working V1 wind interface in days, not weeks: fork `github.com/hatlabs/HALMET-example-firmware`, keep `halmet_*.h/.cpp` and `halmet_serial.h` unchanged, drop in the `sin_cos_angle_transform.h` from above, rewrite `main.cpp` per the dual-output skeleton, change the N2K device info to Function 130 / Class 85 (wind sensor), wire your external 8 V regulator → Raymarine Red, Blue → A1, Green → A2, Yellow → D1, Screen → GND. Initial bench bringup uses the test PWM on GPIO 33 jumpered to D1 (confirms pulse counting), a dual pot on A1/A2 (confirms angle computation). First flash with `pio run -e halmet -t upload`; connect to the `halmet-wind` WiFi AP (password `thisisfine`), browse to 192.168.4.1, configure your home/boat WiFi and (optionally) the SignalK server. Once on the boat WiFi, browse to `http://halmet-wind.local/` for live config of all calibration values. Confirm with `candump can0,09FD0200:1FFFFF00` on the RPi4 that PGN 130306 frames arrive every 100 ms. Confirm in SignalK Data Browser that both `$source`s populate `environment.wind.angleApparent` and `.speedApparent`. Calibrate the angle offset by aligning a known-bow-on wind (or a manually-positioned vane), and the speed multiplier by motoring at GPS-measured speeds in calm air. Set N2K as source priority 1 in the SignalK admin. The masthead transducer is then alive on the backbone, ready to drive any N2K-aware autopilot, MFD, or instrument display, with a parallel SignalK feed for dashboards and debugging.

The two most likely failure modes at first commissioning are the angle direction convention (positive should be clockwise viewed from above per NMEA 2000; if your vane reads backward, swap the sign of either sin or cos input — equivalent to swapping Blue and Green wires) and the speed scaling (default 0.5144 m/s/Hz assumes the newer egg-cup transducer; older square-cup units may need ~0.36 m/s/Hz). Both are tunable from the web UI without reflashing. Beyond that, monitor the serial console (115200 baud, with `esp32_exception_decoder` filter) for any stack traces — and if a transducer is sometimes disconnected at the mast, the SensESP `SinCosAngle` transform emits NaN when sin/cos magnitude drops below `min_magnitude`, which propagates to `N2kDoubleNA` on the bus and shows as "no data" rather than a stuck value on downstream instruments.