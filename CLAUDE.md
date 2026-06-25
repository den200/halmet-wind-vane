# CLAUDE.md — HALMET Wind Vane Firmware

## Mission

Build firmware for the **Hat Labs HALMET** board that reads a legacy **Raymarine ST60+ masthead wind transducer** (the display head died; the masthead itself is healthy) and publishes apparent wind onto the boat's **NMEA 2000 backbone**.

- **V1 (priority, canonical): NMEA 2000 direct.** Transmit **PGN 130306 (Wind Data)** every 100 ms with `WindReference = N2kWind_Apparent` (=2). This is the only path that matters for the deliverable. The boat consumes wind over the wired N2K backbone (Orca Core 2 is N2K-only).
- **V2 (secondary, free if it falls out): SignalK over WiFi.** Same calibrated values to `environment.wind.angleApparent` (rad) and `environment.wind.speedApparent` (m/s). Nice for dashboards and debugging; never a substitute for V1.

**Read `docs/HALMET-wind-skill.md` first.** It is the authoritative reference: full hardware spec, pin map, ST60+ electrical interface, the complete `main.cpp` skeleton, the `SinCosAngle` transform, `platformio.ini`, PGN 130306 field layout, calibration, and validation. This file (`CLAUDE.md`) is *how to work*; the skill is *what to build*.

## Working rules (non-negotiable)

- **KISS / Pareto.** Fix the binding constraint. Do not add features, abstractions, or config the V1 wind path doesn't need.
- **No stubs, no half-measures.** Every function you write is complete and compiles. Don't leave `// TODO` placeholders in a "finished" step.
- **Finish each small step completely before the next.** Compile and (where possible) flash + observe serial before moving on. Order: skeleton compiles → analog reads sane volts → angle computes → pulse counts → N2K frames on the wire → SignalK path → calibration params live.
- **Verify APIs against the actually-resolved libraries, not memory.** The skill's code skeleton is a *guide*. SensESP v3 and the NMEA2000 library have real, specific signatures — read the headers PlatformIO resolves into `.pio/libdeps/` and reconcile. If the skeleton and the real API disagree, the real API wins.
- **Compile early, compile often.** `pio run -e halmet` after every meaningful change. A green build is the unit of progress.
- **Explain tradeoffs, not steps.** When you make a non-obvious choice (a gain setting, an interrupt edge, a buffer size), say why in one line.

## Hardware truths (do not re-derive, do not assume)

These are settled facts about HALMET. Several contradict "obvious" guesses — trust them.

- **CAN is the ESP32 built-in TWAI peripheral (classical CAN 2.0B), NOT an MCP2518FD / CAN-FD.** Use `tNMEA2000_esp32(kCANTxPin, kCANRxPin)` — the same driver the example firmware uses. NMEA 2000 is 250 kbit/s classical CAN; CAN-FD framing never appears. No SPI/CS/INT wiring.
- **No 5 V / 8 V / 12 V rail is exposed at the terminals.** The transducer's 8 V supply comes from the external **MP1584 buck**, not the board.
- **Analog inputs A1–A4 go through an external ADS1115 (16-bit) on I²C @ `0x4b`**, not the ESP32 ADC. Each input has a fixed **10:1 divider** (`kVoltageDividerScale = 33.3/3.3`). Do not change the divider constant; trim with a `calibration_factor` / `Linear` instead.
- **Per-channel 10 mA constant-current source (CCS) exists for resistive senders** — irrelevant here; leave A1/A2 CCS jumpers **OFF** (passive voltage mode for sin/cos).
- **Digital input pin map (note D3/D4 are swapped):** `D1=GPIO23, D2=GPIO25, D3=GPIO27, D4=GPIO26`. Inputs are ±32 V tolerant, hardware Schmitt-triggered. No software debounce in the counter.
- **CAN pins:** `TX=GPIO19, RX=GPIO18`. **I²C:** `SDA=GPIO21, SCL=GPIO22`. **Board target is `esp32dev`** (stock ESP32-WROOM-32E).

## Wiring (transducer → HALMET)

| Wire | Signal | Connect to | Mode |
|---|---|---|---|
| Red | +8.0 V | **MP1584 output** (pre-set to 8.0 V, verified) | external supply |
| Screen/bare | 0 V / GND | HALMET GND (single star point with MP1584 GND) | — |
| Blue | sine | **A1** | passive, CCS OFF |
| Green | cosine | **A2** | passive, CCS OFF |
| Yellow | speed pulse | **D1 (GPIO23)** | passive, RISING edge |

Masthead sin/cos swing ~2.5–5.5 V at the terminal → ~0.25–0.55 V at the ADS after the 10:1 divider → well inside `GAIN_ONE` (±4.096 V). 16-bit resolution far exceeds the transducer's mechanical accuracy, so `GAIN_ONE` is correct; don't over-engineer the gain. Yellow is an actively-driven ~8 V/3.2 V square wave (no pull-up needed); the Schmitt trigger separates it cleanly.

## Toolchain & build

- **Framework:** SensESP **v3** (`SignalK/SensESP @ ^3.1.0`) on PlatformIO, **pioarduino** platform fork, partition `default_8MB.csv`. NMEA 2000 via `ttlappalainen/NMEA2000-library`. ADS via `adafruit/Adafruit ADS1X15`. Copy `platformio.ini` from the skill verbatim as the starting point.
- **v3 gotchas (the skill lists them all):** `ConfigItem(obj)` is mandatory to expose anything in the web UI; `SetupLogging()` not `SetupSerialDebug()`; `-D USE_ESP_IDF_LOG`; no `Startable`/`->start()` — use `event_loop()->onDelay(0, ...)`; global `event_loop()`; `producer->connect_to(consumer)`.
- **Build:** `pio run -e halmet`
- **Flash (USB on the Mac):** `pio run -e halmet -t upload`
- **Serial:** `pio device monitor -b 115200` (the env sets `esp32_exception_decoder`). Mac port is typically `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`; PlatformIO usually auto-detects.

## Firmware architecture

Fork the structure of `github.com/hatlabs/HALMET-example-firmware`. Keep `halmet_const.h`, `halmet_analog.*`, `halmet_serial.h` (board serial → N2K unique ID). Add `sin_cos_angle_transform.h` (from the skill — the two-input "member-consumer" pattern, emits `atan2(sin, cos)`, NaN-guards on low magnitude). Rewrite `main.cpp` per the skill's dual-output skeleton.

Pipeline:
- **Angle:** A1→sin, A2→cos → per-channel centering `Linear` (subtract Vmid, normalize) → `SinCosAngle` → `MovingAverage(5)` → latch for N2K **and** `SKOutput` (throttled to 100 ms).
- **Speed:** `DigitalInputCounter(D1, RISING, 500ms)` → `Frequency` → `Linear(K)` (m/s per Hz) → latch for N2K **and** `SKOutput`.
- **N2K send:** one `event_loop()->onRepeat(100, ...)` builds PGN 130306 from the latched values (convert angle −π..+π → 0..2π; NaN → `N2kDoubleNA`), `SetN2kWindSpeed(...)`, `SendMsg`. `ParseMessages()` on a 1 ms `onRepeat`. **No FreeRTOS task** — everything on the ReactESP loop.

**N2K device identity (change from the example's engine defaults):** Function **130** (Atmospheric), Class **85** (External Environment), Manufacturer **2046** (unregistered), preferred source address 35. Never use a registered manufacturer code.

## Live calibration parameters (expose via ConfigItem, no reflash to change)

- **Angle offset (rad)** — align vane zero to boat centerline.
- **Angle sign / input swap** — flip if wind reads backward (clockwise-from-above is correct).
- **Per-channel Vmid + gain** — center and normalize sin/cos (default Vmid ≈ 4.0 V at terminal; refine from logged free-rotation data).
- **Speed multiplier K (m/s per Hz)** — default 0.5144 (egg-cup, ~1 kn/Hz); ~0.36 for square-cup. Set empirically.

Mast is up, so angle calibration is by **ellipse-fit on logged sin/cos during free-rotation sailing**, not a bench sweep. Make sure raw sin/cos volts are published (SignalK or serial) so the data can be logged for the fit.

## Validation loop

1. **Bench, on the Mac (do this first, before the boat):** jumper the example's test PWM (`GPIO33`, 380 Hz) to D1 → expect ~3.8 Hz raw on serial. Feed A1/A2 from a dual-gang 10 kΩ pot (or two phase-offset sources) → verify computed angle within ±2° around the circle. This proves the whole pipeline without the transducer.
2. **On the boat:** `candump can0,09FD0200:1FFFFF00` on the RPi4 (MacArthur HAT) → confirm PGN 130306 frames every 100 ms. Then SignalK Data Browser on the Cerbo GX → both `$source`s populate the wind paths and agree.

**Definition of done for V1:** PGN 130306 transmits at 10 Hz on the backbone with correct apparent angle and speed; values visible in SignalK Data Browser and on the Orca app; all four calibration parameters adjustable live from `http://halmet-wind.local/`; clean build with no warnings; serial console free of stack traces.

## Guardrails

- Do **not** install or rely on `signalk-to-nmea2000` anywhere downstream — it would re-emit SK wind onto the bus and create a feedback loop. The firmware is the sole PGN 130306 source.
- Do **not** transmit true wind from the firmware. Publish apparent only; let the SignalK server derive true wind if wanted.
- Do **not** drive depth/STW/water-temp through this board — out of scope and electrically wrong for HALMET.
- Leave the on-board **120 Ω CAN termination jumper OPEN** (the backbone is already terminated at both ends).
- Treat the skill's code as a guide, the resolved library headers as truth.