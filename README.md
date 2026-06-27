# HALMET Wind — Raymarine ST60+ masthead → NMEA 2000

Firmware for the [Hat Labs **HALMET**](https://hatlabs.fi/) board that reads a legacy
**Raymarine ST60+ masthead wind transducer** — whose display head died but whose
masthead is still healthy — and republishes **apparent wind** onto the boat's
**NMEA 2000** backbone.

> **The story:** the ST60+ *display head* failed; the masthead 15 m up was fine.
> Rather than climb the mast or replace the instrument, HALMET taps the masthead's
> raw sin/cos and cup-pulse wires directly and puts wind back on the modern bus.

📖 **Project page:** https://den200.github.io/halmet-wind-vane/ · 🔌 **[Wiring & power guide](docs/WIRING.md)**

---

## What it does

- **V1 (primary):** transmits **PGN 130306 (Wind Data)** every 100 ms (10 Hz) with
  `WindReference = Apparent`. This is the deliverable — the boat (Orca Core 2 + N2K
  displays) consume wind over the wired backbone.
- **V2 (secondary):** publishes the same calibrated values to SignalK over WiFi
  (`environment.wind.angleApparent` / `speedApparent`) for dashboards and logging.

## How it works

| Pipeline | Chain |
|---|---|
| **Angle** | A1/A2 (sin/cos) → ADS1115 16-bit → per-channel centring → `atan2(sin,cos)` (NaN-guarded) → MovingAverage(5) → AWA |
| **Speed** | D1 pulse → DigitalInputCounter (RISING, 500 ms) → Frequency → × K (m/s per Hz) → AWS |
| **Output** | latched values → NMEA 2000 PGN 130306 @ 10 Hz **and** SignalK `environment.wind.*` |

The masthead encodes vane angle as two ratiometric voltages (sine and cosine);
recovering the angle is `atan2(sin, cos)`. Since `atan2` is scale-invariant,
calibration only needs to **centre** each channel — see [`src/sin_cos_angle_transform.h`](src/sin_cos_angle_transform.h).

## Wiring (summary)

| Wire | Signal | To | Mode |
|---|---|---|---|
| Red | +8.0 V | external buck OUT+ | supply (set buck to **8.0 V** first!) |
| Screen | 0 V / GND | shared sensor ground | — |
| Blue | sine | **A1** | passive, CCS off |
| Green | cosine | **A2** | passive, CCS off |
| Yellow | speed pulse | **D1 / GPIO23** | passive, RISING |

⚠️ The cheap buck modules ship at ~20 V — set it to **8.0 V on the bench** before
connecting the masthead, or you'll cook it. Full details in [docs/WIRING.md](docs/WIRING.md).

## Live calibration (web UI, no reflash)

All tunables persist to flash and are editable at `http://halmet-wind.local/`:

- **Angle offset** (`offset_rad`) — align vane zero to the bow.
- **Angle direction** (`sin_sign`) — `+1` normal, `−1` if wind reads backward.
- **Per-channel centring** — set each sin/cos midpoint (≈ Vmid).
- **Speed multiplier K** — `0.5144` m/s·Hz⁻¹ for egg-cup ST60+ (~1 kn/Hz), ~`0.36` square-cup.

With the mast up, derive the sin/cos centres by ellipse-fit on logged free-rotation
data ([`tools/ellipse_fit.py`](tools/ellipse_fit.py)); trim K against GPS in calm air.

## NMEA 2000 device identity

Function **130** (Atmospheric), Class **85** (External Environment),
Manufacturer **2046** (unregistered), preferred source address **35**.

## Build & flash

```bash
pio run -e halmet              # build
pio run -e halmet -t upload    # flash over USB
pio device monitor -b 115200   # serial
```

SensESP v3 on PlatformIO (pioarduino platform), NMEA 2000 via the ESP32 TWAI
driver, ADS1115 via Adafruit ADS1X15. Board target `esp32dev`.

## Validation

- **On the wire:** `candump can0,09FD0200:1FFFFF00` on the RPi/MacArthur HAT →
  a PGN 130306 frame from source 35 every 100 ms.
- **SignalK / Orca:** apparent wind angle & speed populate and read correctly.

## License & credits

Open firmware forked from the structure of the Hat Labs HALMET example firmware.
Not affiliated with Raymarine or Orca. Built with [Claude Code](https://claude.com/claude-code).
