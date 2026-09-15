# HALMET Wind — Raymarine ST60+ wind vane → NMEA 2000

Firmware for the [Hat Labs **HALMET**](https://hatlabs.fi/) board that reads a legacy
**Raymarine ST60+ wind transducer** — the vane and cups at the top of the mast,
whose display head died but whose sensor is still healthy — and republishes
**apparent wind** onto the boat's
**NMEA 2000** backbone.

> **The story:** the ST60+ *display head* failed; the wind vane 15 m up the mast
> was fine. Rather than climb the mast or replace the instrument, HALMET taps the
> old sensor's raw sin/cos and cup-pulse wires directly and puts wind back on the bus.

📖 **Project page:** https://den200.github.io/halmet-wind-vane/ · 🔌 **[Wiring & power guide](docs/WIRING.md)**

---

## What it does

- **V1 (primary):** transmits **PGN 130306 (Wind Data)** every 100 ms (10 Hz) with
  `WindReference = Apparent`. This is the deliverable — the boat (Orca Core 2 + N2K
  displays) consume wind over the wired backbone.
- **V2 (secondary):** publishes the same calibrated values to SignalK over WiFi
  (`environment.wind.angleApparent` / `speedApparent`), so any SignalK plugin or
  web app (KIP, Freeboard, instrument panels, loggers) can display them once installed.

## How it works

| Pipeline | Chain |
|---|---|
| **Angle** | A1/A2 (sin/cos) → ADS1115 16-bit → raw-voltage validity gate (2–6 V, else NaN) → per-channel centring → moving average **of the sin/cos vector** (5) → `atan2(sin,cos)` → AWA |
| **Speed** | D1 pulse → DigitalInputCounter (RISING, 500 ms) → Frequency → × K (m/s per Hz) → no-data while the sin/cos rails show a dead transducer → AWS |
| **Output** | latched values → NMEA 2000 PGN 130306 @ 10 Hz **and** SignalK `environment.wind.*` |

The vane encodes its angle as two ratiometric voltages (sine and cosine);
recovering the angle is `atan2(sin, cos)`. Since `atan2` is scale-invariant,
calibration only needs to **centre** each channel — see [`src/sin_cos_angle_transform.h`](src/sin_cos_angle_transform.h).

Two details there are deliberate. Smoothing averages the **sin/cos vector**, not
the emitted angle: averaging the angle breaks at the ±π wrap, where samples
alternate between ≈ +179° and ≈ −179° and mean to 0°, so dead astern would read
as dead ahead. And the no-data check watches the **raw terminal voltage**
(Blue/Green must be inside 2–6 V), not the computed magnitude: an unpowered
transducer puts both channels at 0 V, which centres to a large, perfectly steady
vector and yields a convincing, unchanging −135°. The gate sits **before** the
smoothing, turning a bad reading into NaN, which clears the averaging window —
so nothing from the fault blends into the first angles after recovery. The same
0 V rails also mean the pulse line is dead, so speed goes to no-data as well
instead of reporting a false calm.

## Wiring (summary)

| Wire | Signal | To | Mode |
|---|---|---|---|
| Red | +8.0 V | external buck OUT+ | supply (set buck to **8.0 V** first!) |
| Screen | 0 V / GND | shared sensor ground | — |
| Blue | sine | **A1** | passive, CCS off |
| Green | cosine | **A2** | passive, CCS off |
| Yellow | speed pulse | **D1 / GPIO23** | passive, RISING |

⚠️ The cheap buck modules ship at ~20 V — set it to **8.0 V on the bench** before
connecting the sensor, or you'll cook it. Full details in [docs/WIRING.md](docs/WIRING.md).

## Components

Most of the build is reused — the wind vane, its cable, and the boat's NMEA 2000
network are already there. You really only buy two things:

| Part | For | Link |
|---|---|---|
| **Hat Labs HALMET** | the bridge board — reads the vane/cups, speaks N2K, hosts the web UI | [shop.hatlabs.fi](https://shop.hatlabs.fi/products/halmet) |
| **Adjustable buck converter** (with voltage display) | steps boat 12 V down to the vane's **8.0 V** — set it before wiring | [amazon.de](https://www.amazon.de/dp/B0D8T9HDR7) |
| **Raymarine ST60+ wind vane** | the existing masthead sensor — reused, not bought | — |
| Inline ~1 A fuse + wire | fuse the buck's 12 V input; three signal taps off the existing mast cable | — |

Plus the open-source firmware in this repo.

## Live calibration (web UI, no reflash)

All tunables persist to flash and are editable at `http://halmet-wind.local/`:

- **Angle offset** (`offset_rad`) — align vane zero to the bow.
- **Angle direction** (`sin_sign`) — `+1` normal, `−1` if wind reads backward.
- **Per-channel centring** — set each sin/cos midpoint (≈ Vmid).
- **Smoothing** (`samples`) — sin/cos samples averaged before the angle is computed; 5 ≈ a 2.5 s window at the ~2 Hz sample rate.
- **Speed multiplier K** — `0.5144` m/s·Hz⁻¹ for egg-cup ST60+ (~1 kn/Hz), ~`0.36` square-cup.

With the mast up, derive the sin/cos centres by ellipse-fit on logged free-rotation
data ([`tools/ellipse_fit.py`](tools/ellipse_fit.py)); trim K against GPS in calm air.

## NMEA 2000 device identity

Function **130** (Atmospheric), Class **85** (External Environment),
Manufacturer **2046** (unregistered), preferred source address **35**.

## Build & flash

```bash
pio run -e halmet              # build
pio run -e halmet -t upload    # flash over USB (first time only; then OTA, see below)
pio device monitor -b 115200   # serial
tools/run_host_tests.sh        # angle-transform regression tests, on the laptop
```

`src/sin_cos_angle_transform.h` is pure math over a thin SensESP base, so it is
compiled against minimal stubs in [`test/host/`](test/host/) and exercised on a
laptop — full circle accuracy, behaviour through the ±π wrap, NaN recovery,
sign/offset handling and config clamping. No board required.

SensESP v3 on PlatformIO (pioarduino platform), NMEA 2000 via the ESP32 TWAI
driver, ADS1115 via Adafruit ADS1X15. Board target `esp32dev`.

## Updating firmware from a phone (OTA)

The USB flash is needed once. After that the board updates itself from a
browser at `http://halmet-wind.local/update` (also linked from the config
page), over the boat Wi-Fi or the board's own access point:

1. Bump `FW_VERSION` in `src/version.h`, commit, tag `v<version>`, push the tag.
   [GitHub Actions](.github/workflows/firmware.yml) builds and attaches
   `halmet-wind-v<version>.bin` (+ `.sha256`) to a release. It refuses to
   publish if the tag and `FW_VERSION` disagree.
2. On the phone, open the update page and press **Check latest release**. It
   asks the GitHub API from the browser and links the `.bin`. If the boat Wi-Fi
   has no internet, download the file on mobile data first — it lands in Files.
3. Pick the file, press **Flash**. The board writes it into the spare OTA slot,
   verifies the image (segment checksums and the SHA-256 the build appends),
   only then marks it bootable, and reboots. The page waits and shows the new
   version once it is back. Wind data pauses for about ten seconds.

A truncated or wrong file is refused before the switch — the first bytes are
checked for the ESP32 image magic and chip id, and the whole image is verified
at the end — so the running firmware keeps running. If a new image ever fails
to boot, the bootloader rolls back to the previous slot on the next reset.
Calibration and Wi-Fi settings live in their own partition and survive updates.

## Validation

- **On the wire:** `candump can0,09FD0200:1FFFFF00` on the RPi/MacArthur HAT →
  a PGN 130306 frame from source 35 every 100 ms.
- **SignalK / Orca:** apparent wind angle & speed populate and read correctly.

## License

Licensed under the **Apache License 2.0** — see [LICENSE](LICENSE). This is the same
license as the Hat Labs HALMET example firmware and the SensESP framework it builds on,
keeping everything license-compatible.

## Credits

Forked from the structure of the Hat Labs HALMET example firmware; built on SensESP.
Not affiliated with Raymarine or Orca. Built with [Claude Code](https://claude.com/claude-code).
