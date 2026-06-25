# HALMET Wind — Wiring & Power

How to wire a Raymarine ST60+ masthead wind transducer to a Hat Labs HALMET
board, powering the masthead from an external adjustable buck converter. The
same content is summarised on the device's own web UI (the "Help 1–6" cards at
`http://halmet-wind.local/`).

---

## ⚠️ Safety first: set the buck to 8.0 V before connecting anything

The cheap adjustable step-down (buck) modules ship with the output set to **~20 V**
from the factory. The Raymarine masthead expects **8.0 V (±0.25 V)**. Connecting
20 V to the Red wire can **destroy the masthead electronics**.

With **nothing connected to the buck output**:

1. Feed the buck **input** from 12 V and power it on.
2. Turn the **trim potentiometer** until a **multimeter** on the output reads
   **8.0 V**. (Don't fully trust the onboard LED meter — verify with a DMM.)
3. If the output just follows the input and won't come down, turn the pot
   **counter-clockwise 10+ turns** first, then adjust.
4. The push-button "calibration" procedure in the module's manual only trims the
   **LED voltmeter's display accuracy** — it does **not** set the output. The
   output voltage is set by the **potentiometer**.

Only after it reads a stable 8.0 V: power off and connect the Red wire.

---

## Power topology

| Device | Powered from | Notes |
|---|---|---|
| **HALMET** | NMEA 2000 backbone (4-pin connector) | No separate supply needed; the board makes its own 3.3 V. |
| **Buck converter** | Boat 12 V, **input** side, via a ~1 A fuse | Non-isolated; set output to **8.0 V**. |
| **Masthead** | Buck **output** (8.0 V) | Draws only tens of mA — a 3 A buck is hugely oversized, which is fine. |

- Buck **OUT+** → transducer **Red**.
- Buck **OUT−** → the shared sensor ground (see below).

---

## Grounding (the part that's easy to get wrong)

The masthead's sine/cosine outputs are voltages **relative to its own ground**
(the Screen/bare wire) and ratiometric to its 8 V supply. HALMET's ADC measures
those voltages **relative to HALMET's input-side ground**. For the angle to be
correct, **those two grounds must be the same node.**

Tie together, at one point:

- Buck **OUT−** (= buck **IN−**; these modules are non-isolated, so input and
  output share ground)
- Transducer **Screen / bare**
- **HALMET's input-side GND** — the ground terminal on the **analog-input** side
  of the board (the same ground the A1–A4 / D1–D4 terminals reference). Check the
  board silkscreen for the input-side GND terminal/pad.

**Do not** tie the sensor ground to HALMET's **N2K/CAN connector ground.**
HALMET deliberately isolates its sensor inputs (A1–A4, D1–D4, ADS1115) from the
N2K/MCU domain. Referencing the sensor to the input-side ground keeps the
measurement clean and the isolation intact.

> On a typical boat the buck's 12 V negative and the N2K backbone negative both
> trace back to the same battery negative externally — that's fine. HALMET's
> internal barrier still keeps the two ground domains separate on the board,
> which is exactly the intent. (Full galvanic isolation of the masthead from the
> bus would require an *isolated* DC-DC for the 8 V; these cheap bucks are not
> isolated, so that benefit is not available — not a problem for a wind sensor.)

---

## Transducer wire map

Raymarine ST60+ / ST60 / i60 masthead (same colours across generations):

| Wire | Signal | Connect to | Mode |
|---|---|---|---|
| **Red** | +8.0 V supply | Buck **OUT+** (8.0 V) | external supply |
| **Screen / bare** | 0 V / ground | shared sensor ground (above) | — |
| **Blue** | sine | **A1** | passive, CCS **OFF** |
| **Green** | cosine | **A2** | passive, CCS **OFF** |
| **Yellow** | speed pulse | **D1** (GPIO23) | passive, RISING edge |

If the wind angle rotates the **wrong way** after calibration, set
`sin_sign = −1` on the **Wind angle** card (equivalent to swapping Blue↔Green).

---

## Board jumpers

- **A1 & A2** — constant-current-source (CCS) jumpers **OFF** (passive voltage
  mode; CCS is for resistive senders, not for sin/cos).
- **D1** — no pull-up needed (Yellow is an actively-driven ~8 V/3.2 V square
  wave). Engage D1's **2.3 kHz low-pass** jumper only if a long cable causes
  noise/false pulse counts.
- **On-board 120 Ω CAN termination** — leave **OPEN**. The NMEA 2000 backbone is
  already terminated at both ends; adding a third terminator unbalances the bus.

---

## Calibration (all live in the web UI, no reflash)

- **Speed multiplier K** (`/wind/speed/cal`): `0.5144` for the newer egg-cup
  ST60+ (~1 knot per Hz); ~`0.36` for older square-cup units. Fine-tune by
  motoring at GPS-measured speed in calm air (average both directions).
- **Per-channel centering** (`/wind/sin/cal`, `/wind/cos/cal`): free-rotate the
  vane, log the sin/cos terminal volts, and set each channel's offset so the
  midpoint matches the measured Vmid (~4.0 V at the terminal).
- **Angle offset** (`/wind/angle/cal` → `offset_rad`): align 0° to the boat
  centreline (bow).
- **Angle direction** (`/wind/angle/cal` → `sin_sign`): `+1` normal, `−1` if the
  wind reads backward. NMEA 2000 convention is positive = clockwise viewed from
  above.

---

## Validation

- **On the wire (authoritative):** on the RPi4 / MacArthur HAT,
  `candump can0,09FD0200:1FFFFF00` → a PGN 130306 frame (`09FD0223`, source 35)
  every 100 ms.
- **SignalK Data Browser / Orca app:** confirm the apparent wind angle and speed
  populate and read correctly.
