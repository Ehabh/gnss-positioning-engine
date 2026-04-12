# GNSS Positioning Engine — User Guide

**Version:** 1.0.0  
**Hardware:** Any RTCM3-capable GNSS receiver  
**Platform:** macOS / Linux / Windows (Python 3.10+)

---

## Table of Contents

1. [Installation](#1-installation)
2. [Launching the App](#2-launching-the-app)
3. [UI Overview](#3-ui-overview)
4. [Connecting the Receiver](#4-connecting-the-receiver)
5. [Required Receiver Output](#5-required-receiver-output)
6. [Positioning Modes](#6-positioning-modes)
7. [Understanding the Displays](#7-understanding-the-displays)
8. [Session Recording](#8-session-recording)
9. [Console Log Format](#9-console-log-format)
10. [Reference Accuracy Comparison](#10-reference-accuracy-comparison)
11. [Design Internals](#11-design-internals)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Installation

### Prerequisites

- Python 3.10 or newer
- One (required) or two (optional) GNSS receivers connected via USB or UART adapter
  - The receiver must be configured to output RTCM3 MSM4 or MSM7 observations (messages 1074–1077 / 1084–1087 / 1094–1097 / 1124–1127) and broadcast ephemeris (messages 1019 / 1020 / 1042 / 1046) for all desired constellations
  - **Only SPS is currently operational.** DGNSS and RTK toolbar buttons exist as placeholders for future work.
  - **GLONASS** satellites are decoded and displayed but are not included in the WLS solution until a PZ-90 → WGS-84 coordinate transform is implemented.

### Install

```bash
git clone https://github.com/yourusername/gnss-positioning-engine.git
cd gnss-positioning-engine

# Create and activate a virtual environment (recommended)
python -m venv .venv
source .venv/bin/activate          # macOS / Linux
.venv\Scripts\activate             # Windows

# Install dependencies
pip install PySide6 numpy pyserial
```

`requirements.txt` pins tested versions. `pyproject.toml` defines the package for editable installs (`pip install -e .`).

---

## 2. Launching the App

**From the terminal:**
```bash
python main.py
```

**On macOS — double-click launcher:**  
Double-click `launch_gnss_engine.command` in Finder. It auto-detects the `.venv` and opens the app. The Dock entry appears as **GNSS Engine** with the satellite-dish icon.

**CLI flags:**
```
python main.py --log-level DEBUG     # verbose RTCM/engine logging to terminal
python main.py --log-level WARNING   # quiet mode
```

---

## 3. UI Overview

The interface features a dark theme with dockable panels and a central tabbed workspace.

### Toolbar

| Control | Description |
|---|---|
| **Rover port / baud** | Serial port and baud rate for the rover receiver |
| **Connect** | Opens the serial connection; button changes to **Disconnect** |
| **Base port / baud** | Serial port for the optional base station receiver |
| **Connect Base** | Opens base station connection |
| **SPS / DGNSS / RTK** | Switches active positioning mode. Only **SPS** is currently operational — DGNSS and RTK are present as placeholders |
| **Record** | Starts JSONL session logging; shows **Stop** while active |
| **Refresh Ports** | Re-scans all serial ports |

### Left Dock

| Panel | Contents |
|---|---|
| **Fix Badge** | Large coloured badge: fix type, satellite count used, GPS TOW |
| **Position** | Latitude, longitude, altitude (ellipsoidal) |
| **Quality** | HDOP, VDOP, PDOP, horizontal sigma, vertical sigma |
| **Reference Error** | Source, fix mode (SPS/DGNSS), reference lat/lon/alt, live 2D/3D error, RMS 2D/3D |
| **Ephemeris** | Count of stored broadcast ephemerides per constellation |
| **Session Logging** | Log file path, start/stop button, live recording indicator |
| **Map Settings** | Tile URL, API key (Google Maps), manual reference coordinates |
| **Scatter Controls** | Zoom in/out, fit, clear, auto-scale toggle |

### Right Dock — Satellite Table

Shows every tracked satellite updated each epoch:

| Column | Description |
|---|---|
| **PRN** | Satellite identifier (G01–G32, R01–R24, E01–E36, C01–C63) |
| **System** | Constellation name |
| **CNR (dB-Hz)** | Carrier-to-noise ratio; green ≥ 40, yellow 35–40, orange 25–35, red < 25 |
| **Status** | **Used** (in WLS solution) or **Tracked** (received but excluded) |

### Central Tabs

| Tab | Description |
|---|---|
| **Map** | Live rover track on OpenStreetMap (or Google Maps); reference point shown if available |
| **Signal** | Vertical CNR bar chart, constellation-grouped, colour-coded by signal strength |
| **Scatter** | ENU scatter plot of all rover positions relative to the reference, with CEP50 circle |
| **DOP / 2D** | Scrolling time series of HDOP, VDOP, and live 2D error; selectable window from 5 min to 12 hr |

### Bottom Dock — Console

Time-stamped log of every epoch solution, connection events, and errors. Use the **Clear** button to reset.

---

## 4. Connecting the Receiver

1. Plug the receiver into USB. On macOS it appears as `/dev/cu.usbmodemXXXX`; on Linux as `/dev/ttyUSBX` or `/dev/ttyACMX`; on Windows as `COMX`.
2. Click **Refresh Ports** — the dropdown lists all detected ports with their descriptions.
3. Select the port and set the baud rate to match your receiver's configured output rate (commonly 115200).
4. Click **Connect**. The button turns red. The console prints `Rover connected: /dev/cu.usbmodem...`.
5. If no ports appear, check USB cable and driver. On macOS you may need `CP210x` or `CH340` USB-serial drivers.

---

## 5. Required Receiver Output

The engine is receiver-agnostic — it reads whatever RTCM3 the receiver streams. **Configure your receiver using its own manufacturer software before connecting.** The engine begins processing as soon as valid RTCM frames arrive on the serial port.

| RTCM message | Content | Required for |
|---|---|---|
| 1074 or 1077 | GPS MSM4 / MSM7 observations | GPS |
| 1084 or 1087 | GLONASS MSM4 / MSM7 observations | GLONASS (tracked; not yet in WLS) |
| 1094 or 1097 | Galileo MSM4 / MSM7 observations | Galileo |
| 1124 or 1127 | BeiDou MSM4 / MSM7 observations | BeiDou |
| 1019 | GPS broadcast ephemeris | GPS |
| 1020 | GLONASS broadcast ephemeris | GLONASS |
| 1042 | BeiDou broadcast ephemeris | BeiDou |
| 1046 | Galileo I/NAV broadcast ephemeris | Galileo |
| GGA / RMC (NMEA) | Receiver's own position fix | Optional — enables reference accuracy display |

Enable all constellations you want the engine to track. Any constellation whose MSM or ephemeris messages are absent will simply not appear in the satellite table or contribute to the solution. Both MSM4 and MSM7 formats are decoded correctly.

---

## 6. Positioning Modes

> **Only SPS is currently implemented.** The DGNSS and RTK toolbar buttons exist as placeholders for future work — selecting them has no effect on the solution.

### SPS — Standard Positioning Service (active)

Computes position from rover observations alone. No base station required.

**Constellations used in WLS:** GPS, Galileo, BeiDou MEO/IGSO  
**Constellations tracked but excluded:** GLONASS (PZ-90 → WGS-84 transform pending)

**What it does:**
1. Selects the best pseudorange per satellite (dual-frequency iono-free combination if both signals available; Hatch-smoothed single-frequency otherwise)
2. Applies a hard CNR gate: satellites with CNR < 25 dB-Hz are excluded
3. Computes satellite positions from broadcast ephemeris
4. Applies tropospheric correction (Saastamoinen zenith delay + Niell mapping function)
5. Applies ionospheric correction (Klobuchar model for single-frequency; eliminated by iono-free combination for dual-frequency)
6. Solves Weighted Least Squares iteratively; weights are elevation-sine-squared × CNR factor
7. Estimates one receiver clock offset per constellation (inter-system biases)
8. Performs residual-based outlier rejection then re-solves
9. Applies an Earth-surface validity guard — if WLS converges to a position more than 500 km above or below the Earth's surface, the solution is rejected and the warm-start is cleared

**Typical accuracy:** 2–5 m (2D) against the receiver's own NMEA solution. Improves after the 100-epoch Hatch filter warm-up (~100 s at 1 Hz).

**Minimum satellites required:** 4 usable (after CNR gate and elevation mask).

### DGNSS — Differential GNSS (not yet implemented)

**Status:** Planned. Will apply pseudorange corrections from a co-located base station, targeting sub-metre accuracy.

### RTK — Real-Time Kinematic (not yet implemented)

**Status:** Planned. Will use double-difference carrier phase observations with LAMBDA integer ambiguity resolution, targeting ~2 cm accuracy.

---

## 7. Understanding the Displays

### Fix Badge Colours

| Colour | Label | Meaning |
|---|---|---|
| Red (outline) | NO FIX | Fewer than 4 usable satellites or WLS failed |
| Blue | SPS | Standard point positioning from pseudoranges |
| Orange | DGNSS | Pseudorange-corrected solution from base station |
| Purple | RTK FLOAT | Carrier phase, ambiguities not yet resolved |
| Green | RTK FIXED | Carrier phase, integer ambiguities resolved |

### Reference Error Panel

| Field | Description |
|---|---|
| **Source** | NMEA sentence type (GNGGA, GNRMC, etc.) |
| **Ref Mode** | Fix mode reported by the reference receiver (SPS, DGNSS, RTK Fixed, etc.) |
| **Ref Lat/Lon/Alt** | Reference position from the receiver's own NMEA output |
| **Error 2D / 3D** | Current epoch horizontal / 3D distance to reference |
| **RMS 2D / 3D** | Running RMS across all epochs since last clear |

### Signal Bars (Signal tab)

Satellites are grouped by constellation (GPS=blue, GLONASS=red, Galileo=green, BeiDou=amber). Bar height represents CNR.

| CNR | Colour | Interpretation |
|---|---|---|
| ≥ 40 dB-Hz | Green | Excellent |
| 35–40 dB-Hz | Yellow | Good |
| 25–35 dB-Hz | Orange | Marginal |
| < 25 dB-Hz | Red | Excluded by CNR gate |

Satellites used in the WLS solution render at full opacity; tracked-only satellites are dimmed.

### Scatter Plot (Scatter tab)

Each dot is one epoch's position in East-North metres relative to the reference. The circle is the CEP50 estimate (radius containing 50% of solutions, ≈ 0.59 × RMS 2D). Colour matches the fix type badge.

### DOP / 2D Time Series (DOP/2D tab)

Scrolling time series showing:
- **HDOP** (blue) — horizontal dilution of precision
- **VDOP** (orange) — vertical dilution of precision
- **2D Error** (green) — live horizontal distance to reference in metres (only when a reference is available)

HDOP < 2.0 is excellent; > 4.0 indicates poor satellite geometry.

**Window duration selector** — drop-down in the top-right of the panel:

| Option | Duration |
|--------|----------|
| 5 min  | 300 s    |
| 10 min | 600 s    |
| 30 min | 1 800 s  |
| 1 hr   | 3 600 s  |
| 12 hr  | 43 200 s |

Up to 12 hours of history is retained regardless of the selected window. X-axis labels show time-ago (e.g. `-30m`, `-1h`) with **now** pinned to the right edge.

---

## 8. Session Recording

JSONL (JSON Lines) session logs capture every pipeline event for offline replay and analysis.

### Starting a session

In the left dock **Session Logging** panel:
1. The default path is `~/gnss_session_YYYYMMDD_HHMMSS.jsonl`
2. Edit the path if desired
3. Click **Start Logging** (or press the toolbar **Record** button)
4. The indicator shows `● REC filename.jsonl` while active

### Stopping

Click **Stop** in the toolbar or **Stop Logging** in the panel.

### Log format

Each line is a JSON object with a `type` field:

```json
{"type": "rtcm_frame",        "source": "Rover",  "msg_type": 1074, "length": 212}
{"type": "epoch_observations", "tow_s": 519531.0,  "week": 2311, "num_obs": 34, "num_sats": 12}
{"type": "position_solution",  "timestamp_gps": 519531.0, "fix_type": "SPS",
 "latitude": 53.2378481, "longitude": -113.5324592, "altitude": 748.3,
 "num_satellites": 23, "sigma_horizontal": 17.9}
{"type": "reference_error",    "enu_e_m": -0.23, "enu_n_m": 4.45, "error_2d_m": 4.47}
{"type": "nmea_reference",     "source": "GGA", "latitude": 53.2378411,
 "longitude": -113.5325171, "altitude": 747.37}
{"type": "base_station_arp",   "x": -1527580.0, "y": -3507755.0, "z": 5087245.0}
```

---

## 9. Console Log Format

Each epoch solution prints one line:

```
[HH:MM:SS.mmm] TOW <seconds>: <FIX_TYPE> pos=(<lat>,<lon>,<alt>m) ref=(<lat>,<lon>,<alt>m) 2D=<metres> m
```

**Example:**
```
[18:54:43.023] TOW 3300.0: SPS pos=(53.2378777,-113.5324895,735.98m) ref=(53.2378411,-113.5325171,747.37m) 2D=4.465 m
```

- `TOW` — GPS Time of Week in seconds (rolls over at 604800 = Sunday midnight)
- `pos` — the software-computed WLS position
- `ref` — the receiver's own NMEA GGA/RMC fix (only when receiver is outputting NMEA)
- `2D` — horizontal distance between `pos` and `ref` in metres

When no reference is available, the `ref=` and `2D=` fields are omitted.

---

## 10. Reference Accuracy Comparison

### How the reference is obtained

If the receiver is configured to output NMEA sentences alongside RTCM3, the app separates them (RTCM3 frames start with `0xD3`, NMEA lines start with `$`), parses GGA for latitude/longitude/altitude and RMC for speed/course, and maintains a merged reference position that updates at 1 Hz. The **Reference Error** panel also displays the fix mode reported by the receiver (SPS, DGNSS, etc.) so you can assess the quality of the reference itself.

### How 2D error is computed

1. Convert the WLS ECEF position `(X, Y, Z)` to LLA using the Bowring iteration
2. Convert the reference LLA to ECEF using WGS-84
3. Compute the ECEF difference vector `(ΔX, ΔY, ΔZ)`
4. Rotate to the local East-North-Up frame at the reference point
5. `2D error = √(East² + North²)`

### What the numbers mean

The NMEA GGA/RMC position is the receiver's internal PVT solution, not a surveyed ground truth. A 2D error of ~5 m means the software WLS agrees with the receiver's own positioning engine to within 5 m. For absolute accuracy validation you would need a surveyed reference marker.

---

## 11. Design Internals

### Pipeline flow (one epoch)

```
Serial bytes arrive on read thread
    → RTCM3FrameExtractor buffers and validates frames (CRC24Q)
    → GNSSPipeline._on_rover_frame dispatches by message type
        MSM4/MSM7 (1074/1077, 1084/1087, 1094/1097, 1124/1127) → MSMDecoder.decode()
            → EpochObservations accumulated in _pending_obs[gps_tow]
              (BeiDou epoch time shifted +14 s for accumulation key;
               GLONASS epoch time converted from Moscow TOD to GPS TOW)
            → When a strictly newer TOW arrives: _process_pending_epoch(old_tow)
                → on_satellites callback → UI satellite table / signal bars
                → SPSEngine.process_epoch(epoch)
                    → _select_observations: GLONASS exclusion, BDS GEO exclusion,
                                            CNR gate, Hatch filter
                    → _prepare_observations (coarse pass, no receiver pos)
                    → _solve_wls (coarse solution, all sats, no elevation mask)
                    → _prepare_observations (refined: tropo/iono with coarse pos)
                    → _solve_wls (refined solution, elevation mask applied)
                    → _reject_outliers_and_resolve
                    → Earth-surface validity guard
                    → Assemble PositionSolution
                → on_solution callback → UI position / quality / scatter / map
        Ephemeris (1019/1020/1042/1046) → EphemerisStore
        ARP (1005/1006) → BaseStationInfo
        NMEA on same port → NMEASentenceParser → reference_position
```

### Observation selection and Hatch filter

For each satellite, `_select_observations` tries to form a dual-frequency iono-free combination:

```
PR_IF = (f1² · P1 − f2² · P2) / (f1² − f2²)
CP_IF = (f1² · λ1·φ1 − f2² · λ2·φ2) / (f1² − f2²)
```

If only one frequency is available, the primary frequency pseudorange is used with Klobuchar correction.

The **Hatch filter** smooths the code pseudorange using carrier-phase increments:

```
ρ_smooth(k) = (1/N)·ρ_code(k) + (1 − 1/N)·(ρ_smooth(k−1) + Δφ(k))
```

`N` grows to 100 over the first 100 epochs (~100 s at 1 Hz). The filter resets on:
- Carrier lock-time decrease > 30 ms (cycle slip / signal interruption)
- Half-cycle ambiguity flag set in the MSM message
- Satellite disappears and reappears

GLONASS is excluded from Hatch smoothing because its FDMA frequencies vary per satellite.

### MSM4 vs MSM7 decoding

Both MSM4 and MSM7 formats are supported. MSM7 differs from MSM4 in:
- An extra 14-bit **rough phase range rate** field per satellite in the satellite-data section
- Wider signal-data fields: fine PR 20 bits (vs 15), fine phase 24 bits (vs 22), lock 10 bits (vs 4), CNR 10 bits at 0.0625 dB-Hz resolution (vs 6 bits at 1 dB-Hz)
- An additional 15-bit **fine phase range rate** field per signal cell

The decoder detects the format from the message type and reads all fields with correct widths and scale factors.

### Satellite orbit computation

**GPS / Galileo / BeiDou MEO/IGSO** — Standard Keplerian model (IS-GPS-200N §20.3.3.4.3):
1. `t_k = t_transmit − t_oe` (time from ephemeris epoch)
2. Solve Kepler's equation iteratively for eccentric anomaly `E` (15 iterations, tolerance 1e-14 rad)
3. Compute true anomaly `ν`, argument of latitude `u`, radius `r`, inclination `i` with harmonic corrections
4. Rotate to ECEF via ascending node longitude `Ω`
5. Relativistic correction: `Δt_rel = F · e · √a · sin(E)`
6. Clock: `δt_sv = a_f0 + a_f1·(t−t_oc) + a_f2·(t−t_oc)² + Δt_rel [− TGD]`
   - TGD subtracted only for single-frequency users; omitted for iono-free combinations

**BeiDou GEO (PRN 1–5, 59–63)** — excluded. GEO satellites require an additional Y-axis rotation (BDS-SIS-ICD §5.2.4.15) not implemented in the standard Keplerian model.

**BeiDou time system** — BeiDou MSM4/7 epoch time is in BDT (BeiDou Time = GPS time − 14 s). The raw BDT timestamp is stored in each observation and used for satellite position computation (`t_transmit`). The pipeline epoch accumulator adds 14 s to the accumulation key only, so BeiDou observations merge with GPS/Galileo into the same epoch bucket.

**GLONASS** — 4th-order Runge-Kutta integration of equations of motion in PZ-90 ECEF frame:
- Central gravity + J2 zonal harmonic
- Luni-solar perturbations from the ephemeris message
- Step size 60 s; sub-step for final partial interval
- `dt` computed by converting GPS TOW to Moscow time-of-day before subtracting `eph.tb`
- Integration aborted if ephemeris age > 7200 s
- **Excluded from WLS** — output is in PZ-90; Helmert transform to WGS-84 not yet implemented

**Earth rotation (Sagnac) correction:**
```
[X_corr]   [ cos(ω·τ)  sin(ω·τ)  0] [X]
[Y_corr] = [-sin(ω·τ)  cos(ω·τ)  0] [Y]
[Z_corr]   [    0          0      1] [Z]
```
where `τ = pseudorange / c` is the signal transit time.

### Weighted Least Squares

Observation equation linearised around current estimate `x̂`:

```
δρ_i = H_i · δx + ε_i

H_i = [−e_i^T  |  c·I_constellation(i)]
```

where `e_i` is the unit line-of-sight vector from receiver to satellite, and the clock columns are identity for the constellation of satellite `i` and zero otherwise. One clock state per constellation present in the epoch.

Weights: `W_ii = sin²(elev_i) · clip((CNR_i − 20) / 35, 0.1, 1.0)`

The solution iterates until `|δx| < 1e-4 m` (max 20 iterations), then one pass of residual-based outlier rejection (reject if `|residual| > max(30 m, 4·σ_MAD)`) and re-solve.

Post-fit covariance: `Q = (H^T W H)⁻¹`, giving:
- `σ_H = √((Q_11 + Q_22) · σ²)` — horizontal sigma
- `σ_V = √(Q_33 · σ²)` — vertical sigma

DOP values are computed by rotating `Q[:3,:3]` to the local ENU frame.

### GLONASS time system

GLONASS MSM4/7 messages encode epoch time as **Moscow time** (UTC+3, seconds since start of week). GPS time = UTC + 18 leap seconds. Conversion:

```
GPS TOW = (GLONASS_Moscow_week_second − 10782) mod 604800
```

where `10782 = 10800 (3 h) − 18 (leap seconds)`.

For ephemeris age computation, `eph.tb` is stored in **Moscow time-of-day** (0–86400 s). The correct `dt` requires converting the GPS TOW transmission time to Moscow TOD first:

```
moscow_tod = (t_transmit + 10782) mod 86400
dt = moscow_tod − eph.tb   (wrapped to ±43200 s)
```

### Coordinate conversion (ECEF ↔ LLA)

**LLA → ECEF** (closed-form):
```
N = a / √(1 − e²·sin²φ)
X = (N + h)·cos(φ)·cos(λ)
Y = (N + h)·cos(φ)·sin(λ)
Z = (N·(1−e²) + h)·sin(φ)
```

**ECEF → LLA** (Bowring iterative):
```
p = √(X² + Y²)
φ_0 = atan2(Z, p·(1 − e²))
iterate: N_k = a / √(1 − e²·sin²(φ_k))
         φ_{k+1} = atan2(Z + e²·N_k·sin(φ_k), p)
until |φ_{k+1} − φ_k| < 1e-12 rad
```

The iteration uses **`e²`** (first eccentricity squared = 0.00669438). Using `e′² = e²/(1−e²)` instead shifts latitude by +138 m at 53°N.

---

## 12. Troubleshooting

### No ports listed after Refresh

- Check USB cable and connector
- On macOS: install CP210x (`SiLabs`) or CH340 USB-serial driver
- On Linux: add user to `dialout` group — `sudo usermod -aG dialout $USER` then log out/in
- On Windows: Device Manager → Ports (COM & LPT) should show the device

### Connected but no ephemeris appearing

- Receivers typically take up to 30 s to broadcast the first full ephemeris set after cold start
- Confirm your receiver is configured to output the required RTCM messages (see Section 5)
- Check the **Ephemeris** panel counts; if GPS is 0 after 60 s the receiver may not have sky visibility

### Only GPS satellites tracked / Galileo or BeiDou missing

- This is expected on first connection — non-GPS ephemeris arrives later
- Confirm the receiver is configured to output 1094 (Galileo) and 1124 (BeiDou) MSM messages and the corresponding ephemeris (1042/1046)
- GLONASS will appear as **Tracked** once 1020 messages arrive, but will show **Tracked** (not **Used**) in the satellite table — this is expected

### GLONASS satellites all show "Tracked" instead of "Used"

This is intentional. GLONASS broadcasts satellite positions in the **PZ-90** geodetic frame. The WLS requires all positions in **WGS-84**. The ~0.5 m frame offset plus any residual alignment errors corrupt the WLS solution. GLONASS will be included once the 7-parameter Helmert transform is implemented.

### Satellites tracked but "No Fix"

- Need at least 4 usable satellites after the CNR gate (< 25 dB-Hz excluded) and elevation mask (< 10° excluded)
- Wait for more constellations' ephemeris to arrive — Galileo and BeiDou significantly improve geometry
- Check for `Keplerian computation error` in the console (bad ephemeris data)

### 2D error reported as very large (> 1 km)

- If the reference comes from GGA while the receiver itself is still acquiring, the reference position is unreliable — wait for the receiver's fix quality to stabilise
- Check the **Ref Mode** field in the Reference Error panel; a reference in NO FIX or low-quality mode gives meaningless error numbers

### High CPU usage

- The UI updates at 1 Hz; RTCM parsing runs on a background thread
- `--log-level DEBUG` generates verbose console output which can slow the UI; use `INFO` for normal operation

---

*For bug reports and feature requests, open an issue on GitHub.*
