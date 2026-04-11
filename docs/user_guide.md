# GNSS Positioning Engine — User Guide

**Version:** 1.0.0  
**Hardware:** Quectel LC29HBA  
**Platform:** macOS / Linux / Windows (Python 3.10+)

---

## Table of Contents

1. [Installation](#1-installation)
2. [Launching the App](#2-launching-the-app)
3. [UI Overview](#3-ui-overview)
4. [Connecting the Receiver](#4-connecting-the-receiver)
5. [Receiver Configuration Modes](#5-receiver-configuration-modes)
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
- One or two **Quectel LC29HBA** receivers connected via USB or UART adapter
- For DGNSS / RTK: a second LC29HBA at a precisely known location as a base station

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

The interface is modelled on u-blox u-center 2 with a dark theme, dockable panels, and a central tab area.

```
┌──────────────────────────────────────────────────────────────────────┐
│  TOOLBAR                                                              │
│  Rover: [port▾] [baud▾] [Connect] [RTCM Only] [RTCM+NMEA]          │
│  Base:  [port▾] [baud▾] [Connect Base]                               │
│  Mode: [SPS] [DGNSS] [RTK]   [Record]   [Refresh Ports]             │
├───────────────────────┬──────────────────────────┬────────────────────┤
│  LEFT DOCK            │  CENTRAL TABS            │  RIGHT DOCK        │
│                       │                          │                    │
│  ┌─────────────────┐  │  Map | Signal | Scatter  │  Satellite Table  │
│  │   FIX BADGE     │  │        | DOP/σ           │  PRN · Sys · CNR  │
│  │   (SPS / DGNSS  │  │                          │  Used · Lock(s)   │
│  │    RTK FLOAT /  │  │  [live map / chart]      │                    │
│  │    RTK FIXED /  │  │                          │                    │
│  │     NO FIX)     │  │                          │                    │
│  └─────────────────┘  │                          │                    │
│  Position             │                          │                    │
│  Quality (DOP/σ)      │                          │                    │
│  Reference Error      │                          │                    │
│  Ephemeris counts     │                          │                    │
│  Session Logging      │                          │                    │
│  Map / Scatter cfg    │                          │                    │
├───────────────────────┴──────────────────────────┴────────────────────┤
│  BOTTOM DOCK — Console log (timestamped, scrollable)                  │
└──────────────────────────────────────────────────────────────────────┘
```

### Toolbar

| Control | Description |
|---|---|
| **Rover port / baud** | Serial port and baud rate for the rover receiver |
| **Connect** | Opens the serial connection; button changes to **Disconnect** with a red icon |
| **RTCM Only** | Configures receiver for MSM4 + ephemeris output only (no NMEA) |
| **RTCM+NMEA** | MSM4 + ephemeris + GGA/RMC for automatic reference comparison |
| **Base port / baud** | Serial port for the optional base station receiver |
| **Connect Base** | Opens base station connection |
| **SPS / DGNSS / RTK** | Switches active positioning mode; active button is highlighted |
| **Record** | Starts JSONL session logging; button shows **Stop** while active |
| **Refresh Ports** | Re-scans all serial ports |

### Left Dock

| Panel | Contents |
|---|---|
| **Fix Badge** | Large coloured badge showing current fix type, satellite count, and GPS TOW |
| **Position** | Latitude, longitude, altitude (ellipsoidal) |
| **Quality** | HDOP, VDOP, PDOP, horizontal sigma (σH), vertical sigma (σV) |
| **Reference Error** | Source (GGA/RMC), reference lat/lon/alt, live 2D error, live 3D error, RMS 2D, RMS 3D |
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
| **Status** | **Used** (in WLS solution) or **Tracked** (observed but excluded) |
| **Lock (s)** | Continuous carrier lock time in seconds |

### Central Tabs

| Tab | Description |
|---|---|
| **Map** | Live rover track on OpenStreetMap (or Google Maps); reference point shown if available |
| **Signal** | Vertical CNR bar chart, constellation-grouped, colour-coded by signal strength |
| **Scatter** | ENU scatter plot of all rover positions relative to the reference, with CEP50 circle |
| **DOP / σ** | Scrolling 5-minute time series of HDOP, VDOP, and σH |

### Bottom Dock — Console

Time-stamped log of every epoch solution, connection events, configuration changes, and errors. Use the **Clear** button to reset.

---

## 4. Connecting the Receiver

1. Plug the LC29HBA into USB. On macOS it appears as `/dev/cu.usbmodemXXXX`; on Linux as `/dev/ttyUSBX` or `/dev/ttyACMX`; on Windows as `COMX`.
2. Click **Refresh Ports** — the dropdown lists all detected ports with their descriptions.
3. Select the port. Leave baud at **115200** (LC29HBA default).
4. Click **Connect**. The button turns red with a disconnect icon. The console prints `Rover connected: /dev/cu.usbmodem... @ 115200`.
5. If no ports appear, check USB cable and driver. On macOS you may need `CP210x` or `CH340` USB-serial drivers.

---

## 5. Receiver Configuration Modes

After connecting, choose how the receiver should output data. The app sends Quectel **PAIR434** configuration commands over the same serial port.

### RTCM Only

```
[RTCM Only] button
```

Disables all NMEA sentences (GGA, GLL, GSA, GSV, RMC, VTG, ZDA) and enables:
- **1074** GPS MSM4 observations
- **1084** GLONASS MSM4 observations
- **1094** Galileo MSM4 observations
- **1124** BeiDou MSM4 observations
- **1019** GPS broadcast ephemeris
- **1020** GLONASS broadcast ephemeris
- **1042** BeiDou broadcast ephemeris
- **1046** Galileo I/NAV broadcast ephemeris

Use this when you only care about the computed position and do not need the receiver's own fix for comparison.

### RTCM + NMEA (Recommended for accuracy evaluation)

```
[RTCM+NMEA] button
```

Same as RTCM Only, plus enables:
- **GGA** at 1 Hz — provides the receiver's own position fix (used as the reference truth)
- **RMC** at 1 Hz — provides speed and course

The app automatically extracts GGA/RMC from the mixed stream, stores the reference position, and computes 2D/3D error each epoch. This is the recommended mode for benchmarking.

> **Note:** Configuration commands follow Quectel firmware version 3.x syntax. If commands fail (shown in the console), verify your firmware version and check `serial_handler.py`.

---

## 6. Positioning Modes

Select with the **SPS / DGNSS / RTK** buttons in the toolbar.

### SPS — Standard Positioning Service

Computes position from rover observations alone. No base station required.

**What it does:**
1. Selects the best pseudorange per satellite (dual-frequency iono-free combination if both signals available; Hatch-smoothed single-frequency otherwise)
2. Computes satellite positions from broadcast ephemeris
3. Applies tropospheric correction (Saastamoinen zenith delay + Niell mapping function)
4. Applies ionospheric correction (Klobuchar model for single-frequency; eliminated by iono-free combination for dual-frequency)
5. Solves Weighted Least Squares iteratively; weights are elevation and CNR based
6. Estimates one receiver clock offset per constellation (inter-system biases)
7. Performs residual-based outlier rejection
8. Computes DOP values in the ENU frame

**Typical accuracy:** 1–3 m (2D) against the receiver's own solution, depending on satellite geometry and how many constellations have broadcast ephemeris available.

**Minimum satellites required:** 4 (with GPS only); multi-constellation improves both accuracy and availability.

### DGNSS — Differential GNSS (stub)

Requires a base station (second LC29HBA at a known position). Will apply pseudorange corrections derived from the base station's known position vs its measured ranges. Expected accuracy: sub-metre.

**Status:** Architecture is wired; implementation pending.

### RTK — Real-Time Kinematic (stub)

Requires a base station. Will form double-difference carrier phase observations and resolve integer ambiguities using LAMBDA, targeting 2 cm accuracy.

**Status:** Interface is wired; full implementation pending.

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

### Signal Bars (Signal tab)

Satellites are grouped by constellation (GPS=blue, GLONASS=red, Galileo=green, BeiDou=amber). Bar height represents CNR. Bar colour:

| CNR | Colour | Interpretation |
|---|---|---|
| ≥ 40 dB-Hz | Green | Excellent — ideal for precise positioning |
| 35–40 dB-Hz | Yellow | Good |
| 25–35 dB-Hz | Orange | Marginal — may be excluded by elevation mask |
| < 25 dB-Hz | Red | Poor — likely excluded |

Used satellites (in the WLS solution) render at full opacity; tracked-only satellites are dimmed grey.

### Scatter Plot (Scatter tab)

Each dot is one epoch's position in East-North metres relative to the reference. The circle is the CEP50 estimate (radius containing 50% of solutions, ≈ 0.59 × RMS 2D). Colour matches the fix type badge.

### DOP / σ Time Series (DOP/σ tab)

Scrolling 5-minute window showing:
- **HDOP** (blue) — horizontal dilution of precision
- **VDOP** (orange) — vertical dilution of precision
- **σH** (green dashed) — horizontal position sigma from the WLS covariance

HDOP < 2.0 is excellent; > 4.0 indicates poor satellite geometry.

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
 "num_satellites": 10, "sigma_horizontal": 0.82}
{"type": "reference_error",    "enu_e_m": -0.23, "enu_n_m": 1.85, "error_2d_m": 1.86}
{"type": "nmea_reference",     "source": "GGA", "latitude": 53.2378301,
 "longitude": -113.5324592, "altitude": 746.14}
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
[22:51:19.014] TOW 535897.0: SPS pos=(53.2378481,-113.5324592,748.31m) ref=(53.2378301,-113.5324592,746.14m) 2D=2.01 m
```

- `TOW` — GPS Time of Week in seconds (rolls over at 604800 = Sunday midnight)
- `pos` — the software-computed position from WLS
- `ref` — the receiver's own NMEA GGA/RMC fix (if RTCM+NMEA mode was used)
- `2D` — horizontal distance between `pos` and `ref` in metres

When no reference is available, the `ref=` and `2D=` fields are omitted.

---

## 10. Reference Accuracy Comparison

### How the reference is obtained

When **RTCM+NMEA** mode is configured, the receiver outputs GGA and RMC sentences on the same serial port alongside the RTCM3 binary stream. The app separates them (RTCM3 frames start with `0xD3`, NMEA lines start with `$`), parses GGA for latitude/longitude/altitude and RMC for speed/course, and maintains a merged reference position that updates at 1 Hz.

### How 2D error is computed

1. Convert the WLS ECEF position `(X, Y, Z)` to LLA using the corrected Bowring iteration
2. Convert the reference LLA to ECEF using WGS84
3. Compute the ECEF difference vector `(ΔX, ΔY, ΔZ)`
4. Rotate to the local East-North-Up frame at the reference point
5. `2D error = √(East² + North²)`

The conversion uses `e²` (first eccentricity squared = 0.00669438) in the Bowring iteration. Earlier versions incorrectly used `e′²` (second eccentricity squared = 0.00673950) which caused a **+138 m northward bias** in the displayed latitude at 53°N — the WLS ECEF was correct the whole time.

### What the numbers mean

The NMEA GGA/RMC position is itself the receiver's internal PVT solution, not ground truth. A 2D error of ~2 m means the software WLS agrees with the receiver's own positioning engine to within 2 m. For absolute accuracy you would need a surveyed reference marker.

---

## 11. Design Internals

### Pipeline flow (one epoch)

```
Serial bytes arrive on read thread
    → RTCM3FrameExtractor buffers and validates frames (CRC24Q)
    → GNSSPipeline._on_rover_frame dispatches by message type
        MSM4 (1074/1084/1094/1124) → MSM4Decoder.decode()
            → EpochObservations accumulated in _pending_obs[tow]
            → After 200 ms timeout: _process_pending_epoch(tow)
                → on_satellites callback → UI satellite table/signal bars
                → SPSEngine.process_epoch(epoch)
                    → _select_observations: Hatch filter, BDS GEO exclusion
                    → _prepare_observations (coarse pass, no receiver pos)
                    → _solve_wls (coarse solution)
                    → _prepare_observations (refined with tropo/iono)
                    → _solve_wls (refined solution)
                    → _reject_outliers_and_resolve
                    → Assemble PositionSolution
                → on_solution callback → UI position/quality/scatter/map
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

If only one frequency is available (GLONASS always, others when second signal is absent), the primary frequency pseudorange is used with Klobuchar correction.

The **Hatch filter** then smooths the code pseudorange using carrier-phase increments:

```
ρ_smooth(k) = (1/N)·ρ_code(k) + (1 − 1/N)·(ρ_smooth(k−1) + Δφ(k))
```

where `N` grows to 100 over the first 100 epochs (~100 s at 1 Hz). The filter resets on:
- Carrier lock-time decrease > 30 ms (cycle slip / signal interruption)
- Half-cycle ambiguity flag set in the MSM4 message
- Satellite disappears and reappears

GLONASS is excluded from Hatch smoothing because its FDMA frequencies vary per satellite, making carrier-phase increments from one epoch to the next non-trivially combinable across the signal change.

### Satellite orbit computation

**GPS / Galileo / BeiDou MEO/IGSO** — Standard Keplerian model (IS-GPS-200N §20.3.3.4.3):
1. Compute time from reference epoch `t_k = t − t_oe`
2. Solve Kepler's equation iteratively for eccentric anomaly `E` (15 iterations, tolerance 1e-14 rad)
3. Compute true anomaly `ν`, argument of latitude `u`, radius `r`, inclination `i` with harmonic corrections
4. Rotate to ECEF via ascending node longitude `Ω`
5. Apply relativistic correction: `Δt_rel = F · e · √a · sin(E)`
6. Clock: `δt_sv = a_f0 + a_f1·(t−t_oc) + a_f2·(t−t_oc)² + Δt_rel [− TGD]`
   - TGD subtracted only for single-frequency users; omitted for iono-free combinations

**BeiDou GEO (PRN 1–5, 59–63)** — excluded entirely. GEO satellites require an additional Y-axis rotation (BDS-SIS-ICD §5.2.4.15) not implemented in the standard Keplerian model. Including them corrupts the WLS solution.

**GLONASS** — 4th-order Runge-Kutta integration of equations of motion in PZ-90 ECEF frame:
- Central gravity + J2 zonal harmonic
- Luni-solar perturbations from the ephemeris message
- Step size 60 s; sub-step for final partial interval
- Integration aborted if ephemeris age > 7200 s

**Earth rotation (Sagnac) correction** — the satellite position is computed at signal transmission time in an inertial frame; during propagation the Earth rotates:
```
[X_corr]   [ cos(ω·τ)  sin(ω·τ)  0] [X]
[Y_corr] = [-sin(ω·τ)  cos(ω·τ)  0] [Y]
[Z_corr]   [    0          0      1] [Z]
```
where `τ = |r_sat − r_rx| / c` is the approximate signal transit time.

### Weighted Least Squares

The observation equation linearised around current estimate `x̂`:

```
δρ_i = H_i · δx + ε_i

H_i = [−e_i^T  |  c·I_constellation(i)]
```

where `e_i` is the unit line-of-sight vector from receiver to satellite, and the clock columns are identity for the constellation of satellite `i` and zero otherwise.

Weights are `W_ii = sin²(elev_i) · min(CNR_i / 35, 1)² · σ_code⁻²`, giving higher trust to high-elevation, high-CNR observations.

The solution iterates until `|δx| < 1e-4 m` (or max 10 iterations), then one pass of residual-based outlier rejection (reject if `|residual| > 3σ`) and re-solve.

The post-fit covariance is `Q = (H^T W H)⁻¹`, from which:
- `σ_H = √((Q_11 + Q_22) · σ²)` — horizontal sigma
- `σ_V = √(Q_33 · σ²)` — vertical sigma
- `σ² = Σresiduals² / (n − n_unknowns)` — unit-weight variance

DOP values are computed by rotating `(H^T H)⁻¹` to the local ENU frame.

### GLONASS time system

GLONASS MSM4 messages encode epoch time as Moscow time (UTC+3). GPS time = UTC + 18 leap seconds (as of 2017). The offset is:

```
GPS TOW = (GLONASS Moscow week-second − 10782) mod 604800
```

where 10782 = 10800 (3 hours) − 18 (leap seconds). This conversion ensures GLONASS observations merge with GPS/Galileo/BeiDou in the epoch accumulator, which groups observations by GPS TOW.

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

The iteration uses **`e²`** (first eccentricity squared = 0.00669438). Using `e′² = e²/(1−e²)` instead shifts the latitude by +138 m at 53°N.

---

## 12. Troubleshooting

### No ports listed after Refresh

- Check USB cable and connector
- On macOS: install CP210x (`SiLabs`) or CH340 USB-serial driver
- On Linux: add user to `dialout` group — `sudo usermod -aG dialout $USER` then log out/in
- On Windows: Device Manager → Ports (COM & LPT) should show the device

### Connected but no ephemeris appearing

- Confirm configuration was applied (console should show "Rover configured: RTCM...")
- The LC29HBA takes up to 30 s to broadcast the first full ephemeris set after a cold start
- Check **Ephemeris** panel counts — if GPS is 0 after 60 s, the receiver may not have sky visibility

### Only GPS satellites tracked/used

- This is expected on first connection; Galileo/BeiDou/GLONASS ephemeris arrives later
- GLONASS appears once RTCM 1020 messages arrive (needed for frequency channel numbers)
- If only GPS appears after several minutes, check that the receiver is configured with `PAIR434,1084,1` (GLONASS MSM4) — verify in the console

### Satellites tracked but "No Fix"

- Need at least 4 usable satellites in the WLS solution
- Elevation mask is 5°; satellites below this are excluded
- Check signal quality — CNR < 25 dB-Hz satellites are heavily down-weighted
- Check for `Keplerian computation error` in the console (indicates bad ephemeris data)

### Position jumps by ~138 m in latitude after an update

- This was a bug in `ecef_to_lla` (using `e′²` instead of `e²`). Fixed in v1.0.0.

### 2D error reported as very large (>1 km)

- If reference is from GGA while the receiver is still acquiring (low-accuracy NMEA fix), the reference itself may be wrong
- Wait for the receiver's own fix quality to stabilise (fix quality ≥ 1 in GGA)

### App title shows "Python" in macOS Dock

- This is normal without a macOS `.app` bundle
- Running from `launch_gnss_engine.command` applies the `GNSS Engine` display name via `NSBundle` (requires `pyobjc` — `pip install pyobjc-framework-Cocoa`)
- Without pyobjc, the Qt `setApplicationDisplayName` call still sets the menu bar name; the Dock may show "Python" until the first window appears

### High CPU usage

- The UI updates at 1 Hz; RTCM parsing runs on a background thread
- Enabling `--log-level DEBUG` generates verbose console output which can slow the UI; use `INFO` for normal operation

---

*For bug reports and feature requests, open an issue on GitHub.*
