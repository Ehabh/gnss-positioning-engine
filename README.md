# GNSS Positioning Engine

A software-defined GNSS positioning engine that computes receiver position entirely in software from raw RTCM3 measurements. The Quectel LC29HBA hardware is used purely as a measurement front-end — all satellite orbit computation, atmospheric corrections, and position solutions run in this codebase.

> **Current accuracy:** ~2 m 2D (SPS, GPS-only, Hatch-smoothed) against the receiver's own NMEA solution.  
> Built as a learning platform and stepping stone toward full RTK at centimetre level.

---

## Why?

Most GNSS projects treat the receiver as a black box that outputs NMEA fixes. This project goes deeper: it disables the receiver's internal PVT engine, ingests only raw pseudorange and carrier phase via RTCM3, and solves the positioning problem from first principles — so you can see and modify every step.

---

## Supported Constellations & Signals

| Constellation | Signals    | Ephemeris (RTCM) | Observations (RTCM) | Notes |
|---|---|---|---|---|
| GPS     | L1 C/A, L5 | 1019      | 1074 (MSM4) | Dual-freq iono-free |
| Galileo | E1, E5a    | 1045/1046 | 1094 (MSM4) | Dual-freq iono-free |
| GLONASS | L1, L2     | 1020      | 1084 (MSM4) | FDMA, single-freq smoothed |
| BeiDou  | B1C, B2a   | 1042      | 1124 (MSM4) | Dual-freq; GEO excluded |

---

## Architecture

```
LC29HBA (Rover)                    LC29HBA (Base)  [optional]
    │ RTCM3 binary                     │ RTCM3 binary
    ▼                                  ▼
┌──────────────┐               ┌──────────────┐
│ SerialHandler│               │ SerialHandler│
│  (threaded)  │               │  (threaded)  │
└──────┬───────┘               └──────┬───────┘
       │ raw bytes                    │
       ▼                             ▼
┌──────────────────────────────────────────────┐
│           RTCM3FrameExtractor                │
│   Frame sync · length · CRC24Q validate      │
└──────────────────┬───────────────────────────┘
                   │ (msg_type, payload)
                   ▼
┌──────────────────────────────────────────────┐
│              Message Router                  │
│  1074/1084/1094/1124 → MSM4Decoder          │
│  1019/1020/1042/1045/1046 → EphemerisStore  │
│  1005/1006 → BaseStationInfo (ARP)          │
└──────────────────┬───────────────────────────┘
                   │ EpochObservations
                   ▼
┌──────────────────────────────────────────────┐
│           Positioning Engine                 │
│                                              │
│  ┌─────────────────────────────────────┐    │
│  │  Observation Preparation            │    │
│  │  • BDS GEO exclusion (PRN 1-5,59-63)│    │
│  │  • Hatch carrier-smoothed PR (100 s)│    │
│  │  • Iono-free dual-freq combination  │    │
│  │  • Klobuchar (single-freq fallback) │    │
│  │  • Saastamoinen + Niell tropo       │    │
│  └──────────────────┬──────────────────┘    │
│                     │                        │
│  ┌──────────────────▼──────────────────┐    │
│  │  Satellite State Computation        │    │
│  │  • Keplerian orbit (GPS/GAL/BDS)    │    │
│  │  • RK4 integration (GLONASS PZ-90)  │    │
│  │  • Relativistic clock correction    │    │
│  │  • Earth rotation (Sagnac) fix      │    │
│  └──────────────────┬──────────────────┘    │
│                     │                        │
│  ┌──────────────────▼──────────────────┐    │
│  │  Weighted Least Squares             │    │
│  │  • Elevation + CNR weighting        │    │
│  │  • Per-constellation clock offsets  │    │
│  │  • Residual outlier rejection       │    │
│  │  • DOP computation (ENU frame)      │    │
│  └─────────────────────────────────────┘    │
│                                              │
│  ┌─────────┐  ┌──────────┐  ┌───────────┐  │
│  │   SPS   │  │  DGNSS   │  │    RTK    │  │
│  │ (live)  │  │  [stub]  │  │  [stub]   │  │
│  └─────────┘  └──────────┘  └───────────┘  │
└──────────────────┬───────────────────────────┘
                   │ PositionSolution
                   ▼
┌──────────────────────────────────────────────┐
│              PyQt UI (u-center 2 style)      │
│  Toolbar   · Connect · Mode · Record         │
│  Left dock · Fix badge · Position · Quality  │
│            · Reference error · Ephemeris     │
│  Right dock · Satellite table (CNR/status)   │
│  Bottom    · Console log                     │
│  Tabs      · Map · Signal Bars · Scatter     │
│            · DOP/2D time series              │
└──────────────────────────────────────────────┘
```

---

## Project Structure

```
gnss-positioning-engine/
├── main.py                         # Entry point / CLI
├── launch_gnss_engine.command      # macOS double-click launcher
├── requirements.txt
├── pyproject.toml
├── gnss_positioning/
│   ├── core/
│   │   ├── constants.py            # WGS84, PZ-90, signal frequencies, RTCM IDs
│   │   ├── data_types.py           # Dataclasses: observations, ephemeris, solutions
│   │   ├── pipeline.py             # Central coordinator
│   │   ├── serial_handler.py       # Threaded serial I/O + LC29HBA config
│   │   └── session_logger.py       # JSONL event recording
│   ├── parsers/
│   │   ├── bit_reader.py           # Bit-level RTCM3 field reader
│   │   ├── rtcm3_frame.py          # Frame sync, CRC24Q validation
│   │   ├── rtcm3_msm.py            # MSM4 decoder (pseudorange, phase, CNR)
│   │   ├── rtcm3_ephemeris.py      # Broadcast ephemeris for all constellations
│   │   └── nmea.py                 # GGA/RMC reference parser
│   ├── engines/
│   │   ├── orbit.py                # Satellite positions: Keplerian + GLONASS RK4
│   │   ├── sps.py                  # WLS positioning engine (implemented)
│   │   ├── dgnss.py                # Pseudorange correction engine [stub]
│   │   └── rtk.py                  # Double-difference + LAMBDA [stub]
│   ├── corrections/
│   │   └── atmosphere.py           # Saastamoinen, Niell, Klobuchar, iono-free
│   ├── utils/
│   │   ├── coordinates.py          # ECEF ↔ LLA (Bowring), ECEF ↔ ENU
│   │   └── time_utils.py           # GPS/BeiDou/GLONASS time conversions
│   └── ui/
│       ├── main_window.py          # Full PyQt application (~1900 lines)
│       └── assets/                 # SVG/PNG icons, app .icns
└── docs/
    └── user_guide.md               # Full usage & design reference
```

---

## Quick Start

### Requirements

- Python 3.10+
- Quectel LC29HBA (one for SPS/DGNSS, two for RTK)
- USB–UART adapter or direct USB

### Install

```bash
git clone https://github.com/yourusername/gnss-positioning-engine.git
cd gnss-positioning-engine
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt   # or: pip install PySide6 numpy pyserial
```

### Run

```bash
python main.py
```

Or on macOS, double-click `launch_gnss_engine.command` in Finder.

### First fix (step by step)

1. Click **Refresh Ports** — auto-detects all serial devices
2. Select the rover COM port and baud rate (default 115200), click **Connect**
3. Click **RTCM+NMEA** — configures the receiver to output MSM4 observations, broadcast ephemeris, and GGA/RMC as an accuracy reference
4. Watch the **Ephemeris** panel count up — GPS satellites appear first (~10–30 s)
5. First SPS fix typically arrives within 30–60 s of connecting
6. The **Fix Badge** turns blue (SPS) and position/DOP panels populate
7. Reference error (2D/3D vs the receiver's own NMEA solution) updates each epoch

---

## Accuracy

Tested with Quectel LC29HBA at 53°N (Alberta, Canada):

| Mode | 2D Error (vs NMEA) | Satellites | Notes |
|---|---|---|---|
| SPS GPS-only | ~2 m | 8–10 | After ~100 s Hatch filter warm-up |
| SPS multi-GNSS | expected 1–2 m | 15–22 | Once GLONASS/Galileo/BeiDou ephemeris collected |
| DGNSS | sub-metre | — | Planned |
| RTK Fixed | ~2 cm | — | Planned |

---

## Implementation Status

### Done
- RTCM3 frame extraction (CRC24Q), MSM4 decoding, broadcast ephemeris for all constellations
- Keplerian orbit computation (GPS, Galileo, BeiDou MEO/IGSO)
- GLONASS RK4 orbit integration (PZ-90 frame, J2 perturbation, luni-solar)
- Relativistic clock and Sagnac (Earth rotation) corrections
- Saastamoinen troposphere + Niell mapping function
- Klobuchar ionospheric model (single-frequency fallback)
- Iono-free dual-frequency combination (GPS L1/L5, Galileo E1/E5a, BeiDou B1C/B2a)
- Hatch carrier-smoothed pseudorange (100-epoch window, cycle-slip detection)
- Multi-constellation WLS with per-constellation clock offsets and outlier rejection
- GLONASS time conversion (Moscow TOD → GPS TOW) for proper epoch merging
- Correct Bowring ECEF→LLA conversion (e² vs e′² — was causing 138 m lat offset)
- Full PyQt UI: dockable panels, signal bars, scatter plot, map, DOP time series

### Planned
- **DGNSS** — pseudorange corrections from co-located base, range-rate temporal decorrelation
- **RTK Float/Fixed** — double-difference carrier phase, EKF float solution, LAMBDA ambiguity resolution, ratio test
- **Cycle slip detection** — geometry-free and Melbourne-Wübbena combinations
- **RINEX logging** — observation file output for post-processing
- **NTRIP client** — receive RTK corrections from internet reference networks

---

## References

- IS-GPS-200N — GPS Interface Specification
- Galileo OS SIS ICD Issue 2.1
- GLONASS ICD Edition 5.1
- BDS-SIS-ICD-B1I-3.0
- RTCM Standard 10403.3
- Misra & Enge — *GPS: Signals, Measurements, and Performance*
- Kaplan & Hegarty — *Understanding GPS/GNSS: Principles and Applications*
- Teunissen & Montenbruck — *Springer Handbook of Global Navigation Satellite Systems*
- Hatch (1982) — *The synergism of GPS code and carrier measurements*

---

## License

MIT
