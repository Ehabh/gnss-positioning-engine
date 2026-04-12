# GNSS Positioning Engine

A software-defined GNSS positioning engine that computes receiver position entirely in software from raw RTCM3 measurements. Any GNSS receiver that outputs RTCM3 MSM4 (or MSM7) observations and broadcast ephemeris can serve as the measurement front-end — all satellite orbit computation, atmospheric corrections, and position solutions run in this codebase.

> **Current status:** SPS (Standard Positioning Service) is fully implemented and operational.  
> **Current accuracy:** ~5 m 2D (GPS + Galileo + BeiDou, Hatch-smoothed) against the receiver's own NMEA solution.  
> DGNSS and RTK are planned but not yet implemented.

---

## Why?

Most GNSS projects treat the receiver as a black box that outputs NMEA fixes. This project goes deeper: it ingests only raw pseudorange and carrier phase via RTCM3, and solves the positioning problem from first principles — so you can see and modify every step.

---

## Supported Constellations & Signals

| Constellation | Signals    | Ephemeris (RTCM) | Observations (RTCM)       | WLS Status | Notes |
|---|---|---|---|---|---|
| GPS     | L1 C/A, L5 | 1019      | 1074 (MSM4) / 1077 (MSM7) | **Active** | Dual-freq iono-free |
| Galileo | E1, E5a    | 1045/1046 | 1094 (MSM4) / 1097 (MSM7) | **Active** | Dual-freq iono-free |
| BeiDou  | B1C, B2a   | 1042      | 1124 (MSM4) / 1127 (MSM7) | **Active** | Dual-freq; GEO excluded |
| GLONASS | L1, L2     | 1020      | 1084 (MSM4) / 1087 (MSM7) | Tracked only | PZ-90→WGS-84 transform pending |

GLONASS satellites are decoded, displayed in the satellite table, and ephemeris is stored — but they are not included in the WLS solution until the PZ-90 to WGS-84 Helmert transform is implemented.

---

## Architecture

```
GNSS Receiver (Rover)              GNSS Receiver (Base)  [optional]
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
│  MSM4 1074/1084/1094/1124 → MSMDecoder      │
│  MSM7 1077/1087/1097/1127 → MSMDecoder      │
│  1019/1020/1042/1045/1046 → EphemerisStore  │
│  1005/1006 → BaseStationInfo (ARP)          │
└──────────────────┬───────────────────────────┘
                   │ EpochObservations
                   │ (epoch-boundary detected, constellations merged)
                   ▼
┌──────────────────────────────────────────────┐
│           Positioning Engine                 │
│                                              │
│  ┌─────────────────────────────────────┐    │
│  │  Observation Preparation            │    │
│  │  • GLONASS excluded (PZ-90 pending) │    │
│  │  • BDS GEO exclusion (PRN 1-5,59-63)│    │
│  │  • CNR gate: < 25 dB-Hz excluded    │    │
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
│  │  • Earth-surface validity guard     │    │
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
│   │   ├── pipeline.py             # Central coordinator + epoch accumulator
│   │   ├── serial_handler.py       # Threaded serial I/O
│   │   └── session_logger.py       # JSONL event recording
│   ├── parsers/
│   │   ├── bit_reader.py           # Bit-level RTCM3 field reader
│   │   ├── rtcm3_frame.py          # Frame sync, CRC24Q validation
│   │   ├── rtcm3_msm.py            # MSM4 + MSM7 decoder (pseudorange, phase, CNR)
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
│       ├── main_window.py          # Full PyQt application
│       └── assets/                 # SVG/PNG icons, app .icns
└── docs/
    └── user_guide.md               # Full usage & design reference
```

---

## Quick Start

### Requirements

- Python 3.10+
- A GNSS receiver capable of outputting RTCM3 MSM4 (or MSM7) observations and broadcast ephemeris for all desired constellations
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

1. Configure your receiver to output MSM4 (or MSM7) observations and broadcast ephemeris for all constellations, plus GGA/RMC NMEA sentences for the reference comparison display. The engine reads whatever the receiver streams — no in-app configuration is required.
2. Click **Refresh Ports** — auto-detects all serial devices
3. Select the rover COM port and baud rate (default 115200), click **Connect**
4. Watch the **Ephemeris** panel count up — GPS satellites appear first (~10–30 s)
5. First SPS fix typically arrives within 30–60 s of connecting
6. The **Fix Badge** turns blue (SPS) and position/DOP panels populate
7. Reference error (2D/3D vs the receiver's own NMEA solution) updates each epoch

---

## Accuracy

Tested at 53°N (Alberta, Canada):

| Mode | 2D Error (vs NMEA) | Satellites | Constellations |
|---|---|---|---|
| SPS | ~5 m | 15–25 | GPS + Galileo + BeiDou |
| SPS (Hatch warm, good geometry) | ~2–3 m | 20+ | GPS + Galileo + BeiDou |
| DGNSS | — | — | Not yet implemented |
| RTK Fixed | — | — | Not yet implemented |

Error is measured against the receiver's own NMEA solution (which is itself not ground truth). Accuracy improves after the 100-epoch Hatch filter warm-up period (~100 s at 1 Hz).

---

## Implementation Status

### Done
- RTCM3 frame extraction (CRC24Q), MSM4 **and MSM7** decoding, broadcast ephemeris for all constellations
- Multi-constellation epoch accumulation: constellations arrive in separate MSM frames; BeiDou 14 s time offset handled; all merged per GPS TOW before entering WLS
- Keplerian orbit computation (GPS, Galileo, BeiDou MEO/IGSO)
- GLONASS RK4 orbit integration (PZ-90 frame, J2 perturbation, luni-solar) — orbit computed but GLONASS excluded from WLS pending PZ-90→WGS-84 transform
- Relativistic clock and Sagnac (Earth rotation) corrections
- Saastamoinen troposphere + Niell mapping function
- Klobuchar ionospheric model (single-frequency fallback)
- Iono-free dual-frequency combination (GPS L1/L5, Galileo E1/E5a, BeiDou B1C/B2a)
- Hatch carrier-smoothed pseudorange (100-epoch window, cycle-slip detection)
- Multi-constellation WLS with per-constellation clock offsets and outlier rejection
- Earth-surface validity guard (rejects and clears warm-start if |r| outside 5871–6871 km)
- Correct Bowring ECEF→LLA conversion (e² vs e′² — was causing 138 m latitude error)
- Full PyQt UI: dockable panels, signal bars, scatter plot, live map, DOP time series

### Planned
- **GLONASS in WLS** — 7-parameter Helmert transform (PZ-90.11 → WGS-84) to correctly merge GLONASS positions with other constellations
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
