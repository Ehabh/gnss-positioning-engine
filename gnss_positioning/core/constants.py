"""
GNSS Physical Constants, System Parameters, and RTCM3 Message Definitions.

References:
    - IS-GPS-200N (GPS Interface Specification)
    - Galileo OS SIS ICD Issue 2.1
    - GLONASS ICD Edition 5.1
    - BDS-SIS-ICD-B1I-3.0
    - RTCM Standard 10403.3
"""

import numpy as np
from enum import IntEnum, auto

# ==============================================================================
# Physical Constants
# ==============================================================================

C = 299792458.0                     # Speed of light [m/s]
MU_EARTH = 3.986005e14              # Earth gravitational constant (WGS84) [m^3/s^2]
MU_EARTH_GLO = 3.9860044e14        # Earth gravitational constant (PZ-90) [m^3/s^2]
OMEGA_EARTH = 7.2921151467e-5      # Earth rotation rate (WGS84) [rad/s]
OMEGA_EARTH_GLO = 7.292115e-5      # Earth rotation rate (PZ-90) [rad/s]
PI = 3.1415926535898                # Pi (GPS ICD value, more precise than math.pi for consistency)

# WGS84 Ellipsoid
WGS84_A = 6378137.0                # Semi-major axis [m]
WGS84_F = 1.0 / 298.257223563     # Flattening
WGS84_B = WGS84_A * (1 - WGS84_F) # Semi-minor axis [m]
WGS84_E2 = 2 * WGS84_F - WGS84_F ** 2  # First eccentricity squared

# PZ-90 Ellipsoid (GLONASS)
PZ90_A = 6378136.0                 # Semi-major axis [m]
PZ90_F = 1.0 / 298.257839303      # Flattening
PZ90_J2 = 1.08262575e-3           # Second zonal harmonic

# ==============================================================================
# Carrier Frequencies [Hz]
# ==============================================================================

# GPS
GPS_L1_FREQ = 1575.42e6            # L1 C/A
GPS_L2_FREQ = 1227.60e6            # L2
GPS_L5_FREQ = 1176.45e6            # L5

# Galileo
GAL_E1_FREQ = 1575.42e6            # E1
GAL_E5A_FREQ = 1176.45e6           # E5a
GAL_E5B_FREQ = 1207.14e6           # E5b

# BeiDou
BDS_B1I_FREQ = 1561.098e6          # B1I
BDS_B1C_FREQ = 1575.42e6           # B1C
BDS_B2A_FREQ = 1176.45e6           # B2a
BDS_B3I_FREQ = 1268.52e6           # B3I

# GLONASS (FDMA - frequency depends on channel k)
GLO_L1_BASE = 1602.0e6             # L1 base frequency
GLO_L1_STEP = 562.5e3              # L1 frequency step per channel
GLO_L2_BASE = 1246.0e6             # L2 base frequency
GLO_L2_STEP = 437.5e3              # L2 frequency step per channel
GLO_L3_FREQ = 1202.025e6           # L3 CDMA

# Wavelengths [m]
GPS_L1_WAVELENGTH = C / GPS_L1_FREQ
GPS_L5_WAVELENGTH = C / GPS_L5_FREQ
GAL_E1_WAVELENGTH = C / GAL_E1_FREQ
GAL_E5A_WAVELENGTH = C / GAL_E5A_FREQ
BDS_B1C_WAVELENGTH = C / BDS_B1C_FREQ
BDS_B2A_WAVELENGTH = C / BDS_B2A_FREQ


def glonass_l1_freq(k: int) -> float:
    """GLONASS L1 frequency for channel number k (-7 to +6)."""
    return GLO_L1_BASE + k * GLO_L1_STEP


def glonass_l2_freq(k: int) -> float:
    """GLONASS L2 frequency for channel number k (-7 to +6)."""
    return GLO_L2_BASE + k * GLO_L2_STEP


# ==============================================================================
# Constellation Enums
# ==============================================================================

class Constellation(IntEnum):
    GPS = 0
    GLONASS = 1
    GALILEO = 2
    BEIDOU = 3


class SignalType(IntEnum):
    GPS_L1CA = auto()
    GPS_L5 = auto()
    GAL_E1 = auto()
    GAL_E5A = auto()
    GLO_L1 = auto()
    GLO_L2 = auto()
    BDS_B1C = auto()
    BDS_B2A = auto()


class FixType(IntEnum):
    NO_FIX = 0
    SPS = 1
    DGNSS = 2
    RTK_FLOAT = 3
    RTK_FIXED = 4


# Signal -> Frequency mapping
SIGNAL_FREQUENCY = {
    SignalType.GPS_L1CA: GPS_L1_FREQ,
    SignalType.GPS_L5: GPS_L5_FREQ,
    SignalType.GAL_E1: GAL_E1_FREQ,
    SignalType.GAL_E5A: GAL_E5A_FREQ,
    SignalType.GLO_L1: GLO_L1_BASE,  # Needs channel correction
    SignalType.GLO_L2: GLO_L2_BASE,  # Needs channel correction
    SignalType.BDS_B1C: BDS_B1C_FREQ,
    SignalType.BDS_B2A: BDS_B2A_FREQ,
}

SIGNAL_WAVELENGTH = {
    sig: C / freq for sig, freq in SIGNAL_FREQUENCY.items()
}

# ==============================================================================
# RTCM3 Message Types
# ==============================================================================

# Station/Antenna
RTCM_STATION_ARP = 1005              # Stationary RTK reference station ARP
RTCM_STATION_ARP_HEIGHT = 1006       # ARP + antenna height

# GPS Ephemeris
RTCM_GPS_EPHEMERIS = 1019

# GLONASS Ephemeris
RTCM_GLO_EPHEMERIS = 1020

# Galileo Ephemeris (F/NAV and I/NAV)
RTCM_GAL_FNAV_EPHEMERIS = 1045
RTCM_GAL_INAV_EPHEMERIS = 1046

# BeiDou Ephemeris
RTCM_BDS_EPHEMERIS = 1042

# MSM4 Messages (pseudorange + carrier phase + Doppler + CNR)
RTCM_GPS_MSM4 = 1074
RTCM_GLO_MSM4 = 1084
RTCM_GAL_MSM4 = 1094
RTCM_BDS_MSM4 = 1124

# MSM7 Messages (high-resolution, full set)
RTCM_GPS_MSM7 = 1077
RTCM_GLO_MSM7 = 1087
RTCM_GAL_MSM7 = 1097
RTCM_BDS_MSM7 = 1127

# Map RTCM MSM message type to constellation
MSM_CONSTELLATION_MAP = {
    1071: Constellation.GPS, 1072: Constellation.GPS,
    1073: Constellation.GPS, 1074: Constellation.GPS,
    1075: Constellation.GPS, 1076: Constellation.GPS,
    1077: Constellation.GPS,
    1081: Constellation.GLONASS, 1082: Constellation.GLONASS,
    1083: Constellation.GLONASS, 1084: Constellation.GLONASS,
    1085: Constellation.GLONASS, 1086: Constellation.GLONASS,
    1087: Constellation.GLONASS,
    1091: Constellation.GALILEO, 1092: Constellation.GALILEO,
    1093: Constellation.GALILEO, 1094: Constellation.GALILEO,
    1095: Constellation.GALILEO, 1096: Constellation.GALILEO,
    1097: Constellation.GALILEO,
    1121: Constellation.BEIDOU, 1122: Constellation.BEIDOU,
    1123: Constellation.BEIDOU, 1124: Constellation.BEIDOU,
    1125: Constellation.BEIDOU, 1126: Constellation.BEIDOU,
    1127: Constellation.BEIDOU,
}

# ==============================================================================
# GPS Time Constants
# ==============================================================================

GPS_EPOCH_UNIX = 315964800          # GPS epoch (Jan 6, 1980) in Unix time
SECONDS_PER_WEEK = 604800           # Seconds in a GPS week
LEAP_SECONDS = 18                   # Current GPS-UTC leap seconds (as of 2024)

# Galileo uses GST which has same epoch as GPS but different week numbering
# BeiDou epoch: Jan 1, 2006 00:00:00 UTC
BDS_EPOCH_UNIX = 1136073600         # BeiDou epoch in Unix time
BDS_GPS_OFFSET = BDS_EPOCH_UNIX - GPS_EPOCH_UNIX  # Offset in seconds

# ==============================================================================
# Positioning Defaults
# ==============================================================================

ELEVATION_MASK_DEG = 10.0           # Default elevation mask [degrees]
MAX_ITERATIONS = 20                 # Max iterations for least squares
CONVERGENCE_THRESHOLD = 1e-4        # Position convergence threshold [m]
INITIAL_POS_ECEF = np.array([0.0, 0.0, 0.0])  # Initial position guess (will be updated)

# DOP thresholds
GDOP_THRESHOLD = 30.0              # Reject solutions with GDOP > this
PDOP_THRESHOLD = 25.0              # Reject solutions with PDOP > this

# Pseudorange validity
PR_MIN = 15e6                       # Minimum valid pseudorange [m] (~50ms)
PR_MAX = 90e6                       # Maximum valid pseudorange [m] (~300ms)
