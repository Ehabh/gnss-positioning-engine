"""
Core data structures for GNSS observations, ephemeris, and solutions.

Uses dataclasses for clean, typed representations of all GNSS data flowing
through the positioning pipeline.
"""

import time
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Tuple
from .constants import Constellation, SignalType, FixType


# ==============================================================================
# Observation Data (from RTCM MSM)
# ==============================================================================

@dataclass
class RawObservation:
    """Single raw observation from one satellite on one signal."""
    constellation: Constellation
    svn: int                              # Satellite vehicle number (PRN)
    signal: SignalType
    timestamp_gps: float                  # GPS time of week [s]
    week: int                             # GPS week number
    pseudorange: float                    # Pseudorange [m]
    carrier_phase: float                  # Carrier phase [cycles]
    doppler: float                        # Doppler shift [Hz]
    cnr: float                            # Carrier-to-noise ratio [dB-Hz]
    lock_time: float                      # Carrier lock time indicator [s]
    half_cycle_ambiguity: bool = False    # Half-cycle ambiguity flag
    pseudorange_valid: bool = True
    carrier_phase_valid: bool = True

    @property
    def sat_id(self) -> str:
        """Unique satellite identifier, e.g., 'G01', 'E05', 'R24', 'C11'."""
        prefix = {
            Constellation.GPS: 'G',
            Constellation.GLONASS: 'R',
            Constellation.GALILEO: 'E',
            Constellation.BEIDOU: 'C',
        }
        return f"{prefix[self.constellation]}{self.svn:02d}"


@dataclass
class EpochObservations:
    """All observations from a single epoch (time step)."""
    timestamp_gps: float                  # GPS time of week [s]
    week: int                             # GPS week number
    observations: List[RawObservation] = field(default_factory=list)
    receiver_clock_offset: float = 0.0    # Receiver clock offset estimate [s]

    @property
    def satellite_count(self) -> int:
        """Number of unique satellites observed."""
        return len(set(obs.sat_id for obs in self.observations))

    def get_by_constellation(self, const: Constellation) -> List[RawObservation]:
        """Get observations filtered by constellation."""
        return [o for o in self.observations if o.constellation == const]

    def get_by_satellite(self, sat_id: str) -> List[RawObservation]:
        """Get all signal observations for a specific satellite."""
        return [o for o in self.observations if o.sat_id == sat_id]


# ==============================================================================
# Ephemeris Data
# ==============================================================================

@dataclass
class GPSEphemeris:
    """GPS broadcast ephemeris (from RTCM 1019)."""
    svn: int                   # PRN number
    week: int                  # GPS week
    toc: float                 # Clock reference time [s]
    toe: float                 # Ephemeris reference time [s]
    # Clock correction
    af0: float = 0.0          # Clock bias [s]
    af1: float = 0.0          # Clock drift [s/s]
    af2: float = 0.0          # Clock drift rate [s/s^2]
    tgd: float = 0.0          # Group delay [s]
    # Keplerian orbital parameters
    sqrt_a: float = 0.0       # Square root of semi-major axis [m^0.5]
    e: float = 0.0            # Eccentricity
    i0: float = 0.0           # Inclination at reference [rad]
    omega0: float = 0.0       # Longitude of ascending node at reference [rad]
    omega: float = 0.0        # Argument of perigee [rad]
    m0: float = 0.0           # Mean anomaly at reference [rad]
    # Perturbation corrections
    delta_n: float = 0.0      # Mean motion correction [rad/s]
    omega_dot: float = 0.0    # Rate of right ascension [rad/s]
    idot: float = 0.0         # Rate of inclination [rad/s]
    cuc: float = 0.0          # Cos correction to argument of latitude [rad]
    cus: float = 0.0          # Sin correction to argument of latitude [rad]
    crc: float = 0.0          # Cos correction to orbital radius [m]
    crs: float = 0.0          # Sin correction to orbital radius [m]
    cic: float = 0.0          # Cos correction to inclination [rad]
    cis: float = 0.0          # Sin correction to inclination [rad]
    # Health and accuracy
    ura: int = 0              # User Range Accuracy index
    health: int = 0           # SV health
    iodc: int = 0             # Issue of data, clock
    iode: int = 0             # Issue of data, ephemeris
    fit_interval: int = 0     # Fit interval flag
    received_time: float = field(default_factory=time.time)

    @property
    def is_healthy(self) -> bool:
        return self.health == 0

    @property
    def sat_id(self) -> str:
        return f"G{self.svn:02d}"


@dataclass
class GalileoEphemeris:
    """Galileo broadcast ephemeris (from RTCM 1046 I/NAV or 1045 F/NAV)."""
    svn: int
    week: int                  # Galileo week
    toc: float
    toe: float
    af0: float = 0.0
    af1: float = 0.0
    af2: float = 0.0
    bgd_e5a_e1: float = 0.0   # E1/E5a broadcast group delay [s]
    bgd_e5b_e1: float = 0.0   # E1/E5b broadcast group delay [s]
    sqrt_a: float = 0.0
    e: float = 0.0
    i0: float = 0.0
    omega0: float = 0.0
    omega: float = 0.0
    m0: float = 0.0
    delta_n: float = 0.0
    omega_dot: float = 0.0
    idot: float = 0.0
    cuc: float = 0.0
    cus: float = 0.0
    crc: float = 0.0
    crs: float = 0.0
    cic: float = 0.0
    cis: float = 0.0
    sisa: float = 0.0         # Signal-in-space accuracy [m]
    health_e1b: int = 0
    health_e5a: int = 0
    data_source: int = 0       # I/NAV or F/NAV
    iodnav: int = 0            # Issue of data
    received_time: float = field(default_factory=time.time)

    @property
    def is_healthy(self) -> bool:
        return self.health_e1b == 0 and self.health_e5a == 0

    @property
    def sat_id(self) -> str:
        return f"E{self.svn:02d}"


@dataclass
class GLONASSEphemeris:
    """GLONASS broadcast ephemeris (from RTCM 1020).

    GLONASS uses position/velocity/acceleration instead of Keplerian elements.
    Integration of equations of motion is required (4th-order Runge-Kutta).
    """
    svn: int                   # Slot number
    freq_channel: int          # Frequency channel number (-7 to +6)
    tb: float                  # Reference time within the day [s] (Moscow time)
    # Position at tb [m] (PZ-90 ECEF)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    # Velocity at tb [m/s]
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    # Acceleration (luni-solar) [m/s^2]
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    # Clock
    tau_n: float = 0.0         # Clock bias [s] (note: sign convention is negative)
    gamma_n: float = 0.0       # Frequency bias
    # Status
    health: int = 0
    age: int = 0               # Age of ephemeris [days]
    nt: int = 0                # Calendar day number within 4-year period
    n4: int = 0                # 4-year period number
    received_time: float = field(default_factory=time.time)

    @property
    def is_healthy(self) -> bool:
        return self.health == 0

    @property
    def sat_id(self) -> str:
        return f"R{self.svn:02d}"


@dataclass
class BeidouEphemeris:
    """BeiDou broadcast ephemeris (from RTCM 1042)."""
    svn: int
    week: int                  # BeiDou week
    toc: float
    toe: float
    af0: float = 0.0
    af1: float = 0.0
    af2: float = 0.0
    tgd1: float = 0.0         # B1 group delay [s]
    tgd2: float = 0.0         # B2 group delay [s]
    sqrt_a: float = 0.0
    e: float = 0.0
    i0: float = 0.0
    omega0: float = 0.0
    omega: float = 0.0
    m0: float = 0.0
    delta_n: float = 0.0
    omega_dot: float = 0.0
    idot: float = 0.0
    cuc: float = 0.0
    cus: float = 0.0
    crc: float = 0.0
    crs: float = 0.0
    cic: float = 0.0
    cis: float = 0.0
    ura: float = 0.0
    health: int = 0
    aode: int = 0              # Age of data, ephemeris
    aodc: int = 0              # Age of data, clock
    received_time: float = field(default_factory=time.time)

    @property
    def is_healthy(self) -> bool:
        return self.health == 0

    @property
    def sat_id(self) -> str:
        return f"C{self.svn:02d}"


# ==============================================================================
# Base Station Data
# ==============================================================================

@dataclass
class BaseStationInfo:
    """Base station information from RTCM 1005/1006."""
    station_id: int
    x_ecef: float              # ECEF X [m]
    y_ecef: float              # ECEF Y [m]
    z_ecef: float              # ECEF Z [m]
    antenna_height: float = 0.0  # Only in 1006
    itrf_year: int = 0
    gps_indicator: bool = True
    glonass_indicator: bool = True
    galileo_indicator: bool = True
    beidou_indicator: bool = False

    @property
    def position_ecef(self) -> np.ndarray:
        return np.array([self.x_ecef, self.y_ecef, self.z_ecef])


# ==============================================================================
# Satellite Computed State
# ==============================================================================

@dataclass
class SatelliteState:
    """Computed satellite position, velocity, and clock at a given time."""
    sat_id: str
    constellation: Constellation
    # Position in ECEF [m]
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    # Velocity in ECEF [m/s]
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    # Clock
    clock_bias: float = 0.0       # [s]
    clock_drift: float = 0.0      # [s/s]
    # Geometry relative to receiver
    elevation: float = 0.0        # [deg]
    azimuth: float = 0.0          # [deg]
    geometric_range: float = 0.0  # [m]
    # Corrections applied
    iono_correction: float = 0.0  # [m]
    tropo_correction: float = 0.0 # [m]
    relativistic_correction: float = 0.0  # [s]

    @property
    def position_ecef(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @property
    def velocity_ecef(self) -> np.ndarray:
        return np.array([self.vx, self.vy, self.vz])


# ==============================================================================
# Position Solution
# ==============================================================================

@dataclass
class DOPValues:
    """Dilution of Precision values."""
    gdop: float = 99.9
    pdop: float = 99.9
    hdop: float = 99.9
    vdop: float = 99.9
    tdop: float = 99.9


@dataclass
class PositionSolution:
    """Complete position solution for one epoch."""
    timestamp_gps: float
    week: int
    fix_type: FixType = FixType.NO_FIX
    # Position in ECEF [m]
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    # Position in geodetic (WGS84)
    latitude: float = 0.0         # [deg]
    longitude: float = 0.0        # [deg]
    altitude: float = 0.0         # [m] above ellipsoid
    # Receiver clock
    clock_bias: float = 0.0       # [s]
    clock_drift: float = 0.0      # [s/s]
    # Inter-system biases (relative to GPS)
    isb_glonass: float = 0.0      # [s]
    isb_galileo: float = 0.0      # [s]
    isb_beidou: float = 0.0       # [s]
    # Quality indicators
    dop: DOPValues = field(default_factory=DOPValues)
    num_satellites: int = 0
    residuals: Optional[np.ndarray] = None
    covariance: Optional[np.ndarray] = None  # 3x3 or 4x4
    sigma_pos: float = 0.0        # 3D position sigma [m]
    sigma_horizontal: float = 0.0  # Horizontal sigma [m]
    sigma_vertical: float = 0.0    # Vertical sigma [m]
    # RTK specific
    ambiguities_fixed: bool = False
    num_ambiguities: int = 0
    ratio_test: float = 0.0       # LAMBDA ratio test value
    # Satellites used
    satellites_used: List[str] = field(default_factory=list)
    constellations_used: List[Constellation] = field(default_factory=list)
    # Epoch processing time
    processing_time_ms: float = 0.0

    @property
    def position_ecef(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @property
    def position_lla(self) -> Tuple[float, float, float]:
        return (self.latitude, self.longitude, self.altitude)

    @property
    def is_valid(self) -> bool:
        return self.fix_type != FixType.NO_FIX and self.num_satellites >= 4


# ==============================================================================
# Ephemeris Store
# ==============================================================================

class EphemerisStore:
    """Manages broadcast ephemeris for all constellations.

    Stores the latest ephemeris for each satellite and provides lookup
    by satellite ID with age checking.
    """

    def __init__(self, max_age_seconds: float = 7200.0):
        self.max_age = max_age_seconds
        self._gps: Dict[int, GPSEphemeris] = {}
        self._galileo: Dict[int, GalileoEphemeris] = {}
        self._glonass: Dict[int, GLONASSEphemeris] = {}
        self._beidou: Dict[int, BeidouEphemeris] = {}

    def update_gps(self, eph: GPSEphemeris):
        existing = self._gps.get(eph.svn)
        if existing is None or eph.iode != existing.iode:
            self._gps[eph.svn] = eph

    def update_galileo(self, eph: GalileoEphemeris):
        existing = self._galileo.get(eph.svn)
        if existing is None or eph.iodnav != existing.iodnav:
            self._galileo[eph.svn] = eph

    def update_glonass(self, eph: GLONASSEphemeris):
        self._glonass[eph.svn] = eph

    def update_beidou(self, eph: BeidouEphemeris):
        self._beidou[eph.svn] = eph

    def get_gps(self, svn: int) -> Optional[GPSEphemeris]:
        eph = self._gps.get(svn)
        if eph and eph.is_healthy:
            return eph
        return None

    def get_galileo(self, svn: int) -> Optional[GalileoEphemeris]:
        eph = self._galileo.get(svn)
        if eph and eph.is_healthy:
            return eph
        return None

    def get_glonass(self, svn: int) -> Optional[GLONASSEphemeris]:
        eph = self._glonass.get(svn)
        if eph and eph.is_healthy:
            return eph
        return None

    def get_beidou(self, svn: int) -> Optional[BeidouEphemeris]:
        eph = self._beidou.get(svn)
        if eph and eph.is_healthy:
            return eph
        return None

    def get_ephemeris(self, constellation: Constellation, svn: int):
        """Generic ephemeris lookup by constellation."""
        lookup = {
            Constellation.GPS: self.get_gps,
            Constellation.GLONASS: self.get_glonass,
            Constellation.GALILEO: self.get_galileo,
            Constellation.BEIDOU: self.get_beidou,
        }
        return lookup[constellation](svn)

    @property
    def summary(self) -> Dict[str, int]:
        return {
            'GPS': len(self._gps),
            'GLONASS': len(self._glonass),
            'Galileo': len(self._galileo),
            'BeiDou': len(self._beidou),
        }