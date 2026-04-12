"""
Satellite Position and Clock Computation from Broadcast Ephemeris.

Implements:
    - Keplerian orbit computation (GPS, Galileo, BeiDou)
    - Runge-Kutta 4th order integration (GLONASS)
    - Relativistic clock correction
    - Earth rotation correction during signal transit

References:
    - IS-GPS-200N, Section 20.3.3.4.3
    - Galileo OS SIS ICD, Section 5.1.1
    - BDS-SIS-ICD, Section 5.2.4.10
    - GLONASS ICD, Section 4.3.1
"""

import numpy as np
import logging
from typing import Optional

from ..core.constants import (
    C, PI, MU_EARTH, MU_EARTH_GLO, OMEGA_EARTH, OMEGA_EARTH_GLO,
    PZ90_A, PZ90_J2, SECONDS_PER_WEEK,
)
from ..core.data_types import (
    GPSEphemeris, GalileoEphemeris, GLONASSEphemeris, BeidouEphemeris,
    SatelliteState, Constellation,
)

logger = logging.getLogger(__name__)

# Relativistic correction constant F = -2*sqrt(mu)/c^2
F_REL = -2.0 * np.sqrt(MU_EARTH) / C**2


def compute_gps_satellite(eph: GPSEphemeris, t_transmit: float,
                           apply_tgd: bool = True) -> Optional[SatelliteState]:
    """Compute GPS satellite position and clock from broadcast ephemeris.

    Args:
        eph: GPS ephemeris data
        t_transmit: Transmission time (GPS TOW) [s]
        apply_tgd: Apply group delay correction (True for L1-only, False for iono-free)

    Returns:
        SatelliteState with position, velocity, and clock
    """
    return _compute_keplerian(eph, t_transmit, MU_EARTH, OMEGA_EARTH,
                               Constellation.GPS, eph.tgd, apply_tgd)


def compute_galileo_satellite(eph: GalileoEphemeris, t_transmit: float,
                               apply_tgd: bool = True) -> Optional[SatelliteState]:
    """Compute Galileo satellite position and clock.

    Args:
        apply_tgd: Apply BGD correction (True for E1-only, False for iono-free)
    """
    return _compute_keplerian(eph, t_transmit, MU_EARTH, OMEGA_EARTH,
                               Constellation.GALILEO, eph.bgd_e5a_e1, apply_tgd)


def compute_beidou_satellite(eph: BeidouEphemeris, t_transmit: float,
                              apply_tgd: bool = True) -> Optional[SatelliteState]:
    """Compute BeiDou MEO/IGSO satellite position and clock.

    NOTE: BeiDou GEO satellites (PRN 1-5, 59-63) require a different rotation
    formula (BDS-SIS-ICD §5.2.4.15). They must be excluded before calling this.

    Args:
        apply_tgd: Apply TGD1 correction (True for B1C-only, False for iono-free)
    """
    return _compute_keplerian(eph, t_transmit, MU_EARTH, OMEGA_EARTH,
                               Constellation.BEIDOU, eph.tgd1, apply_tgd)


def _compute_keplerian(eph, t_transmit: float, mu: float, omega_e: float,
                        constellation: Constellation, tgd: float,
                        apply_tgd: bool = True) -> Optional[SatelliteState]:
    """Generic Keplerian orbit computation.

    This implements the standard broadcast ephemeris orbit model
    used by GPS, Galileo, and BeiDou (MEO/IGSO).

    Args:
        eph: Ephemeris dataclass (GPS, Galileo, or BeiDou)
        t_transmit: Signal transmission time [s] (TOW)
        mu: Gravitational parameter [m^3/s^2]
        omega_e: Earth rotation rate [rad/s]
        constellation: Constellation enum
        tgd: Group delay correction [s]
    """
    try:
        # Time from ephemeris reference epoch
        tk = t_transmit - eph.toe
        if tk > SECONDS_PER_WEEK / 2:
            tk -= SECONDS_PER_WEEK
        elif tk < -SECONDS_PER_WEEK / 2:
            tk += SECONDS_PER_WEEK

        # Semi-major axis
        a = eph.sqrt_a ** 2

        # Computed mean motion [rad/s]
        n0 = np.sqrt(mu / a**3)
        n = n0 + eph.delta_n

        # Mean anomaly
        M = eph.m0 + n * tk

        # Eccentric anomaly (Kepler's equation, iterative)
        E = M
        for _ in range(15):
            E_new = M + eph.e * np.sin(E)
            if abs(E_new - E) < 1e-14:
                break
            E = E_new
        E = E_new

        sin_E = np.sin(E)
        cos_E = np.cos(E)

        # True anomaly
        nu = np.arctan2(np.sqrt(1 - eph.e**2) * sin_E, cos_E - eph.e)

        # Argument of latitude
        phi = nu + eph.omega

        sin_2phi = np.sin(2 * phi)
        cos_2phi = np.cos(2 * phi)

        # Second harmonic perturbations
        delta_u = eph.cus * sin_2phi + eph.cuc * cos_2phi
        delta_r = eph.crs * sin_2phi + eph.crc * cos_2phi
        delta_i = eph.cis * sin_2phi + eph.cic * cos_2phi

        # Corrected argument of latitude, radius, inclination
        u = phi + delta_u
        r = a * (1 - eph.e * cos_E) + delta_r
        i = eph.i0 + delta_i + eph.idot * tk

        # Positions in orbital plane
        x_orb = r * np.cos(u)
        y_orb = r * np.sin(u)

        # Corrected longitude of ascending node
        omega = eph.omega0 + (eph.omega_dot - omega_e) * tk - omega_e * eph.toe

        sin_omega = np.sin(omega)
        cos_omega = np.cos(omega)
        sin_i = np.sin(i)
        cos_i = np.cos(i)

        # ECEF coordinates
        x = x_orb * cos_omega - y_orb * cos_i * sin_omega
        y = x_orb * sin_omega + y_orb * cos_i * cos_omega
        z = y_orb * sin_i

        # --- Satellite velocity (for Doppler) ---
        E_dot = n / (1 - eph.e * cos_E)
        nu_dot = E_dot * np.sqrt(1 - eph.e**2) / (1 - eph.e * cos_E)
        phi_dot = nu_dot

        u_dot = phi_dot + 2 * phi_dot * (eph.cus * cos_2phi - eph.cuc * sin_2phi)
        r_dot = a * eph.e * sin_E * E_dot + \
                2 * phi_dot * (eph.crs * cos_2phi - eph.crc * sin_2phi)
        i_dot = eph.idot + 2 * phi_dot * (eph.cis * cos_2phi - eph.cic * sin_2phi)

        x_orb_dot = r_dot * np.cos(u) - r * u_dot * np.sin(u)
        y_orb_dot = r_dot * np.sin(u) + r * u_dot * np.cos(u)

        omega_dot = eph.omega_dot - omega_e

        vx = (x_orb_dot * cos_omega - y_orb_dot * cos_i * sin_omega +
              y_orb * sin_i * sin_omega * i_dot -
              (x_orb * sin_omega + y_orb * cos_i * cos_omega) * omega_dot)
        vy = (x_orb_dot * sin_omega + y_orb_dot * cos_i * cos_omega -
              y_orb * sin_i * cos_omega * i_dot +
              (x_orb * cos_omega - y_orb * cos_i * sin_omega) * omega_dot)
        vz = y_orb_dot * sin_i + y_orb * cos_i * i_dot

        # --- Clock correction ---
        dt_sv = t_transmit - eph.toc
        if dt_sv > SECONDS_PER_WEEK / 2:
            dt_sv -= SECONDS_PER_WEEK
        elif dt_sv < -SECONDS_PER_WEEK / 2:
            dt_sv += SECONDS_PER_WEEK

        # Relativistic correction
        delta_t_rel = F_REL * eph.e * eph.sqrt_a * sin_E

        # SV clock bias.  TGD is only subtracted for single-frequency users.
        # For iono-free dual-freq combinations the bias is frequency-independent
        # and must NOT be applied (it is already eliminated by the combination).
        clock_bias = eph.af0 + eph.af1 * dt_sv + eph.af2 * dt_sv**2 + delta_t_rel
        if apply_tgd:
            clock_bias -= tgd
        clock_drift = eph.af1 + 2 * eph.af2 * dt_sv

        return SatelliteState(
            sat_id=f"{['G','R','E','C'][constellation]}{eph.svn:02d}",
            constellation=constellation,
            x=x, y=y, z=z,
            vx=vx, vy=vy, vz=vz,
            clock_bias=clock_bias,
            clock_drift=clock_drift,
            relativistic_correction=delta_t_rel,
        )

    except Exception as e:
        logger.error(f"Keplerian computation error: {e}")
        return None


def compute_glonass_satellite(eph: GLONASSEphemeris,
                               t_transmit: float) -> Optional[SatelliteState]:
    """Compute GLONASS satellite position using Runge-Kutta 4th order integration.

    GLONASS ephemeris provides position, velocity, and luni-solar accelerations
    at a reference time. We integrate the equations of motion to the desired time.

    The equations of motion include:
        - Central gravity (with J2 perturbation)
        - Luni-solar perturbations (from ephemeris)

    Args:
        eph: GLONASS ephemeris
        t_transmit: Transmission time [s] (approximate GPS TOW)

    Returns:
        SatelliteState
    """
    try:
        # eph.tb is in Moscow-time seconds-of-day (0..86400).
        # t_transmit is GPS TOW (0..604800).
        # Convert t_transmit to Moscow seconds-of-day before computing dt.
        #   Moscow TOD = (GPS_TOW + 10782) mod 86400
        #   (10782 = 10800 Moscow offset − 18 GPS-UTC leap seconds)
        moscow_tod = (t_transmit + 10782.0) % 86400.0
        dt = moscow_tod - eph.tb
        if dt > 43200.0:
            dt -= 86400.0
        elif dt < -43200.0:
            dt += 86400.0

        if abs(dt) > 3600:
            logger.warning(f"GLONASS R{eph.svn:02d} ephemeris age: {dt:.0f}s")
            if abs(dt) > 7200:
                return None

        # Initial state vector [x, y, z, vx, vy, vz] in PZ-90
        state = np.array([eph.x, eph.y, eph.z, eph.vx, eph.vy, eph.vz])

        # Luni-solar accelerations (constant over integration)
        acc_ls = np.array([eph.ax, eph.ay, eph.az])

        # Integration step (60 seconds, sign depends on direction)
        h = 60.0 if dt > 0 else -60.0
        t_remaining = dt
        t_current = 0.0

        while abs(t_remaining) > 1e-9:
            if abs(t_remaining) < abs(h):
                h = t_remaining

            # RK4 step
            k1 = _glonass_derivatives(state, acc_ls)
            k2 = _glonass_derivatives(state + 0.5 * h * k1, acc_ls)
            k3 = _glonass_derivatives(state + 0.5 * h * k2, acc_ls)
            k4 = _glonass_derivatives(state + h * k3, acc_ls)

            state = state + (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
            t_remaining -= h

        x, y, z, vx, vy, vz = state

        # Clock correction
        # GLONASS: tau_n has negative sign convention in ICD
        clock_bias = -eph.tau_n + eph.gamma_n * dt

        return SatelliteState(
            sat_id=f"R{eph.svn:02d}",
            constellation=Constellation.GLONASS,
            x=x, y=y, z=z,
            vx=vx, vy=vy, vz=vz,
            clock_bias=clock_bias,
            clock_drift=eph.gamma_n,
        )

    except Exception as e:
        logger.error(f"GLONASS orbit computation error: {e}")
        return None


def _glonass_derivatives(state: np.ndarray,
                          acc_luni_solar: np.ndarray) -> np.ndarray:
    """Compute derivatives for GLONASS equations of motion.

    Includes central gravity with J2 perturbation in PZ-90 frame.

    Args:
        state: [x, y, z, vx, vy, vz] in meters and m/s
        acc_luni_solar: Luni-solar accelerations [ax, ay, az] in m/s^2

    Returns:
        Derivative of state vector [vx, vy, vz, ax, ay, az]
    """
    x, y, z, vx, vy, vz = state
    r = np.sqrt(x**2 + y**2 + z**2)
    r2 = r**2
    r5 = r**5

    # Common terms
    mu_r3 = MU_EARTH_GLO / (r**3)
    j2_coeff = 1.5 * PZ90_J2 * MU_EARTH_GLO * PZ90_A**2 / r5

    z2_r2 = (z / r)**2

    # Accelerations (central body + J2 + luni-solar)
    ax = (-mu_r3 * x + j2_coeff * x * (1 - 5 * z2_r2) +
          OMEGA_EARTH_GLO**2 * x + 2 * OMEGA_EARTH_GLO * vy +
          acc_luni_solar[0])
    ay = (-mu_r3 * y + j2_coeff * y * (1 - 5 * z2_r2) +
          OMEGA_EARTH_GLO**2 * y - 2 * OMEGA_EARTH_GLO * vx +
          acc_luni_solar[1])
    az = (-mu_r3 * z + j2_coeff * z * (3 - 5 * z2_r2) +
          acc_luni_solar[2])

    return np.array([vx, vy, vz, ax, ay, az])


def earth_rotation_correction(sat_pos: np.ndarray,
                                transit_time: float) -> np.ndarray:
    """Apply Earth rotation correction during signal transit.

    The satellite position must be rotated by the angle the Earth
    rotates during signal propagation time.

    Args:
        sat_pos: Satellite ECEF position [m]
        transit_time: Signal transit time [s]

    Returns:
        Corrected satellite ECEF position [m]
    """
    angle = OMEGA_EARTH * transit_time

    cos_a = np.cos(angle)
    sin_a = np.sin(angle)

    x_corr = cos_a * sat_pos[0] + sin_a * sat_pos[1]
    y_corr = -sin_a * sat_pos[0] + cos_a * sat_pos[1]
    z_corr = sat_pos[2]

    return np.array([x_corr, y_corr, z_corr])