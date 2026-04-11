"""
Atmospheric Correction Models.

Troposphere:
    - Saastamoinen model (zenith delay)
    - Niell mapping function (elevation-dependent)

Ionosphere:
    - Klobuchar model (single-frequency, broadcast parameters)
    - Iono-free linear combination (dual-frequency)

References:
    - Saastamoinen (1972), "Atmospheric correction for the troposphere"
    - Klobuchar (1987), "Ionospheric time-delay algorithm for single-frequency GPS users"
    - IS-GPS-200N, Section 20.3.3.5.2.5
"""

import numpy as np
import logging

from ..core.constants import C, GPS_L1_FREQ, GPS_L5_FREQ

logger = logging.getLogger(__name__)


# ==============================================================================
# Troposphere
# ==============================================================================

def saastamoinen_zenith_delay(lat_deg: float, alt_m: float,
                               pressure_hpa: float = None,
                               temperature_k: float = None,
                               humidity_pct: float = 50.0) -> tuple:
    """Saastamoinen model for zenith tropospheric delay.

    If pressure/temperature not provided, uses standard atmosphere model.

    Args:
        lat_deg: Geodetic latitude [degrees]
        alt_m: Altitude above ellipsoid [m]
        pressure_hpa: Atmospheric pressure [hPa] (optional)
        temperature_k: Temperature [K] (optional)
        humidity_pct: Relative humidity [%]

    Returns:
        (zhd, zwd): Zenith hydrostatic delay [m], zenith wet delay [m]
    """
    # Standard atmosphere if not provided
    if temperature_k is None:
        temperature_k = 288.15 - 0.0065 * alt_m  # ISA lapse rate

    if pressure_hpa is None:
        pressure_hpa = 1013.25 * (1 - 2.2557e-5 * alt_m) ** 5.2568

    # Water vapor partial pressure [hPa]
    # Approximate using relative humidity and temperature
    e_sat = 6.108 * np.exp(17.15 * (temperature_k - 273.15) /
                            (temperature_k - 38.25))
    e = (humidity_pct / 100.0) * e_sat

    lat_rad = np.radians(lat_deg)

    # Zenith hydrostatic delay [m]
    zhd = 0.0022768 * pressure_hpa / (1 - 0.00266 * np.cos(2 * lat_rad) -
                                        0.00028 * alt_m / 1000.0)

    # Zenith wet delay [m]
    zwd = 0.002277 * (1255.0 / temperature_k + 0.05) * e

    return zhd, zwd


def niell_mapping_function(elevation_deg: float, lat_deg: float,
                            doy: int = 180) -> tuple:
    """Niell mapping function for tropospheric delay.

    Maps zenith delay to slant delay as a function of elevation angle.

    Args:
        elevation_deg: Satellite elevation angle [degrees]
        lat_deg: Receiver latitude [degrees]
        doy: Day of year

    Returns:
        (mf_hydro, mf_wet): Hydrostatic and wet mapping factors
    """
    el_rad = np.radians(max(elevation_deg, 5.0))

    # Simplified mapping: 1/sin(elevation) with correction
    # This is a simplified version; full Niell uses latitude-dependent coefficients
    sin_el = np.sin(el_rad)

    # Hydrostatic mapping function (simplified Marini)
    a_h = 0.00124
    b_h = 0.00307
    c_h = 0.05018
    mf_h = (1 + a_h / (1 + b_h / (1 + c_h))) / \
            (sin_el + a_h / (sin_el + b_h / (sin_el + c_h)))

    # Wet mapping function
    a_w = 0.00058
    b_w = 0.00142
    c_w = 0.04318
    mf_w = (1 + a_w / (1 + b_w / (1 + c_w))) / \
            (sin_el + a_w / (sin_el + b_w / (sin_el + c_w)))

    return mf_h, mf_w


def troposphere_correction(elevation_deg: float, lat_deg: float,
                            alt_m: float) -> float:
    """Compute total tropospheric correction for a satellite.

    Args:
        elevation_deg: Satellite elevation [degrees]
        lat_deg: Receiver latitude [degrees]
        alt_m: Receiver altitude [m]

    Returns:
        Tropospheric delay [m] (always positive - to be subtracted from pseudorange)
    """
    if elevation_deg < 2.0:
        return 0.0

    zhd, zwd = saastamoinen_zenith_delay(lat_deg, alt_m)
    mf_h, mf_w = niell_mapping_function(elevation_deg, lat_deg)

    return zhd * mf_h + zwd * mf_w


# ==============================================================================
# Ionosphere
# ==============================================================================

# Default Klobuchar parameters (GPS almanac typical values)
DEFAULT_IONO_ALPHA = [0.1118e-07, -0.7451e-08, -0.5961e-07, 0.1192e-06]
DEFAULT_IONO_BETA = [0.1167e+06, -0.2294e+06, -0.1311e+06, 0.1049e+07]


def klobuchar_correction(lat_deg: float, lon_deg: float,
                          elevation_deg: float, azimuth_deg: float,
                          gps_tow: float,
                          alpha: list = None, beta: list = None) -> float:
    """Klobuchar ionospheric delay model (L1).

    Single-frequency ionospheric correction using broadcast parameters.
    Accuracy: ~50% RMS correction of actual ionospheric delay.

    Args:
        lat_deg: Receiver geodetic latitude [degrees]
        lon_deg: Receiver geodetic longitude [degrees]
        elevation_deg: Satellite elevation [degrees]
        azimuth_deg: Satellite azimuth [degrees]
        gps_tow: GPS time of week [s]
        alpha: Klobuchar alpha parameters [4]
        beta: Klobuchar beta parameters [4]

    Returns:
        L1 ionospheric delay [m]
    """
    if alpha is None:
        alpha = DEFAULT_IONO_ALPHA
    if beta is None:
        beta = DEFAULT_IONO_BETA

    # Semi-circles
    lat_u = lat_deg / 180.0
    lon_u = lon_deg / 180.0
    el = elevation_deg / 180.0
    az = np.radians(azimuth_deg)

    # Earth-centered angle
    psi = 0.0137 / (el + 0.11) - 0.022

    # Ionospheric pierce point (IPP) latitude
    lat_i = lat_u + psi * np.cos(az)
    if lat_i > 0.416:
        lat_i = 0.416
    elif lat_i < -0.416:
        lat_i = -0.416

    # IPP longitude
    lon_i = lon_u + psi * np.sin(az) / np.cos(lat_i * np.pi)

    # Geomagnetic latitude
    lat_m = lat_i + 0.064 * np.cos((lon_i - 1.617) * np.pi)

    # Local time
    t = 43200.0 * lon_i + gps_tow
    t = t % 86400
    if t < 0:
        t += 86400

    # Obliquity factor
    F = 1.0 + 16.0 * (0.53 - el)**3

    # Period and amplitude of ionospheric delay
    PER = sum(beta[n] * lat_m**n for n in range(4))
    if PER < 72000:
        PER = 72000

    AMP = sum(alpha[n] * lat_m**n for n in range(4))
    if AMP < 0:
        AMP = 0

    # Phase
    x = 2.0 * np.pi * (t - 50400.0) / PER

    # Ionospheric delay [s]
    if abs(x) < 1.57:
        iono_delay_s = F * (5e-9 + AMP * (1 - x**2/2 + x**4/24))
    else:
        iono_delay_s = F * 5e-9

    # Convert to meters (L1)
    return iono_delay_s * C


def ionofree_combination(pr_l1: float, pr_l5: float,
                          f1: float = GPS_L1_FREQ,
                          f2: float = GPS_L5_FREQ) -> float:
    """Ionosphere-free linear combination of dual-frequency pseudoranges.

    This eliminates first-order ionospheric delay.

    PR_IF = (f1^2 * PR1 - f2^2 * PR2) / (f1^2 - f2^2)

    Args:
        pr_l1: L1/E1 pseudorange [m]
        pr_l5: L5/E5a pseudorange [m]
        f1: First frequency [Hz]
        f2: Second frequency [Hz]

    Returns:
        Ionosphere-free pseudorange [m]
    """
    f1_sq = f1**2
    f2_sq = f2**2
    return (f1_sq * pr_l1 - f2_sq * pr_l5) / (f1_sq - f2_sq)


def ionofree_carrier_phase(cp_l1: float, cp_l5: float,
                             f1: float = GPS_L1_FREQ,
                             f2: float = GPS_L5_FREQ) -> float:
    """Ionosphere-free linear combination of dual-frequency carrier phases.

    Carrier phases should be in meters (cycles * wavelength).

    Args:
        cp_l1: L1 carrier phase [m]
        cp_l5: L5 carrier phase [m]
        f1: First frequency [Hz]
        f2: Second frequency [Hz]

    Returns:
        Ionosphere-free carrier phase [m]
    """
    f1_sq = f1**2
    f2_sq = f2**2
    return (f1_sq * cp_l1 - f2_sq * cp_l5) / (f1_sq - f2_sq)


def ionosphere_correction(lat_deg: float, lon_deg: float,
                            elevation_deg: float, azimuth_deg: float,
                            gps_tow: float,
                            signal_freq: float = GPS_L1_FREQ) -> float:
    """Compute ionospheric correction for a given signal frequency.

    Scales Klobuchar L1 correction to the target frequency.

    Args:
        lat_deg, lon_deg: Receiver position [deg]
        elevation_deg, azimuth_deg: Satellite geometry [deg]
        gps_tow: GPS time of week [s]
        signal_freq: Target signal frequency [Hz]

    Returns:
        Ionospheric delay at target frequency [m]
    """
    # Klobuchar gives L1 delay
    iono_l1 = klobuchar_correction(lat_deg, lon_deg, elevation_deg,
                                     azimuth_deg, gps_tow)

    # Scale to target frequency: delay ~ 1/f^2
    scale = (GPS_L1_FREQ / signal_freq)**2
    return iono_l1 * scale