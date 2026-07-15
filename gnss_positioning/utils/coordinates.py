"""
Coordinate transformations: ECEF <-> LLA <-> ENU.

All angles in radians internally unless specified otherwise.
"""

import numpy as np
from ..core.constants import WGS84_A, WGS84_B, WGS84_E2, WGS84_F

# [EG_C] PZ-90(.11) -> WGS-84 (EPSG:15843, "Coordinate Frame rotation" method)
# ---------------------------------------------------------------------------
PZ90_DX = 0.0                                      # [m]
PZ90_DY = 0.0                                      # [m]
PZ90_DZ = 1.5                                      # [m]
PZ90_RZ_ARCSEC = -0.076                            # [arc-seconds]
PZ90_RZ_RAD = np.radians(PZ90_RZ_ARCSEC / 3600.0)  # [rad]

def ecef_to_lla(x: float, y: float, z: float) -> tuple:
    """Convert ECEF coordinates to geodetic (WGS84).

    Uses Bowring's iterative method (converges in 2-3 iterations).

    Args:
        x, y, z: ECEF coordinates [m]

    Returns:
        (lat_deg, lon_deg, alt_m): Geodetic latitude [deg], longitude [deg],
                                    altitude above ellipsoid [m]
    """
    lon = np.arctan2(y, x)
    p = np.sqrt(x**2 + y**2)

    # Initial estimate using spherical approximation
    lat = np.arctan2(z, p * (1 - WGS84_E2))

    for _ in range(10):
        sin_lat = np.sin(lat)
        N = WGS84_A / np.sqrt(1 - WGS84_E2 * sin_lat**2)
        # Standard Bowring iteration: uses e² (first eccentricity squared).
        # Using e'² = e²/(1-e²) instead causes ~138 m latitude error at 53°N.
        lat_new = np.arctan2(z + WGS84_E2 * N * sin_lat, p)
        if abs(lat_new - lat) < 1e-12:
            break
        lat = lat_new

    sin_lat = np.sin(lat)
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * sin_lat**2)

    cos_lat = np.cos(lat)
    if abs(cos_lat) > 1e-10:
        alt = p / cos_lat - N
    else:
        alt = abs(z) - WGS84_B

    return np.degrees(lat), np.degrees(lon), alt


def lla_to_ecef(lat_deg: float, lon_deg: float, alt: float) -> np.ndarray:
    """Convert geodetic (WGS84) to ECEF.

    Args:
        lat_deg: Latitude [degrees]
        lon_deg: Longitude [degrees]
        alt: Altitude above ellipsoid [m]

    Returns:
        np.array([x, y, z]) in ECEF [m]
    """
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)

    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    N = WGS84_A / np.sqrt(1 - WGS84_E2 * sin_lat**2)

    x = (N + alt) * cos_lat * cos_lon
    y = (N + alt) * cos_lat * sin_lon
    z = (N * (1 - WGS84_E2) + alt) * sin_lat

    return np.array([x, y, z])


def ecef_to_enu(dx: float, dy: float, dz: float,
                ref_lat_deg: float, ref_lon_deg: float) -> np.ndarray:
    """Convert ECEF difference vector to ENU at a reference point.

    Args:
        dx, dy, dz: ECEF difference (target - reference) [m]
        ref_lat_deg, ref_lon_deg: Reference point geodetic coords [deg]

    Returns:
        np.array([east, north, up]) [m]
    """
    lat = np.radians(ref_lat_deg)
    lon = np.radians(ref_lon_deg)

    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    # Rotation matrix ECEF -> ENU
    R = np.array([
        [-sin_lon,            cos_lon,           0       ],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat ],
        [ cos_lat * cos_lon,  cos_lat * sin_lon, sin_lat ]
    ])

    return R @ np.array([dx, dy, dz])


def enu_to_ecef(e: float, n: float, u: float,
                ref_lat_deg: float, ref_lon_deg: float) -> np.ndarray:
    """Convert ENU to ECEF difference vector.

    Args:
        e, n, u: East, North, Up [m]
        ref_lat_deg, ref_lon_deg: Reference point [deg]

    Returns:
        np.array([dx, dy, dz]) ECEF difference [m]
    """
    lat = np.radians(ref_lat_deg)
    lon = np.radians(ref_lon_deg)

    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    # Rotation matrix ENU -> ECEF (transpose of ECEF->ENU)
    R = np.array([
        [-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
        [ cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
        [ 0,        cos_lat,           sin_lat           ]
    ])

    return R @ np.array([e, n, u])


def compute_elevation_azimuth(receiver_ecef: np.ndarray,
                               satellite_ecef: np.ndarray) -> tuple:
    """Compute elevation and azimuth from receiver to satellite.

    Args:
        receiver_ecef: Receiver position [m] ECEF
        satellite_ecef: Satellite position [m] ECEF

    Returns:
        (elevation_deg, azimuth_deg)
    """
    diff = satellite_ecef - receiver_ecef
    lat, lon, _ = ecef_to_lla(*receiver_ecef)

    enu = ecef_to_enu(diff[0], diff[1], diff[2], lat, lon)
    e, n, u = enu

    horiz_dist = np.sqrt(e**2 + n**2)
    elevation = np.degrees(np.arctan2(u, horiz_dist))
    azimuth = np.degrees(np.arctan2(e, n)) % 360

    return elevation, azimuth


def compute_geometric_range(receiver_ecef: np.ndarray,
                             satellite_ecef: np.ndarray) -> float:
    """Geometric range between receiver and satellite [m]."""
    return np.linalg.norm(satellite_ecef - receiver_ecef)


# [EG-C]
def pz90_to_wgs84(x: float, y: float, z: float) -> np.ndarray:
    """Transform a PZ-90(.11) ECEF position to WGS-84 ECEF.

    Small-angle 7-parameter Helmert, EPSG:1032 "Coordinate Frame
    rotation" convention (matches EPSG:15843's parameter set). With
    rX = rY = scale = 0, this reduces to a rotation about Z plus a
    Z-axis translation:

        x_w = x + rz*y + dx
        y_w = -rz*x + y + dy
        z_w = z + dz

    Args:
        x, y, z: PZ-90(.11) ECEF coordinates [m]

    Returns:
        np.array([x, y, z]) in WGS-84 ECEF [m]
    """
    x_w = x + PZ90_RZ_RAD * y + PZ90_DX
    y_w = -PZ90_RZ_RAD * x + y + PZ90_DY
    z_w = z + PZ90_DZ
    return np.array([x_w, y_w, z_w])


def pz90_to_wgs84_velocity(vx: float, vy: float, vz: float) -> np.ndarray:
    """Rotate a PZ-90(.11) ECEF velocity vector into WGS-84.

    Velocities only need the rotation component of the Helmert
    transform (translation and scale don't apply to a rate vector).

    Args:
        vx, vy, vz: PZ-90(.11) ECEF velocity [m/s]

    Returns:
        np.array([vx, vy, vz]) in WGS-84 ECEF [m/s]
    """
    vx_w = vx + PZ90_RZ_RAD * vy
    vy_w = -PZ90_RZ_RAD * vx + vy
    return np.array([vx_w, vy_w, vz])
