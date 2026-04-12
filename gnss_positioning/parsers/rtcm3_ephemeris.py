"""
RTCM3 Ephemeris Decoders.

Decodes broadcast ephemeris from:
    - 1019: GPS
    - 1020: GLONASS
    - 1042: BeiDou
    - 1045: Galileo F/NAV
    - 1046: Galileo I/NAV

Reference: RTCM Standard 10403.3, Tables 3.5-18 through 3.5-43
"""

import logging
from typing import Optional

from .bit_reader import BitReader
from ..core.constants import PI
from ..core.data_types import (
    GPSEphemeris, GalileoEphemeris, GLONASSEphemeris, BeidouEphemeris
)

logger = logging.getLogger(__name__)


def decode_gps_ephemeris(data: bytes) -> Optional[GPSEphemeris]:
    """Decode RTCM 1019 GPS Ephemeris.

    Args:
        data: Raw message data bytes

    Returns:
        GPSEphemeris or None
    """
    try:
        r = BitReader(data)
        msg_type = r.read_uint(12)     # 1019
        svn = r.read_uint(6)           # PRN
        week = r.read_uint(10)         # GPS week (mod 1024)
        ura = r.read_uint(4)           # URA index
        _code_l2 = r.read_uint(2)      # Code on L2
        idot = r.read_int(14) * (PI * 2**-43)
        iode = r.read_uint(8)
        toc = r.read_uint(16) * 16.0   # [s]
        af2 = r.read_int(8) * 2**-55   # [s/s^2]
        af1 = r.read_int(16) * 2**-43  # [s/s]
        af0 = r.read_int(22) * 2**-31  # [s]
        iodc = r.read_uint(10)
        crs = r.read_int(16) * 2**-5   # [m]
        delta_n = r.read_int(16) * (PI * 2**-43)  # [rad/s]
        m0 = r.read_int(32) * (PI * 2**-31)       # [rad]
        cuc = r.read_int(16) * 2**-29  # [rad]
        e = r.read_uint(32) * 2**-33   # eccentricity
        cus = r.read_int(16) * 2**-29  # [rad]
        sqrt_a = r.read_uint(32) * 2**-19  # [m^0.5]
        toe = r.read_uint(16) * 16.0   # [s]
        cic = r.read_int(16) * 2**-29  # [rad]
        omega0 = r.read_int(32) * (PI * 2**-31)   # [rad]
        cis = r.read_int(16) * 2**-29  # [rad]
        i0 = r.read_int(32) * (PI * 2**-31)       # [rad]
        crc = r.read_int(16) * 2**-5   # [m]
        omega = r.read_int(32) * (PI * 2**-31)    # [rad]
        omega_dot = r.read_int(24) * (PI * 2**-43)  # [rad/s]
        tgd = r.read_int(8) * 2**-31   # [s]
        health = r.read_uint(6)
        _l2p_flag = r.read_bool()
        fit_interval = r.read_uint(1)

        # Resolve GPS week ambiguity (mod 1024)
        # Assume current cycle: week + 2048 if needed
        if week < 1024:
            week += 2048  # Adjust for current GPS week cycle

        eph = GPSEphemeris(
            svn=svn, week=week, toc=toc, toe=toe,
            af0=af0, af1=af1, af2=af2, tgd=tgd,
            sqrt_a=sqrt_a, e=e, i0=i0,
            omega0=omega0, omega=omega, m0=m0,
            delta_n=delta_n, omega_dot=omega_dot, idot=idot,
            cuc=cuc, cus=cus, crc=crc, crs=crs, cic=cic, cis=cis,
            ura=ura, health=health, iodc=iodc, iode=iode,
            fit_interval=fit_interval,
        )
        logger.debug(f"GPS ephemeris decoded: G{svn:02d} IODE={iode}")
        return eph

    except Exception as e:
        logger.error(f"GPS ephemeris decode error: {e}")
        return None


def decode_glonass_ephemeris(data: bytes) -> Optional[GLONASSEphemeris]:
    """Decode RTCM 1020 GLONASS Ephemeris.

    GLONASS uses position/velocity/acceleration model, not Keplerian.

    Args:
        data: Raw message data bytes

    Returns:
        GLONASSEphemeris or None
    """
    try:
        r = BitReader(data)
        msg_type = r.read_uint(12)      # 1020
        svn = r.read_uint(6)            # Slot number
        freq_ch = r.read_uint(5) - 7    # Frequency channel (-7 to +13)
        _almanac_health = r.read_bool()
        _almanac_health_avail = r.read_bool()
        _p1 = r.read_uint(2)
        tk_h = r.read_uint(5)           # Hours
        tk_m = r.read_uint(6)           # Minutes
        tk_s = r.read_uint(1) * 30      # 30-second intervals
        _bn_health = r.read_uint(1)
        _p2 = r.read_bool()
        tb = r.read_uint(7) * 15 * 60   # Reference time [s] (15-min intervals, Moscow TOD)

        # Velocity [km/s] -> [m/s]
        vx = r.read_int(24) * 2**-20 * 1000.0
        x = r.read_int(27) * 2**-11 * 1000.0    # [km] -> [m]
        ax = r.read_int(5) * 2**-30 * 1000.0     # [km/s^2] -> [m/s^2]
        vy = r.read_int(24) * 2**-20 * 1000.0
        y = r.read_int(27) * 2**-11 * 1000.0
        ay = r.read_int(5) * 2**-30 * 1000.0
        vz = r.read_int(24) * 2**-20 * 1000.0
        z = r.read_int(27) * 2**-11 * 1000.0
        az = r.read_int(5) * 2**-30 * 1000.0

        _p3 = r.read_bool()
        gamma_n = r.read_int(11) * 2**-40
        _p = r.read_uint(2)
        _ln = r.read_bool()
        tau_n = r.read_int(22) * 2**-30  # [s]
        _delta_tau = r.read_int(5) * 2**-30
        age = r.read_uint(5)
        _p4 = r.read_bool()
        _ft = r.read_uint(4)
        nt = r.read_uint(11)
        _m = r.read_uint(2)
        _additional = r.read_bool()
        _na = r.read_uint(11)
        _tau_c = r.read_int(32) * 2**-31
        n4 = r.read_uint(5)
        _tau_gps = r.read_int(22) * 2**-30
        _ln_flag = r.read_bool()

        health = 1 if _bn_health else 0

        eph = GLONASSEphemeris(
            svn=svn, freq_channel=freq_ch, tb=tb,
            x=x, y=y, z=z,
            vx=vx, vy=vy, vz=vz,
            ax=ax, ay=ay, az=az,
            tau_n=tau_n, gamma_n=gamma_n,
            health=health, age=age, nt=nt, n4=n4,
        )
        logger.debug(f"GLONASS ephemeris decoded: R{svn:02d} ch={freq_ch}")
        return eph

    except Exception as e:
        logger.error(f"GLONASS ephemeris decode error: {e}")
        return None


def decode_galileo_ephemeris(data: bytes) -> Optional[GalileoEphemeris]:
    """Decode RTCM 1046 Galileo I/NAV Ephemeris.

    Args:
        data: Raw message data bytes

    Returns:
        GalileoEphemeris or None
    """
    try:
        r = BitReader(data)
        msg_type = r.read_uint(12)      # 1046
        svn = r.read_uint(6)
        week = r.read_uint(12)          # Galileo week
        iodnav = r.read_uint(10)
        sisa = r.read_uint(8)           # SISA index
        idot = r.read_int(14) * (PI * 2**-43)
        toc = r.read_uint(14) * 60.0    # [s]
        af2 = r.read_int(6) * 2**-59
        af1 = r.read_int(21) * 2**-46
        af0 = r.read_int(31) * 2**-34
        crs = r.read_int(16) * 2**-5
        delta_n = r.read_int(16) * (PI * 2**-43)
        m0 = r.read_int(32) * (PI * 2**-31)
        cuc = r.read_int(16) * 2**-29
        e = r.read_uint(32) * 2**-33
        cus = r.read_int(16) * 2**-29
        sqrt_a = r.read_uint(32) * 2**-19
        toe = r.read_uint(14) * 60.0
        cic = r.read_int(16) * 2**-29
        omega0 = r.read_int(32) * (PI * 2**-31)
        cis = r.read_int(16) * 2**-29
        i0 = r.read_int(32) * (PI * 2**-31)
        crc = r.read_int(16) * 2**-5
        omega = r.read_int(32) * (PI * 2**-31)
        omega_dot = r.read_int(24) * (PI * 2**-43)
        bgd_e5a_e1 = r.read_int(10) * 2**-32
        bgd_e5b_e1 = r.read_int(10) * 2**-32
        health_e5b = r.read_uint(2)
        _e5b_valid = r.read_bool()
        health_e1b = r.read_uint(2)
        _e1b_valid = r.read_bool()

        eph = GalileoEphemeris(
            svn=svn, week=week, toc=toc, toe=toe,
            af0=af0, af1=af1, af2=af2,
            bgd_e5a_e1=bgd_e5a_e1, bgd_e5b_e1=bgd_e5b_e1,
            sqrt_a=sqrt_a, e=e, i0=i0,
            omega0=omega0, omega=omega, m0=m0,
            delta_n=delta_n, omega_dot=omega_dot, idot=idot,
            cuc=cuc, cus=cus, crc=crc, crs=crs, cic=cic, cis=cis,
            sisa=sisa, health_e1b=health_e1b, health_e5a=health_e5b,
            data_source=1,  # I/NAV
            iodnav=iodnav,
        )
        logger.debug(f"Galileo ephemeris decoded: E{svn:02d} IOD={iodnav}")
        return eph

    except Exception as e:
        logger.error(f"Galileo ephemeris decode error: {e}")
        return None


def decode_beidou_ephemeris(data: bytes) -> Optional[BeidouEphemeris]:
    """Decode RTCM 1042 BeiDou Ephemeris.

    Args:
        data: Raw message data bytes

    Returns:
        BeidouEphemeris or None
    """
    try:
        r = BitReader(data)
        msg_type = r.read_uint(12)      # 1042
        svn = r.read_uint(6)
        week = r.read_uint(13)          # BeiDou week
        ura = r.read_uint(4)
        idot = r.read_int(14) * (PI * 2**-43)
        aode = r.read_uint(5)
        toc = r.read_uint(17) * 8.0     # [s]
        af2 = r.read_int(11) * 2**-66
        af1 = r.read_int(22) * 2**-50
        af0 = r.read_int(24) * 2**-33
        aodc = r.read_uint(5)
        crs = r.read_int(18) * 2**-6
        delta_n = r.read_int(16) * (PI * 2**-43)
        m0 = r.read_int(32) * (PI * 2**-31)
        cuc = r.read_int(18) * 2**-31
        e = r.read_uint(32) * 2**-33
        cus = r.read_int(18) * 2**-31
        sqrt_a = r.read_uint(32) * 2**-19
        toe = r.read_uint(17) * 8.0
        cic = r.read_int(18) * 2**-31
        omega0 = r.read_int(32) * (PI * 2**-31)
        cis = r.read_int(18) * 2**-31
        i0 = r.read_int(32) * (PI * 2**-31)
        crc = r.read_int(18) * 2**-6
        omega = r.read_int(32) * (PI * 2**-31)
        omega_dot = r.read_int(24) * (PI * 2**-43)
        tgd1 = r.read_int(10) * 1e-10   # [s]
        tgd2 = r.read_int(10) * 1e-10   # [s]
        health = r.read_bool()

        eph = BeidouEphemeris(
            svn=svn, week=week, toc=toc, toe=toe,
            af0=af0, af1=af1, af2=af2,
            tgd1=tgd1, tgd2=tgd2,
            sqrt_a=sqrt_a, e=e, i0=i0,
            omega0=omega0, omega=omega, m0=m0,
            delta_n=delta_n, omega_dot=omega_dot, idot=idot,
            cuc=cuc, cus=cus, crc=crc, crs=crs, cic=cic, cis=cis,
            ura=ura, health=int(health), aode=aode, aodc=aodc,
        )
        logger.debug(f"BeiDou ephemeris decoded: C{svn:02d} AODE={aode}")
        return eph

    except Exception as e:
        logger.error(f"BeiDou ephemeris decode error: {e}")
        return None


def decode_station_arp(data: bytes) -> Optional[dict]:
    """Decode RTCM 1005/1006 Station ARP.

    Args:
        data: Raw message data bytes

    Returns:
        Dict with station_id, x, y, z, antenna_height or None
    """
    try:
        r = BitReader(data)
        msg_type = r.read_uint(12)
        station_id = r.read_uint(12)
        itrf_year = r.read_uint(6)
        _gps_ind = r.read_bool()
        _glo_ind = r.read_bool()
        _gal_ind = r.read_bool()
        _ref_station = r.read_bool()
        x = r.read_int(38) * 0.0001    # [m]
        _single_osc = r.read_bool()
        r.skip(1)                        # Reserved
        y = r.read_int(38) * 0.0001
        r.skip(2)                        # Reserved
        z = r.read_int(38) * 0.0001

        antenna_height = 0.0
        if msg_type == 1006:
            antenna_height = r.read_uint(16) * 0.0001

        result = {
            'station_id': station_id,
            'x': x, 'y': y, 'z': z,
            'antenna_height': antenna_height,
            'itrf_year': itrf_year,
        }
        logger.debug(f"Station ARP decoded: ID={station_id} "
                     f"({x:.4f}, {y:.4f}, {z:.4f})")
        return result

    except Exception as e:
        logger.error(f"Station ARP decode error: {e}")
        return None