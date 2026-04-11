"""
GNSS Time Utilities.

Handles GPS time, UTC, constellation-specific time systems,
and epoch management.
"""

import time
import datetime
import numpy as np
from ..core.constants import (
    GPS_EPOCH_UNIX, SECONDS_PER_WEEK, LEAP_SECONDS, BDS_GPS_OFFSET
)


def gps_time_now() -> tuple:
    """Current GPS time as (week, tow).

    Returns:
        (gps_week, time_of_week_seconds)
    """
    unix_now = time.time()
    gps_seconds = unix_now - GPS_EPOCH_UNIX + LEAP_SECONDS
    week = int(gps_seconds // SECONDS_PER_WEEK)
    tow = gps_seconds % SECONDS_PER_WEEK
    return week, tow


def gps_to_unix(week: int, tow: float) -> float:
    """Convert GPS week/TOW to Unix timestamp."""
    return GPS_EPOCH_UNIX + week * SECONDS_PER_WEEK + tow - LEAP_SECONDS


def unix_to_gps(unix_time: float) -> tuple:
    """Convert Unix timestamp to GPS week/TOW."""
    gps_seconds = unix_time - GPS_EPOCH_UNIX + LEAP_SECONDS
    week = int(gps_seconds // SECONDS_PER_WEEK)
    tow = gps_seconds % SECONDS_PER_WEEK
    return week, tow


def gps_to_datetime(week: int, tow: float) -> datetime.datetime:
    """Convert GPS week/TOW to Python datetime (UTC)."""
    unix_ts = gps_to_unix(week, tow)
    return datetime.datetime.utcfromtimestamp(unix_ts)


def bds_to_gps_time(bds_week: int, bds_tow: float) -> tuple:
    """Convert BeiDou time to GPS time.

    BeiDou epoch is Jan 1, 2006 00:00:00 UTC.
    GPS epoch is Jan 6, 1980 00:00:00 UTC.
    """
    # BDS seconds since BDS epoch
    bds_seconds = bds_week * SECONDS_PER_WEEK + bds_tow
    # Convert to GPS seconds since GPS epoch
    gps_seconds = bds_seconds + BDS_GPS_OFFSET
    gps_week = int(gps_seconds // SECONDS_PER_WEEK)
    gps_tow = gps_seconds % SECONDS_PER_WEEK
    return gps_week, gps_tow


def glo_tod_to_gps_tow(tb_moscow: float, nt: int, n4: int,
                         gps_week: int) -> float:
    """Convert GLONASS time (Moscow) to GPS time of week.

    GLONASS time = UTC + 3h (Moscow time), no leap seconds.

    Args:
        tb_moscow: Time of day in Moscow time [s]
        nt: Day number within 4-year period
        n4: 4-year period number
        gps_week: Current GPS week for reference

    Returns:
        GPS time of week [s]
    """
    # Convert Moscow time to UTC
    utc_tod = tb_moscow - 3 * 3600
    if utc_tod < 0:
        utc_tod += 86400

    # For TOW computation, we need the UTC time and add leap seconds
    # This is an approximation; in production you'd track the full date
    gps_tow_approx = utc_tod + LEAP_SECONDS

    return gps_tow_approx % SECONDS_PER_WEEK


def check_time_validity(toe: float, current_tow: float,
                         max_age: float = 7200.0) -> bool:
    """Check if ephemeris is still valid based on time of ephemeris.

    Args:
        toe: Time of ephemeris [s]
        current_tow: Current GPS time of week [s]
        max_age: Maximum acceptable age [s]

    Returns:
        True if ephemeris is valid
    """
    dt = current_tow - toe
    # Handle week rollover
    if dt > SECONDS_PER_WEEK / 2:
        dt -= SECONDS_PER_WEEK
    elif dt < -SECONDS_PER_WEEK / 2:
        dt += SECONDS_PER_WEEK

    return abs(dt) <= max_age