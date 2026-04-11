"""
NMEA sentence parsing utilities.

Focused on reference-grade fields needed by the UI and pipeline:
    - GGA (fix position + altitude + quality)
    - RMC (position + speed/course + UTC date/time)
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional


@dataclass
class NMEAReference:
    """Parsed NMEA position/reference payload."""
    sentence_type: str
    talker: str
    latitude: float
    longitude: float
    altitude_m: float = 0.0
    fix_quality: int = 0
    num_satellites: int = 0
    speed_knots: float = 0.0
    course_deg: float = 0.0
    timestamp_utc: Optional[datetime] = None

    @property
    def is_valid_fix(self) -> bool:
        if self.sentence_type == "GGA":
            return self.fix_quality > 0
        if self.sentence_type == "RMC":
            return self.timestamp_utc is not None
        return False

    @property
    def source(self) -> str:
        return f"{self.talker}{self.sentence_type}"


class NMEASentenceParser:
    """Minimal parser for NMEA GGA/RMC sentences."""

    def parse_reference(self, sentence: str) -> Optional[NMEAReference]:
        """Parse a NMEA sentence into a reference payload if supported."""
        if not sentence:
            return None

        line = sentence.strip()
        if not line.startswith("$"):
            return None

        if not self._validate_checksum(line):
            return None

        body = line[1:].split("*", 1)[0]
        fields = body.split(",")
        if not fields:
            return None

        if len(fields[0]) < 5:
            return None

        talker = fields[0][:2]
        msg_type = fields[0][2:]

        if msg_type == "GGA":
            return self._parse_gga(talker, fields)
        if msg_type == "RMC":
            return self._parse_rmc(talker, fields)
        return None

    @staticmethod
    def _validate_checksum(sentence: str) -> bool:
        if "*" not in sentence:
            return False

        body, checksum_str = sentence[1:].split("*", 1)
        checksum_str = checksum_str.strip()
        if len(checksum_str) < 2:
            return False

        try:
            expected = int(checksum_str[:2], 16)
        except ValueError:
            return False

        computed = 0
        for ch in body:
            computed ^= ord(ch)
        return computed == expected

    @staticmethod
    def _parse_lat_lon(lat_str: str, lat_hemi: str,
                       lon_str: str, lon_hemi: str) -> Optional[tuple]:
        if not lat_str or not lon_str or not lat_hemi or not lon_hemi:
            return None

        try:
            lat_raw = float(lat_str)
            lon_raw = float(lon_str)
        except ValueError:
            return None

        lat_deg = int(lat_raw / 100)
        lat_min = lat_raw - lat_deg * 100
        latitude = lat_deg + lat_min / 60.0

        lon_deg = int(lon_raw / 100)
        lon_min = lon_raw - lon_deg * 100
        longitude = lon_deg + lon_min / 60.0

        if lat_hemi == "S":
            latitude = -latitude
        if lon_hemi == "W":
            longitude = -longitude

        return latitude, longitude

    def _parse_gga(self, talker: str, fields: list[str]) -> Optional[NMEAReference]:
        # GGA: 0=msg,1=utc,2=lat,3=N/S,4=lon,5=E/W,6=fix_q,7=nsat,8=hdop,9=alt
        if len(fields) < 10:
            return None

        latlon = self._parse_lat_lon(fields[2], fields[3], fields[4], fields[5])
        if latlon is None:
            return None

        try:
            fix_quality = int(fields[6] or "0")
            num_satellites = int(fields[7] or "0")
            altitude_m = float(fields[9] or "0.0")
        except ValueError:
            return None

        ts = self._parse_hhmmss_utc(fields[1])

        return NMEAReference(
            sentence_type="GGA",
            talker=talker,
            latitude=latlon[0],
            longitude=latlon[1],
            altitude_m=altitude_m,
            fix_quality=fix_quality,
            num_satellites=num_satellites,
            timestamp_utc=ts,
        )

    def _parse_rmc(self, talker: str, fields: list[str]) -> Optional[NMEAReference]:
        # RMC: 0=msg,1=utc,2=status,3=lat,4=N/S,5=lon,6=E/W,7=sog,8=cog,9=date
        if len(fields) < 10:
            return None

        if fields[2] != "A":  # Active/valid
            return None

        latlon = self._parse_lat_lon(fields[3], fields[4], fields[5], fields[6])
        if latlon is None:
            return None

        try:
            speed_knots = float(fields[7] or "0.0")
            course_deg = float(fields[8] or "0.0")
        except ValueError:
            return None

        ts = self._parse_rmc_datetime(fields[1], fields[9])

        return NMEAReference(
            sentence_type="RMC",
            talker=talker,
            latitude=latlon[0],
            longitude=latlon[1],
            speed_knots=speed_knots,
            course_deg=course_deg,
            timestamp_utc=ts,
        )

    @staticmethod
    def _parse_hhmmss_utc(value: str) -> Optional[datetime]:
        if not value or len(value) < 6:
            return None

        try:
            hh = int(value[0:2])
            mm = int(value[2:4])
            ss = int(value[4:6])
        except ValueError:
            return None

        now = datetime.now(timezone.utc)
        return now.replace(hour=hh, minute=mm, second=ss, microsecond=0)

    @staticmethod
    def _parse_rmc_datetime(time_str: str, date_str: str) -> Optional[datetime]:
        if not time_str or len(time_str) < 6 or not date_str or len(date_str) != 6:
            return None

        try:
            hh = int(time_str[0:2])
            mm = int(time_str[2:4])
            ss = int(time_str[4:6])
            dd = int(date_str[0:2])
            mo = int(date_str[2:4])
            yy = int(date_str[4:6])
        except ValueError:
            return None

        year = 2000 + yy if yy < 80 else 1900 + yy
        try:
            return datetime(year, mo, dd, hh, mm, ss, tzinfo=timezone.utc)
        except ValueError:
            return None
