"""
RTCM3 MSM4 Message Decoder.

MSM4 provides: full pseudorange, full carrier phase, Doppler, and CNR.
This decoder handles messages 1074 (GPS), 1084 (GLONASS), 1094 (Galileo),
1124 (BeiDou).

Reference: RTCM Standard 10403.3, Section 3.5 (MSM Messages)
"""

import logging
import numpy as np
from typing import List, Optional, Tuple

from .bit_reader import BitReader
from ..core.constants import (
    C, Constellation, SignalType, MSM_CONSTELLATION_MAP,
    GPS_L1_FREQ, GPS_L5_FREQ, GAL_E1_FREQ, GAL_E5A_FREQ,
    BDS_B1C_FREQ, BDS_B2A_FREQ,
    glonass_l1_freq, glonass_l2_freq,
    SECONDS_PER_WEEK,
)
from ..core.data_types import RawObservation, EpochObservations

logger = logging.getLogger(__name__)

# MSM signal ID to SignalType mapping per constellation
# Signal IDs are defined in RTCM 10403.3 Table 3.5-91 onwards
GPS_SIGNAL_MAP = {
    2: SignalType.GPS_L1CA,     # L1 C/A
    15: SignalType.GPS_L5,      # L5 I+Q
    16: SignalType.GPS_L5,      # L5 Q
}

GLONASS_SIGNAL_MAP = {
    2: SignalType.GLO_L1,       # L1 C/A
    8: SignalType.GLO_L2,       # L2 C/A
}

GALILEO_SIGNAL_MAP = {
    2: SignalType.GAL_E1,       # E1 C
    8: SignalType.GAL_E1,       # E1 B+C
    17: SignalType.GAL_E5A,     # E5a I+Q
}

BEIDOU_SIGNAL_MAP = {
    2: SignalType.BDS_B1C,      # B1C
    8: SignalType.BDS_B2A,      # B2a
}

CONSTELLATION_SIGNAL_MAP = {
    Constellation.GPS: GPS_SIGNAL_MAP,
    Constellation.GLONASS: GLONASS_SIGNAL_MAP,
    Constellation.GALILEO: GALILEO_SIGNAL_MAP,
    Constellation.BEIDOU: BEIDOU_SIGNAL_MAP,
}

# Signal ID to frequency for pseudorange/phase computation
GPS_SIGNAL_FREQ = {
    2: GPS_L1_FREQ,
    15: GPS_L5_FREQ,
    16: GPS_L5_FREQ,
}

GALILEO_SIGNAL_FREQ = {
    2: GAL_E1_FREQ,
    8: GAL_E1_FREQ,
    17: GAL_E5A_FREQ,
}

BEIDOU_SIGNAL_FREQ = {
    2: BDS_B1C_FREQ,
    8: BDS_B2A_FREQ,
}

# Range constants for MSM
RANGE_MS = C * 0.001  # 1 ms in meters (range for light-time ~299792.458 m)
P2_10 = 2**-10
P2_24 = 2**-24
P2_29 = 2**-29
P2_31 = 2**-31


class MSM4Decoder:
    """Decoder for RTCM3 MSM4 messages.

    MSM4 data fields per satellite-signal cell:
        - Satellite rough range (integer ms + fractional)
        - Signal fine pseudorange (15 bits, signed, resolution 2^-24 ms)
        - Signal fine carrier phase (22 bits, signed, resolution 2^-29 ms)
        - Signal lock time indicator (4 bits)
        - Signal half-cycle ambiguity (1 bit)
        - Signal CNR (6 bits, resolution 1 dB-Hz)
    """

    def __init__(self):
        self._glonass_channels = {}  # SVN -> freq channel mapping

    def set_glonass_channel(self, svn: int, channel: int):
        """Set GLONASS frequency channel for a satellite."""
        self._glonass_channels[svn] = channel

    def decode(self, msg_type: int, data: bytes,
               current_week: int = 0) -> Optional[EpochObservations]:
        """Decode an MSM4 message.

        Args:
            msg_type: RTCM message type (1074, 1084, 1094, 1124)
            data: Raw message data bytes (after frame header)
            current_week: Current GPS week number

        Returns:
            EpochObservations or None if decoding fails
        """
        constellation = MSM_CONSTELLATION_MAP.get(msg_type)
        if constellation is None:
            logger.warning(f"Unknown MSM message type: {msg_type}")
            return None

        try:
            reader = BitReader(data)
            return self._decode_msm4(reader, constellation, current_week)
        except Exception as e:
            logger.error(f"MSM4 decode error (msg {msg_type}): {e}")
            return None

    def _decode_msm4(self, reader: BitReader, constellation: Constellation,
                      week: int) -> Optional[EpochObservations]:
        """Internal MSM4 decoding."""

        # --- MSM Header ---
        msg_type = reader.read_uint(12)       # Message number
        station_id = reader.read_uint(12)      # Reference station ID

        # Epoch time (constellation-dependent)
        if constellation == Constellation.GLONASS:
            # GLONASS: day-of-week (3 bits) + time-of-day in ms (27 bits).
            # GLONASS epoch time is in Moscow time (UTC+3).
            # Convert to GPS TOW so GLONASS observations merge with
            # GPS/Galileo/BeiDou epochs in the pipeline accumulator.
            #   GPS time = UTC + 18 leap-seconds (as of 2017)
            #   Moscow time = UTC + 10800 s (3 hours)
            #   Offset = 10800 - 18 = 10782 s
            dow = reader.read_uint(3)
            tod_ms = reader.read_uint(27)
            glonass_tod = dow * 86400.0 + tod_ms * 0.001
            epoch_time = (glonass_tod - 10782.0) % 604800.0
        else:
            # GPS/Galileo/BeiDou: GPS epoch time in ms (30 bits)
            epoch_ms = reader.read_uint(30)
            epoch_time = epoch_ms * 0.001  # Convert to seconds (TOW)

        multiple_msg = reader.read_bool()      # Multiple message flag
        iods = reader.read_uint(3)             # Issue of data station
        reserved = reader.read_uint(7)         # Reserved
        clock_steering = reader.read_uint(2)   # Clock steering indicator
        ext_clock = reader.read_uint(2)        # External clock indicator
        smoothing_indicator = reader.read_bool()
        smoothing_interval = reader.read_uint(3)

        # Satellite mask (64 bits)
        sat_mask = reader.read_bitmask(64)
        n_sat = len(sat_mask)

        # Signal mask (32 bits)
        sig_mask = reader.read_bitmask(32)
        n_sig = len(sig_mask)

        if n_sat == 0 or n_sig == 0:
            return None

        # Cell mask: which satellite-signal combinations exist
        n_cells = n_sat * n_sig
        cell_mask_bits = reader.read_uint(n_cells)
        cell_mask = []
        for i in range(n_cells):
            if cell_mask_bits & (1 << (n_cells - 1 - i)):
                cell_mask.append(True)
            else:
                cell_mask.append(False)

        n_active_cells = sum(cell_mask)

        # --- Satellite Data ---
        # Rough ranges (integer ms, 8 bits unsigned per satellite)
        sat_rough_range_int = []
        for _ in range(n_sat):
            sat_rough_range_int.append(reader.read_uint(8))

        # Rough ranges (fractional ms, 10 bits unsigned per satellite)
        sat_rough_range_frac = []
        for _ in range(n_sat):
            sat_rough_range_frac.append(reader.read_uint(10))

        # Rough phase range rates (14 bits signed per satellite) - MSM4 doesn't have this
        # MSM4 has no satellite phase range rate

        # --- Signal Data ---
        # Fine pseudorange (15 bits signed per cell)
        sig_fine_pr = []
        for _ in range(n_active_cells):
            sig_fine_pr.append(reader.read_int(15))

        # Fine carrier phase (22 bits signed per cell)
        sig_fine_cp = []
        for _ in range(n_active_cells):
            sig_fine_cp.append(reader.read_int(22))

        # Lock time indicator (4 bits per cell)
        sig_lock = []
        for _ in range(n_active_cells):
            sig_lock.append(reader.read_uint(4))

        # Half-cycle ambiguity (1 bit per cell)
        sig_half_cycle = []
        for _ in range(n_active_cells):
            sig_half_cycle.append(reader.read_bool())

        # CNR (6 bits per cell, resolution 1 dB-Hz)
        sig_cnr = []
        for _ in range(n_active_cells):
            sig_cnr.append(reader.read_uint(6))

        # --- Build Observations ---
        observations = []
        cell_idx = 0

        sig_map = CONSTELLATION_SIGNAL_MAP.get(constellation, {})

        for sat_i, sat_prn in enumerate(sat_mask):
            # Compute rough range for this satellite [m]
            rough_range_ms = sat_rough_range_int[sat_i] + \
                            sat_rough_range_frac[sat_i] * P2_10

            if sat_rough_range_int[sat_i] == 255:
                # Invalid satellite - skip all signals
                for sig_j in range(n_sig):
                    idx = sat_i * n_sig + sig_j
                    if cell_mask[idx]:
                        cell_idx += 1
                continue

            rough_range_m = rough_range_ms * RANGE_MS

            for sig_j, sig_id in enumerate(sig_mask):
                idx = sat_i * n_sig + sig_j
                if not cell_mask[idx]:
                    continue

                # Fine pseudorange [m]
                fine_pr = sig_fine_pr[cell_idx] * P2_24 * RANGE_MS
                pseudorange = rough_range_m + fine_pr

                # Get frequency for carrier phase conversion
                freq = self._get_signal_frequency(
                    constellation, sig_id, sat_prn)

                if freq is None:
                    cell_idx += 1
                    continue

                wavelength = C / freq

                # Fine carrier phase [cycles]
                fine_cp_ms = sig_fine_cp[cell_idx] * P2_29
                fine_cp_m = fine_cp_ms * RANGE_MS
                # Total phase = rough_range / wavelength + fine_phase / wavelength
                carrier_phase = rough_range_m / wavelength + fine_cp_m / wavelength

                # CNR
                cnr = float(sig_cnr[cell_idx])

                # Lock time (convert indicator to approximate seconds)
                lock_time = self._lock_indicator_to_seconds(sig_lock[cell_idx])

                # Doppler - MSM4 doesn't directly provide Doppler
                # We can estimate it from phase rate if available, set 0 for now
                doppler = 0.0

                # Map signal ID to our SignalType
                signal_type = sig_map.get(sig_id)
                if signal_type is None:
                    cell_idx += 1
                    continue

                obs = RawObservation(
                    constellation=constellation,
                    svn=sat_prn,
                    signal=signal_type,
                    timestamp_gps=epoch_time,
                    week=week,
                    pseudorange=pseudorange,
                    carrier_phase=carrier_phase,
                    doppler=doppler,
                    cnr=cnr,
                    lock_time=lock_time,
                    half_cycle_ambiguity=sig_half_cycle[cell_idx],
                    pseudorange_valid=(sig_fine_pr[cell_idx] != -16384),
                    carrier_phase_valid=(sig_fine_cp[cell_idx] != -2097152),
                )
                observations.append(obs)
                cell_idx += 1

        if not observations:
            return None

        epoch = EpochObservations(
            timestamp_gps=epoch_time,
            week=week,
            observations=observations,
        )

        return epoch

    def _get_signal_frequency(self, constellation: Constellation,
                               sig_id: int, svn: int) -> Optional[float]:
        """Get carrier frequency for a given constellation and signal ID."""
        if constellation == Constellation.GPS:
            return GPS_SIGNAL_FREQ.get(sig_id)
        elif constellation == Constellation.GALILEO:
            return GALILEO_SIGNAL_FREQ.get(sig_id)
        elif constellation == Constellation.BEIDOU:
            return BEIDOU_SIGNAL_FREQ.get(sig_id)
        elif constellation == Constellation.GLONASS:
            ch = self._glonass_channels.get(svn, 0)
            if sig_id in (1, 2):
                return glonass_l1_freq(ch)
            elif sig_id in (7, 8):
                return glonass_l2_freq(ch)
        return None

    @staticmethod
    def _lock_indicator_to_seconds(indicator: int) -> float:
        """Convert MSM4 lock time indicator (4 bits) to seconds.

        Table 3.5-74 in RTCM 10403.3.
        """
        # Simplified mapping
        thresholds = [0, 24, 72, 168, 360, 744, 937, 1500,
                      2400, 4800, 9600, 19200, 38400, 76800,
                      153600, 307200]
        if indicator < len(thresholds):
            return thresholds[indicator] / 1000.0
        return 307.2
