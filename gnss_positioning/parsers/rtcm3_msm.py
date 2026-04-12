"""
RTCM3 MSM4 and MSM7 Message Decoder.

MSM4 provides: full pseudorange, full carrier phase, lock time, half-cycle, CNR.
MSM7 provides: same as MSM4 but with higher-resolution fields and Doppler.

Handled message types:
    MSM4: 1074 (GPS), 1084 (GLONASS), 1094 (Galileo), 1124 (BeiDou)
    MSM7: 1077 (GPS), 1087 (GLONASS), 1097 (Galileo), 1127 (BeiDou)

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

# MSM variant → message-type set
_MSM4_TYPES = frozenset({1074, 1084, 1094, 1124})
_MSM7_TYPES = frozenset({1077, 1087, 1097, 1127})


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
        """Decode an MSM4 or MSM7 message.

        Args:
            msg_type: RTCM message type (1074/1077, 1084/1087, 1094/1097, 1124/1127)
            data: Raw message data bytes (after frame header)
            current_week: Current GPS week number

        Returns:
            EpochObservations or None if decoding fails
        """
        constellation = MSM_CONSTELLATION_MAP.get(msg_type)
        if constellation is None:
            logger.warning(f"Unknown MSM message type: {msg_type}")
            return None

        is_msm7 = msg_type in _MSM7_TYPES
        if is_msm7:
            logger.debug(f"Decoding MSM7 message {msg_type} for {constellation.name}")
        try:
            reader = BitReader(data)
            return self._decode_msm4(reader, constellation, current_week,
                                     is_msm7=is_msm7)
        except Exception as e:
            variant = "MSM7" if is_msm7 else "MSM4"
            logger.error(f"{variant} decode error (msg {msg_type}): {e}")
            return None

    def _decode_msm4(self, reader: BitReader, constellation: Constellation,
                      week: int, is_msm7: bool = False) -> Optional[EpochObservations]:
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
            # GPS / Galileo / BeiDou: 30-bit epoch time in ms.
            # GPS and Galileo use GPS TOW directly.
            # BeiDou uses BDT (BeiDou Time = GPS time − 14 s). The epoch_time
            # is kept in BDT here so that satellite positions are computed at
            # the correct BDT emission time. The pipeline accumulator adds 14 s
            # to the BeiDou key to merge with GPS/Galileo epochs.
            epoch_ms = reader.read_uint(30)
            epoch_time = epoch_ms * 0.001

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
        # Rough ranges: integer ms (8 bits unsigned per satellite, DF397)
        sat_rough_range_int = []
        for _ in range(n_sat):
            sat_rough_range_int.append(reader.read_uint(8))

        # Rough ranges: fractional ms (10 bits unsigned per satellite, DF398)
        sat_rough_range_frac = []
        for _ in range(n_sat):
            sat_rough_range_frac.append(reader.read_uint(10))

        # MSM7 only: rough phase range rate (14 bits signed per satellite, DF399)
        # MSM4 does not include this field — skipping it is what caused the
        # bit-misalignment that corrupted all pseudoranges when MSM7 was decoded
        # as MSM4.
        if is_msm7:
            for _ in range(n_sat):
                reader.read_int(14)   # discard rough phase range rate

        # --- Signal Data ---
        # Field widths and scales differ between MSM4 and MSM7:
        #   MSM4 fine PR:    DF400  15 bits  scale 2^-24 ms
        #   MSM7 fine PR:    DF405  20 bits  scale 2^-29 ms
        #   MSM4 fine phase: DF401  22 bits  scale 2^-29 ms
        #   MSM7 fine phase: DF406  24 bits  scale 2^-31 ms
        #   MSM4 lock time:  DF402   4 bits  (indicator table)
        #   MSM7 lock time:  DF407  10 bits  (indicator table, wider)
        #   CNR:             DF403   6 bits  1 dB-Hz   (MSM4)
        #                    DF408  10 bits  0.0625 dB-Hz (MSM7)
        #   MSM7 also adds fine phase rate: DF404 15 bits per cell

        if is_msm7:
            pr_bits, pr_scale   = 20, P2_29          # DF405
            cp_bits, cp_scale   = 24, P2_31          # DF406
            lock_bits           = 10                  # DF407
            cnr_bits            = 10                  # DF408
            cnr_scale           = 0.0625             # dB-Hz per LSB
        else:
            pr_bits, pr_scale   = 15, P2_24          # DF400
            cp_bits, cp_scale   = 22, P2_29          # DF401
            lock_bits           = 4                   # DF402
            cnr_bits            = 6                   # DF403
            cnr_scale           = 1.0                # dB-Hz per LSB

        # Fine pseudorange (signed per cell)
        sig_fine_pr = []
        for _ in range(n_active_cells):
            sig_fine_pr.append(reader.read_int(pr_bits))

        # Fine carrier phase (signed per cell)
        sig_fine_cp = []
        for _ in range(n_active_cells):
            sig_fine_cp.append(reader.read_int(cp_bits))

        # Lock time indicator (unsigned per cell)
        sig_lock = []
        for _ in range(n_active_cells):
            sig_lock.append(reader.read_uint(lock_bits))

        # Half-cycle ambiguity (1 bit per cell) — same in both MSM4 and MSM7
        sig_half_cycle = []
        for _ in range(n_active_cells):
            sig_half_cycle.append(reader.read_bool())

        # CNR (unsigned per cell)
        sig_cnr = []
        for _ in range(n_active_cells):
            sig_cnr.append(reader.read_uint(cnr_bits) * cnr_scale)

        # MSM7 only: fine phase range rate (15 bits signed per cell, DF404)
        if is_msm7:
            for _ in range(n_active_cells):
                reader.read_int(15)   # discard fine phase range rate

        # --- Build Observations ---
        # Compute invalid sentinels once (most-negative value for each field width)
        pr_invalid = -(1 << (pr_bits - 1))
        cp_invalid = -(1 << (cp_bits - 1))

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
                fine_pr = sig_fine_pr[cell_idx] * pr_scale * RANGE_MS
                pseudorange = rough_range_m + fine_pr

                # Get frequency for carrier phase conversion
                freq = self._get_signal_frequency(
                    constellation, sig_id, sat_prn)

                if freq is None:
                    cell_idx += 1
                    continue

                wavelength = C / freq

                # Fine carrier phase [cycles]
                fine_cp_ms = sig_fine_cp[cell_idx] * cp_scale
                fine_cp_m = fine_cp_ms * RANGE_MS
                # Total phase = rough_range / wavelength + fine_phase / wavelength
                carrier_phase = rough_range_m / wavelength + fine_cp_m / wavelength

                # CNR — already scaled to dB-Hz by sig_cnr[cell_idx]
                cnr = float(sig_cnr[cell_idx])

                # Lock time (convert indicator to approximate seconds)
                lock_time = (self._lock_indicator_msm7(sig_lock[cell_idx])
                             if is_msm7
                             else self._lock_indicator_to_seconds(sig_lock[cell_idx]))

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
                    pseudorange_valid=(sig_fine_pr[cell_idx] != pr_invalid),
                    carrier_phase_valid=(sig_fine_cp[cell_idx] != cp_invalid),
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
        """Convert MSM4 lock time indicator (4 bits, DF402) to seconds.

        Table 3.5-74 in RTCM 10403.3.
        """
        thresholds = [0, 24, 72, 168, 360, 744, 937, 1500,
                      2400, 4800, 9600, 19200, 38400, 76800,
                      153600, 307200]
        if indicator < len(thresholds):
            return thresholds[indicator] / 1000.0
        return 307.2

    @staticmethod
    def _lock_indicator_msm7(indicator: int) -> float:
        """Convert MSM7 lock time indicator (10 bits, DF407) to seconds.

        The 10-bit indicator encodes lock time in ms directly:
            lock_time_ms = 2^(indicator >> 4) * (indicator & 0x0F) - bias
        Simplified: use the indicator directly scaled to a coarse bin.
        RTCM 10403.3 Table 3.5-74 (extended).
        """
        # Decode DF407: lock_time = 2^(I>>4) * (1 + (I & 0xF)/4) - 1) ms
        # For simplicity use the same mapping but extended to 10 bits
        if indicator == 0:
            return 0.0
        # Coarse: indicator ≈ lock_ms / 2 for small values
        # More precise: use formula from RTCM spec
        try:
            i = indicator
            e = (i >> 4) & 0x3F          # 6 exponent bits
            m = i & 0x0F                  # 4 mantissa bits
            lock_ms = (1 << e) * (m + 1) - 1
            return lock_ms / 1000.0
        except Exception:
            return float(indicator) / 1000.0
