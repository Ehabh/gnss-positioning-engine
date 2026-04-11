"""
DGNSS Positioning Engine — Differential GNSS.

Applies pseudorange corrections from a base station at known position
to the rover pseudoranges, eliminating common-mode errors (orbit, clock,
atmosphere).

Correction model:
    PRC_i = PR_base_i - R_base_i - c*dt_base
    PR_rover_corrected = PR_rover - PRC_i - RRC_i * dt_age

Where:
    PRC = Pseudorange correction
    RRC = Range rate correction (for temporal decorrelation)
    dt_age = Age of correction

This engine will be fully implemented in Phase 2.
"""

import time
import numpy as np
import logging
from typing import List, Optional, Dict

from ..core.constants import C, Constellation, ELEVATION_MASK_DEG
from ..core.data_types import (
    EpochObservations, PositionSolution, EphemerisStore,
    BaseStationInfo, SatelliteState, FixType, DOPValues,
)
from ..engines.sps import SPSEngine

logger = logging.getLogger(__name__)


class PseudorangeCorrection:
    """Single satellite pseudorange correction."""

    def __init__(self, sat_id: str, prc: float, rrc: float,
                 timestamp: float, iod: int = 0):
        self.sat_id = sat_id
        self.prc = prc          # Pseudorange correction [m]
        self.rrc = rrc          # Range rate correction [m/s]
        self.timestamp = timestamp  # GPS TOW when correction was computed
        self.iod = iod          # Issue of data (ephemeris)

    def apply(self, pseudorange: float, current_tow: float) -> float:
        """Apply correction to a pseudorange.

        Args:
            pseudorange: Raw pseudorange [m]
            current_tow: Current GPS TOW [s]

        Returns:
            Corrected pseudorange [m]
        """
        dt = current_tow - self.timestamp
        return pseudorange - self.prc - self.rrc * dt


class DGNSSEngine:
    """Differential GNSS positioning engine.

    Uses pseudorange corrections from a base station to improve
    rover positioning accuracy. Expected improvement: ~1-3m horizontal.
    """

    def __init__(self, ephemeris_store: EphemerisStore):
        self.eph_store = ephemeris_store
        self.base_info: Optional[BaseStationInfo] = None
        self.corrections: Dict[str, PseudorangeCorrection] = {}
        self.max_correction_age = 30.0  # Maximum age of corrections [s]
        self._sps_engine = SPSEngine(ephemeris_store)

        # Base station observations for correction computation
        self._base_observations = None

    def set_base_station(self, info: BaseStationInfo):
        """Set the base station position.

        Args:
            info: Base station coordinates and metadata
        """
        self.base_info = info
        logger.info(f"Base station set: ID={info.station_id}, "
                    f"pos=({info.x_ecef:.3f}, {info.y_ecef:.3f}, {info.z_ecef:.3f})")

    def update_base_observations(self, epoch: EpochObservations):
        """Process base station observations to compute corrections.

        For each satellite observed at the base station:
        1. Compute satellite position from ephemeris
        2. Compute geometric range to known base position
        3. Compute pseudorange correction = observed - computed

        Args:
            epoch: Base station observations
        """
        if self.base_info is None:
            logger.warning("No base station position set")
            return

        self._base_observations = epoch

        # TODO Phase 2: Full implementation
        # For each satellite:
        #   1. Get ephemeris
        #   2. Compute sat position at transmission time
        #   3. Compute geometric range to known base position
        #   4. PRC = PR_measured - geometric_range - c*dt_sv
        #   5. RRC = rate of change of PRC (from consecutive epochs)
        #   6. Store correction

        logger.info(f"DGNSS: Base observations received "
                   f"({epoch.satellite_count} satellites)")

    def process_epoch(self, rover_epoch: EpochObservations) -> PositionSolution:
        """Process rover epoch with differential corrections.

        Args:
            rover_epoch: Rover receiver observations

        Returns:
            PositionSolution with DGNSS fix type
        """
        t_start = time.time()

        # TODO Phase 2: Apply corrections to rover pseudoranges
        # For now, fall back to SPS
        if not self.corrections:
            logger.debug("No DGNSS corrections available, falling back to SPS")
            solution = self._sps_engine.process_epoch(rover_epoch)
            # Still mark as SPS since we don't have corrections
            return solution

        # Phase 2 will:
        # 1. Match rover satellites with available corrections
        # 2. Apply PRC + RRC*dt to each matched pseudorange
        # 3. Run WLS with corrected pseudoranges
        # 4. Set fix_type = FixType.DGNSS

        solution = PositionSolution(
            timestamp_gps=rover_epoch.timestamp_gps,
            week=rover_epoch.week,
            fix_type=FixType.NO_FIX,
        )

        solution.processing_time_ms = (time.time() - t_start) * 1000
        return solution