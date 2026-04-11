"""
RTK Positioning Engine — Real-Time Kinematic.

Implements carrier-phase-based relative positioning using
double-differenced observations and integer ambiguity resolution.

Pipeline:
    1. Form single differences (base - rover) to eliminate satellite clocks
    2. Form double differences (between satellites) to eliminate receiver clocks
    3. Float solution via Extended Kalman Filter
    4. Integer ambiguity resolution via LAMBDA method
    5. Fixed solution with resolved ambiguities

Expected accuracy: 1-2 cm horizontal with fixed ambiguities.

This engine will be fully implemented in Phase 3.
"""

import time
import numpy as np
import logging
from typing import List, Optional, Dict, Tuple

from ..core.constants import C, Constellation, ELEVATION_MASK_DEG
from ..core.data_types import (
    EpochObservations, PositionSolution, EphemerisStore,
    BaseStationInfo, FixType, DOPValues,
)

logger = logging.getLogger(__name__)


class AmbiguityState:
    """Tracks carrier phase ambiguity state for one satellite pair."""

    def __init__(self, sat_ref: str, sat_j: str, signal: str):
        self.sat_ref = sat_ref        # Reference satellite
        self.sat_j = sat_j            # Other satellite
        self.signal = signal          # Signal type
        self.float_value: float = 0.0 # Float ambiguity estimate [cycles]
        self.fixed_value: Optional[int] = None  # Fixed integer ambiguity
        self.variance: float = 1e6    # Ambiguity variance
        self.is_fixed: bool = False


class RTKEngine:
    """Real-Time Kinematic positioning engine.

    Uses double-differenced carrier phase observations for
    centimeter-level positioning.
    """

    def __init__(self, ephemeris_store: EphemerisStore):
        self.eph_store = ephemeris_store
        self.base_info: Optional[BaseStationInfo] = None

        # Kalman filter state
        self._state = None           # [dx, dy, dz, N1, N2, ..., Nn]
        self._covariance = None      # State covariance matrix
        self._ambiguities: Dict[str, AmbiguityState] = {}

        # Configuration
        self.elevation_mask = ELEVATION_MASK_DEG
        self.min_lock_time = 10.0    # Minimum lock time for phase [s]
        self.ratio_threshold = 3.0   # LAMBDA ratio test threshold
        self.max_age = 5.0           # Max age of base observations [s]

        # Base station data
        self._base_epoch: Optional[EpochObservations] = None

    def set_base_station(self, info: BaseStationInfo):
        """Set the known base station position."""
        self.base_info = info
        logger.info(f"RTK base station set: ID={info.station_id}")

    def update_base_observations(self, epoch: EpochObservations):
        """Update base station observations.

        These are synchronized with rover observations for
        double-differencing.
        """
        self._base_epoch = epoch

    def process_epoch(self, rover_epoch: EpochObservations) -> PositionSolution:
        """Process one epoch of rover observations.

        Args:
            rover_epoch: Rover receiver observations

        Returns:
            PositionSolution (RTK_FLOAT or RTK_FIXED)
        """
        t_start = time.time()

        solution = PositionSolution(
            timestamp_gps=rover_epoch.timestamp_gps,
            week=rover_epoch.week,
            fix_type=FixType.NO_FIX,
        )

        if self.base_info is None or self._base_epoch is None:
            logger.debug("RTK: No base station data available")
            solution.processing_time_ms = (time.time() - t_start) * 1000
            return solution

        # TODO Phase 3: Full RTK implementation
        # The implementation will follow these steps:

        # Step 1: Time-match base and rover observations
        # matched = self._match_observations(rover_epoch, self._base_epoch)

        # Step 2: Select reference satellite per constellation
        # (highest elevation, best CNR)
        # ref_sats = self._select_reference_satellites(matched)

        # Step 3: Form double differences
        # dd_obs = self._form_double_differences(matched, ref_sats)

        # Step 4: Kalman filter time update (prediction)
        # self._kf_predict()

        # Step 5: Kalman filter measurement update
        # self._kf_update(dd_obs)

        # Step 6: LAMBDA ambiguity resolution
        # fixed, ratio = self._lambda_resolution()

        # Step 7: If fixed, compute fixed solution
        # if fixed:
        #     solution = self._fixed_solution(dd_obs)
        #     solution.fix_type = FixType.RTK_FIXED
        #     solution.ratio_test = ratio
        # else:
        #     solution = self._float_solution()
        #     solution.fix_type = FixType.RTK_FLOAT

        solution.processing_time_ms = (time.time() - t_start) * 1000
        return solution

    def _lambda_search(self, float_ambiguities: np.ndarray,
                        Q_a: np.ndarray,
                        n_candidates: int = 2) -> Tuple[np.ndarray, float]:
        """LAMBDA method for integer ambiguity resolution.

        Implements the LAMBDA (Least-squares AMBiguity Decorrelation Adjustment)
        method for finding the integer least squares solution.

        Steps:
            1. Z-transform (decorrelation) of ambiguity space
            2. Integer search in decorrelated space
            3. Inverse transform to get original ambiguities
            4. Ratio test for validation

        Args:
            float_ambiguities: Float ambiguity estimates [n]
            Q_a: Ambiguity covariance matrix [n x n]
            n_candidates: Number of candidates to find

        Returns:
            (best_integers, ratio_test_value)
        """
        # TODO Phase 3: Implement LAMBDA
        #
        # The LAMBDA method consists of:
        # 1. LDLT decomposition of Q_a
        # 2. Integer Gauss transformations for decorrelation (Z-transform)
        # 3. Search in decorrelated space using branch-and-bound
        # 4. Inverse Z-transform
        # 5. Ratio test: ratio = cost(2nd_best) / cost(best)
        #
        # If ratio > threshold (typically 3.0), ambiguities are "fixed"

        n = len(float_ambiguities)
        best = np.round(float_ambiguities).astype(int)
        ratio = 0.0

        return best, ratio

    def reset(self):
        """Reset the RTK engine state (e.g., after cycle slip)."""
        self._state = None
        self._covariance = None
        self._ambiguities.clear()
        logger.info("RTK engine state reset")