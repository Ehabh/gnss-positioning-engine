"""
SPS Positioning Engine — Standard Positioning Service.

Computes receiver position from pseudorange observations using
Weighted Least Squares (WLS). Supports multi-constellation with
inter-system bias estimation.

The observation equation for each pseudorange:
    PR = geometric_range + c*dt_rx - c*dt_sv + T + I + epsilon

State vector: [X, Y, Z, c*dt_GPS, c*dt_GLO, c*dt_GAL, c*dt_BDS]
    (only include constellation clock offsets that have observations)

Accuracy improvements over a basic SPS engine:
  - Hatch filter (carrier-smoothed pseudoranges) reduces code noise ~5-10x
  - TGD correction skipped for iono-free dual-freq observations (Bug fix)
  - Klobuchar iono correction applied to single-freq fallback sats (Bug fix)
  - BeiDou GEO satellites (PRN 1-5, 59-63) excluded (wrong orbital model)
  - Two-pass WLS with elevation-based tropo/iono refinement
  - Residual-based outlier rejection

References:
    - Misra & Enge, "Global Positioning System: Signals, Measurements, and Performance"
    - Kaplan & Hegarty, "Understanding GPS/GNSS: Principles and Applications"
    - Hatch (1982), "The synergism of GPS code and carrier measurements"
"""

import time
import numpy as np
import logging
from typing import List, Optional, Dict, Tuple

from ..core.constants import (
    C, Constellation, SignalType, ELEVATION_MASK_DEG, CNR_MASK_DBH,
    MAX_ITERATIONS, CONVERGENCE_THRESHOLD, PR_MIN, PR_MAX,
    SIGNAL_FREQUENCY,
    GPS_L1_FREQ, GPS_L5_FREQ,
    GAL_E1_FREQ, GAL_E5A_FREQ,
    BDS_B1C_FREQ, BDS_B2A_FREQ,
)
from ..core.data_types import (
    EpochObservations, RawObservation, SatelliteState, PositionSolution,
    EphemerisStore, DOPValues, FixType,
)
from ..engines.orbit import (
    compute_gps_satellite, compute_galileo_satellite,
    compute_beidou_satellite, compute_glonass_satellite,
    earth_rotation_correction,
)
from ..corrections.atmosphere import (
    troposphere_correction, ionosphere_correction, ionofree_combination,
)
from ..utils.coordinates import (
    ecef_to_lla, compute_elevation_azimuth, compute_geometric_range,
)

logger = logging.getLogger(__name__)

# BeiDou GEO satellites: their ECEF position needs an extra Y-axis rotation
# (BDS-SIS-ICD §5.2.4.15) that _compute_keplerian does not implement.
# Including them corrupts the WLS solution; exclude them entirely.
_BDS_GEO_PRNS: frozenset = frozenset(range(1, 6)) | frozenset(range(59, 64))

# Iono-free dual-freq signal pairs: (primary_sig, secondary_sig, f1, f2)
_IF_PAIRS: Dict[Constellation, tuple] = {
    Constellation.GPS:     (SignalType.GPS_L1CA, SignalType.GPS_L5,
                             GPS_L1_FREQ, GPS_L5_FREQ),
    Constellation.GALILEO: (SignalType.GAL_E1,   SignalType.GAL_E5A,
                             GAL_E1_FREQ, GAL_E5A_FREQ),
    Constellation.BEIDOU:  (SignalType.BDS_B1C,  SignalType.BDS_B2A,
                             BDS_B1C_FREQ, BDS_B2A_FREQ),
}


# ===========================================================================
# Hatch Filter (carrier-smoothed pseudorange)
# ===========================================================================

class HatchFilter:
    """Carrier-smoothed code pseudorange (Hatch 1982).

    Uses carrier-phase increments to smooth the noisier code pseudorange.
    Reduces thermal noise by roughly sqrt(N_smooth) without introducing
    ionospheric divergence (provided the correct combination is used).

    For dual-frequency observations: smoothes the iono-free code combination
    with the iono-free carrier-phase combination (bias-free).

    For single-frequency observations: smoothes with the L1/E1/B1C carrier
    phase, accepting the slow iono divergence (still useful for short arcs
    and when Klobuchar removes most of the bias).
    """

    def __init__(self, n_smooth: int = 100):
        """
        Args:
            n_smooth: Maximum smoothing count (100 ≈ 100 s at 1 Hz).
        """
        self._n_max = n_smooth
        # key → {'smooth': m, 'phase': m, 'lock': s, 'n': int}
        self._state: Dict[str, dict] = {}

    def smooth(self, key: str, code_m: float, phase_m: float,
               lock_s: float, half_cycle: bool) -> float:
        """Return the Hatch-filtered pseudorange.

        Resets automatically on:
          - First observation for this satellite/key
          - Half-cycle ambiguity flag set
          - Lock-time counter reset (cycle slip / signal interruption)
        """
        st = self._state.get(key)

        # Detect cycle slip: lock time decreased by more than the coarsest
        # quantisation step (~24 ms).  A genuine decrease always means the
        # receiver re-acquired the carrier.
        cycle_slip = (
            st is not None
            and lock_s < st['lock'] - 0.030   # 30 ms margin
        )

        if st is None or half_cycle or cycle_slip:
            self._state[key] = {
                'smooth': code_m,
                'phase':  phase_m,
                'lock':   lock_s,
                'n':      1,
            }
            return code_m

        n     = min(st['n'] + 1, self._n_max)
        alpha = 1.0 / n
        delta = phase_m - st['phase']              # carrier-phase increment [m]

        # Hatch recursion
        smooth = alpha * code_m + (1.0 - alpha) * (st['smooth'] + delta)

        self._state[key].update(
            smooth=smooth, phase=phase_m, lock=lock_s, n=n
        )
        return smooth

    def reset(self, key: str) -> None:
        self._state.pop(key, None)

    def clear(self) -> None:
        self._state.clear()


# ===========================================================================
# SPS Engine
# ===========================================================================

class SPSEngine:
    """Standard Positioning Service engine.

    Implements multi-constellation pseudorange positioning using
    Weighted Least Squares with carrier-smoothed pseudoranges.
    """

    def __init__(self, ephemeris_store: EphemerisStore):
        self.eph_store = ephemeris_store
        self.elevation_mask  = ELEVATION_MASK_DEG
        self.max_iterations  = MAX_ITERATIONS
        self.convergence_threshold = CONVERGENCE_THRESHOLD
        self.use_dual_freq   = True   # Form iono-free combination when available

        # Carrier-smoothed pseudorange filter (resets on pipeline restart)
        self._hatch = HatchFilter(n_smooth=100)

        # Warm-start state
        self._last_position       = np.zeros(3)
        self._last_clock_offsets: Dict[Constellation, float] = {}

    # -----------------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------------

    def process_epoch(self, epoch: EpochObservations) -> PositionSolution:
        """Process one epoch of observations to compute position."""
        t_start = time.time()

        solution = PositionSolution(
            timestamp_gps=epoch.timestamp_gps,
            week=epoch.week,
        )

        # Step 1: select best (possibly Hatch-smoothed) pseudorange per sat
        sat_obs = self._select_observations(epoch)
        if len(sat_obs) < 4:
            logger.warning(f"Only {len(sat_obs)} usable satellites (need ≥ 4)")
            solution.fix_type = FixType.NO_FIX
            return solution

        # Step 2: coarse WLS (no tropo/iono — position unknown)
        sat_states, pseudoranges, weights, _ = self._prepare_observations(
            sat_obs, epoch.timestamp_gps, epoch.week
        )
        if len(sat_states) < 4:
            logger.warning(
                f"Coarse pass: only {len(sat_states)} sats after prep "
                f"(had {len(sat_obs)} from select); TOW={epoch.timestamp_gps:.3f}"
            )
            solution.fix_type = FixType.NO_FIX
            return solution

        solve = self._solve_wls(sat_states, pseudoranges, weights,
                                epoch.timestamp_gps)
        if solve is None:
            solution.fix_type = FixType.NO_FIX
            return solution
        x_state, const_indices, H, dz, W = solve

        # Step 3: refine with elevation-dependent corrections (tropo + iono)
        sat_states_r, pseudoranges_r, weights_r, _ = self._prepare_observations(
            sat_obs, epoch.timestamp_gps, epoch.week,
            receiver_pos=x_state[:3]
        )
        if len(sat_states_r) >= 4:
            solve_r = self._solve_wls(sat_states_r, pseudoranges_r, weights_r,
                                      epoch.timestamp_gps)
            if solve_r is not None:
                x_state, const_indices, H, dz, W = solve_r
                sat_states    = sat_states_r
                pseudoranges  = pseudoranges_r
                weights       = weights_r

        # Step 4: residual outlier rejection then final solve
        x_state, const_indices, sat_states, pseudoranges, weights, H, dz, W = \
            self._reject_outliers_and_resolve(
                x_state, const_indices, sat_states, pseudoranges, weights,
                epoch.timestamp_gps
            )

        # Step 5: assemble solution
        pos_ecef = x_state[:3]
        lat, lon, alt = ecef_to_lla(*pos_ecef)

        try:
            Q   = np.linalg.inv(H.T @ W @ H)
            dop = self._compute_dop(Q, lat, lon)
        except np.linalg.LinAlgError:
            Q   = None
            dop = DOPValues()

        H_f, dz_f, _ = self._build_system(
            x_state, sat_states, pseudoranges, weights, const_indices,
            epoch.timestamp_gps
        )
        residuals  = dz_f if dz_f is not None else dz
        n_unknowns = 3 + len(const_indices)
        sigma      = np.sqrt(
            np.sum(residuals**2) / max(len(residuals) - n_unknowns, 1)
        )

        clock_bias_gps = 0.0
        for const, idx in const_indices.items():
            if const == Constellation.GPS:
                clock_bias_gps = x_state[3 + idx] / C

        # Sanity-check the ECEF result before accepting it.
        # A valid position must be within ±500 km of Earth's surface
        # (mean radius 6371 km). If the WLS diverged, reject the solution
        # and clear the warm-start so the next epoch starts from scratch.
        r = float(np.linalg.norm(pos_ecef))
        if not (5_871_000.0 < r < 6_871_000.0):
            logger.warning(
                f"WLS position outside Earth surface (|r|={r/1e3:.0f} km) — "
                "rejecting and clearing warm-start"
            )
            # Diagnostic: dump the satellite states and pseudoranges that caused divergence
            for i, (ss, prc) in enumerate(zip(sat_states, pseudoranges)):
                sp = ss.position_ecef
                geom = float(np.linalg.norm(sp))
                logger.debug(
                    f"  sat[{i}] {ss.sat_id}: pos=({sp[0]/1e6:.3f},{sp[1]/1e6:.3f},{sp[2]/1e6:.3f}) Mm "
                    f"|pos|={geom/1e3:.0f}km  PR_corr={prc/1e3:.0f}km  "
                    f"elev={ss.elevation:.1f}°  cnr={sat_obs.get(ss.sat_id, {}).get('cnr', 0):.0f}dBHz"
                )
            self._last_position      = np.zeros(3)
            self._last_clock_offsets = {}
            solution.fix_type = FixType.NO_FIX
            return solution

        self._last_position      = pos_ecef.copy()
        self._last_clock_offsets = {
            const: x_state[3 + idx] for const, idx in const_indices.items()
        }

        solution.fix_type          = FixType.SPS
        solution.x, solution.y, solution.z = pos_ecef
        solution.latitude          = lat
        solution.longitude         = lon
        solution.altitude          = alt
        solution.clock_bias        = clock_bias_gps
        solution.dop               = dop
        solution.num_satellites    = len(sat_states)
        solution.residuals         = residuals
        solution.covariance        = Q[:3, :3] * sigma**2 if Q is not None else None
        solution.sigma_pos         = sigma
        solution.sigma_horizontal  = np.sqrt(
            (Q[0, 0] + Q[1, 1]) * sigma**2
        ) if Q is not None else sigma
        solution.sigma_vertical    = np.sqrt(
            Q[2, 2] * sigma**2
        ) if Q is not None else sigma
        solution.satellites_used   = [s.sat_id for s in sat_states]
        solution.constellations_used = list(const_indices.keys())
        solution.processing_time_ms = (time.time() - t_start) * 1000

        for const, idx in const_indices.items():
            bias_m = x_state[3 + idx] - clock_bias_gps * C
            if const == Constellation.GLONASS:
                solution.isb_glonass = bias_m / C
            elif const == Constellation.GALILEO:
                solution.isb_galileo = bias_m / C
            elif const == Constellation.BEIDOU:
                solution.isb_beidou  = bias_m / C

        return solution

    # -----------------------------------------------------------------------
    # Observation selection (Hatch filter, BDS GEO exclusion, iono-free)
    # -----------------------------------------------------------------------

    def _select_observations(self, epoch: EpochObservations) -> Dict[str, dict]:
        """Select the best pseudorange per satellite and apply the Hatch filter.

        Returns:
            Dict[sat_id → {
                'pseudorange': float,  # Hatch-smoothed, sat-clock-NOT-yet-removed
                'constellation': Constellation,
                'svn': int,
                'cnr': float,
                'dual_freq': bool,     # True ↔ iono-free combination was formed
            }]
        """
        # Group raw observations by satellite
        sat_groups: Dict[str, List[RawObservation]] = {}
        for obs in epoch.observations:
            if not obs.pseudorange_valid:
                continue
            if not (PR_MIN <= obs.pseudorange <= PR_MAX):
                continue
            sat_groups.setdefault(obs.sat_id, []).append(obs)

        result: Dict[str, dict] = {}

        for sat_id, obs_list in sat_groups.items():
            const = obs_list[0].constellation
            svn   = obs_list[0].svn

            # ----- Exclude BeiDou GEO (wrong orbital model) ----------------
            if const == Constellation.BEIDOU and svn in _BDS_GEO_PRNS:
                self._hatch.reset(sat_id)
                continue

            obs_by_sig = {o.signal: o for o in obs_list}
            dual_freq  = False
            pr: Optional[float] = None

            # ----- Try iono-free dual-frequency combination -----------------
            if self.use_dual_freq and const in _IF_PAIRS:
                pr_val = self._try_ionofree_hatch(
                    sat_id, obs_by_sig, const)
                if pr_val is not None:
                    pr        = pr_val
                    dual_freq = True

            # ----- Single-frequency fallback --------------------------------
            if pr is None:
                best = max(obs_list, key=lambda o: o.cnr)
                # Hatch filter for CDMA constellations (fixed known wavelength)
                if const != Constellation.GLONASS:
                    freq = SIGNAL_FREQUENCY.get(best.signal)
                    if freq and best.carrier_phase_valid:
                        wavelength = C / freq
                        phase_m    = best.carrier_phase * wavelength
                        hkey       = sat_id + '_sf'
                        pr = self._hatch.smooth(
                            hkey, best.pseudorange, phase_m,
                            best.lock_time, best.half_cycle_ambiguity
                        )
                    else:
                        # No carrier — reset and use raw code
                        self._hatch.reset(sat_id + '_sf')
                        pr = best.pseudorange
                else:
                    # GLONASS: FDMA wavelength varies — skip Hatch
                    pr = best.pseudorange

            result[sat_id] = {
                'pseudorange':   pr,
                'constellation': const,
                'svn':           svn,
                'cnr':           max(o.cnr for o in obs_list),
                'dual_freq':     dual_freq,
                # Native constellation timestamp: BDT for BeiDou, GPS TOW for others.
                # Used for t_transmit so satellite positions are computed in the
                # correct time reference regardless of the merged epoch GPS TOW.
                'obs_tow':       obs_list[0].timestamp_gps,
            }

        return result

    def _try_ionofree_hatch(self, sat_id: str,
                             obs_by_sig: dict,
                             constellation: Constellation) -> Optional[float]:
        """Form iono-free code+carrier and apply dual-freq Hatch filter.

        Returns:
            Hatch-smoothed iono-free pseudorange [m], or None if not possible.
        """
        sig1, sig2, f1, f2 = _IF_PAIRS[constellation]
        obs1 = obs_by_sig.get(sig1)
        obs2 = obs_by_sig.get(sig2)

        if not (obs1 and obs2
                and obs1.pseudorange_valid and obs2.pseudorange_valid):
            return None

        # Iono-free code combination
        pr_if = ionofree_combination(obs1.pseudorange, obs2.pseudorange, f1, f2)

        # Iono-free carrier-phase combination [m]
        lambda1, lambda2 = C / f1, C / f2
        cp1_m   = obs1.carrier_phase * lambda1
        cp2_m   = obs2.carrier_phase * lambda2
        cp_if_m = (f1**2 * cp1_m - f2**2 * cp2_m) / (f1**2 - f2**2)

        lock_s    = min(obs1.lock_time, obs2.lock_time)
        half_cyc  = obs1.half_cycle_ambiguity or obs2.half_cycle_ambiguity
        carrier_valid = obs1.carrier_phase_valid and obs2.carrier_phase_valid

        hkey = sat_id + '_2f'
        if carrier_valid:
            return self._hatch.smooth(hkey, pr_if, cp_if_m, lock_s, half_cyc)
        else:
            self._hatch.reset(hkey)
            return pr_if

    # -----------------------------------------------------------------------
    # Observation preparation (satellite state + corrections)
    # -----------------------------------------------------------------------

    def _prepare_observations(self, sat_obs: dict, tow: float, week: int,
                               receiver_pos: Optional[np.ndarray] = None):
        """Compute satellite states and apply atmospheric corrections.

        Returns:
            (sat_states, pseudoranges, weights, constellations_present)
        """
        if receiver_pos is None:
            lp = self._last_position
            r_last = float(np.linalg.norm(lp))
            receiver_pos = lp if (5_871_000.0 < r_last < 6_871_000.0) else None

        sat_states:   List[SatelliteState] = []
        pseudoranges: List[float]          = []
        weights:      List[float]          = []
        constellations: set                = set()

        for sat_id, obs in sat_obs.items():
            const     = obs['constellation']
            svn       = obs['svn']
            pr        = obs['pseudorange']
            dual_freq = obs['dual_freq']

            # GLONASS broadcasts in PZ-90 frame; WGS-84 alignment not yet
            # implemented. Skip GLONASS to avoid corrupting the WLS solution.
            # [EG-C: these next 2 lines can be deleted after testing]
            #if const == Constellation.GLONASS:
            #   continue

            eph = self.eph_store.get_ephemeris(const, svn)
            if eph is None:
                continue

            transit_time = pr / C
            # Use the observation's own native timestamp (BDT for BeiDou,
            # GPS TOW for GPS/Galileo/GLONASS) so the orbit engine gets the
            # correct time reference for satellite position computation.
            t_transmit   = obs['obs_tow'] - transit_time

            # Satellite clock correction.
            # apply_tgd=False for iono-free: TGD already cancelled by combination.
            sat_state = self._compute_sat_state(
                eph, const, t_transmit, apply_tgd=not dual_freq
            )
            if sat_state is None:
                continue

            # Remove satellite clock from pseudorange
            pr_corr = pr + sat_state.clock_bias * C

            # Earth rotation correction
            sat_pos_corr = earth_rotation_correction(
                sat_state.position_ecef, transit_time
            )
            sat_state.x, sat_state.y, sat_state.z = sat_pos_corr

            # Elevation-dependent corrections (available only after coarse fix)
            if receiver_pos is not None:
                elev, azim = compute_elevation_azimuth(
                    receiver_pos, sat_state.position_ecef
                )
                if elev < self.elevation_mask:
                    continue
                sat_state.elevation = elev
                sat_state.azimuth   = azim

                lat, lon, alt = ecef_to_lla(*receiver_pos)

                # Tropospheric delay (always apply)
                tropo = troposphere_correction(elev, lat, alt)
                sat_state.tropo_correction = tropo
                pr_corr -= tropo

                # Ionospheric delay.
                # - dual_freq: already eliminated by iono-free combination → skip
                # - single_freq (including GLONASS fallback): apply Klobuchar
                if not dual_freq:
                    iono = ionosphere_correction(lat, lon, elev, azim, tow)
                    sat_state.iono_correction = iono
                    pr_corr -= iono

            # Hard CNR floor — exclude satellites too weak to contribute
            if obs['cnr'] < CNR_MASK_DBH:
                continue

            w = self._observation_weight(sat_state.elevation, obs['cnr'])

            sat_states.append(sat_state)
            pseudoranges.append(pr_corr)
            weights.append(w)
            constellations.add(const)

        return (sat_states, np.array(pseudoranges),
                np.array(weights), constellations)

    # -----------------------------------------------------------------------
    # WLS solve
    # -----------------------------------------------------------------------

    def _solve_wls(self, sat_states: List[SatelliteState],
                   pseudoranges: np.ndarray,
                   weights: np.ndarray,
                   tow: float):
        """Iterative Weighted Least Squares.

        Returns (x_state, const_indices, H, dz, W) or None on failure.
        """
        consts_present = set(s.constellation for s in sat_states)
        const_indices  = self._build_constellation_indices(consts_present)
        n_unknowns     = 3 + len(const_indices)

        if len(sat_states) < n_unknowns:
            logger.warning(
                f"Under-determined: {len(sat_states)} obs, {n_unknowns} unknowns"
            )
            return None

        x_state = self._initialize_state(const_indices)
        H = dz = W = None

        for _ in range(self.max_iterations):
            H, dz, W = self._build_system(
                x_state, sat_states, pseudoranges, weights,
                const_indices, tow
            )
            if H is None:
                return None

            try:
                HtW = H.T @ W
                dx  = np.linalg.solve(HtW @ H, HtW @ dz)
            except np.linalg.LinAlgError:
                logger.error("Singular normal matrix in WLS")
                return None

            x_state += dx
            if np.linalg.norm(dx[:3]) < self.convergence_threshold:
                break

        return x_state, const_indices, H, dz, W

    def _reject_outliers_and_resolve(
            self, x_state, const_indices, sat_states,
            pseudoranges, weights, tow):
        """Remove gross residual outliers (|dz| > max(30m, 4·σ_MAD)) and re-solve."""
        H, dz, W = self._build_system(
            x_state, sat_states, pseudoranges, weights, const_indices, tow
        )
        if dz is None or len(dz) < 6:
            return x_state, const_indices, sat_states, pseudoranges, weights, H, dz, W

        mad           = np.median(np.abs(dz - np.median(dz)))
        robust_sigma  = 1.4826 * mad if mad > 1e-9 else np.std(dz)
        thresh        = max(30.0, 4.0 * robust_sigma)
        keep          = np.abs(dz) <= thresh

        if np.all(keep):
            return x_state, const_indices, sat_states, pseudoranges, weights, H, dz, W

        kept_states = [s for s, k in zip(sat_states, keep) if k]
        kept_pr     = pseudoranges[keep]
        kept_w      = weights[keep]

        if len(kept_states) < 4:
            return x_state, const_indices, sat_states, pseudoranges, weights, H, dz, W

        solve = self._solve_wls(kept_states, kept_pr, kept_w, tow)
        if solve is None:
            return x_state, const_indices, sat_states, pseudoranges, weights, H, dz, W

        x2, ci2, H2, dz2, W2 = solve
        n_removed = int(np.sum(~keep))
        logger.debug(
            f"Outlier rejection: removed {n_removed} obs (thresh={thresh:.1f}m)"
        )
        return x2, ci2, kept_states, kept_pr, kept_w, H2, dz2, W2

    # -----------------------------------------------------------------------
    # Linearised observation system
    # -----------------------------------------------------------------------

    def _build_system(self, x_state, sat_states, pseudoranges,
                       weights, const_indices, tow):
        """Build H, dz, W for the current state estimate."""
        n_obs      = len(sat_states)
        n_unknowns = 3 + len(const_indices)

        H  = np.zeros((n_obs, n_unknowns))
        dz = np.zeros(n_obs)
        W  = np.diag(weights)
        pos = x_state[:3]

        for i, sat_state in enumerate(sat_states):
            sat_pos = sat_state.position_ecef
            r       = compute_geometric_range(pos, sat_pos)

            if r < 1e3:
                continue   # degenerate geometry — leave row as zero

            # Unit line-of-sight vector (receiver → satellite)
            los = (pos - sat_pos) / r
            H[i, 0] = los[0]
            H[i, 1] = los[1]
            H[i, 2] = los[2]

            ci = const_indices.get(sat_state.constellation)
            if ci is not None:
                H[i, 3 + ci] = 1.0

            clock_offset  = x_state[3 + ci] if ci is not None else 0.0
            pr_computed   = r + clock_offset
            dz[i]         = pseudoranges[i] - pr_computed

        return H, dz, W

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _compute_sat_state(self, eph, constellation: Constellation,
                            t_transmit: float,
                            apply_tgd: bool = True) -> Optional[SatelliteState]:
        if constellation == Constellation.GPS:
            return compute_gps_satellite(eph, t_transmit, apply_tgd=apply_tgd)
        elif constellation == Constellation.GALILEO:
            return compute_galileo_satellite(eph, t_transmit, apply_tgd=apply_tgd)
        elif constellation == Constellation.BEIDOU:
            return compute_beidou_satellite(eph, t_transmit, apply_tgd=apply_tgd)
        elif constellation == Constellation.GLONASS:
            # GLONASS clock has no TGD in the broadcast message
            return compute_glonass_satellite(eph, t_transmit)
        return None

    def _build_constellation_indices(self,
                                      constellations: set) -> Dict[Constellation, int]:
        """Assign a state-vector index to each constellation present."""
        ordered = sorted(constellations, key=lambda c: c.value)
        return {const: i for i, const in enumerate(ordered)}

    def _initialize_state(self, const_indices: dict) -> np.ndarray:
        n = 3 + len(const_indices)
        x = np.zeros(n)
        if np.any(self._last_position != 0):
            x[:3] = self._last_position
        for const, idx in const_indices.items():
            if const in self._last_clock_offsets:
                x[3 + idx] = self._last_clock_offsets[const]
        return x

    def _observation_weight(self, elevation_deg: float, cnr: float) -> float:
        """Elevation-sine-squared weighting combined with CNR quality factor."""
        if elevation_deg <= 0:
            return 0.1   # unknown elevation — low weight

        sin_el = np.sin(np.radians(elevation_deg))
        w_elev = sin_el ** 2

        # Normalise CNR to (0.1, 1.0] over a typical 20–55 dB-Hz range
        w_cnr = float(np.clip((cnr - 20.0) / 35.0, 0.1, 1.0))

        return w_elev * w_cnr

    def _compute_dop(self, Q: np.ndarray,
                      lat_deg: float, lon_deg: float) -> DOPValues:
        """DOP values from the cofactor matrix Q (= inv(H^T W H))."""
        dop = DOPValues()
        try:
            dop.gdop = float(np.sqrt(np.trace(Q)))
            dop.pdop = float(np.sqrt(Q[0, 0] + Q[1, 1] + Q[2, 2]))
            if Q.shape[0] > 3:
                dop.tdop = float(np.sqrt(Q[3, 3]))

            lat = np.radians(lat_deg)
            lon = np.radians(lon_deg)
            sl, cl = np.sin(lat), np.cos(lat)
            sn, cn = np.sin(lon), np.cos(lon)

            R = np.array([
                [-sn,       cn,      0.0],
                [-sl * cn, -sl * sn, cl ],
                [ cl * cn,  cl * sn, sl ],
            ])
            Q_enu    = R @ Q[:3, :3] @ R.T
            dop.hdop = float(np.sqrt(Q_enu[0, 0] + Q_enu[1, 1]))
            dop.vdop = float(np.sqrt(Q_enu[2, 2]))
        except Exception as e:
            logger.warning(f"DOP computation error: {e}")
        return dop
