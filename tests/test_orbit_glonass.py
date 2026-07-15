"""
Integration tests for the PZ-90(.11) -> WGS-84 transform as wired into
`orbit.compute_glonass_satellite`.

These tests deliberately don't re-verify the transform's own math (that's
covered by tests/test_coordinates.py) — they verify the *integration
point*: that compute_glonass_satellite actually calls the transform, at
the right place in the pipeline (after RK4 propagation, not before),
on both position and velocity, without disturbing the clock terms, and
without silently swallowing failures.
"""

import numpy as np
import pytest

from gnss_positioning.core.data_types import GLONASSEphemeris, Constellation
from gnss_positioning.engines import orbit
from gnss_positioning.engines.orbit import (
    compute_glonass_satellite,
    compute_gps_satellite,
)
from gnss_positioning.utils.coordinates import (
    pz90_to_wgs84,
    pz90_to_wgs84_velocity,
)


def _make_glonass_eph(**overrides) -> GLONASSEphemeris:
    """A realistic GLONASS ephemeris fixture. tb=0.0 (Moscow midnight)
    by default, paired with T_TRANSMIT_ZERO_DT below so the RK4 loop
    executes zero steps and the propagated state exactly equals the
    input state — isolating the transform from integration error."""
    defaults = dict(
        svn=7,
        freq_channel=1,
        tb=0.0,
        x=1.2e7, y=-1.9e7, z=1.3e7,
        vx=1500.0, vy=-2200.0, vz=2100.0,
        ax=0.0, ay=0.0, az=0.0,
        tau_n=-1.234e-5,
        gamma_n=3.0e-12,
        health=0, age=0, nt=1, n4=1,
    )
    defaults.update(overrides)
    return GLONASSEphemeris(**defaults)


# t_transmit chosen so moscow_tod = (t_transmit + 10782) % 86400 == 0,
# matching tb=0.0 above exactly -> dt = 0 -> RK4 loop is skipped.
T_TRANSMIT_ZERO_DT = 86400.0 - 10782.0  # 75618.0

# A t_transmit 300 s later -> dt = 300 s -> RK4 actually integrates.
T_TRANSMIT_NONZERO_DT = T_TRANSMIT_ZERO_DT + 300.0


class TestTransformIsApplied:

    def test_position_matches_transform_applied_to_raw_state(self):
        """With dt=0, no integration happens, so the propagated PZ-90
        state is exactly the ephemeris's own (x, y, z). The returned
        SatelliteState must equal pz90_to_wgs84 of that raw state —
        an exact, non-approximate check."""
        eph = _make_glonass_eph()
        result = compute_glonass_satellite(eph, T_TRANSMIT_ZERO_DT)

        expected_pos = pz90_to_wgs84(eph.x, eph.y, eph.z)

        assert result is not None
        np.testing.assert_array_equal(
            [result.x, result.y, result.z], expected_pos
        )

    def test_velocity_matches_transform_applied_to_raw_state(self):
        eph = _make_glonass_eph()
        result = compute_glonass_satellite(eph, T_TRANSMIT_ZERO_DT)

        expected_vel = pz90_to_wgs84_velocity(eph.vx, eph.vy, eph.vz)

        assert result is not None
        np.testing.assert_array_equal(
            [result.vx, result.vy, result.vz], expected_vel
        )

    def test_output_differs_from_untransformed_pz90_position(self):
        """Regression guard: if someone removes the transform call,
        this test fails because the output would equal the raw PZ-90
        ephemeris position instead of the WGS-84 one."""
        eph = _make_glonass_eph()
        result = compute_glonass_satellite(eph, T_TRANSMIT_ZERO_DT)

        assert result is not None
        raw_pz90 = np.array([eph.x, eph.y, eph.z])
        transformed = np.array([result.x, result.y, result.z])
        assert not np.array_equal(raw_pz90, transformed)


class TestTransformOrderOfOperations:
    """Verify the transform is applied to the *propagated* state, not
    the raw ephemeris input — order matters, since rotating first and
    integrating in the wrong frame would silently corrupt everything.
    """

    def test_transform_called_after_propagation_not_before(self, monkeypatch):
        # Replace the transform with identity (minus translation, so
        # we can tell it ran) to capture the pre-transform propagated
        # state, then compare against a second real run.
        captured = {}

        def identity_pos(x, y, z):
            captured['pos'] = np.array([x, y, z])
            return np.array([x, y, z])

        def identity_vel(vx, vy, vz):
            captured['vel'] = np.array([vx, vy, vz])
            return np.array([vx, vy, vz])

        monkeypatch.setattr(orbit, 'pz90_to_wgs84', identity_pos)
        monkeypatch.setattr(orbit, 'pz90_to_wgs84_velocity', identity_vel)

        eph = _make_glonass_eph()
        untransformed_result = compute_glonass_satellite(
            eph, T_TRANSMIT_NONZERO_DT
        )
        assert untransformed_result is not None

        # With dt != 0 the RK4 loop ran, so the propagated state must
        # differ from the raw ephemeris input.
        raw = np.array([eph.x, eph.y, eph.z])
        assert not np.allclose(captured['pos'], raw), (
            "Transform received the raw ephemeris state, not the "
            "RK4-propagated state — it's being applied before "
            "propagation instead of after."
        )

        # Now run for real and confirm applying the transform to the
        # captured (pre-transform) state reproduces the real output.
        monkeypatch.undo()
        real_result = compute_glonass_satellite(eph, T_TRANSMIT_NONZERO_DT)
        expected_pos = pz90_to_wgs84(*captured['pos'])
        np.testing.assert_allclose(
            [real_result.x, real_result.y, real_result.z],
            expected_pos, atol=1e-6
        )

    def test_transform_functions_are_actually_invoked(self, monkeypatch):
        """Guard against a future refactor that quietly deletes the
        call site — assert the transform is on the call path at all."""
        calls = {'pos': 0, 'vel': 0}
        orig_pos, orig_vel = pz90_to_wgs84, pz90_to_wgs84_velocity

        def counting_pos(x, y, z):
            calls['pos'] += 1
            return orig_pos(x, y, z)

        def counting_vel(vx, vy, vz):
            calls['vel'] += 1
            return orig_vel(vx, vy, vz)

        monkeypatch.setattr(orbit, 'pz90_to_wgs84', counting_pos)
        monkeypatch.setattr(orbit, 'pz90_to_wgs84_velocity', counting_vel)

        eph = _make_glonass_eph()
        compute_glonass_satellite(eph, T_TRANSMIT_ZERO_DT)

        assert calls['pos'] == 1
        assert calls['vel'] == 1


class TestClockTermsUnaffected:
    """The transform must only touch geometry — clock bias/drift are
    computed independently and must pass through untouched."""

    def test_clock_bias_ignores_position_transform(self):
        eph = _make_glonass_eph(tau_n=-9.876e-5, gamma_n=1.5e-11)
        result = compute_glonass_satellite(eph, T_TRANSMIT_ZERO_DT)

        assert result is not None
        # dt = 0 at T_TRANSMIT_ZERO_DT, so clock_bias = -tau_n exactly.
        assert result.clock_bias == pytest.approx(-eph.tau_n, abs=1e-15)
        assert result.clock_drift == pytest.approx(eph.gamma_n, abs=1e-18)

    def test_clock_bias_unaffected_by_which_transform_is_active(
        self, monkeypatch
    ):
        """Swapping the position transform for identity must not
        change the clock terms at all — they're computed from
        completely different ephemeris fields."""
        eph = _make_glonass_eph()

        result_real = compute_glonass_satellite(eph, T_TRANSMIT_ZERO_DT)

        monkeypatch.setattr(
            orbit, 'pz90_to_wgs84', lambda x, y, z: np.array([x, y, z])
        )
        result_identity = compute_glonass_satellite(eph, T_TRANSMIT_ZERO_DT)

        assert result_real.clock_bias == result_identity.clock_bias
        assert result_real.clock_drift == result_identity.clock_drift


class TestOtherConstellationsUnaffected:
    """Guard against the PZ-90 transform accidentally leaking into the
    Keplerian (GPS/Galileo/BeiDou) path, which is already in WGS-84 and
    must never be rotated."""

    def test_gps_path_does_not_call_pz90_transform(self, monkeypatch):
        called = {'flag': False}

        def flagging(*args, **kwargs):
            called['flag'] = True
            return np.array(args)

        monkeypatch.setattr(orbit, 'pz90_to_wgs84', flagging)
        monkeypatch.setattr(orbit, 'pz90_to_wgs84_velocity', flagging)

        from gnss_positioning.core.data_types import GPSEphemeris
        # Minimal valid-ish GPS ephemeris; only fields touched by
        # _compute_keplerian need real values.
        gps_eph = GPSEphemeris(
            svn=5, week=2300, toe=100000.0, toc=100000.0,
            sqrt_a=5153.7, e=0.01, i0=0.95, omega0=1.0, omega=0.5,
            m0=0.2, delta_n=4.5e-9, idot=1e-10, omega_dot=-8e-9,
            cus=1e-6, cuc=1e-6, crs=10.0, crc=10.0, cis=1e-7, cic=1e-7,
            af0=1e-5, af1=1e-11, af2=0.0, tgd=1e-8,
        )
        result = compute_gps_satellite(gps_eph, 100050.0)

        assert result is not None
        assert called['flag'] is False, (
            "PZ-90 transform was invoked on the GPS (Keplerian) path — "
            "it must only apply to GLONASS."
        )


class TestFailureHandling:
    """A broken/raising transform must degrade safely (return None,
    matching the existing behaviour for any other error in this
    function) rather than propagate an unhandled exception up into
    the WLS solver and crash the epoch."""

    def test_transform_exception_yields_none_not_a_crash(self, monkeypatch):
        def raiser(x, y, z):
            raise ValueError("simulated transform failure")

        monkeypatch.setattr(orbit, 'pz90_to_wgs84', raiser)

        eph = _make_glonass_eph()
        result = compute_glonass_satellite(eph, T_TRANSMIT_ZERO_DT)

        assert result is None
