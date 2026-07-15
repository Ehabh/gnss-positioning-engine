"""
Unit tests for the PZ-90(.11) -> WGS-84 Helmert transform.

Reference: EPSG:15843 "PZ-90 to WGS 84 (1)", Coordinate Frame rotation
method, params (dX, dY, dZ, rX, rY, rZ, scale) = (0, 0, 1.5 m, 0, 0,
-0.076", 0), stated accuracy 1.5 m.
"""

import numpy as np
import pytest

from gnss_positioning.utils.coordinates import (
    pz90_to_wgs84,
    pz90_to_wgs84_velocity,
    PZ90_DZ,
    PZ90_RZ_RAD,
)

# Approximate GLONASS orbital radius (~19,140 km altitude + Earth radius)
GLONASS_ORBIT_RADIUS_M = 25_508_000.0


def _explicit_helmert_matrix() -> np.ndarray:
    """Build the full 3x3 rotation matrix independently of the
    production code's simplified formula, so tests don't just
    check the implementation against itself.

    Coordinate Frame convention, small-angle approximation:
        R = I + [[0, rz, -ry], [-rz, 0, rx], [ry, -rx, 0]]
    with rx = ry = 0.
    """
    rz = PZ90_RZ_RAD
    return np.array([
        [1.0,  rz, 0.0],
        [-rz, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _reference_transform(x, y, z):
    """Independent reference implementation for cross-checking."""
    R = _explicit_helmert_matrix()
    translation = np.array([0.0, 0.0, PZ90_DZ])
    return R @ np.array([x, y, z]) + translation


class TestPZ90ToWGS84Position:

    def test_returns_3_element_array(self):
        result = pz90_to_wgs84(1.0, 2.0, 3.0)
        assert isinstance(result, np.ndarray)
        assert result.shape == (3,)

    def test_origin_translates_by_dz_only(self):
        """At the origin, rotation has no effect — only dz translation
        should show up."""
        result = pz90_to_wgs84(0.0, 0.0, 0.0)
        np.testing.assert_allclose(result, [0.0, 0.0, PZ90_DZ], atol=1e-12)

    def test_matches_independent_reference_matrix(self):
        """Cross-check the production formula against a matrix built
        a different way, at a realistic GLONASS orbital position."""
        x, y, z = GLONASS_ORBIT_RADIUS_M * np.array(
            [0.6, -0.5, 0.624]  # arbitrary unit-ish direction
        )
        result = pz90_to_wgs84(x, y, z)
        expected = _reference_transform(x, y, z)
        np.testing.assert_allclose(result, expected, atol=1e-9)

    @pytest.mark.parametrize("x,y,z", [
        (GLONASS_ORBIT_RADIUS_M, 0.0, 0.0),
        (0.0, GLONASS_ORBIT_RADIUS_M, 0.0),
        (0.0, 0.0, GLONASS_ORBIT_RADIUS_M),
        (-GLONASS_ORBIT_RADIUS_M, 0.0, 0.0),
        (18_000_000.0, -12_000_000.0, 9_500_000.0),
    ])
    def test_correction_magnitude_within_stated_accuracy(self, x, y, z):
        """The EPSG entry states ~1.5 m accuracy for this transform.
        The applied correction itself (not the residual error) should
        be small and bounded — a regression here (e.g. degrees instead
        of arc-seconds) would blow this up by orders of magnitude."""
        result = pz90_to_wgs84(x, y, z)
        original = np.array([x, y, z])
        correction = np.linalg.norm(result - original)
        # dz alone is 1.5 m; rotation term at GLONASS-orbit radius is
        # ~|rz_rad| * r ~ 3.7e-7 * 2.55e7 ~ 9.4 m. Bound generously.
        assert correction < 15.0, (
            f"Correction magnitude {correction:.3f} m is implausibly "
            f"large for a sub-arcsecond rotation + 1.5 m translation"
        )

    def test_z_component_shift_is_dz_plus_second_order(self):
        """z_w = z + dz exactly (z has no rotation dependence in this
        transform since rx = ry = 0)."""
        x, y, z = 1.0e7, -2.0e7, 3.0e6
        result = pz90_to_wgs84(x, y, z)
        assert result[2] == pytest.approx(z + PZ90_DZ, abs=1e-9)

    def test_rotation_direction_is_deterministic(self):
        """Sanity check on rotation sign/direction, per EPSG:1032
        (Coordinate Frame rotation, geocentric domain):
            Yt = -rz*Xs + Ys
        For a point on the +X axis with rz < 0, y_w = -rz*x > 0, so
        the point should move toward +Y."""
        x, y, z = GLONASS_ORBIT_RADIUS_M, 0.0, 0.0
        result = pz90_to_wgs84(x, y, z)
        assert PZ90_RZ_RAD < 0  # sanity: confirm the sign we're relying on
        assert result[1] > 0  # y should move positive given rz < 0

    def test_zero_rotation_reduces_to_pure_translation(self, monkeypatch):
        """With rz forced to 0, the transform must be a pure Z
        translation regardless of input."""
        import gnss_positioning.utils.coordinates as coords
        monkeypatch.setattr(coords, "PZ90_RZ_RAD", 0.0)
        x, y, z = 1.23e7, -4.56e6, 7.89e6
        result = coords.pz90_to_wgs84(x, y, z)
        np.testing.assert_allclose(
            result, [x, y, z + PZ90_DZ], atol=1e-9
        )

    def test_not_a_no_op(self):
        """Guard against an accidental identity-function regression —
        the transform must actually change the input by a measurable
        amount. Uses an absolute check: at GLONASS-orbit scale
        (~2.5e7 m), np.allclose's default relative tolerance would
        happily call a multi-metre shift 'close', so it can't be used
        here."""
        x, y, z = GLONASS_ORBIT_RADIUS_M, 5_000_000.0, 3_000_000.0
        result = pz90_to_wgs84(x, y, z)
        displacement = np.linalg.norm(result - np.array([x, y, z]))
        assert displacement > 0.5  # dz alone (1.5 m) guarantees this


class TestPZ90ToWGS84Velocity:

    def test_returns_3_element_array(self):
        result = pz90_to_wgs84_velocity(100.0, -200.0, 50.0)
        assert isinstance(result, np.ndarray)
        assert result.shape == (3,)

    def test_no_translation_applied_to_velocity(self):
        """Velocity is a rate, not a position — dz must NOT leak in."""
        result = pz90_to_wgs84_velocity(0.0, 0.0, 0.0)
        np.testing.assert_allclose(result, [0.0, 0.0, 0.0], atol=1e-12)

    def test_z_component_unaffected_by_rotation(self):
        """rz only mixes x/y; vz must pass through unchanged."""
        vx, vy, vz = 1500.0, -3000.0, 800.0  # ~typical GLONASS MEO speeds
        result = pz90_to_wgs84_velocity(vx, vy, vz)
        assert result[2] == pytest.approx(vz, abs=1e-12)

    def test_matches_rotation_only_reference(self):
        """Velocity transform should equal the rotation-only part of
        the position transform (no translation term)."""
        vx, vy, vz = 1500.0, -3000.0, 800.0
        R = _explicit_helmert_matrix()
        expected = R @ np.array([vx, vy, vz])
        result = pz90_to_wgs84_velocity(vx, vy, vz)
        np.testing.assert_allclose(result, expected, atol=1e-9)

    def test_correction_magnitude_is_small_at_orbital_speed(self):
        """At GLONASS orbital speed (~3.9 km/s), the rotation-induced
        velocity correction should be sub-mm/s — this is a tiny effect,
        included mainly for Doppler/range-rate consistency."""
        speed = 3900.0
        vx, vy, vz = speed, 0.0, 0.0
        result = pz90_to_wgs84_velocity(vx, vy, vz)
        correction = np.linalg.norm(result - np.array([vx, vy, vz]))
        assert correction < 0.01, (
            f"Velocity correction {correction:.6f} m/s is larger than "
            f"expected for a sub-arcsecond rotation rate"
        )


class TestConsistencyWithExistingConversions:
    """Cross-check against the module's other coordinate functions to
    make sure the new transform composes sanely with them."""

    def test_transformed_point_still_near_earth_surface_scale(self):
        """A GLONASS-orbit-scale point, after PZ-90->WGS-84, should
        still have a norm consistent with the same orbital radius
        (i.e. the transform must not distort the magnitude by more
        than a few metres)."""
        x, y, z = GLONASS_ORBIT_RADIUS_M * np.array([0.6, -0.5, 0.624])
        result = pz90_to_wgs84(x, y, z)
        original_norm = np.linalg.norm([x, y, z])
        new_norm = np.linalg.norm(result)
        assert abs(new_norm - original_norm) < 15.0

    def test_deterministic_and_stateless(self):
        """Calling twice with the same input must give identical
        output — no hidden state, no reliance on call order."""
        args = (12_345_678.9, -9_876_543.2, 4_321_000.5)
        r1 = pz90_to_wgs84(*args)
        r2 = pz90_to_wgs84(*args)
        np.testing.assert_array_equal(r1, r2)
