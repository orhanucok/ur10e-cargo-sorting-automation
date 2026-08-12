import pytest

from sorting_cell_core.physics import Cuboid, VacuumGripper


@pytest.fixture
def epick() -> VacuumGripper:
    return VacuumGripper(
        cup_count=4,
        cup_diameter_m=0.055,
        maximum_vacuum_kpa=80.0,
        seal_quality=0.72,
        tangential_friction=0.65,
        maximum_total_moment_nm=150.0,
    )


def test_cuboid_inertia_uses_real_dimensions() -> None:
    inertia = Cuboid(0.40, 0.30, 0.25, 8.0).inertia_kg_m2
    assert inertia == pytest.approx((0.1016666667, 0.1483333333, 0.1666666667))


def test_nominal_box_is_within_vacuum_capacity(epick: VacuumGripper) -> None:
    safe, failures = epick.evaluate_hold(
        payload_mass_kg=4.0,
        normal_acceleration_m_s2=1.0,
        tangential_acceleration_m_s2=1.5,
        applied_moment_nm=8.0,
    )
    assert safe
    assert failures == ()


def test_bad_seal_causes_physical_failure() -> None:
    leaky = VacuumGripper(
        cup_count=1,
        cup_diameter_m=0.040,
        maximum_vacuum_kpa=30.0,
        seal_quality=0.25,
        tangential_friction=0.30,
        maximum_total_moment_nm=5.0,
    )
    safe, failures = leaky.evaluate_hold(
        payload_mass_kg=8.0,
        normal_acceleration_m_s2=2.0,
        tangential_acceleration_m_s2=4.0,
        applied_moment_nm=4.0,
    )
    assert not safe
    assert set(failures) == {"normal_capacity", "tangential_capacity", "moment_capacity"}

