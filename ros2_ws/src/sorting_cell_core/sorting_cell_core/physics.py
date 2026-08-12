from __future__ import annotations

from dataclasses import dataclass
import math


GRAVITY_M_S2 = 9.80665


@dataclass(frozen=True, slots=True)
class Cuboid:
    size_x_m: float
    size_y_m: float
    size_z_m: float
    mass_kg: float

    def __post_init__(self) -> None:
        if min(self.size_x_m, self.size_y_m, self.size_z_m, self.mass_kg) <= 0:
            raise ValueError("cuboid dimensions and mass must be positive")

    @property
    def inertia_kg_m2(self) -> tuple[float, float, float]:
        x, y, z, mass = self.size_x_m, self.size_y_m, self.size_z_m, self.mass_kg
        return (
            mass * (y * y + z * z) / 12.0,
            mass * (x * x + z * z) / 12.0,
            mass * (x * x + y * y) / 12.0,
        )


@dataclass(frozen=True, slots=True)
class VacuumGripper:
    cup_count: int
    cup_diameter_m: float
    maximum_vacuum_kpa: float
    seal_quality: float
    tangential_friction: float
    maximum_total_moment_nm: float

    def __post_init__(self) -> None:
        if self.cup_count <= 0:
            raise ValueError("cup_count must be positive")
        if self.cup_diameter_m <= 0 or self.maximum_vacuum_kpa <= 0:
            raise ValueError("vacuum geometry and pressure must be positive")
        if not 0.0 <= self.seal_quality <= 1.0:
            raise ValueError("seal_quality must be in [0, 1]")
        if self.tangential_friction <= 0 or self.maximum_total_moment_nm <= 0:
            raise ValueError("friction and moment capacity must be positive")

    @property
    def total_cup_area_m2(self) -> float:
        radius = self.cup_diameter_m / 2.0
        return self.cup_count * math.pi * radius * radius

    @property
    def normal_capacity_n(self) -> float:
        return self.maximum_vacuum_kpa * 1000.0 * self.total_cup_area_m2 * self.seal_quality

    @property
    def tangential_capacity_n(self) -> float:
        return self.normal_capacity_n * self.tangential_friction

    def evaluate_hold(
        self,
        payload_mass_kg: float,
        normal_acceleration_m_s2: float = 0.0,
        tangential_acceleration_m_s2: float = 0.0,
        applied_moment_nm: float = 0.0,
        safety_factor: float = 2.0,
    ) -> tuple[bool, tuple[str, ...]]:
        if payload_mass_kg <= 0 or safety_factor < 1.0:
            raise ValueError("payload must be positive and safety_factor at least 1")

        normal_load = payload_mass_kg * (GRAVITY_M_S2 + max(0.0, normal_acceleration_m_s2))
        tangential_load = payload_mass_kg * abs(tangential_acceleration_m_s2)
        failures: list[str] = []

        if normal_load * safety_factor > self.normal_capacity_n:
            failures.append("normal_capacity")
        if tangential_load * safety_factor > self.tangential_capacity_n:
            failures.append("tangential_capacity")
        if abs(applied_moment_nm) * safety_factor > self.maximum_total_moment_nm:
            failures.append("moment_capacity")
        return (not failures, tuple(failures))

