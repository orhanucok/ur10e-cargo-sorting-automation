from __future__ import annotations

from dataclasses import dataclass, replace
from enum import StrEnum
import math


class Destination(StrEnum):
    RED = "red"
    BLUE = "blue"
    REJECT = "reject"


class BoxState(StrEnum):
    ON_CONVEYOR = "on_conveyor"
    CLASSIFIED = "classified"
    RESERVED = "reserved"
    ENTERING_PICK_ZONE = "entering_pick_zone"
    GRASPED = "grasped"
    SORTED = "sorted"
    REJECTED = "rejected"
    LOST = "lost"


class TaskStatus(StrEnum):
    RESERVED = "reserved"
    ACTIVE = "active"
    PICKED = "picked"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True, slots=True)
class BoxObservation:
    box_id: str
    destination: Destination
    confidence: float
    belt_position_m: float
    belt_velocity_m_s: float
    observed_at_s: float

    def __post_init__(self) -> None:
        if not self.box_id.strip():
            raise ValueError("box_id must not be empty")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        for value, label in (
            (self.belt_position_m, "belt_position_m"),
            (self.belt_velocity_m_s, "belt_velocity_m_s"),
            (self.observed_at_s, "observed_at_s"),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{label} must be finite")
        if self.belt_velocity_m_s < 0:
            raise ValueError("reverse belt motion is not enabled")

    def predicted_position(self, at_s: float) -> float:
        if at_s < self.observed_at_s:
            raise ValueError("prediction time cannot precede observation time")
        return self.belt_position_m + self.belt_velocity_m_s * (
            at_s - self.observed_at_s
        )

    def with_destination(self, destination: Destination) -> BoxObservation:
        return replace(self, destination=destination)


@dataclass(frozen=True, slots=True)
class PickTask:
    box_id: str
    destination: Destination
    robot_name: str
    pick_zone_center_m: float
    observation: BoxObservation
    status: TaskStatus = TaskStatus.RESERVED

    def eta_s(self, now_s: float) -> float:
        position = self.observation.predicted_position(now_s)
        remaining = self.pick_zone_center_m - position
        if remaining <= 0:
            return 0.0
        velocity = self.observation.belt_velocity_m_s
        return math.inf if velocity <= 0 else remaining / velocity

    def with_observation(self, observation: BoxObservation) -> PickTask:
        return replace(self, observation=observation)

    def with_status(self, status: TaskStatus) -> PickTask:
        return replace(self, status=status)

