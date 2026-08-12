from __future__ import annotations

from dataclasses import dataclass

from .domain import BoxObservation, Destination, PickTask, TaskStatus


@dataclass(frozen=True, slots=True)
class RoutingPolicy:
    minimum_confidence: float
    reservation_horizon_s: float

    def __post_init__(self) -> None:
        if not 0.0 < self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be in (0, 1]")
        if self.reservation_horizon_s <= 0:
            raise ValueError("reservation_horizon_s must be positive")


class TaskCoordinator:
    """Owns box-to-robot reservations; array order is never used as identity."""

    def __init__(
        self,
        policy: RoutingPolicy,
        robot_by_destination: dict[Destination, str],
        pick_zone_by_destination_m: dict[Destination, float],
    ) -> None:
        required = {Destination.RED, Destination.BLUE, Destination.REJECT}
        if set(robot_by_destination) != required:
            raise ValueError("a robot or reject handler is required for every destination")
        if set(pick_zone_by_destination_m) != required:
            raise ValueError("a pick zone is required for every destination")
        self._policy = policy
        self._robot_by_destination = dict(robot_by_destination)
        self._pick_zone_by_destination_m = dict(pick_zone_by_destination_m)
        self._tasks: dict[str, PickTask] = {}

    def reserve(self, observation: BoxObservation) -> PickTask:
        destination = observation.destination
        if observation.confidence < self._policy.minimum_confidence:
            destination = Destination.REJECT
            observation = observation.with_destination(destination)

        existing = self._tasks.get(observation.box_id)
        if existing is not None:
            if existing.status in {TaskStatus.PICKED, TaskStatus.CANCELLED}:
                raise ValueError(f"terminal task {observation.box_id} cannot be updated")
            if existing.destination != destination:
                raise ValueError(
                    f"destination for reserved box {observation.box_id} cannot change"
                )
            updated = existing.with_observation(observation)
            self._tasks[observation.box_id] = updated
            return updated

        task = PickTask(
            box_id=observation.box_id,
            destination=destination,
            robot_name=self._robot_by_destination[destination],
            pick_zone_center_m=self._pick_zone_by_destination_m[destination],
            observation=observation,
        )
        self._tasks[task.box_id] = task
        return task

    def tasks_due_for(self, robot_name: str, now_s: float) -> list[PickTask]:
        due = [
            task
            for task in self._tasks.values()
            if task.robot_name == robot_name
            and task.status in {TaskStatus.RESERVED, TaskStatus.ACTIVE}
            and task.eta_s(now_s) <= self._policy.reservation_horizon_s
        ]
        return sorted(due, key=lambda task: (task.eta_s(now_s), task.box_id))

    def set_status(self, box_id: str, status: TaskStatus) -> PickTask:
        task = self._tasks[box_id]
        allowed = {
            TaskStatus.RESERVED: {TaskStatus.ACTIVE, TaskStatus.FAILED, TaskStatus.CANCELLED},
            TaskStatus.ACTIVE: {TaskStatus.PICKED, TaskStatus.FAILED, TaskStatus.CANCELLED},
            TaskStatus.FAILED: {TaskStatus.RESERVED, TaskStatus.CANCELLED},
            TaskStatus.PICKED: set(),
            TaskStatus.CANCELLED: set(),
        }
        if status not in allowed[task.status]:
            raise ValueError(f"invalid task transition: {task.status} -> {status}")
        updated = task.with_status(status)
        self._tasks[box_id] = updated
        return updated

    def get(self, box_id: str) -> PickTask:
        return self._tasks[box_id]

