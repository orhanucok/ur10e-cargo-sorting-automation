import pytest

from sorting_cell_core.coordinator import RoutingPolicy, TaskCoordinator
from sorting_cell_core.domain import BoxObservation, Destination, TaskStatus


def coordinator() -> TaskCoordinator:
    return TaskCoordinator(
        policy=RoutingPolicy(minimum_confidence=0.985, reservation_horizon_s=8.0),
        robot_by_destination={
            Destination.RED: "ur10e_red",
            Destination.BLUE: "ur10e_blue",
            Destination.REJECT: "reject_chute",
        },
        pick_zone_by_destination_m={
            Destination.RED: 3.10,
            Destination.BLUE: 4.35,
            Destination.REJECT: 5.00,
        },
    )


def observation(box_id: str, destination: Destination, confidence: float = 0.999) -> BoxObservation:
    return BoxObservation(
        box_id=box_id,
        destination=destination,
        confidence=confidence,
        belt_position_m=2.25,
        belt_velocity_m_s=0.15,
        observed_at_s=10.0,
    )


def test_color_assigns_exact_robot() -> None:
    service = coordinator()
    assert service.reserve(observation("BOX_1", Destination.BLUE)).robot_name == "ur10e_blue"
    assert service.reserve(observation("BOX_2", Destination.RED)).robot_name == "ur10e_red"


def test_low_confidence_is_rejected_instead_of_guessed() -> None:
    task = coordinator().reserve(observation("BOX_3", Destination.BLUE, confidence=0.70))
    assert task.destination is Destination.REJECT
    assert task.robot_name == "reject_chute"


def test_reobservation_updates_position_without_changing_owner() -> None:
    service = coordinator()
    first = service.reserve(observation("BOX_4", Destination.BLUE))
    later = BoxObservation(
        box_id="BOX_4",
        destination=Destination.BLUE,
        confidence=0.998,
        belt_position_m=2.55,
        belt_velocity_m_s=0.15,
        observed_at_s=12.0,
    )
    updated = service.reserve(later)
    assert updated.robot_name == first.robot_name
    assert updated.observation.belt_position_m == pytest.approx(2.55)


def test_reserved_destination_cannot_flip() -> None:
    service = coordinator()
    service.reserve(observation("BOX_5", Destination.BLUE))
    with pytest.raises(ValueError, match="cannot change"):
        service.reserve(observation("BOX_5", Destination.RED))


def test_state_machine_rejects_skipping_active_state() -> None:
    service = coordinator()
    service.reserve(observation("BOX_6", Destination.RED))
    with pytest.raises(ValueError, match="invalid task transition"):
        service.set_status("BOX_6", TaskStatus.PICKED)


def test_due_queue_is_time_ordered_not_detection_ordered() -> None:
    service = coordinator()
    service.reserve(observation("BOX_LATE", Destination.RED))
    early = BoxObservation(
        box_id="BOX_EARLY",
        destination=Destination.RED,
        confidence=0.999,
        belt_position_m=2.70,
        belt_velocity_m_s=0.15,
        observed_at_s=10.0,
    )
    service.reserve(early)
    due = service.tasks_due_for("ur10e_red", now_s=10.0)
    assert [task.box_id for task in due] == ["BOX_EARLY", "BOX_LATE"]

