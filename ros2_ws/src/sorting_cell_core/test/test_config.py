from pathlib import Path

import pytest

from sorting_cell_core.config import load_yaml, validate_cell_config


ROOT = Path(__file__).resolve().parents[4]


def test_checked_in_cell_configuration_is_valid() -> None:
    config = load_yaml(ROOT / "config" / "cell.yaml")
    validate_cell_config(config)


def test_local_camera_cannot_become_routing_owner() -> None:
    config = load_yaml(ROOT / "config" / "cell.yaml")
    config["cameras"]["local"]["decision_owner"] = True
    with pytest.raises(ValueError, match="local cameras"):
        validate_cell_config(config)


def test_robot_base_below_conveyor_clearance_is_rejected() -> None:
    config = load_yaml(ROOT / "config" / "cell.yaml")
    config["robots"][1]["base_position_m"][2] = 0.70
    with pytest.raises(ValueError, match="150 mm clearance"):
        validate_cell_config(config)


def test_infeed_supply_pool_must_sustain_continuous_circulation() -> None:
    config = load_yaml(ROOT / "config" / "cell.yaml")
    config["infeed_supply"]["pool_size"] = 3
    config["infeed_supply"]["release_interval_s"] = 3.0
    config["infeed_supply"]["recycle_delay_s"] = 10.0
    with pytest.raises(ValueError, match="too small"):
        validate_cell_config(config)
