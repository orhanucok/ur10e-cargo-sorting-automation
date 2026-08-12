from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"configuration root must be a mapping: {path}")
    return data


def validate_cell_config(config: dict[str, Any]) -> None:
    if config.get("schema_version") != 1:
        raise ValueError("unsupported cell configuration schema")

    physics = config["physics"]
    if physics["engine"] != "mujoco":
        raise ValueError("this project is calibrated for the MuJoCo engine")
    if not 0 < physics["validation_step_s"] <= physics["development_step_s"] <= 0.005:
        raise ValueError("physics steps must be positive and no larger than 5 ms")

    robots = config["robots"]
    if len(robots) != 3:
        raise ValueError("exactly three robots are required by this cell layout")
    names = [robot["name"] for robot in robots]
    if len(names) != len(set(names)):
        raise ValueError("robot names must be unique")
    if any(robot["model"] != "ur10e" for robot in robots):
        raise ValueError("all robots must use the locked UR10e model")

    destinations = {
        robot["destination"]
        for robot in robots
        if robot["role"] == "sort"
    }
    if destinations != {"red", "blue"}:
        raise ValueError("one red and one blue sorting robot are required")

    cameras = config["cameras"]
    if cameras["main"]["decision_owner"] is not True:
        raise ValueError("main camera must be the sole routing decision owner")
    if cameras["local"]["decision_owner"] is not False:
        raise ValueError("local cameras must not own routing decisions")

    conveyor = config["conveyor"]
    if not 0 < conveyor["speed_m_s"] <= conveyor["max_speed_m_s"]:
        raise ValueError("invalid conveyor speed range")

    deck_top = conveyor["origin_m"][2] + conveyor["deck_thickness_m"] / 2
    minimum_robot_base_z = deck_top + 0.15
    if any(robot["base_position_m"][2] < minimum_robot_base_z for robot in robots):
        raise ValueError("robot bases require at least 150 mm clearance above the conveyor deck")

    supply = config["infeed_supply"]
    chute_exit = supply["chute_exit_m"]
    chute_top = supply["chute_top_m"]
    if chute_top[1] <= chute_exit[1] or chute_top[2] <= chute_exit[2]:
        raise ValueError("infeed chute must rise away from the bin")
    if not 0 < supply["chute_width_m"] < config["bins"]["infeed"]["inner_size_m"][0]:
        raise ValueError("infeed chute must fit inside the infeed bin width")
    if not isinstance(supply["pool_size"], int) or not 3 <= supply["pool_size"] <= 24:
        raise ValueError("infeed supply pool size must be between 3 and 24")
    if not 0.5 <= supply["release_interval_s"] <= 30.0:
        raise ValueError("infeed release interval must be between 0.5 and 30 seconds")
    if supply["recycle_delay_s"] <= supply["release_interval_s"]:
        raise ValueError("infeed recycle delay must exceed the release interval")
    if supply["pool_size"] * supply["release_interval_s"] <= supply["recycle_delay_s"] + 3.0:
        raise ValueError("infeed supply pool is too small for continuous circulation")
