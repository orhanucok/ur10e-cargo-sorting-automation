from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import sys
import time
from typing import Any

from sorting_cell_core.config import load_yaml, validate_cell_config


ROOT_ALIAS = "sorting_cell"
JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
HOME_JOINTS_DEG = (0.0, -90.0, 90.0, -90.0, -90.0, 0.0)
PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_PATH = Path(__file__).with_name("sorting_cell_controller.lua")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build the full UR10e sorting cell in CoppeliaSim")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--replace", action="store_true")
    parser.add_argument("--start", action="store_true")
    parser.add_argument("--save", type=Path, default=None)
    return parser.parse_args()


def set_alias(sim: Any, handle: int, alias: str) -> None:
    sim.setObjectAlias(handle, alias)


def set_parent(sim: Any, child: int, parent: int, *, keep_in_place: bool = True) -> None:
    sim.setObjectParent(child, parent, keep_in_place)


def short_alias(sim: Any, handle: int) -> str:
    path = sim.getObjectAlias(handle, 1)
    return re.sub(r"\[\d+\]$", "", path.rsplit("/", 1)[-1])


def create_box(
    sim: Any,
    parent: int,
    alias: str,
    size: list[float],
    position: list[float],
    color: list[float],
    *,
    static: bool,
    respondable: bool = True,
) -> int:
    handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 2)
    set_alias(sim, handle, alias)
    sim.setObjectPosition(handle, sim.handle_world, position)
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, color)
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, int(static))
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, int(respondable))
    set_parent(sim, handle, parent)
    return handle


def create_cylinder(
    sim: Any,
    parent: int,
    alias: str,
    diameter: float,
    length: float,
    position: list[float],
    color: list[float],
    *,
    static: bool,
    respondable: bool = True,
    orientation: list[float] | None = None,
) -> int:
    handle = sim.createPrimitiveShape(
        sim.primitiveshape_cylinder,
        [diameter, diameter, length],
        2,
    )
    set_alias(sim, handle, alias)
    sim.setObjectPosition(handle, sim.handle_world, position)
    if orientation is not None:
        sim.setObjectOrientation(handle, sim.handle_world, orientation)
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, color)
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, int(static))
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, int(respondable))
    set_parent(sim, handle, parent)
    return handle


def set_local_pose(
    sim: Any,
    handle: int,
    parent: int,
    position: list[float],
    orientation: list[float] | None = None,
) -> None:
    sim.setObjectPosition(handle, parent, position)
    if orientation is not None:
        sim.setObjectOrientation(handle, parent, orientation)


def configure_physics(sim: Any, config: dict[str, Any]) -> None:
    sim.setInt32Param(sim.intparam_dynamic_engine, sim.physics_mujoco)
    sim.setFloatParam(sim.floatparam_simulation_time_step, config["physics"]["development_step_s"])
    sim.setArrayParam(sim.arrayparam_gravity, [0.0, 0.0, -config["world"]["gravity_m_s2"]])


def create_open_bin(sim: Any, parent: int, alias: str, spec: dict[str, Any]) -> int:
    root = sim.createDummy(0.01)
    set_alias(sim, root, alias)
    set_parent(sim, root, parent)
    cx, cy, cz = spec["center_m"]
    sx, sy, sz = spec["inner_size_m"]
    color = spec["color_rgb"]
    wall = 0.045

    create_box(
        sim, root, f"{alias}_floor", [sx + 2 * wall, sy + 2 * wall, wall],
        [cx, cy, cz - sz / 2], color, static=True,
    )
    create_box(
        sim, root, f"{alias}_wall_left", [wall, sy + 2 * wall, sz],
        [cx - sx / 2 - wall / 2, cy, cz], color, static=True,
    )
    create_box(
        sim, root, f"{alias}_wall_right", [wall, sy + 2 * wall, sz],
        [cx + sx / 2 + wall / 2, cy, cz], color, static=True,
    )
    create_box(
        sim, root, f"{alias}_wall_front", [sx, wall, sz],
        [cx, cy - sy / 2 - wall / 2, cz], color, static=True,
    )
    create_box(
        sim, root, f"{alias}_wall_back", [sx, wall, sz],
        [cx, cy + sy / 2 + wall / 2, cz], color, static=True,
    )

    if alias == "bin_infeed":
        # A shallow gravity/vibration insert brings parcels from the chute at
        # the rear toward a repeatable, robot-reachable pick pocket at the
        # front. Without it, a random parcel can settle beyond UR10e reach.
        insert_angle = math.radians(18.0)
        insert_length = sy * 0.82
        insert_thickness = 0.035
        surface_center = [cx, cy, 0.215]
        normal = [0.0, -math.sin(insert_angle), math.cos(insert_angle)]
        insert_center = [
            surface_center[i] - normal[i] * insert_thickness / 2 for i in range(3)
        ]
        insert = create_box(
            sim,
            root,
            "infeed_bin_vibratory_insert",
            [sx * 0.88, insert_length, insert_thickness],
            insert_center,
            [0.25, 0.27, 0.29],
            static=True,
        )
        sim.setObjectOrientation(insert, sim.handle_world, [insert_angle, 0.0, 0.0])
    return root


def create_infeed_chute(
    sim: Any,
    parent: int,
    bin_spec: dict[str, Any],
    supply_spec: dict[str, Any],
) -> dict[str, int]:
    chute_root = sim.createDummy(0.01)
    set_alias(sim, chute_root, "continuous_infeed_chute")
    set_parent(sim, chute_root, parent)

    lower = supply_spec["chute_exit_m"]
    upper = supply_spec["chute_top_m"]
    width = supply_spec["chute_width_m"]
    thickness = supply_spec["chute_plate_thickness_m"]
    wall_height = supply_spec["side_wall_height_m"]
    delta_y = upper[1] - lower[1]
    delta_z = upper[2] - lower[2]
    length = math.hypot(delta_y, delta_z)
    angle = math.atan2(delta_z, delta_y)
    direction = [0.0, math.cos(angle), math.sin(angle)]
    normal = [0.0, -math.sin(angle), math.cos(angle)]
    surface_center = [(lower[i] + upper[i]) / 2 for i in range(3)]
    steel = [0.24, 0.27, 0.30]
    galvanized = [0.50, 0.54, 0.57]
    safety_yellow = [0.92, 0.62, 0.04]

    # The configured end points describe the upper contact surface. Offset the
    # primitive down its normal so parcels touch the plate instead of starting
    # interpenetrated with it.
    plate_center = [
        surface_center[i] - normal[i] * thickness / 2 for i in range(3)
    ]
    plate = create_box(
        sim,
        chute_root,
        "infeed_chute_plate",
        [width, length, thickness],
        plate_center,
        galvanized,
        static=True,
    )

    for side, sign in (("left", -1.0), ("right", 1.0)):
        wall_center = [
            surface_center[0] + sign * (width / 2 + 0.0225),
            surface_center[1] + normal[1] * wall_height / 2,
            surface_center[2] + normal[2] * wall_height / 2,
        ]
        wall = create_box(
            sim,
            chute_root,
            f"infeed_chute_wall_{side}",
            [0.045, length, wall_height],
            wall_center,
            safety_yellow,
            static=True,
        )
        sim.setObjectOrientation(wall, sim.handle_world, [angle, 0.0, 0.0])

    sim.setObjectOrientation(plate, sim.handle_world, [angle, 0.0, 0.0])

    # A rigid magazine hood and four floor-mounted posts make the source of the
    # parcels visually and structurally credible. The lower lip remains open.
    hood_depth = 0.48
    hood_height = 0.58
    hood_center_y = upper[1] + 0.08
    hood_center_z = upper[2] + 0.17
    create_box(
        sim,
        chute_root,
        "infeed_magazine_back",
        [width + 0.18, 0.055, hood_height],
        [upper[0], upper[1] + hood_depth / 2, hood_center_z],
        steel,
        static=True,
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        create_box(
            sim,
            chute_root,
            f"infeed_magazine_side_{side}",
            [0.055, hood_depth, hood_height],
            [upper[0] + sign * (width / 2 + 0.06), hood_center_y, hood_center_z],
            steel,
            static=True,
        )
    create_box(
        sim,
        chute_root,
        "infeed_magazine_header",
        [width + 0.18, hood_depth, 0.07],
        [upper[0], hood_center_y, upper[2] + hood_height / 2 + 0.17],
        safety_yellow,
        static=True,
    )

    support_y = upper[1] + hood_depth / 2
    support_height = upper[2] + 0.25
    for side, sign in (("left", -1.0), ("right", 1.0)):
        create_box(
            sim,
            chute_root,
            f"infeed_magazine_support_{side}",
            [0.075, 0.075, support_height],
            [upper[0] + sign * (width / 2 + 0.06), support_y, support_height / 2],
            steel,
            static=True,
        )

    # A concrete pad keeps the elevated feeder visibly supported even though
    # the infeed bin sits at the edge of the main factory floor.
    bin_x, bin_y, _ = bin_spec["center_m"]
    create_box(
        sim,
        chute_root,
        "infeed_feeder_foundation",
        [1.15, 1.70, 0.08],
        [bin_x, bin_y + 0.75, 0.0],
        [0.38, 0.39, 0.40],
        static=True,
    )

    # Spawn the parcel slightly down-slope and above the upper contact surface.
    along_slope = 0.13
    normal_clearance = 0.115
    spawn_position = [
        upper[i] - direction[i] * along_slope + normal[i] * normal_clearance
        for i in range(3)
    ]
    spawn = sim.createDummy(0.025)
    set_alias(sim, spawn, "infeed_chute_spawn")
    sim.setObjectPosition(spawn, sim.handle_world, spawn_position)
    sim.setObjectInt32Param(spawn, sim.objintparam_visibility_layer, 0)
    set_parent(sim, spawn, chute_root)

    staging = sim.createDummy(0.025)
    set_alias(sim, staging, "infeed_supply_staging")
    sim.setObjectPosition(staging, sim.handle_world, [upper[0], upper[1] + 0.65, -0.80])
    sim.setObjectInt32Param(staging, sim.objintparam_visibility_layer, 0)
    set_parent(sim, staging, chute_root)
    return {"spawn": spawn, "staging": staging, "plate": plate}


def create_conveyor(sim: Any, root: int, spec: dict[str, Any]) -> tuple[int, list[int]]:
    conveyor_root = sim.createDummy(0.01)
    set_alias(sim, conveyor_root, "conveyor")
    set_parent(sim, conveyor_root, root)

    x, y, z = spec["origin_m"]
    length = spec["length_m"]
    width = spec["width_m"]
    deck_thickness = spec["deck_thickness_m"]
    steel = [0.20, 0.23, 0.27]
    belt = [0.055, 0.06, 0.065]
    safety_yellow = [0.92, 0.62, 0.04]

    create_box(
        sim,
        conveyor_root,
        "conveyor_deck",
        [length, width, deck_thickness],
        [x + length / 2, y, z],
        steel,
        static=True,
    )

    slat_pitch = 0.125
    slat_height = 0.026
    slat_z = z + deck_thickness / 2 + slat_height / 2 + 0.003
    belt_drop = 0.18
    belt_path_length = 2 * length + 2 * belt_drop
    slat_count = math.ceil(belt_path_length / slat_pitch)
    slats: list[int] = []
    for index in range(slat_count):
        phase = index * belt_path_length / slat_count
        if phase < length:
            slat_x, current_z = x + phase, slat_z
        elif phase < length + belt_drop:
            slat_x, current_z = x + length, slat_z - (phase - length)
        elif phase < 2 * length + belt_drop:
            slat_x = x + length - (phase - length - belt_drop)
            current_z = slat_z - belt_drop
        else:
            slat_x = x
            current_z = slat_z - belt_drop + (phase - 2 * length - belt_drop)
        slat = create_box(
            sim,
            conveyor_root,
            f"belt_slat_{index + 1:02d}",
            [slat_pitch * 0.92, width * 0.94, slat_height],
            [slat_x, y, current_z],
            belt,
            static=True,
        )
        sim.setObjectInt32Param(slat, sim.shapeintparam_kinematic, 1)
        slats.append(slat)

    rail_z = slat_z + 0.105
    # Physical access windows replace an impossible full-length guide rail at
    # robot pick zones. The deck and moving belt remain continuous.
    rail_openings = {
        # The infeed robot approaches from negative Y and needs a physical
        # window to place an oriented parcel on the first belt section.
        "left": ((0.0, 0.80), (2.65, 3.55)),
        "right": ((3.90, 4.80),),
    }
    for side, sign in (("left", -1.0), ("right", 1.0)):
        segments: list[tuple[float, float]] = []
        segment_start = x
        for opening_start, opening_end in rail_openings[side]:
            if opening_start > segment_start:
                segments.append((segment_start, opening_start))
            segment_start = max(segment_start, opening_end)
        if segment_start < x + length:
            segments.append((segment_start, x + length))
        for segment_index, (segment_start, segment_end) in enumerate(segments, 1):
            segment_length = segment_end - segment_start
            if segment_length <= 0:
                continue
            create_box(
                sim,
                conveyor_root,
                f"guide_rail_{side}_{segment_index}",
                [segment_length, 0.035, 0.11],
                [segment_start + segment_length / 2, y + sign * (width / 2 + 0.025), rail_z],
                safety_yellow,
                static=True,
            )

    roller_z = slat_z - 0.005
    for name, roller_x in (("drive", x + 0.045), ("tail", x + length - 0.045)):
        create_cylinder(
            sim,
            conveyor_root,
            f"roller_{name}",
            0.13,
            width + 0.08,
            [roller_x, y, roller_z],
            [0.32, 0.34, 0.36],
            static=True,
            orientation=[math.pi / 2, 0.0, 0.0],
        )

    for leg_x in (x + 0.45, x + length / 2, x + length - 0.45):
        for leg_y in (y - width / 2 + 0.07, y + width / 2 - 0.07):
            create_box(
                sim,
                conveyor_root,
                f"conveyor_leg_{leg_x:.2f}_{leg_y:.2f}",
                [0.075, 0.075, z - deck_thickness / 2],
                [leg_x, leg_y, (z - deck_thickness / 2) / 2],
                steel,
                static=True,
            )
    return conveyor_root, slats


def create_main_camera(sim: Any, parent: int, spec: dict[str, Any]) -> int:
    x, y, z = spec["position_m"]
    steel = [0.22, 0.24, 0.27]
    for support_y in (-0.58, 0.58):
        create_box(
            sim,
            parent,
            f"camera_gantry_post_{support_y:+.2f}",
            [0.065, 0.065, z - 0.72],
            [x, support_y, 0.72 + (z - 0.72) / 2],
            steel,
            static=True,
        )
    create_box(
        sim,
        parent,
        "camera_gantry_beam",
        [0.09, 1.25, 0.075],
        [x, 0.0, z + 0.08],
        steel,
        static=True,
    )
    create_box(
        sim,
        parent,
        "basler_a2_camera_body",
        [0.13, 0.11, 0.09],
        [x, y, z + 0.025],
        [0.06, 0.08, 0.10],
        static=True,
        respondable=False,
    )

    resolution = spec["development_resolution"]
    # Explicit handling keeps four GPU cameras from rendering at the 200 Hz
    # physics rate. The vision pipeline requests frames at each camera's FPS.
    options = 1 | 2 | 4 | 64
    float_params = [
        spec["near_clip_m"],
        spec["far_clip_m"],
        math.radians(spec["horizontal_fov_deg"]),
        0.05,
        0.0,
        0.0,
        0.06,
        0.06,
        0.07,
        0.0,
        0.0,
    ]
    sensor = sim.createVisionSensor(options, [*resolution, 0, 0], float_params)
    set_alias(sim, sensor, "main_camera_basler_a2A2448")
    sim.setObjectPosition(sensor, sim.handle_world, spec["position_m"])
    sim.setObjectOrientation(
        sensor,
        sim.handle_world,
        [math.radians(value) for value in spec["orientation_rpy_deg"]],
    )
    set_parent(sim, sensor, parent)
    return sensor


def create_parcel(
    sim: Any,
    parent: int,
    alias: str,
    position: list[float],
    route: str,
    mass_kg: float,
    *,
    static: bool = False,
) -> int:
    size = [0.31, 0.23, 0.18]
    cardboard = [0.56, 0.36, 0.18]
    box = create_box(sim, parent, alias, size, position, cardboard, static=static)
    sim.computeMassAndInertia(box, mass_kg / math.prod(size))
    sim.setShapeMass(box, mass_kg)

    # Use thin, non-respondable child shapes instead of free drawing objects.
    # Their transforms are genuinely relative to the parcel, so the sticker
    # follows every physics translation, tumble and robot re-parenting step.
    def add_sticker_part(
        part_alias: str,
        local_position: list[float],
        part_size: list[float],
        color: list[float],
    ) -> int:
        part = create_box(
            sim,
            box,
            f"{alias}_{part_alias}",
            part_size,
            [0.0, 0.0, 0.0],
            color,
            static=True,
            respondable=False,
        )
        set_local_pose(sim, part, box, local_position)
        return part

    top_z = size[2] / 2
    add_sticker_part(
        "packing_tape",
        [0.0, 0.0, top_z + 0.0008],
        [0.032, size[1] * 0.98, 0.0015],
        [0.77, 0.66, 0.44],
    )
    add_sticker_part(
        "shipping_label",
        [0.015, 0.0, top_z + 0.0018],
        [0.18, 0.13, 0.0015],
        [0.93, 0.93, 0.89],
    )
    route_color = [0.80, 0.04, 0.04] if route == "red" else [0.04, 0.20, 0.86]
    add_sticker_part(
        "route_mark",
        [-0.045, -0.015, top_z + 0.0028],
        [0.052, 0.052, 0.0015],
        route_color,
    )

    for stripe in range(7):
        stripe_x = 0.035 + stripe * 0.012
        stripe_width = 0.003 if stripe in (0, 3, 5) else 0.005
        add_sticker_part(
            f"barcode_{stripe + 1:02d}",
            [stripe_x, 0.014, top_z + 0.0037],
            [stripe_width, 0.048, 0.0012],
            [0.025, 0.025, 0.025],
        )
    return box


def create_gripper_and_camera(
    sim: Any,
    root: int,
    wrist: int,
    robot_name: str,
    local_camera: dict[str, Any],
) -> tuple[int, int]:
    assembly = sim.createDummy(0.01)
    set_alias(sim, assembly, f"{robot_name}_robotiq_epick")
    set_parent(sim, assembly, wrist)
    set_local_pose(sim, assembly, wrist, [0.0, 0.0, 0.0])

    dark = [0.09, 0.10, 0.11]
    aluminum = [0.48, 0.50, 0.52]
    rubber = [0.035, 0.035, 0.038]

    flange = create_cylinder(
        sim, root, f"{robot_name}_epick_flange", 0.095, 0.055,
        [0.0, 0.0, 0.0], aluminum, static=True, respondable=False,
    )
    set_parent(sim, flange, assembly)
    set_local_pose(sim, flange, assembly, [0.0, 0.0, -0.035])

    manifold = create_box(
        sim, root, f"{robot_name}_epick_manifold", [0.19, 0.15, 0.055],
        [0.0, 0.0, 0.0], dark, static=True, respondable=False,
    )
    set_parent(sim, manifold, assembly)
    set_local_pose(sim, manifold, assembly, [0.0, 0.0, -0.085])

    for cup_index, (cup_x, cup_y) in enumerate(((-0.055, -0.04), (-0.055, 0.04), (0.055, -0.04), (0.055, 0.04)), 1):
        cup = create_cylinder(
            sim, root, f"{robot_name}_vacuum_cup_{cup_index}", 0.052, 0.055,
            [0.0, 0.0, 0.0], rubber, static=True, respondable=False,
        )
        set_parent(sim, cup, assembly)
        set_local_pose(sim, cup, assembly, [cup_x, cup_y, -0.14])

    camera_body = create_box(
        sim, root, f"{robot_name}_d455_body", [0.12, 0.035, 0.03],
        [0.0, 0.0, 0.0], [0.04, 0.05, 0.055], static=True, respondable=False,
    )
    set_parent(sim, camera_body, assembly)
    set_local_pose(sim, camera_body, assembly, [0.105, 0.0, -0.085], [0.0, math.pi / 2, 0.0])

    resolution = local_camera["development_resolution"]
    sensor = sim.createVisionSensor(
        1 | 2 | 4 | 64,
        [*resolution, 0, 0],
        [
            local_camera["near_clip_m"], local_camera["far_clip_m"],
            math.radians(local_camera["horizontal_fov_deg"]), 0.03,
            0.0, 0.0, 0.04, 0.04, 0.045, 0.0, 0.0,
        ],
    )
    set_alias(sim, sensor, f"{robot_name}_realsense_d455")
    set_parent(sim, sensor, assembly)
    set_local_pose(sim, sensor, assembly, [0.125, 0.0, -0.085], [0.0, math.pi / 2, 0.0])

    tip = sim.createDummy(0.025)
    set_alias(sim, tip, f"{robot_name}_vacuum_tcp")
    set_parent(sim, tip, assembly)
    set_local_pose(sim, tip, assembly, [0.0, 0.0, -0.18])
    sim.setObjectOrientation(tip, sim.handle_world, [math.pi, 0.0, 0.0])

    target = sim.createDummy(0.035)
    set_alias(sim, target, f"{robot_name}_ik_target")
    sim.setObjectPose(target, sim.getObjectPose(tip, sim.handle_world), sim.handle_world)
    sim.setObjectInt32Param(target, sim.objintparam_visibility_layer, 0)
    set_parent(sim, target, root)
    return tip, target


def import_ur10e(
    sim: Any,
    client: Any,
    root: int,
    urdf_path: Path,
    robot: dict[str, Any],
    local_camera: dict[str, Any],
) -> dict[str, Any]:
    sim_urdf = client.require("simURDF")
    result = getattr(sim_urdf, "import")(str(urdf_path.resolve()).replace("\\", "/"), 32)

    if isinstance(result, (tuple, list)) and len(result) >= 2 and result[1]:
        model = int(result[1][0])
    else:
        model_name = result[0] if isinstance(result, (tuple, list)) else result
        model_path = model_name if str(model_name).startswith("/") else f"/{model_name}"
        model = sim.getObject(model_path)

    set_alias(sim, model, robot["name"])
    sim.setObjectPosition(model, sim.handle_world, robot["base_position_m"])
    sim.setObjectOrientation(
        model,
        sim.handle_world,
        [0.0, 0.0, math.radians(robot["base_yaw_deg"])],
    )
    sim.setObjectInt32Param(model, sim.shapeintparam_static, 1)
    set_parent(sim, model, root)

    joints_by_name: dict[str, int] = {}
    for handle in sim.getObjectsInTree(model, sim.object_joint_type):
        alias = short_alias(sim, handle)
        if alias in JOINT_NAMES:
            joints_by_name[alias] = handle
    missing = [name for name in JOINT_NAMES if name not in joints_by_name]
    if missing:
        raise RuntimeError(f"UR10e importer omitted joints: {missing}")
    joints = [joints_by_name[name] for name in JOINT_NAMES]
    for name, handle, position_deg in zip(JOINT_NAMES, joints, HOME_JOINTS_DEG, strict=True):
        set_alias(sim, handle, f"{robot['name']}_{name}")
        position = math.radians(position_deg)
        sim.setJointPosition(handle, position)
        sim.setJointTargetPosition(handle, position)

    wrist = next(
        handle
        for handle in sim.getObjectsInTree(model, sim.object_shape_type)
        if "wrist_3_link_respondable" in short_alias(sim, handle)
    )
    tip, target = create_gripper_and_camera(sim, root, wrist, robot["name"], local_camera)
    return {"base": model, "joints": joints, "tip": tip, "target": target}


def create_pedestal(sim: Any, root: int, robot: dict[str, Any]) -> None:
    x, y, z = robot["base_position_m"]
    height = max(z, 0.16)
    flange_color = [0.16, 0.18, 0.20]
    body_color = [0.32, 0.35, 0.38]
    plate_height = 0.055
    body_height = height - 2 * plate_height

    create_cylinder(
        sim,
        root,
        f"{robot['name']}_pedestal_bottom_flange",
        0.50,
        plate_height,
        [x, y, plate_height / 2],
        flange_color,
        static=True,
    )
    create_cylinder(
        sim,
        root,
        f"{robot['name']}_pedestal_column",
        0.36,
        body_height,
        [x, y, plate_height + body_height / 2],
        body_color,
        static=True,
    )
    create_cylinder(
        sim,
        root,
        f"{robot['name']}_pedestal_top_flange",
        0.46,
        plate_height,
        [x, y, height - plate_height / 2],
        flange_color,
        static=True,
    )
    for bolt_index, angle_deg in enumerate((45.0, 135.0, 225.0, 315.0), 1):
        angle = math.radians(angle_deg)
        create_cylinder(
            sim,
            root,
            f"{robot['name']}_anchor_bolt_{bolt_index}",
            0.026,
            0.022,
            [x + 0.19 * math.cos(angle), y + 0.19 * math.sin(angle), plate_height + 0.011],
            [0.05, 0.055, 0.06],
            static=True,
        )


def attach_controller(
    sim: Any,
    root: int,
    slats: list[int],
    conveyor_shapes: list[int],
    red_boxes: list[int],
    blue_boxes: list[int],
    supply_boxes: list[int],
    supply_handles: dict[str, int],
    supply_spec: dict[str, Any],
    robots: list[dict[str, Any]],
) -> int:
    script_text = CONTROLLER_PATH.read_text(encoding="utf-8")
    script = sim.createScript(sim.scripttype_simulation, script_text, 0, "lua")
    set_alias(sim, script, "sorting_cell_controller")
    set_parent(sim, script, root)
    sim.setObjectInt32Param(script, sim.objintparam_visibility_layer, 0)
    sim.setReferencedHandles(script, [root], "cell.root")
    sim.setReferencedHandles(script, slats, "belt.slats")
    sim.setReferencedHandles(script, conveyor_shapes, "belt.collision")
    sim.setReferencedHandles(script, red_boxes, "boxes.red")
    sim.setReferencedHandles(script, blue_boxes, "boxes.blue")
    sim.setReferencedHandles(script, supply_boxes, "boxes.supply")
    sim.setReferencedHandles(script, [supply_handles["spawn"]], "supply.spawn")
    sim.setReferencedHandles(script, [supply_handles["staging"]], "supply.staging")
    sim.setReferencedHandles(script, [supply_handles["plate"]], "supply.chute")
    sim.setFloatSignal(
        "sorting_cell_supply_release_interval",
        float(supply_spec["release_interval_s"]),
    )
    sim.setFloatSignal(
        "sorting_cell_supply_recycle_delay",
        float(supply_spec["recycle_delay_s"]),
    )
    for index, robot in enumerate(robots, 1):
        sim.setReferencedHandles(script, [robot["base"]], f"robot.{index}.base")
        sim.setReferencedHandles(script, robot["joints"], f"robot.{index}.joints")
        sim.setReferencedHandles(script, [robot["tip"]], f"robot.{index}.tip")
        sim.setReferencedHandles(script, [robot["target"]], f"robot.{index}.target")
    return script


def position_default_camera(sim: Any) -> None:
    camera = sim.getObject("/DefaultCamera", {"noError": True})
    if camera == -1:
        return
    position = [2.0, -8.5, 9.5]
    target = [2.0, 0.0, 0.95]
    forward = [target[i] - position[i] for i in range(3)]
    forward_length = math.sqrt(sum(value * value for value in forward))
    forward = [value / forward_length for value in forward]
    # Coppelia's camera image uses -Y as screen-up. Negating both horizontal
    # and vertical basis axes keeps a right-handed pose and an upright view.
    right = [-forward[1], forward[0], 0.0]
    right_length = math.sqrt(sum(value * value for value in right))
    right = [value / right_length for value in right]
    up = [
        forward[1] * right[2] - forward[2] * right[1],
        forward[2] * right[0] - forward[0] * right[2],
        forward[0] * right[1] - forward[1] * right[0],
    ]
    matrix = [
        right[0], up[0], forward[0], position[0],
        right[1], up[1], forward[1], position[1],
        right[2], up[2], forward[2], position[2],
    ]
    sim.setObjectMatrix(camera, sim.handle_world, matrix)


def build_scene(sim: Any, client: Any, config: dict[str, Any], urdf_path: Path | None) -> int:
    configure_physics(sim, config)
    default_floor = sim.getObject("/Floor", {"noError": True})
    if default_floor != -1:
        sim.removeObjects(sim.getObjectsInTree(default_floor))
    root = sim.createDummy(0.01)
    set_alias(sim, root, ROOT_ALIAS)

    world_x, world_y, _ = config["world"]["size_m"]
    create_box(
        sim,
        root,
        "factory_floor",
        [world_x, world_y, 0.10],
        [world_x / 2 - 1.0, 0.0, -0.05],
        [0.30, 0.32, 0.35],
        static=True,
    )

    conveyor_root, slats = create_conveyor(sim, root, config["conveyor"])

    station = config["orientation_station"]
    create_box(
        sim,
        root,
        "orientation_station",
        station["size_m"],
        station["center_m"],
        [0.22, 0.24, 0.27],
        static=True,
    )
    station_x, station_y, station_z = station["center_m"]
    for offset in (-0.24, -0.12, 0.0, 0.12, 0.24):
        create_cylinder(
            sim, root, f"orientation_roller_{offset:+.2f}", 0.055, station["size_m"][1] * 0.90,
            [station_x + offset, station_y, station_z + 0.055], [0.38, 0.40, 0.42],
            static=True, orientation=[math.pi / 2, 0.0, 0.0],
        )

    bins_root = sim.createDummy(0.01)
    set_alias(sim, bins_root, "bins")
    set_parent(sim, bins_root, root)
    for name, spec in config["bins"].items():
        create_open_bin(sim, bins_root, f"bin_{name}", spec)

    supply_spec = config["infeed_supply"]
    supply_handles = create_infeed_chute(
        sim,
        root,
        config["bins"]["infeed"],
        supply_spec,
    )

    create_main_camera(sim, root, config["cameras"]["main"])

    if urdf_path is None:
        raise ValueError("the full cell requires --urdf artifacts/ur10e_coppelia.urdf")
    if not urdf_path.is_file():
        raise FileNotFoundError(f"UR10e URDF does not exist: {urdf_path}")

    imported_robots: list[dict[str, Any]] = []
    for robot_spec in config["robots"]:
        create_pedestal(sim, root, robot_spec)
        imported_robots.append(
            import_ur10e(
                sim,
                client,
                root,
                urdf_path,
                robot_spec,
                config["cameras"]["local"],
            )
        )

    belt_surface = (
        config["conveyor"]["origin_m"][2]
        + config["conveyor"]["deck_thickness_m"] / 2
        + 0.029
    )
    box_z = belt_surface + 0.09 + 0.004
    parcel_specs = (
        (2.55, "red", 2.2),
        (3.10, "blue", 1.6),
        (1.45, "red", 3.1),
        (2.00, "blue", 2.7),
        (0.35, "red", 1.9),
        (0.90, "blue", 3.4),
    )
    red_boxes: list[int] = []
    blue_boxes: list[int] = []
    for index, (box_x, route, mass) in enumerate(parcel_specs, 1):
        box = create_parcel(
            sim,
            root,
            f"parcel_{index:02d}_{route}",
            [box_x, 0.0, box_z],
            route,
            mass,
        )
        (red_boxes if route == "red" else blue_boxes).append(box)

    supply_boxes: list[int] = []
    supply_routes = ("red", "blue")
    supply_masses = (1.7, 2.4, 3.0, 2.0, 2.7, 1.9)
    staging_position = sim.getObjectPosition(supply_handles["staging"], sim.handle_world)
    for index in range(supply_spec["pool_size"]):
        route = supply_routes[index % len(supply_routes)]
        mass = supply_masses[index % len(supply_masses)]
        box = create_parcel(
            sim,
            root,
            f"supply_parcel_{index + 1:02d}_{route}",
            [staging_position[0], staging_position[1], staging_position[2] - index * 0.25],
            route,
            mass,
            static=True,
        )
        sim.setObjectInt32Param(box, sim.objintparam_visibility_layer, 0)
        supply_boxes.append(box)

    conveyor_shapes = sim.getObjectsInTree(conveyor_root, sim.object_shape_type)
    attach_controller(
        sim,
        root,
        slats,
        conveyor_shapes,
        red_boxes,
        blue_boxes,
        supply_boxes,
        supply_handles,
        supply_spec,
        imported_robots,
    )
    position_default_camera(sim)
    sim.setObjectSelection([])
    sim.setBoolParam(sim.boolparam_infotext_visible, False)
    return root


def stop_simulation(sim: Any) -> None:
    if sim.getSimulationState() == sim.simulation_stopped:
        return
    sim.stopSimulation()
    deadline = time.monotonic() + 15.0
    while sim.getSimulationState() != sim.simulation_stopped:
        if time.monotonic() >= deadline:
            raise TimeoutError("CoppeliaSim did not stop within 15 seconds")
        time.sleep(0.1)


def main() -> int:
    args = parse_args()
    config = load_yaml(args.config)
    validate_cell_config(config)

    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    except ImportError:
        print("Install the simulator extra: python -m pip install -e '.[sim]'", file=sys.stderr)
        return 2

    client = RemoteAPIClient(host=args.host, port=args.port)
    sim = client.require("sim")
    stop_simulation(sim)

    existing = sim.getObject(f"/{ROOT_ALIAS}", {"noError": True})
    if existing != -1:
        if not args.replace:
            raise RuntimeError(f"/{ROOT_ALIAS} already exists; use --replace explicitly")
        sim.removeObjects(sim.getObjectsInTree(existing))

    root = build_scene(sim, client, config, args.urdf)
    print(f"Created full CoppeliaSim cell root handle: {root}")
    print("Physics: MuJoCo | timestep: 0.005 s | Robots: 3x official UR10e")

    if args.save is not None:
        save_path = args.save.resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            sim.saveScene(str(save_path).replace("\\", "/"))
            print(f"Saved scene: {save_path}")
        except Exception as exc:
            # Some unactivated Pro installations allow live simulation but
            # reject scene serialization. The launcher can always rebuild it.
            print(f"Warning: scene serialization was unavailable: {exc}", file=sys.stderr)

    if args.start:
        sim.startSimulation()
        print("Simulation started: belt, cameras, IK motion and red/blue routing are active.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
