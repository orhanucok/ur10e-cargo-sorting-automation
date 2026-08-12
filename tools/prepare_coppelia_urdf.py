from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


AUXILIARY_LINKS = {"world", "base_link", "ft_frame", "base", "flange", "tool0"}
AUXILIARY_JOINTS = {
    "base_joint",
    "base_link-base_link_inertia",
    "wrist_3_link-ft_frame",
    "base_link-base_fixed_joint",
    "wrist_3-flange",
    "flange-tool0",
}


def prepare(source: Path, destination: Path) -> None:
    tree = ET.parse(source)
    robot = tree.getroot()
    if robot.tag != "robot":
        raise ValueError("source is not a URDF robot")

    for link in list(robot.findall("link")):
        if link.attrib.get("name") in AUXILIARY_LINKS:
            robot.remove(link)
    for joint in list(robot.findall("joint")):
        if joint.attrib.get("name") in AUXILIARY_JOINTS:
            robot.remove(joint)

    # The physical base link becomes the root of CoppeliaSim's dynamic tree.
    base = robot.find("link[@name='base_link_inertia']")
    if base is None:
        raise ValueError("UR10e physical base link is missing")
    base.attrib["name"] = "base_link"

    shoulder_joint = robot.find("joint[@name='shoulder_pan_joint']")
    if shoulder_joint is None:
        raise ValueError("UR10e shoulder joint is missing")
    parent = shoulder_joint.find("parent")
    if parent is None:
        raise ValueError("UR10e shoulder joint parent is missing")
    parent.attrib["link"] = "base_link"

    remaining_links = {link.attrib["name"] for link in robot.findall("link")}
    for joint in robot.findall("joint"):
        parent_name = joint.find("parent").attrib["link"]
        child_name = joint.find("child").attrib["link"]
        if parent_name not in remaining_links or child_name not in remaining_links:
            raise ValueError(f"dangling joint {joint.attrib['name']}")

    if len(robot.findall("link")) != 7 or len(robot.findall("joint")) != 6:
        raise ValueError("Coppelia URDF must contain exactly 7 physical links and 6 joints")

    # CoppeliaSim 4.10's URDF plugin resolves visual meshes from file:/// URIs,
    # but fails while combining visual and collision meshes when the collision
    # filename uses that URI form. Plain absolute Windows paths with forward
    # slashes are accepted by both the Assimp and STL import paths.
    for mesh in robot.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if filename.startswith("file:///"):
            filename = filename[len("file:///") :]
        mesh.attrib["filename"] = filename.replace("\\", "/")

    ET.indent(tree, space="  ")
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    prepare(args.source, args.destination)
    print(args.destination.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
