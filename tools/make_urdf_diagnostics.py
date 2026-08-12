from __future__ import annotations

"""Generate reduced URDFs used to isolate CoppeliaSim importer failures."""

import copy
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "artifacts" / "ur10e_coppelia.urdf"
OUTPUT = ROOT / "work" / "debug" / "urdf_variants"


def write_variant(name: str, link_count: int, *, visuals: bool, collisions: bool, inertials: bool) -> None:
    source_root = ET.parse(SOURCE).getroot()
    links = source_root.findall("link")[:link_count]
    link_names = {link.attrib["name"] for link in links}
    robot = ET.Element("robot", {"name": name})
    for source_link in links:
        link = copy.deepcopy(source_link)
        if not visuals:
            for element in list(link.findall("visual")):
                link.remove(element)
        if not collisions:
            for element in list(link.findall("collision")):
                link.remove(element)
        if not inertials:
            for element in list(link.findall("inertial")):
                link.remove(element)
        robot.append(link)
    for source_joint in source_root.findall("joint"):
        parent = source_joint.find("parent").attrib["link"]
        child = source_joint.find("child").attrib["link"]
        if parent in link_names and child in link_names:
            robot.append(copy.deepcopy(source_joint))
    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ")
    tree.write(OUTPUT / f"{name}.urdf", encoding="utf-8", xml_declaration=True)


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for count in range(1, 8):
        write_variant(f"ur10e_{count}_full", count, visuals=True, collisions=True, inertials=True)
    write_variant("ur10e_7_visual", 7, visuals=True, collisions=False, inertials=True)
    write_variant("ur10e_7_geometry", 7, visuals=True, collisions=True, inertials=False)
    print(OUTPUT)


if __name__ == "__main__":
    main()
