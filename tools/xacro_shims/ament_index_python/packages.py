from __future__ import annotations

import os
from pathlib import Path


class PackageNotFoundError(LookupError):
    pass


def get_package_share_directory(package_name: str) -> str:
    """Resolve only the pinned package required during Windows bootstrap.

    A real ROS 2 installation provides ament_index_python. This deliberately
    narrow shim prevents Windows-side URDF generation from silently resolving
    arbitrary or stale ROS packages.
    """

    if package_name != "ur_description":
        raise PackageNotFoundError(package_name)
    share = os.environ.get("UR_DESCRIPTION_SHARE")
    if not share:
        raise PackageNotFoundError(
            "UR_DESCRIPTION_SHARE must point to Universal_Robots_ROS2_Description"
        )
    resolved = Path(share).resolve()
    if not (resolved / "package.xml").is_file():
        raise PackageNotFoundError(f"invalid ur_description share: {resolved}")
    return str(resolved)

