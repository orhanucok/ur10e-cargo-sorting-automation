from __future__ import annotations

import ctypes
from datetime import datetime
from pathlib import Path
import subprocess
import sys
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
BUILDER = PROJECT_ROOT / "coppeliasim" / "build_scene.py"
CONFIG = PROJECT_ROOT / "config" / "cell.yaml"
URDF = PROJECT_ROOT / "artifacts" / "ur10e_coppelia.urdf"
LOG_PATH = PROJECT_ROOT / "outputs" / "simulation_watchdog.log"
MAINTENANCE_FLAG = PROJECT_ROOT / "work" / "watchdog_maintenance.flag"
POLL_SECONDS = 2.0
HEARTBEAT_TIMEOUT_SECONDS = 20.0
REBUILD_COOLDOWN_SECONDS = 45.0
ERROR_ALREADY_EXISTS = 183


def log(message: str) -> None:
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
    with LOG_PATH.open("a", encoding="utf-8") as stream:
        stream.write(f"{timestamp} {message}\n")


def acquire_single_instance() -> int | None:
    kernel32 = ctypes.windll.kernel32
    handle = kernel32.CreateMutexW(None, False, "Local\\UR10eSortingCellSimulationWatchdog")
    if not handle or kernel32.GetLastError() == ERROR_ALREADY_EXISTS:
        return None
    return int(handle)


def connect():
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    client = RemoteAPIClient(host="localhost", port=23000)
    return client, client.require("sim")


def rebuild_scene() -> bool:
    command = [
        sys.executable,
        str(BUILDER),
        "--config",
        str(CONFIG),
        "--urdf",
        str(URDF),
        "--replace",
        "--start",
    ]
    creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    result = subprocess.run(
        command,
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=150,
        creationflags=creation_flags,
        check=False,
    )
    if result.returncode == 0:
        log("Sahne otomatik olarak yeniden kuruldu ve baslatildi.")
        return True
    error = (result.stderr or result.stdout).strip().replace("\n", " | ")
    log(f"Sahne yeniden kurulamadi: kod={result.returncode} {error}")
    return False


def main() -> int:
    mutex = acquire_single_instance()
    if mutex is None:
        return 0

    log("Saglik gozcusu baslatildi.")
    client = None
    sim = None
    heartbeat_value = None
    heartbeat_changed_at = time.monotonic()
    last_rebuild_at = 0.0
    connection_failures = 0

    while True:
        try:
            if MAINTENANCE_FLAG.exists():
                heartbeat_changed_at = time.monotonic()
                time.sleep(POLL_SECONDS)
                continue
            if sim is None:
                client, sim = connect()
                heartbeat_value = None
                heartbeat_changed_at = time.monotonic()
                connection_failures = 0
                log("CoppeliaSim ZeroMQ connection is ready.")

            state = sim.getSimulationState()
            now = time.monotonic()
            needs_rebuild = False
            reason = ""

            if state == sim.simulation_paused:
                log("Unexpected pause detected; resuming the simulation.")
                sim.startSimulation()
                heartbeat_changed_at = now
            elif state == sim.simulation_stopped:
                needs_rebuild = True
                reason = "the simulation has stopped"
            else:
                controller_state = sim.getInt32Signal("sorting_cell_controller_state")
                heartbeat = sim.getInt32Signal("sorting_cell_heartbeat_ms")
                progress_stalled = sim.getInt32Signal("sorting_cell_progress_stalled") or 0
                if heartbeat != heartbeat_value:
                    heartbeat_value = heartbeat
                    heartbeat_changed_at = now
                elif now - heartbeat_changed_at >= HEARTBEAT_TIMEOUT_SECONDS:
                    needs_rebuild = True
                    reason = "the controller heartbeat is not advancing"
                if controller_state != 2:
                    needs_rebuild = True
                    reason = f"the controller is not ready (state={controller_state})"
                if progress_stalled == 1:
                    needs_rebuild = True
                    reason = "production has not advanced for 60 simulation seconds"

            if needs_rebuild and now - last_rebuild_at >= REBUILD_COOLDOWN_SECONDS:
                log(f"Automatic recovery started: {reason}.")
                last_rebuild_at = now
                rebuild_scene()
                client = None
                sim = None

            time.sleep(POLL_SECONDS)
        except KeyboardInterrupt:
            log("The health watchdog was stopped by the user.")
            return 0
        except Exception as exc:
            connection_failures += 1
            log(f"Watchdog connection error {connection_failures}: {exc}")
            client = None
            sim = None
            if connection_failures >= 5:
                log("CoppeliaSim appears to be closed; stopping the health watchdog.")
                return 1
            time.sleep(POLL_SECONDS)


if __name__ == "__main__":
    raise SystemExit(main())
