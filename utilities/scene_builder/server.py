#!/usr/bin/env python3
"""Local MAVS scene-builder server."""

from __future__ import annotations

import json
import math
import mimetypes
import re
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse


ROOT = Path(__file__).resolve().parents[2]
STATIC_ROOT = Path(__file__).resolve().parent
DATA_ROOT = ROOT / "data"
SAVE_ROOT = DATA_ROOT / "scenes"
SIM_ROOT = DATA_ROOT / "sims"
PREVIEW_SCRIPT = STATIC_ROOT / "preview_scene.py"
SIM_SCRIPT = STATIC_ROOT / "run_simulation.py"
PREVIEW_LOG = SAVE_ROOT / "scene_builder_preview.log"
SIM_LOG = SAVE_ROOT / "simulation_preview.log"
WAYPOINTS_ROOT = DATA_ROOT / "waypoints"
ACTORS_ROOT = DATA_ROOT / "actors" / "actors"
VEHICLE_DEFS_ROOT = DATA_ROOT / "vehicles" / "rp3d_vehicles"
SENSOR_DEFS_ROOT = DATA_ROOT / "sensors"
NATIVE_SENSOR_MODELS = {
    ("lidar", "M8"),
    ("lidar", "HDL-64E"),
    ("lidar", "HDL-32E"),
    ("lidar", "VLP-16"),
    ("lidar", "LMS-291"),
    ("camera", "Flea3-4mm"),
    ("camera", "XCD-V60"),
    ("radar", "Delphi Long Range"),
    ("radar", "Delphi Mid Range"),
}
NATIVE_SENSOR_TYPES = {"lidar", "camera", "gps", "compass", "fisheye", "radar", "imu"}
HOST = "127.0.0.1"
PORT = 8765


def choose_json_file(title: str, initial_dir: Path) -> Path | None:
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    try:
        selected = filedialog.askopenfilename(
            parent=root,
            title=title,
            initialdir=str(initial_dir),
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
    finally:
        root.destroy()
    return Path(selected).resolve() if selected else None


def choose_json_save_file(title: str, initial_dir: Path, initial_name: str) -> Path | None:
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    try:
        selected = filedialog.asksaveasfilename(
            parent=root,
            title=title,
            initialdir=str(initial_dir),
            initialfile=initial_name,
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
    finally:
        root.destroy()
    return Path(selected).resolve() if selected else None


def vehicle_def_records() -> list[dict]:
    records: list[dict] = []
    if not VEHICLE_DEFS_ROOT.exists():
        return records
    for path in VEHICLE_DEFS_ROOT.glob("*.json"):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            chassis = data.get("Mesh", {})
            tire = data.get("Tire Mesh", {})
            chassis_file = str(chassis.get("File", ""))
            tire_file = str(tire.get("File", ""))
            if not chassis_file:
                continue
            axles = []
            for axle in data.get("Axles", []):
                tire_data = axle.get("Tire", {})
                axles.append({
                    "longitudinal_offset": float(axle.get("Longitudinal Offset", 0.0)),
                    "track_width": float(axle.get("Track Width", 1.5)),
                    "tire_radius": float(tire_data.get("Radius", 0.35)),
                    "spring_length": float(axle.get("Spring Length", 0.5)),
                })
            chassis_data = data.get("Chassis", {})
            axle_offsets = [axle["longitudinal_offset"] for axle in axles]
            def editor_path(raw: str) -> str:
                """Vehicle JSONs store mesh paths relative to scenes/meshes/."""
                if not raw:
                    return ""
                candidate = DATA_ROOT / "scenes" / "meshes" / raw
                if candidate.exists():
                    return f"scenes/meshes/{raw}"
                return raw
            records.append({
                "name": path.stem,
                "definition_file": path.name,
                "cg_offset": float(chassis_data.get("CG Offset", 0.0)),
                "wheelbase": max(axle_offsets) - min(axle_offsets) if len(axle_offsets) >= 2 else 1.25,
                "chassis_mesh": editor_path(chassis_file),
                "chassis_offset": chassis.get("Offset", [0.0, 0.0, 0.0]),
                "chassis_rotation": [
                    90.0 if chassis.get("Rotate Y to Z", False) else 0.0,
                    0.0,
                    (90.0 if chassis.get("Rotate X to Y", False) else 0.0)
                    + (-90.0 if chassis.get("Rotate Y to X", False) else 0.0),
                ],
                "chassis_scale": chassis.get("Scale", [1.0, 1.0, 1.0]),
                "tire_mesh": editor_path(tire_file),
                "tire_offset": tire.get("Offset", [0.0, 0.0, 0.0]),
                "tire_rotation": [
                    90.0 if tire.get("Rotate Y to Z", False) else 0.0,
                    0.0,
                    (90.0 if tire.get("Rotate X to Y", False) else 0.0)
                    + (-90.0 if tire.get("Rotate Y to X", False) else 0.0),
                ],
                "tire_scale": tire.get("Scale", [1.0, 1.0, 1.0]),
                "axles": axles,
            })
        except Exception:
            continue
    return sorted(records, key=lambda r: r["name"])


def _linear_angles(low: float, high: float, step: float) -> list[float]:
    if step <= 0:
        return [low]
    count = max(1, int(round((high - low) / step)) + 1)
    return [low + index * (high - low) / max(1, count - 1) for index in range(count)]


def _sensor_type(data: dict) -> str:
    explicit = str(data.get("Type", "")).lower()
    if explicit:
        return explicit
    if "Scan Pattern" in data:
        return "lidar"
    if "Pixels" in data or "FocalLength" in data or "FocalPlaneDimensions" in data:
        return "camera"
    if "Field of View" in data or "Lobe Size" in data:
        return "radar"
    if "Accelerometer" in data or "Gyroscope" in data or "Sample Rate" in data:
        return "imu"
    return ""


def _sensor_metadata(sensor_type: str, data: dict) -> dict:
    metadata: dict = {}
    if sensor_type in {"camera", "fisheye"}:
        focal_length = float(data.get("FocalLength", 0) or 0)
        dimensions = data.get("FocalPlaneDimensions", [])
        if focal_length > 0 and isinstance(dimensions, list) and len(dimensions) >= 2:
            metadata["cameraFov"] = {
                "tanHalfHorizontal": float(dimensions[0]) / (2 * focal_length),
                "tanHalfVertical": float(dimensions[1]) / (2 * focal_length),
            }
        if isinstance(data.get("Pixels"), list):
            metadata["pixels"] = data["Pixels"][:2]
    elif sensor_type == "lidar":
        scan = data.get("Scan Pattern", {})
        horizontal = scan.get("Horizontal Range", [])
        vertical = scan.get("Vertical Range", [])
        if isinstance(horizontal, list) and len(horizontal) >= 2:
            metadata["horizontalRange"] = horizontal[:2]
            metadata["horizontalStep"] = float(scan.get("Horizontal Step", 5) or 5)
        if isinstance(vertical, list) and len(vertical) >= 2:
            step = float(scan.get("Vertical Step", 1) or 1)
            metadata["verticalAngles"] = _linear_angles(float(vertical[0]), float(vertical[1]), step)
        if "Max Range" in data:
            metadata["maxRange"] = float(data["Max Range"])
        if "Min Range" in data:
            metadata["minRange"] = float(data["Min Range"])
    elif sensor_type == "radar":
        if "Field of View" in data:
            metadata["fieldOfView"] = float(data["Field of View"])
        if "Max Range" in data:
            metadata["maxRange"] = float(data["Max Range"])
    return metadata


def sensor_catalog() -> dict:
    entries = [
        {"id": "builtin:lidar:M8", "type": "lidar", "model": "M8", "label": "M8", "source": "builtin",
         "metadata": {"verticalAngles": _linear_angles(-18.22, 3.2, 3.06), "maxRange": 100.0}},
        {"id": "builtin:lidar:HDL-64E", "type": "lidar", "model": "HDL-64E", "label": "HDL-64E", "source": "builtin",
         "metadata": {"verticalAngles": _linear_angles(-24.8, 2.0, 0.425396825), "maxRange": 90.0}},
        {"id": "builtin:lidar:HDL-32E", "type": "lidar", "model": "HDL-32E", "label": "HDL-32E", "source": "builtin",
         "metadata": {"verticalAngles": _linear_angles(-30.6623, 10.67, 1.3333)}},
        {"id": "builtin:lidar:VLP-16", "type": "lidar", "model": "VLP-16", "label": "VLP-16", "source": "builtin",
         "metadata": {"verticalAngles": _linear_angles(-15.0, 15.0, 2.0)}},
        {"id": "builtin:lidar:LMS-291", "type": "lidar", "model": "LMS-291", "label": "LMS-291", "source": "builtin",
         "metadata": {"verticalAngles": [0.0], "horizontalRange": [-90.0, 90.0]}},
        {"id": "builtin:camera:Flea3-4mm", "type": "camera", "model": "Flea3-4mm", "label": "Flea3-4mm", "source": "builtin",
         "metadata": {"cameraFov": {"tanHalfHorizontal": 0.006784 / 0.008, "tanHalfVertical": 0.0054272 / 0.008}}},
        {"id": "builtin:camera:XCD-V60", "type": "camera", "model": "XCD-V60", "label": "XCD-V60", "source": "builtin",
         "metadata": {"cameraFov": {"tanHalfHorizontal": 0.0024 / 0.007, "tanHalfVertical": 0.0018 / 0.007}}},
        {"id": "builtin:radar:Delphi Long Range", "type": "radar", "model": "Delphi Long Range", "label": "Delphi Long Range", "source": "builtin", "metadata": {}},
        {"id": "builtin:radar:Delphi Mid Range", "type": "radar", "model": "Delphi Mid Range", "label": "Delphi Mid Range", "source": "builtin", "metadata": {}},
    ]
    if SENSOR_DEFS_ROOT.exists():
        for path in sorted(SENSOR_DEFS_ROOT.rglob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                sensor_type = _sensor_type(data)
                if not sensor_type:
                    continue
                rel = path.relative_to(DATA_ROOT).as_posix()
                entries.append({
                    "id": f"json:{rel}",
                    "type": sensor_type,
                    "model": "",
                    "label": path.stem,
                    "source": "json",
                    "inputFile": str(path.resolve()),
                    "relativePath": rel,
                    "metadata": _sensor_metadata(sensor_type, data),
                })
            except Exception:
                continue
    return {
        "types": ["lidar", "camera", "gps", "compass", "fisheye", "radar", "imu"],
        "models": entries,
    }


def native_sensor_record(raw_sensor: object) -> dict | None:
    if not isinstance(raw_sensor, dict):
        return None
    sensor = dict(raw_sensor)
    sensor_type = str(sensor.get("Type", "")).lower()
    sensor["Type"] = sensor_type
    if sensor_type not in NATIVE_SENSOR_TYPES:
        return None
    if sensor_type == "compass":
        sensor.setdefault("Input File", "compass")
    elif sensor_type == "gps":
        pass
    elif sensor.get("Model") and (sensor_type, str(sensor["Model"])) not in NATIVE_SENSOR_MODELS:
        return None
    elif not sensor.get("Model") and not sensor.get("Input File"):
        return None
    sensor.setdefault("Number Processors", 1)
    return sensor


def safe_data_path(relative_path: str) -> Path:
    candidate = (DATA_ROOT / unquote(relative_path)).resolve()
    if not candidate.is_relative_to(DATA_ROOT.resolve()):
        raise ValueError("Path escapes data directory")
    return candidate


def asset_records() -> list[dict[str, str]]:
    records: list[dict[str, str]] = []
    for path in DATA_ROOT.rglob("*.obj"):
        rel = path.relative_to(DATA_ROOT).as_posix()
        records.append(
            {
                "name": path.name,
                "path": rel,
                "folder": path.parent.relative_to(DATA_ROOT).as_posix(),
            }
        )
    return sorted(records, key=lambda item: item["path"].lower())


def scene_records() -> list[dict[str, str]]:
    records: list[dict[str, str]] = []
    for path in SAVE_ROOT.glob("*.json"):
        records.append(
            {
                "name": path.name,
                "path": path.relative_to(SAVE_ROOT).as_posix(),
                "relative_path": path.relative_to(ROOT).as_posix(),
            }
        )
    return sorted(records, key=lambda item: item["name"].lower())


def safe_waypoint_path(relative_path: str) -> Path:
    candidate = (WAYPOINTS_ROOT / unquote(relative_path)).resolve()
    if not candidate.is_relative_to(WAYPOINTS_ROOT.resolve()):
        raise ValueError("Path escapes waypoints directory")
    if candidate.suffix.lower() != ".json":
        raise ValueError("Waypoint file must be a JSON file")
    return candidate


def sanitize_waypoint_name(raw_name: str) -> str:
    name = raw_name.strip() or "waypoints"
    name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name)
    if not name.lower().endswith(".json"):
        name += ".json"
    return name


def waypoint_records() -> list[dict[str, str]]:
    records: list[dict[str, str]] = []
    if WAYPOINTS_ROOT.exists():
        for path in WAYPOINTS_ROOT.glob("*.json"):
            records.append(
                {
                    "name": path.name,
                    "path": path.relative_to(WAYPOINTS_ROOT).as_posix(),
                    "relative_path": path.relative_to(ROOT).as_posix(),
                }
            )
    return sorted(records, key=lambda item: item["name"].lower())


def safe_scene_path(relative_path: str) -> Path:
    candidate = (SAVE_ROOT / unquote(relative_path)).resolve()
    if not candidate.is_relative_to(SAVE_ROOT.resolve()):
        raise ValueError("Path escapes scene directory")
    if candidate.suffix.lower() != ".json":
        raise ValueError("Scene must be a JSON file")
    return candidate


def sanitize_scene_name(raw_name: str) -> str:
    name = raw_name.strip() or "scene_builder_scene"
    name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name)
    if not name.lower().endswith(".json"):
        name += ".json"
    return name


class SceneBuilderHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args: object) -> None:
        print(f"{self.client_address[0]} - {fmt % args}")

    def send_json(self, payload: object, status: int = 200) -> None:
        body = json.dumps(payload, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_file(self, path: Path) -> None:
        if not path.exists() or not path.is_file():
            self.send_error(404)
            return
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _save_waypoints_list(self, waypoints_list: list) -> int:
        """Save a list of {name, waypoints} dicts to WAYPOINTS_ROOT. Returns count saved."""
        WAYPOINTS_ROOT.mkdir(parents=True, exist_ok=True)
        saved = 0
        for entry in waypoints_list:
            if not isinstance(entry, dict):
                continue
            wp_name = sanitize_waypoint_name(str(entry.get("name", "")))
            wp_path = (WAYPOINTS_ROOT / wp_name).resolve()
            if not wp_path.is_relative_to(WAYPOINTS_ROOT.resolve()):
                continue
            raw = entry.get("waypoints", [])
            if not isinstance(raw, list) or not raw:
                continue
            wp_data = {
                "Waypoints": [[float(p[0]), float(p[1])] for p in raw],
                "Solve Potential": True,
                "Map Resolution": 1.0,
            }
            wp_path.write_text(json.dumps(wp_data, indent=4), encoding="utf-8")
            saved += 1
        return saved

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/assets":
            self.send_json({"assets": asset_records()})
            return
        if parsed.path == "/api/scenes":
            self.send_json({"scenes": scene_records()})
            return
        if parsed.path == "/api/waypoints":
            self.send_json({"waypoints": waypoint_records()})
            return
        if parsed.path == "/api/vehicle-defs":
            self.send_json({"vehicles": vehicle_def_records()})
            return
        if parsed.path == "/api/sensor-catalog":
            self.send_json(sensor_catalog())
            return
        if parsed.path == "/api/scene":
            path = parse_qs(parsed.query).get("path", [""])[0]
            try:
                scene_path = safe_scene_path(path)
                self.send_json({"name": scene_path.name, "scene": json.loads(scene_path.read_text(encoding="utf-8"))})
            except Exception as exc:
                self.send_json({"error": str(exc)}, status=400)
            return
        if parsed.path == "/api/file":
            path = parse_qs(parsed.query).get("path", [""])[0]
            try:
                self.send_file(safe_data_path(path))
            except ValueError:
                self.send_error(400, "Invalid data path")
            return

        rel = "index.html" if parsed.path in {"", "/"} else parsed.path.lstrip("/")
        target = (STATIC_ROOT / rel).resolve()
        if not target.is_relative_to(STATIC_ROOT.resolve()):
            self.send_error(400, "Invalid static path")
            return
        self.send_file(target)

    def do_POST(self) -> None:
        parsed_path = urlparse(self.path).path
        if parsed_path == "/api/load-scene-file":
            try:
                target = choose_json_file("Load Scene", SAVE_ROOT)
                if target is None:
                    self.send_json({"cancelled": True})
                    return
                self.send_json({
                    "name": target.name,
                    "path": str(target),
                    "scene": json.loads(target.read_text(encoding="utf-8")),
                })
            except Exception as exc:
                self.send_json({"error": str(exc)}, status=400)
            return

        if parsed_path == "/api/save-scene-file":
            length = int(self.headers.get("Content-Length", "0"))
            try:
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                requested_path = payload.get("path")
                target = Path(requested_path).resolve() if requested_path else choose_json_save_file(
                    "Save Scene",
                    SAVE_ROOT,
                    sanitize_scene_name(str(payload.get("name", "scene_builder_scene.json"))),
                )
                if target is None:
                    self.send_json({"cancelled": True})
                    return
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(json.dumps(payload["scene"], indent=2), encoding="utf-8")
                self.send_json({"saved": True, "name": target.name, "path": str(target)})
            except Exception as exc:
                self.send_json({"saved": False, "error": str(exc)}, status=400)
            return

        if parsed_path == "/api/load-simulation-file":
            try:
                target = choose_json_file("Load Simulation", SIM_ROOT)
                if target is None:
                    self.send_json({"cancelled": True})
                    return
                simulation = json.loads(target.read_text(encoding="utf-8"))
                scene_value = simulation.get("Scene", {}).get("Input File")
                if not scene_value:
                    raise ValueError("Simulation does not contain Scene.Input File")
                scene_path = Path(str(scene_value))
                if not scene_path.is_absolute():
                    scene_path = (DATA_ROOT / scene_path).resolve()
                if not scene_path.exists():
                    raise ValueError(f"Simulation scene does not exist: {scene_path}")
                driver_value = simulation.get("Driver", {}).get("Input File")
                driver_name = "simulation_path.json"
                waypoints = []
                if driver_value:
                    driver_path = Path(str(driver_value))
                    if not driver_path.is_absolute():
                        driver_path = (DATA_ROOT / driver_path).resolve()
                    driver_name = driver_path.name
                    if driver_path.exists():
                        driver_data = json.loads(driver_path.read_text(encoding="utf-8"))
                        waypoints = driver_data.get("Waypoints", [])
                self.send_json({
                    "name": target.name,
                    "path": str(target),
                    "simulation": simulation,
                    "driver_name": driver_name,
                    "waypoints": waypoints,
                    "scene_name": scene_path.name,
                    "scene_path": str(scene_path),
                    "scene": json.loads(scene_path.read_text(encoding="utf-8")),
                })
            except Exception as exc:
                self.send_json({"error": str(exc)}, status=400)
            return

        if parsed_path == "/api/preview-scene":
            length = int(self.headers.get("Content-Length", "0"))
            try:
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                name = sanitize_scene_name(str(payload.get("name", "scene_builder_preview.json")))
                scene = payload["scene"]
                preview_name = f"_preview_{name}"
                SAVE_ROOT.mkdir(parents=True, exist_ok=True)
                target = (SAVE_ROOT / preview_name).resolve()
                if not target.is_relative_to(SAVE_ROOT.resolve()):
                    raise ValueError("Invalid preview path")
                target.write_text(json.dumps(scene, indent=2), encoding="utf-8")
                cmd = [sys.executable, str(PREVIEW_SCRIPT), str(target)]
                cam = payload.get("camera")
                if isinstance(cam, dict):
                    pos = cam.get("position")
                    if isinstance(pos, list) and len(pos) == 3:
                        cmd += [
                            "--x",       str(float(pos[0])),
                            "--y",       str(float(pos[1])),
                            "--z",       str(float(pos[2])),
                            "--heading", str(float(cam.get("heading", 0.0))),
                            "--pitch",   str(float(cam.get("pitch",   0.0))),
                        ]
                if sys.platform.startswith("win"):
                    with PREVIEW_LOG.open("w", encoding="utf-8") as preview_log:
                        process = subprocess.Popen(
                            cmd,
                            cwd=str(ROOT),
                            stdout=preview_log,
                            stderr=subprocess.STDOUT,
                            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW,
                        )
                else:
                    with PREVIEW_LOG.open("w", encoding="utf-8") as preview_log:
                        process = subprocess.Popen(
                            cmd,
                            cwd=str(ROOT),
                            stdout=preview_log,
                            stderr=subprocess.STDOUT,
                        )
                time.sleep(1.0)
                if process.poll() is not None:
                    details = PREVIEW_LOG.read_text(encoding="utf-8", errors="replace").strip() if PREVIEW_LOG.exists() else ""
                    raise RuntimeError(details[-1200:] or f"Preview exited with code {process.returncode}")
                self.send_json(
                    {
                        "started": True,
                        "scene": target.relative_to(ROOT).as_posix(),
                        "log": PREVIEW_LOG.relative_to(ROOT).as_posix(),
                    }
                )
            except Exception as exc:
                self.send_json({"started": False, "error": str(exc)}, status=400)
            return

        if parsed_path == "/api/save-waypoints":
            length = int(self.headers.get("Content-Length", "0"))
            try:
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                name = sanitize_waypoint_name(str(payload.get("name", "")))
                waypoints = payload["waypoints"]
                if not isinstance(waypoints, list):
                    raise ValueError("waypoints must be a list")
                WAYPOINTS_ROOT.mkdir(parents=True, exist_ok=True)
                target = (WAYPOINTS_ROOT / name).resolve()
                if not target.is_relative_to(WAYPOINTS_ROOT.resolve()):
                    raise ValueError("Invalid waypoint path")
                waypoint_data = {
                    "Waypoints": [[float(p[0]), float(p[1])] for p in waypoints],
                    "Solve Potential": True,
                    "Map Resolution": 1.0,
                }
                target.write_text(json.dumps(waypoint_data, indent=4), encoding="utf-8")
                self.send_json(
                    {
                        "saved": True,
                        "path": str(target),
                        "relative_path": target.relative_to(ROOT).as_posix(),
                    }
                )
            except Exception as exc:
                self.send_json({"saved": False, "error": str(exc)}, status=400)
            return

        if parsed_path == "/api/export-sim-config":
            length = int(self.headers.get("Content-Length", "0"))
            try:
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                base_name = sanitize_scene_name(str(payload.get("name", "sim")))
                name_stem = base_name[:-5] if base_name.lower().endswith(".json") else base_name
                scene_data = payload["scene"]
                vehicles_data = payload.get("vehicles", {}).get("vehicles", [])
                waypoints_list = payload.get("waypoints", [])
                primary_path_name = sanitize_waypoint_name(str(payload.get("primary_path_name", "path.json")))

                # Save scene JSON
                SAVE_ROOT.mkdir(parents=True, exist_ok=True)
                scene_path = (SAVE_ROOT / base_name).resolve()
                if not scene_path.is_relative_to(SAVE_ROOT.resolve()):
                    raise ValueError("Invalid scene path")
                scene_path.write_text(json.dumps(scene_data, indent=2), encoding="utf-8")

                # Save all waypoints
                saved_wp = self._save_waypoints_list(waypoints_list)

                # Pick the first path-following vehicle
                path_vehicle = next(
                    (v for v in vehicles_data
                     if str(v.get("controller_mode", "")).lower() == "path"
                     and len(v.get("waypoints", [])) >= 2),
                    None,
                )
                if not path_vehicle:
                    raise ValueError("No vehicle assigned to a path with at least 2 waypoints")

                # Read vehicle def; patch out veg forces if scene has no vegetation file
                def_name = str(path_vehicle["definition_file"])
                def_path = DATA_ROOT / "vehicles" / "rp3d_vehicles" / def_name
                cg_height = 0.5
                try:
                    def_data = json.loads(def_path.read_text(encoding="utf-8"))
                    cg_height = float(def_data.get("Chassis", {}).get("CG Offset", 0.5))
                    if def_data.get("Using Veg Forces") and not scene_data.get("Vegetation File"):
                        def_data["Using Veg Forces"] = False
                        runtime_root = DATA_ROOT / "vehicles" / "rp3d_vehicles" / "_scene_builder_runtime"
                        runtime_root.mkdir(parents=True, exist_ok=True)
                        patched_path = (runtime_root / def_name).resolve()
                        patched_path.write_text(json.dumps(def_data), encoding="utf-8")
                        def_path = patched_path
                except Exception:
                    pass

                # Convert yaw heading to quaternion [w, x, y, z] (rotation around Z)
                heading_rad = math.radians(float(path_vehicle.get("initial_heading_degrees", 0.0)))
                half = heading_rad / 2.0
                orientation = [math.cos(half), 0.0, 0.0, math.sin(half)]

                # Build sensor list; inject GPS and compass if absent (both required by A* path planner)
                sensors = []
                for raw_sensor in path_vehicle.get("sensors", []):
                    sensor = native_sensor_record(raw_sensor)
                    if sensor is not None:
                        sensors.append(sensor)
                if not any(str(s.get("Type", "")).lower() == "gps" for s in sensors):
                    sensors.append({
                        "Type": "gps",
                        "Name": "gps",
                        "Offset": [0.0, 0.0, 0.0],
                        "Orientation": [1.0, 0.0, 0.0, 0.0],
                        "Repitition Rate (Hz)": 10.0,
                        "Number Processors": 1,
                    })
                if not any(str(s.get("Type", "")).lower() == "compass" for s in sensors):
                    sensors.append({
                        "Type": "compass",
                        "Name": "compass",
                        "Input File": "compass",
                        "Offset": [0.0, 0.0, 0.0],
                        "Orientation": [1.0, 0.0, 0.0, 0.0],
                        "Repitition Rate (Hz)": 10.0,
                        "Number Processors": 1,
                    })
                if not any(str(s.get("Type", "")).lower() == "lidar" for s in sensors):
                    sensors.append({
                        "Type": "lidar",
                        "Model": "VLP-16",
                        "Name": "lidar",
                        "Offset": [0.0, 0.0, 1.0],
                        "Orientation": [1.0, 0.0, 0.0, 0.0],
                        "Repitition Rate (Hz)": 10.0,
                        "Number Processors": 1,
                    })
                initial_pos = path_vehicle.get("initial_position", [0.0, 0.0, 0.0])

                waypoint_abs = str((WAYPOINTS_ROOT / primary_path_name).resolve())
                vehicle_abs = str(def_path.resolve())
                scene_abs = str(scene_path)

                sim_config = {
                    "Environment": {
                        "Month": 6,
                        "Year": 2025,
                        "Day": 1,
                        "Hour": 12,
                        "Minute": 0,
                        "Second": 0,
                        "Turbidity": 2.0,
                        "Local Albedo": [0.1, 0.1, 0.1],
                    },
                    "Driver": {
                        "Type": "A* Planner",
                        "Input File": waypoint_abs,
                        "Number Processors": 1,
                        "Update Rate": 10.0,
                    },
                    "Vehicle": {
                        "Type": "Rp3d",
                        "Initial Position": [float(initial_pos[0]), float(initial_pos[1]), float(initial_pos[2] if len(initial_pos) > 2 else 0.0)],
                        "Initial Orientation": orientation,
                        "CG Height": cg_height,
                        "Update Rate": 100.0,
                        "Input File": vehicle_abs,
                        "Number Processors": 1,
                    },
                    "Scene": {
                        "Input File": scene_abs,
                        "Origin": [32.3526, -90.8779, 73.152],
                        "Time Zone": -6,
                    },
                    "Display Sensors": True,
                    "Save Data": False,
                    "Sensors": sensors,
                    "Max Sim Time": 300.0,
                    "Time Step": 0.01,
                }

                SIM_ROOT.mkdir(parents=True, exist_ok=True)
                if payload.get("choose_path"):
                    sim_config_path = choose_json_save_file(
                        "Export Simulation",
                        SIM_ROOT,
                        f"{name_stem}_sim.json",
                    )
                    if sim_config_path is None:
                        self.send_json({"cancelled": True})
                        return
                else:
                    sim_config_path = (SIM_ROOT / f"{name_stem}_sim.json").resolve()
                sim_config_path.write_text(json.dumps(sim_config, indent=4), encoding="utf-8")

                self.send_json({
                    "saved": True,
                    "sim_config_path": str(sim_config_path),
                    "scene_path": scene_path.relative_to(ROOT).as_posix(),
                    "waypoints_saved": saved_wp,
                })
            except Exception as exc:
                self.send_json({"saved": False, "error": str(exc)}, status=400)
            return

        if parsed_path == "/api/export-simulation":
            length = int(self.headers.get("Content-Length", "0"))
            try:
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                name = sanitize_scene_name(str(payload.get("name", "actors.json")))
                actors_data = payload["actors"]
                waypoints_list = payload.get("waypoints", [])
                saved_wp = self._save_waypoints_list(waypoints_list)
                ACTORS_ROOT.mkdir(parents=True, exist_ok=True)
                target = (ACTORS_ROOT / name).resolve()
                if not target.is_relative_to(ACTORS_ROOT.resolve()):
                    raise ValueError("Invalid actors path")
                target.write_text(json.dumps(actors_data, indent=4), encoding="utf-8")
                self.send_json(
                    {
                        "saved": True,
                        "actors_path": target.relative_to(ROOT).as_posix(),
                        "waypoints_saved": saved_wp,
                    }
                )
            except Exception as exc:
                self.send_json({"saved": False, "error": str(exc)}, status=400)
            return

        if parsed_path == "/api/run-simulation":
            length = int(self.headers.get("Content-Length", "0"))
            try:
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                base_name = sanitize_scene_name(str(payload.get("name", "sim_preview")))
                scene = payload["scene"]
                vehicles_data = payload["vehicles"]
                waypoints_list = payload.get("waypoints", [])

                # Write temp scene
                SAVE_ROOT.mkdir(parents=True, exist_ok=True)
                scene_target = (SAVE_ROOT / f"_preview_{base_name}").resolve()
                if not scene_target.is_relative_to(SAVE_ROOT.resolve()):
                    raise ValueError("Invalid preview scene path")
                scene_target.write_text(json.dumps(scene, indent=2), encoding="utf-8")

                # Write waypoints (permanent — same as export)
                self._save_waypoints_list(waypoints_list)

                # Write temp vehicle/controller configuration
                ACTORS_ROOT.mkdir(parents=True, exist_ok=True)
                vehicles_target = (ACTORS_ROOT / f"_preview_{base_name}").resolve()
                if not vehicles_target.is_relative_to(ACTORS_ROOT.resolve()):
                    raise ValueError("Invalid preview vehicles path")
                vehicles_target.write_text(json.dumps(vehicles_data, indent=4), encoding="utf-8")

                if sys.platform.startswith("win"):
                    with SIM_LOG.open("w", encoding="utf-8") as sim_log:
                        process = subprocess.Popen(
                            [sys.executable, str(SIM_SCRIPT), str(scene_target), str(vehicles_target)],
                            cwd=str(ROOT),
                            stdout=sim_log,
                            stderr=subprocess.STDOUT,
                            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW,
                        )
                else:
                    with SIM_LOG.open("w", encoding="utf-8") as sim_log_f:
                        process = subprocess.Popen(
                            [sys.executable, str(SIM_SCRIPT), str(scene_target), str(vehicles_target)],
                            cwd=str(ROOT),
                            stdout=sim_log_f,
                            stderr=subprocess.STDOUT,
                        )

                time.sleep(1.0)
                if process.poll() is not None:
                    details = SIM_LOG.read_text(encoding="utf-8", errors="replace").strip() if SIM_LOG.exists() else ""
                    raise RuntimeError(details[-1200:] or f"Simulation exited early with code {process.returncode}")

                self.send_json({"started": True})
            except Exception as exc:
                self.send_json({"started": False, "error": str(exc)}, status=400)
            return

        if parsed_path != "/api/save-scene":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", "0"))
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            name = sanitize_scene_name(str(payload.get("name", "")))
            scene = payload["scene"]
            SAVE_ROOT.mkdir(parents=True, exist_ok=True)
            target = (SAVE_ROOT / name).resolve()
            if not target.is_relative_to(SAVE_ROOT.resolve()):
                raise ValueError("Invalid save path")
            target.write_text(json.dumps(scene, indent=2), encoding="utf-8")
            self.send_json(
                {
                    "saved": True,
                    "path": str(target),
                    "relative_path": target.relative_to(ROOT).as_posix(),
                }
            )
        except Exception as exc:
            self.send_json({"saved": False, "error": str(exc)}, status=400)


def main() -> None:
    server = ThreadingHTTPServer((HOST, PORT), SceneBuilderHandler)
    print(f"MAVS scene builder: http://{HOST}:{PORT}")
    print(f"Browsing assets from: {DATA_ROOT}")
    print(f"Saving scenes to: {SAVE_ROOT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nScene builder server stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
