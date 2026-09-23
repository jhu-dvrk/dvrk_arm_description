"""Load and validate canonical dVRK arm-description YAML documents."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
import yaml


@dataclass(frozen=True)
class JointConfig:
    name: str
    type: str
    lower: float
    upper: float
    velocity: float


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(result.get(key), dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_robot_document(path: str | Path, _stack: tuple[Path, ...] = ()) -> dict[str, Any]:
    """Load an arm-description YAML document and resolve relative includes."""
    source = Path(path).expanduser().resolve()
    if source in _stack:
        chain = " -> ".join(str(item) for item in (*_stack, source))
        raise ValueError(f"cyclic robot YAML include: {chain}")
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError(f"{source}: expected a YAML mapping")
    includes = document.pop("include", [])
    if isinstance(includes, (str, Path)):
        includes = [includes]
    if not isinstance(includes, list):
        raise ValueError(f"{source}: include must be a path or list of paths")
    merged: dict[str, Any] = {}
    for include in includes:
        include_path = Path(include)
        if not include_path.is_absolute():
            include_path = source.parent / include_path
        merged = _deep_merge(merged, load_robot_document(include_path, (*_stack, source)))
    return _deep_merge(merged, document)


@dataclass(frozen=True)
class RobotConfig:
    name: str
    type: str
    instrument: str | None
    endoscope: str | None
    parent_frame: str
    base_frame: str
    tool_frame: str
    adaptor_frame: str
    base_position: np.ndarray
    base_orientation_xyzw: np.ndarray
    joints: tuple[JointConfig, ...]
    home_position: np.ndarray
    raw: dict[str, Any]


def with_base_pose(config: RobotConfig, *, position: Any | None = None,
                   orientation_xyzw: Any | None = None) -> RobotConfig:
    """Return a robot configuration with a validated base-pose override."""
    base_position = config.base_position if position is None else np.asarray(position, dtype=float)
    base_orientation = (config.base_orientation_xyzw if orientation_xyzw is None
                        else np.asarray(orientation_xyzw, dtype=float))
    if base_position.shape != (3,) or not np.all(np.isfinite(base_position)):
        raise ValueError("overridden base position must have three finite values")
    if (base_orientation.shape != (4,) or not np.all(np.isfinite(base_orientation))
            or np.linalg.norm(base_orientation) == 0.0):
        raise ValueError("overridden base orientation must be a non-zero quaternion")
    base_position = np.array(base_position, dtype=float, copy=True)
    base_orientation = np.array(base_orientation / np.linalg.norm(base_orientation), dtype=float, copy=True)
    base_position.setflags(write=False)
    base_orientation.setflags(write=False)
    return replace(config, base_position=base_position, base_orientation_xyzw=base_orientation)


def load_robot_config(path: str | Path, base_position: Any | None = None,
                      base_orientation_xyzw: Any | None = None,
                      instrument: str | None = None,
                      endoscope: str | None = None) -> RobotConfig:
    """Load and validate one canonical dVRK arm-description YAML document."""
    source = Path(path)
    document = load_robot_document(source)
    if not isinstance(document.get("robot"), dict):
        raise ValueError(f"{source}: expected a top-level 'robot' mapping")
    robot = document["robot"]
    asset = robot.setdefault("asset", {})
    if not isinstance(asset, dict):
        raise ValueError(f"{source}: robot.asset must be a mapping")
    if instrument is not None:
        asset["instrument"] = str(instrument)
    if endoscope is not None:
        asset["endoscope"] = str(endoscope)
    robot_type = str(robot.get("type", "")).upper()
    if robot_type not in {"PSM", "ECM"}:
        raise ValueError(f"{source}: robot.type must be PSM or ECM")
    robot["type"] = robot_type
    required = ("name", "type", "parent_frame", "base_frame", "tool_frame",
                "adaptor_frame", "joints", "home_position")
    missing = [key for key in required if key not in robot]
    if missing:
        raise ValueError(f"{source}: missing robot fields: {', '.join(missing)}")
    base_pose = robot.get("base_pose", {})
    position = np.asarray(base_pose.get("position", [0.0, 0.0, 0.0]), dtype=float)
    orientation = np.asarray(base_pose.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]), dtype=float)
    if position.shape != (3,) or orientation.shape != (4,):
        raise ValueError(f"{source}: base_pose must contain a three-vector and quaternion")
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(orientation)) or np.linalg.norm(orientation) == 0.0:
        raise ValueError(f"{source}: base_pose must contain finite values and a non-zero quaternion")
    joints = []
    for joint in robot["joints"]:
        try:
            item = JointConfig(str(joint["name"]), str(joint["type"]), float(joint["lower"]),
                               float(joint["upper"]), float(joint["velocity"]))
        except KeyError as error:
            raise ValueError(f"{source}: joint missing field {error.args[0]}") from error
        if item.type not in {"revolute", "prismatic"}:
            raise ValueError(f"{source}: unsupported joint type {item.type!r}")
        if (not np.all(np.isfinite([item.lower, item.upper, item.velocity]))
                or item.lower > item.upper or item.velocity <= 0.0):
            raise ValueError(f"{source}: invalid limits for joint {item.name!r}")
        joints.append(item)
    if len({joint.name for joint in joints}) != len(joints):
        raise ValueError(f"{source}: joint names must be unique")
    home = np.asarray(robot["home_position"], dtype=float)
    if home.shape != (len(joints),) or not np.all(np.isfinite(home)):
        raise ValueError(f"{source}: home_position must be a finite vector matching joint count")
    for value, joint in zip(home, joints):
        if not joint.lower <= value <= joint.upper:
            raise ValueError(f"{source}: home position exceeds limits for {joint.name!r}")
    home = np.array(home, dtype=float, copy=True)
    home.setflags(write=False)
    config = RobotConfig(str(robot["name"]), robot_type,
                         str(asset["instrument"]) if asset.get("instrument") is not None else None,
                         str(asset["endoscope"]) if asset.get("endoscope") is not None else None,
                         str(robot["parent_frame"]), str(robot["base_frame"]),
                         str(robot["tool_frame"]), str(robot["adaptor_frame"]), position, orientation,
                         tuple(joints), home, document)
    return with_base_pose(config, position=base_position if base_position is not None else position,
                          orientation_xyzw=(base_orientation_xyzw if base_orientation_xyzw is not None else orientation))
