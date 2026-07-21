"""Shared configuration loading for both halves of the system.

Both the Windows (Fritz scraper) side and the robot (ROS2 container) side
load YAML configs from a single directory, resolved as:

1. ``$CHESS_CONFIG_DIR`` environment variable, if set.
2. ``<repo root>/config`` -- valid for editable installs and the Docker
   bind-mount layout documented in the README.
"""
import os
from pathlib import Path
from typing import Any

import yaml

_REPO_CONFIG = Path(__file__).resolve().parents[2] / "config"


def config_dir() -> Path:
    """Return the active config directory."""
    env = os.environ.get("CHESS_CONFIG_DIR")
    return Path(env) if env else _REPO_CONFIG


def load_config(name: str) -> dict[str, Any]:
    """Load a YAML config file by stem, e.g. ``load_config('board_config')``."""
    path = config_dir() / f"{name}.yaml"
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)
