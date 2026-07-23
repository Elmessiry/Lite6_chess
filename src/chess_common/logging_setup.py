"""Logging configuration shared by both halves of the system.

Loads ``logging_config.yaml`` via :mod:`chess_common.config`, points the file
handler at a timestamped file under the logs directory, and falls back to
basic console logging if configuration fails.

The logs directory is resolved as ``$CHESS_LOG_DIR`` if set, else
``<repo root>/logs``.
"""
import logging
import logging.config
import os
from datetime import datetime
from pathlib import Path

import yaml

from chess_common.config import config_dir

_REPO_LOGS = Path(__file__).resolve().parents[2] / "logs"


def logs_dir() -> Path:
    """Return the active logs directory, creating it if needed."""
    env = os.environ.get("CHESS_LOG_DIR")
    path = Path(env) if env else _REPO_LOGS
    path.mkdir(parents=True, exist_ok=True)
    return path


def setup_logging(component: str | None = None) -> logging.Logger:
    """Configure logging for the application and return a logger.

    Args:
        component: Optional component name for a specific logger.
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = logs_dir() / f"chess_robot_{timestamp}.log"

    try:
        with open(config_dir() / "logging_config.yaml", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        config["handlers"]["file"]["filename"] = str(log_file)
        logging.config.dictConfig(config)
    except Exception:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        )
        logging.exception("Failed to load logging configuration; using basic config")

    return logging.getLogger(component) if component else logging.getLogger()
