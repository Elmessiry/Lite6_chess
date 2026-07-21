"""Tests for the shared config loader."""
import pytest

from chess_common.config import config_dir, load_config


def test_env_var_overrides_default(monkeypatch, tmp_path):
    monkeypatch.setenv("CHESS_CONFIG_DIR", str(tmp_path))
    assert config_dir() == tmp_path


def test_default_points_at_repo_config(monkeypatch):
    monkeypatch.delenv("CHESS_CONFIG_DIR", raising=False)
    assert config_dir().name == "config"
    assert (config_dir() / "board_config.yaml").exists()


def test_missing_file_raises(monkeypatch, tmp_path):
    monkeypatch.setenv("CHESS_CONFIG_DIR", str(tmp_path))
    with pytest.raises(FileNotFoundError):
        load_config("nonexistent")


def test_real_board_config_loads(monkeypatch):
    monkeypatch.delenv("CHESS_CONFIG_DIR", raising=False)
    config = load_config("board_config")
    assert {"board", "capture_zone", "robot"} <= config.keys()
    assert config["board"]["square_size"] == 0.03


def test_real_messaging_config_has_split_hosts(monkeypatch):
    monkeypatch.delenv("CHESS_CONFIG_DIR", raising=False)
    rabbit = load_config("messaging_config")["rabbitmq"]
    assert "host_from_windows" in rabbit
    assert "host_from_container" in rabbit
    assert "host" not in rabbit  # old ambiguous key must stay dead


def test_env_config_dir_is_used_for_loading(monkeypatch, tmp_path):
    (tmp_path / "custom.yaml").write_text("value: 42\n")
    monkeypatch.setenv("CHESS_CONFIG_DIR", str(tmp_path))
    assert load_config("custom") == {"value": 42}
