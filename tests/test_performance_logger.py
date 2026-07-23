"""Tests for the performance logger.

PerformanceLogger imports chess_common.logging_setup (importable, no ROS).
setup_logging and export both write files under the logs directory, so
CHESS_LOG_DIR is redirected to a tmp path for every test.
"""
import json
import threading
import time

import pytest

from chess_robot.performance_logger import PerformanceLogger


@pytest.fixture
def logger(monkeypatch, tmp_path):
    monkeypatch.setenv("CHESS_LOG_DIR", str(tmp_path))
    # Large interval so the background timer never fires mid-test.
    pl = PerformanceLogger(export_interval=3600)
    yield pl
    pl.close()


def _read(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def test_log_latency_returns_sane_value(logger):
    start = time.time() - 0.05  # ~50ms ago
    duration = logger.log_latency("op", start)
    assert duration >= 0
    assert 40 <= duration <= 5000  # generous upper bound for slow CI
    assert len(logger.metrics["latency"]) == 1
    assert logger.metrics["latency"][0]["operation"] == "op"


def test_export_writes_valid_json_with_summary(logger):
    logger.log_latency("planning", time.time() - 0.01)
    logger.log_move_execution("e2-e4", True, planning_time=12.0, execution_time=34.0)
    logger.log_error("controller", "boom", "details")

    path = logger.export_metrics()
    assert path is not None and path.exists()

    data = _read(path)
    assert set(data.keys()) == {"metrics", "summary", "start_time", "end_time"}
    assert data["summary"]["latency"]["planning"]["count"] == 1
    assert data["summary"]["move_execution"]["success"] == 1
    assert data["summary"]["errors"]["total"] == 1
    assert data["summary"]["errors"]["by_component"]["controller"] == 1


def test_zero_timing_is_not_dropped(logger):
    # Bug #14: `if planning_time and execution_time` dropped genuine 0.0ms.
    logger.log_move_execution("a2-a3", True, planning_time=0.0, execution_time=0.0)
    path = logger.export_metrics()
    data = _read(path)
    move = data["metrics"]["move_execution"][0]
    assert move["planning_time"] == 0.0
    assert move["execution_time"] == 0.0
    # 0.0 must still appear in the summary timing stats.
    assert data["summary"]["move_execution"]["planning_time"]["min"] == 0.0
    assert data["summary"]["move_execution"]["execution_time"]["max"] == 0.0


def test_export_clears_latency_but_retains_errors(logger):
    logger.log_latency("op", time.time())
    logger.log_error("c", "t", "d")
    logger.export_metrics()
    # Latency bucket cleared, errors retained for cumulative summary.
    assert logger.metrics["latency"] == []
    assert len(logger.metrics["errors"]) == 1


def test_close_flushes_final_export(monkeypatch, tmp_path):
    monkeypatch.setenv("CHESS_LOG_DIR", str(tmp_path))
    pl = PerformanceLogger(export_interval=3600)
    pl.log_latency("op", time.time())
    pl.close()

    metrics_files = list(tmp_path.glob("metrics_*.json"))
    assert metrics_files, "close() should have written a final export"
    # close() is idempotent.
    pl.close()


def test_concurrent_logging_no_loss(monkeypatch, tmp_path):
    monkeypatch.setenv("CHESS_LOG_DIR", str(tmp_path))
    pl = PerformanceLogger(export_interval=3600)

    threads_count = 4
    per_thread = 250
    exported_counts = []
    stop = threading.Event()

    def worker():
        for _ in range(per_thread):
            pl.log_latency("concurrent", time.time())

    def exporter():
        # Concurrently snapshot-and-clear while workers append.
        while not stop.is_set():
            path = pl.export_metrics()
            if path is not None:
                exported_counts.append(len(_read(path)["metrics"]["latency"]))

    workers = [threading.Thread(target=worker) for _ in range(threads_count)]
    exp = threading.Thread(target=exporter)
    exp.start()
    for w in workers:
        w.start()
    for w in workers:
        w.join()
    stop.set()
    exp.join()

    # Final export captures whatever remains.
    final = pl.export_metrics()
    remaining = len(_read(final)["metrics"]["latency"]) if final else 0

    total_exported = sum(exported_counts) + remaining
    assert total_exported == threads_count * per_thread
    pl.close()
