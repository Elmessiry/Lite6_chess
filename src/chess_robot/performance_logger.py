"""Performance / reliability metrics for the chess robot.

Collects latency, move-execution and error metrics, exports them to
timestamped JSON files under the shared logs directory on a periodic
timer, and flushes a final export on shutdown.

Thread-safety: metric lists are mutated and snapshotted under a single
lock, so ``export_metrics`` never races with concurrent ``log_*`` calls.
"""
import atexit
import json
import threading
import time
from datetime import datetime

from chess_common.logging_setup import logs_dir, setup_logging


class PerformanceLogger:
    """Collects and periodically exports robot performance metrics."""

    def __init__(self, export_interval: float = 300):
        """
        Args:
            export_interval: How often to export metrics, in seconds.
        """
        self.logger = setup_logging("chess_robot.performance")
        self._lock = threading.Lock()
        self.metrics = {
            "latency": [],
            "move_execution": [],
            "errors": [],
        }
        self.export_interval = export_interval
        self.start_time = time.time()
        self._closed = False

        # Event-driven timer so close() can wake the thread immediately.
        self._stop_event = threading.Event()
        self._export_thread = threading.Thread(
            target=self._export_loop, name="metrics-export", daemon=True)
        self._export_thread.start()
        self.logger.info(
            f"Started metrics export thread (interval: {export_interval}s)")

        atexit.register(self.close)

    def _export_loop(self):
        # Event.wait returns True once close() sets the event -> stop.
        while not self._stop_event.wait(self.export_interval):
            self.export_metrics()

    def log_latency(self, operation, start_time, end_time=None):
        """Record and log the latency of an operation, returning it in ms."""
        if end_time is None:
            end_time = time.time()
        duration_ms = (end_time - start_time) * 1000
        with self._lock:
            self.metrics["latency"].append({
                "timestamp": time.time(),
                "operation": operation,
                "duration_ms": duration_ms,
            })
        self.logger.info(f"LATENCY: {operation} took {duration_ms:.2f}ms")
        return duration_ms

    def log_move_execution(self, move, success, planning_time=None, execution_time=None):
        """Record and log a move execution with optional timing breakdown."""
        with self._lock:
            self.metrics["move_execution"].append({
                "timestamp": time.time(),
                "move": move,
                "success": success,
                "planning_time": planning_time,
                "execution_time": execution_time,
            })
        status = "SUCCESS" if success else "FAILURE"
        timing = ""
        # `is not None` so a genuine 0.0ms measurement is not dropped.
        if planning_time is not None and execution_time is not None:
            timing = (f" (planning: {planning_time:.2f}ms, "
                      f"execution: {execution_time:.2f}ms)")
        self.logger.info(f"MOVE: {move} {status}{timing}")

    def log_error(self, component, error_type, details):
        """Record and log an error event."""
        with self._lock:
            self.metrics["errors"].append({
                "timestamp": time.time(),
                "component": component,
                "error_type": error_type,
                "details": details,
            })
        self.logger.error(f"ERROR: {component} - {error_type}: {details}")

    def export_metrics(self):
        """Snapshot metrics under lock and write them to a JSON file.

        Latency and move-execution buckets are cleared after each export;
        errors are retained so the summary stays cumulative. Returns the
        written path, or None on failure.
        """
        try:
            with self._lock:
                snapshot = {
                    "latency": list(self.metrics["latency"]),
                    "move_execution": list(self.metrics["move_execution"]),
                    "errors": list(self.metrics["errors"]),
                }
                self.metrics["latency"] = []
                self.metrics["move_execution"] = []
                # Errors intentionally retained across exports.

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filepath = logs_dir() / f"metrics_{timestamp}.json"
            export_data = {
                "metrics": snapshot,
                "summary": self._generate_summary(snapshot),
                "start_time": self.start_time,
                "end_time": time.time(),
            }
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(export_data, f, indent=2)
            self.logger.info(f"Exported metrics to {filepath}")
            return filepath
        except Exception:
            self.logger.error("Failed to export metrics", exc_info=True)
            return None

    def close(self):
        """Stop the export thread and write a final export. Idempotent."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self._stop_event.set()
        if self._export_thread.is_alive():
            self._export_thread.join(timeout=5)
        self.export_metrics()

    def _generate_summary(self, metrics):
        """Build summary statistics from a metrics snapshot."""
        summary = {}

        # Latency stats
        if metrics["latency"]:
            latencies = {}
            for entry in metrics["latency"]:
                latencies.setdefault(entry["operation"], []).append(
                    entry["duration_ms"])
            summary["latency"] = {
                op: {
                    "min": min(vals),
                    "max": max(vals),
                    "avg": sum(vals) / len(vals),
                    "count": len(vals),
                }
                for op, vals in latencies.items()
            }

        # Move execution stats
        if metrics["move_execution"]:
            total = len(metrics["move_execution"])
            success = sum(1 for m in metrics["move_execution"] if m["success"])
            planning_times = [
                m["planning_time"] for m in metrics["move_execution"]
                if m["planning_time"] is not None
            ]
            execution_times = [
                m["execution_time"] for m in metrics["move_execution"]
                if m["execution_time"] is not None
            ]

            summary["move_execution"] = {
                "total": total,
                "success": success,
                "failure": total - success,
                "success_rate": (success / total) if total else 0,
            }
            if planning_times:
                summary["move_execution"]["planning_time"] = {
                    "min": min(planning_times),
                    "max": max(planning_times),
                    "avg": sum(planning_times) / len(planning_times),
                }
            if execution_times:
                summary["move_execution"]["execution_time"] = {
                    "min": min(execution_times),
                    "max": max(execution_times),
                    "avg": sum(execution_times) / len(execution_times),
                }

        # Error stats
        if metrics["errors"]:
            by_component = {}
            by_type = {}
            for error in metrics["errors"]:
                component = error["component"]
                error_type = error["error_type"]
                by_component[component] = by_component.get(component, 0) + 1
                by_type[error_type] = by_type.get(error_type, 0) + 1
            summary["errors"] = {
                "total": len(metrics["errors"]),
                "by_component": by_component,
                "by_type": by_type,
            }

        return summary
