import os
import logging
import logging.config
import time
from datetime import datetime
import json
import threading
from typing import Optional
from .logging_utils import setup_logging

class PerformanceMetrics:
    """Simple performance metrics collector for chess robot"""
    
    def __init__(self):
        """Initialize with a logger name"""
        self.logger = setup_logging('chess_robot.performance')
    
    def log_latency(self, operation: str, start_time: float) -> float:
        """Log latency for an operation"""
        duration_ms = (time.time() - start_time) * 1000
        self.logger.info(f"LATENCY: {operation} took {duration_ms:.2f}ms")
        return duration_ms
    
    def log_move_execution(self, move: str, success: bool, 
                         planning_time: Optional[float] = None, 
                         execution_time: Optional[float] = None):
        """Log move execution metrics"""
        status = "SUCCESS" if success else "FAILURE"
        timing = ""
        if planning_time and execution_time:
            timing = f" (planning: {planning_time:.2f}ms, execution: {execution_time:.2f}ms)"
        self.logger.info(f"MOVE: {move} {status}{timing}")
    
    def log_error(self, component: str, error_type: str, details: str):
        """Log error information"""
        self.logger.error(f"ERROR: {component} - {error_type}: {details}")

class PerformanceLogger:
    """Enhanced logger for collecting performance and reliability metrics"""
    
    def __init__(self, export_interval=300):
        """
        Initialize the performance logger
        
        Args:
            export_interval: How often to export metrics (in seconds)
        """
        self.logger = setup_logging('chess_robot.performance')
        self.metrics = {
            "latency": [],
            "message_delivery": [],
            "move_execution": [],
            "errors": [],
            "recovery_events": []
        }
        self.export_interval = export_interval
        self.start_time = time.time()
        
        # Create logs directory if it doesn't exist
        logs_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'logs')
        os.makedirs(logs_dir, exist_ok=True)
        
        # Start the export thread
        self._start_export_thread()
    
    def _start_export_thread(self):
        """Start a thread that exports metrics periodically"""
        def export_thread():
            while True:
                time.sleep(self.export_interval)
                self.export_metrics()
        
        thread = threading.Thread(target=export_thread, daemon=True)
        thread.start()
        self.logger.info(f"Started metrics export thread (interval: {self.export_interval}s)")
    
    def log_latency(self, operation, start_time, end_time=None):
        """Log latency for an operation"""
        if end_time is None:
            end_time = time.time()
        
        duration_ms = (end_time - start_time) * 1000
        
        self.metrics["latency"].append({
            "timestamp": time.time(),
            "operation": operation,
            "duration_ms": duration_ms
        })
        
        self.logger.info(f"LATENCY: {operation} took {duration_ms:.2f}ms")
        return duration_ms
    
    def log_message_delivery(self, message_id, status, latency=None):
        """Log message delivery status"""
        self.metrics["message_delivery"].append({
            "timestamp": time.time(),
            "message_id": message_id,
            "status": status,
            "latency": latency
        })
        
        self.logger.info(f"MESSAGE: {message_id} delivery {status}" + 
                        (f" ({latency:.2f}ms)" if latency else ""))
    
    def log_move_execution(self, move, success, planning_time=None, execution_time=None):
        """Log move execution metrics"""
        self.metrics["move_execution"].append({
            "timestamp": time.time(),
            "move": move,
            "success": success,
            "planning_time": planning_time,
            "execution_time": execution_time
        })
        
        status = "SUCCESS" if success else "FAILURE"
        self.logger.info(f"MOVE: {move} {status}" + 
                        (f" (planning: {planning_time:.2f}ms, execution: {execution_time:.2f}ms)" 
                         if planning_time and execution_time else ""))
    
    def log_error(self, component, error_type, details):
        """Log error information"""
        self.metrics["errors"].append({
            "timestamp": time.time(),
            "component": component,
            "error_type": error_type,
            "details": details
        })
        
        self.logger.error(f"ERROR: {component} - {error_type}: {details}")
    
    def log_recovery(self, component, event_type, success, duration=None):
        """Log recovery event"""
        self.metrics["recovery_events"].append({
            "timestamp": time.time(),
            "component": component,
            "event_type": event_type,
            "success": success,
            "duration": duration
        })
        
        status = "SUCCESS" if success else "FAILURE"
        self.logger.info(f"RECOVERY: {component} {event_type} {status}" + 
                        (f" ({duration:.2f}ms)" if duration else ""))
    
    def export_metrics(self):
        """Export collected metrics to a file"""
        try:
            # Create timestamp for filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Create filepath
            logs_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'logs')
            filepath = os.path.join(logs_dir, f"metrics_{timestamp}.json")
            
            # Add summary statistics
            export_data = {
                "metrics": self.metrics,
                "summary": self._generate_summary(),
                "start_time": self.start_time,
                "end_time": time.time()
            }
            
            # Write to file
            with open(filepath, 'w') as f:
                json.dump(export_data, f, indent=2)
            
            self.logger.info(f"Exported metrics to {filepath}")
            
            # Clear metrics after export (but keep a copy of errors)
            errors = self.metrics["errors"].copy()
            recovery = self.metrics["recovery_events"].copy()
            self.metrics = {
                "latency": [],
                "message_delivery": [],
                "move_execution": [],
                "errors": errors,
                "recovery_events": recovery
            }
            
        except Exception as e:
            self.logger.error(f"Failed to export metrics: {e}")
    
    def _generate_summary(self):
        """Generate summary statistics from collected metrics"""
        summary = {}
        
        # Latency stats
        if self.metrics["latency"]:
            latencies = {}
            for entry in self.metrics["latency"]:
                op = entry["operation"]
                if op not in latencies:
                    latencies[op] = []
                latencies[op].append(entry["duration_ms"])
            
            summary["latency"] = {
                op: {
                    "min": min(vals),
                    "max": max(vals),
                    "avg": sum(vals) / len(vals),
                    "count": len(vals)
                }
                for op, vals in latencies.items()
            }
        
        # Message delivery stats
        if self.metrics["message_delivery"]:
            total = len(self.metrics["message_delivery"])
            success = sum(1 for m in self.metrics["message_delivery"] if m["status"] == "success")
            
            summary["message_delivery"] = {
                "total": total,
                "success": success,
                "failure": total - success,
                "success_rate": (success / total) if total else 0
            }
        
        # Move execution stats
        if self.metrics["move_execution"]:
            total = len(self.metrics["move_execution"])
            success = sum(1 for m in self.metrics["move_execution"] if m["success"])
            
            planning_times = [m["planning_time"] for m in self.metrics["move_execution"] 
                             if m["planning_time"] is not None]
            execution_times = [m["execution_time"] for m in self.metrics["move_execution"] 
                              if m["execution_time"] is not None]
            
            summary["move_execution"] = {
                "total": total,
                "success": success,
                "failure": total - success,
                "success_rate": (success / total) if total else 0
            }
            
            if planning_times:
                summary["move_execution"]["planning_time"] = {
                    "min": min(planning_times),
                    "max": max(planning_times),
                    "avg": sum(planning_times) / len(planning_times)
                }
                
            if execution_times:
                summary["move_execution"]["execution_time"] = {
                    "min": min(execution_times),
                    "max": max(execution_times),
                    "avg": sum(execution_times) / len(execution_times)
                }
        
        # Error stats
        if self.metrics["errors"]:
            by_component = {}
            by_type = {}
            
            for error in self.metrics["errors"]:
                component = error["component"]
                error_type = error["error_type"]
                
                if component not in by_component:
                    by_component[component] = 0
                by_component[component] += 1
                
                if error_type not in by_type:
                    by_type[error_type] = 0
                by_type[error_type] += 1
            
            summary["errors"] = {
                "total": len(self.metrics["errors"]),
                "by_component": by_component,
                "by_type": by_type
            }
        
        return summary 