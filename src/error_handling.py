from datetime import datetime
from typing import List, Dict
from src.models import SystemState

class ErrorHandler:
    def __init__(self):
        self.logs: List[Dict] = []
        self.active_error = False

    def log_error(self, message: str, code: int = 0):
        entry = {
            "timestamp": datetime.now().isoformat(),
            "message": message,
            "code": code
        }
        self.logs.append(entry)
        self.active_error = True
        # For real system, maybe write to file or stderr

    def clear_error(self):
        self.active_error = False

    def has_active_error(self) -> bool:
        return self.active_error

    def get_logs(self) -> List[Dict]:
        return self.logs
