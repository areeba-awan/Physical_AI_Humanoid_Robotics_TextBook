"""
Metrics and monitoring service for the RAG chatbot application.
Tracks API usage, response times, error rates, and other key metrics.
"""

import time
from typing import Dict, Any
from datetime import datetime
import logging

logger = logging.getLogger(__name__)

class MetricsService:
    def __init__(self):
        self.metrics = {
            'api_calls': 0,
            'successful_queries': 0,
            'failed_queries': 0,
            'total_response_time': 0.0,
            'avg_response_time': 0.0,
            'active_sessions': 0
        }

    def record_api_call(self):
        """Record an incoming API call."""
        self.metrics['api_calls'] += 1

    def record_successful_query(self, response_time: float):
        """Record a successful query with its response time."""
        self.metrics['successful_queries'] += 1
        self.metrics['total_response_time'] += response_time
        self.metrics['avg_response_time'] = (
            self.metrics['total_response_time'] / self.metrics['successful_queries']
        )

    def record_failed_query(self):
        """Record a failed query."""
        self.metrics['failed_queries'] += 1

    def increment_active_sessions(self):
        """Increment the active sessions counter."""
        self.metrics['active_sessions'] += 1

    def decrement_active_sessions(self):
        """Decrement the active sessions counter."""
        if self.metrics['active_sessions'] > 0:
            self.metrics['active_sessions'] -= 1

    def get_metrics(self) -> Dict[str, Any]:
        """Get current metrics."""
        return self.metrics.copy()

    def reset_metrics(self):
        """Reset all metrics to zero."""
        for key in self.metrics:
            if isinstance(self.metrics[key], (int, float)):
                self.metrics[key] = 0.0 if isinstance(self.metrics[key], float) else 0

# Global instance
metrics_service = MetricsService()