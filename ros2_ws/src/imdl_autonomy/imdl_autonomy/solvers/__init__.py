"""Task-level mission strategies.

Each task solver decides which robot capability is called and in which order.
ROS communication and low-level motion remain owned by MissionExecutor.
"""

from .registry import create_solver

__all__ = ['create_solver']
