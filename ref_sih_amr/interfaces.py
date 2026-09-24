"""
Abstract Base Classes defining the frozen interfaces for pluggable modules.

Phase 0 architecture freeze — do not modify signatures without team agreement.
"""

from abc import ABC, abstractmethod
from typing import List, Dict, Any, Tuple, Optional
from models import RobotState, Task, IntentMessage


class Planner(ABC):
    @abstractmethod
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int], costmap: Any, reservation_table: Optional[Any] = None, start_time: float = 0.0, robot_id: str = "") -> List[Tuple[int, int]]:
        """Plan a path from start to goal given a costmap."""
        pass


class TaskAllocator(ABC):
    @abstractmethod
    def allocate(self, robots: List[RobotState], tasks: List[Task]) -> Dict[str, str]:
        """
        Allocate tasks to robots.
        Returns a dictionary mapping robot_id to task_id.
        """
        pass


class ConflictResolver(ABC):
    @abstractmethod
    def check_and_resolve(self, reservations: List[Tuple[Tuple[int, int], float]], proposed_path: List[Tuple[int, int]]) -> Any:
        """Check for conflicts and resolve them for a proposed path."""
        pass


class CommsChannel(ABC):
    @abstractmethod
    def send(self, message: IntentMessage) -> None:
        """Send a message through the channel."""
        pass

    @abstractmethod
    def receive(self) -> List[IntentMessage]:
        """Receive a list of messages from the channel."""
        pass


class SecurityValidator(ABC):
    @abstractmethod
    def validate(self, message: IntentMessage) -> str:
        """
        Validate an incoming message.
        Returns 'accept', 'quarantine', or 'reject'.
        """
        pass
