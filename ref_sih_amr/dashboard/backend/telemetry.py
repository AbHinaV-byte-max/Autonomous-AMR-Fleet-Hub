"""
TelemetryBus — write-only publish from Simulator, read-only subscribe by Dashboard.
This is a one-way event conduit; nothing in /robot or /allocator reads from it.
"""
import asyncio
from typing import Optional


class TelemetryBus:
    """
    Thread-safe asyncio queue.  The Simulator calls publish() synchronously.
    The dashboard backend awaits subscribe() in an async context.
    """
    def __init__(self, maxsize: int = 200):
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=maxsize)

    def publish(self, snapshot: dict) -> None:
        """Called by Simulator.tick() — non-blocking, drops oldest if full."""
        try:
            self._queue.put_nowait(snapshot)
        except asyncio.QueueFull:
            try:
                self._queue.get_nowait()   # drop oldest
                self._queue.put_nowait(snapshot)
            except Exception:
                pass

    async def subscribe(self, timeout: float = 0.5):
        """Yields snapshots; yields None on timeout (lets caller send keep-alives)."""
        try:
            return await asyncio.wait_for(self._queue.get(), timeout=timeout)
        except asyncio.TimeoutError:
            return None
