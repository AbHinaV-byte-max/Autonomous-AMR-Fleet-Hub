from typing import List
from interfaces import CommsChannel
from models import IntentMessage

class PubSubChannel(CommsChannel):
    def __init__(self):
        self.current_messages: List[IntentMessage] = []
        self.next_messages: List[IntentMessage] = []

    def send(self, message: IntentMessage) -> None:
        """Broadcast a message to the channel for the next tick."""
        self.next_messages.append(message)

    def receive(self) -> List[IntentMessage]:
        """Receive all broadcast messages sent in the previous tick."""
        return self.current_messages
        
    def clear(self) -> None:
        """Swap buffers. Called by the simulator at the end of a tick."""
        self.current_messages = self.next_messages
        self.next_messages = []
