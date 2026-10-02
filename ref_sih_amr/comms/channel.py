"""Robot-to-robot communication transports.

The deterministic PubSubChannel is kept for lockstep unit tests. Production
robot processes can use UdpPeerChannel: every AMR owns a UDP socket and sends
intent directly to its configured peers. There is no broker or shared
reservation state in the network transport.
"""

import json
import socket
from typing import Dict, List, Tuple

from interfaces import CommsChannel
from models import Intent, IntentMessage, TaskBid
from robot.security import verify_hmac, compute_hmac, is_authorized_robot, verify_bid_hmac, compute_bid_hmac


class PubSubChannel(CommsChannel):
    """Deterministic one-tick in-process transport used by the simulator tests."""

    def __init__(self):
        self.current_messages: List[IntentMessage] = []
        self.next_messages: List[IntentMessage] = []
        self.current_bids: List[TaskBid] = []
        self.next_bids: List[TaskBid] = []

    def send(self, message: IntentMessage) -> None:
        if not message.auth_tag:
            message.auth_tag = compute_hmac(message.robot_id, message)
        self.next_messages.append(message)

    def send_bid(self, bid: TaskBid) -> None:
        if not bid.auth_tag:
            bid.auth_tag = compute_bid_hmac(bid)
        self.next_bids.append(bid)

    def collect_bids(self) -> List[TaskBid]:
        return [bid for bid in self.next_bids if verify_bid_hmac(bid)]

    def receive(self) -> List[IntentMessage]:
        return [
            msg for msg in self.current_messages
            if isinstance(msg, IntentMessage)
            and is_authorized_robot(msg.robot_id)
            and verify_hmac(msg)
        ]

    def clear(self) -> None:
        self.current_messages = self.next_messages
        self.next_messages = []
        self.current_bids = self.next_bids
        self.next_bids = []


class UdpPeerChannel(CommsChannel):
    """Direct UDP unicast mesh transport for one robot process."""

    def __init__(
        self,
        robot_id: str,
        bind: Tuple[str, int],
        peers: Dict[str, Tuple[str, int]],
        recv_buffer: int = 65535,
    ):
        self.robot_id = robot_id
        self.peers = dict(peers)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(bind)
        self.socket.setblocking(False)
        self.recv_buffer = recv_buffer
        self.last_seq: Dict[tuple[str, str], int] = {}
        self.last_epoch: Dict[str, str] = {}
        self.last_bid_seq: Dict[tuple[str, str], int] = {}
        self.last_bid_epoch: Dict[str, str] = {}
        self.received_bids: List[TaskBid] = []

    @staticmethod
    def _encode(message: IntentMessage) -> bytes:
        payload = {
            "robot_id": message.robot_id,
            "seq": message.seq,
            "timestamp": message.timestamp,
            "position": list(message.position),
            "velocity": message.velocity,
            "intent": message.intent.value,
            "next_intersection": list(message.next_intersection) if message.next_intersection else None,
            "task_id": message.task_id,
            "priority": message.priority,
            "planned_path": [list(p) for p in message.planned_path],
            "reservation_horizon": message.reservation_horizon,
            "battery": message.battery,
            "waiting_on": message.waiting_on,
            "heartbeat": message.heartbeat,
            "auth_tag": message.auth_tag,
            "session_epoch": message.session_epoch,
            "type": "intent",
        }
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")

    @staticmethod
    def _decode(payload: bytes) -> IntentMessage:
        data = json.loads(payload.decode("utf-8"))
        return IntentMessage(
            robot_id=str(data["robot_id"]),
            seq=int(data["seq"]),
            timestamp=float(data["timestamp"]),
            position=(float(data["position"][0]), float(data["position"][1])),
            velocity=float(data["velocity"]),
            intent=Intent(data["intent"]),
            next_intersection=(
                (int(data["next_intersection"][0]), int(data["next_intersection"][1]))
                if data.get("next_intersection") is not None else None
            ),
            task_id=data.get("task_id"),
            priority=int(data["priority"]),
            planned_path=[(int(p[0]), int(p[1])) for p in data.get("planned_path", [])],
            reservation_horizon=float(data.get("reservation_horizon", 0.0)),
            battery=float(data.get("battery", 100.0)),
            waiting_on=data.get("waiting_on"),
            heartbeat=float(data.get("heartbeat", 0.0)),
            auth_tag=data.get("auth_tag", ""),
            session_epoch=data.get("session_epoch", ""),
        )

    def send(self, message: IntentMessage) -> None:
        if not message.auth_tag:
            message.auth_tag = compute_hmac(message.robot_id, message)
        packet = self._encode(message)
        for peer_id, endpoint in self.peers.items():
            if peer_id == self.robot_id:
                continue
            try:
                self.socket.sendto(packet, endpoint)
            except OSError:
                continue

    @staticmethod
    def _encode_bid(bid: TaskBid) -> bytes:
        payload = {
            "type": "task_bid",
            "robot_id": bid.robot_id,
            "seq": bid.seq,
            "timestamp": bid.timestamp,
            "session_epoch": bid.session_epoch,
            "task_id": bid.task_id,
            "bid": bid.bid,
            "auth_tag": bid.auth_tag,
        }
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")

    @staticmethod
    def _decode_bid(payload: bytes) -> TaskBid:
        data = json.loads(payload.decode("utf-8"))
        return TaskBid(
            robot_id=str(data["robot_id"]),
            seq=int(data["seq"]),
            timestamp=float(data["timestamp"]),
            session_epoch=str(data["session_epoch"]),
            task_id=str(data["task_id"]),
            bid=float(data["bid"]),
            auth_tag=data.get("auth_tag", ""),
        )

    def send_bid(self, bid: TaskBid) -> None:
        if not bid.auth_tag:
            bid.auth_tag = compute_bid_hmac(bid)
        packet = self._encode_bid(bid)
        for peer_id, endpoint in self.peers.items():
            if peer_id == self.robot_id:
                continue
            try:
                self.socket.sendto(packet, endpoint)
            except OSError:
                continue

    def collect_bids(self) -> List[TaskBid]:
        bids = list(self.received_bids)
        self.received_bids.clear()
        return bids

    def receive(self) -> List[IntentMessage]:
        messages: List[IntentMessage] = []
        while True:
            try:
                packet, _ = self.socket.recvfrom(self.recv_buffer)
            except BlockingIOError:
                break
            except OSError:
                break
            try:
                raw = json.loads(packet.decode("utf-8"))
                if raw.get("type") == "task_bid":
                    bid = self._decode_bid(packet)
                    if not is_authorized_robot(bid.robot_id) or not verify_bid_hmac(bid):
                        continue
                    epoch = bid.session_epoch
                    previous_epoch = self.last_bid_epoch.get(bid.robot_id)
                    if previous_epoch is not None and epoch != previous_epoch:
                        self.last_bid_seq.pop((bid.robot_id, previous_epoch), None)
                    last_seq = self.last_bid_seq.get((bid.robot_id, epoch), -1)
                    if bid.seq <= last_seq:
                        continue
                    self.last_bid_epoch[bid.robot_id] = epoch
                    self.last_bid_seq[(bid.robot_id, epoch)] = bid.seq
                    self.received_bids.append(bid)
                    continue
                msg = self._decode(packet)
            except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                continue
            if msg.robot_id != self.robot_id:
                if not is_authorized_robot(msg.robot_id) or not verify_hmac(msg):
                    continue
                epoch = msg.session_epoch
                if not epoch:
                    continue
                previous_epoch = self.last_epoch.get(msg.robot_id)
                if previous_epoch is not None and epoch != previous_epoch:
                    self.last_seq.pop((msg.robot_id, previous_epoch), None)
                last_seq = self.last_seq.get((msg.robot_id, epoch), -1)
                if msg.seq <= last_seq:
                    continue
                self.last_epoch[msg.robot_id] = epoch
                self.last_seq[(msg.robot_id, epoch)] = msg.seq
                messages.append(msg)
        return messages

    def clear(self) -> None:
        pass

    def close(self) -> None:
        try:
            self.socket.close()
        except OSError:
            pass
