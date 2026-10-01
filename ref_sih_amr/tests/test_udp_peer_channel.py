import time

from comms.channel import UdpPeerChannel
from models import Intent, IntentMessage


def _msg(robot_id: str) -> IntentMessage:
    return IntentMessage(
        robot_id=robot_id,
        seq=1,
        timestamp=1.0,
        position=(1.0, 2.0),
        velocity=1.0,
        intent=Intent.MOVE,
        next_intersection=(2, 2),
        task_id="T1",
        priority=1,
        planned_path=[(1, 2), (2, 2)],
        heartbeat=1.0,
    )


def test_udp_peer_channel_exchanges_intents_directly():
    a = UdpPeerChannel("robot-a", ("127.0.0.1", 19101), {"robot-b": ("127.0.0.1", 19102)})
    b = UdpPeerChannel("robot-b", ("127.0.0.1", 19102), {"robot-a": ("127.0.0.1", 19101)})
    try:
        a.send(_msg("robot-a"))
        deadline = time.monotonic() + 1.0
        received = []
        while time.monotonic() < deadline and not received:
            received = b.receive()
            if not received:
                time.sleep(0.01)

        assert len(received) == 1
        assert received[0].robot_id == "robot-a"
        assert received[0].planned_path == [(1, 2), (2, 2)]
        assert received[0].intent == Intent.MOVE
    finally:
        a.close()
        b.close()
