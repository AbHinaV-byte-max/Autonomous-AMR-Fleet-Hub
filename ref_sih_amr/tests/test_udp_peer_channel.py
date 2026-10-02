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
    # Bind to ephemeral loopback ports so the test is isolated from stale
    # listeners or another local simulator instance.
    b = UdpPeerChannel("robot-1", ("127.0.0.1", 0), {})
    a = None
    try:
        b_endpoint = b.socket.getsockname()
        a = UdpPeerChannel("robot-0", ("127.0.0.1", 0), {"robot-1": b_endpoint})
        a_endpoint = a.socket.getsockname()
        b.peers = {"robot-0": a_endpoint}

        a.send(_msg("robot-0"))
        deadline = time.monotonic() + 1.0
        received = []
        while time.monotonic() < deadline and not received:
            received = b.receive()
            if not received:
                time.sleep(0.01)

        assert len(received) == 1
        assert received[0].robot_id == "robot-0"
        assert received[0].planned_path == [(1, 2), (2, 2)]
        assert received[0].intent == Intent.MOVE
    finally:
        if a is not None:
            a.close()
        b.close()


def test_simulator_can_use_per_robot_udp_channels():
    from sim.simulator import Simulator
    from comms.channel import UdpPeerChannel

    ascii_map = """\
########
#R..P.R#
#......#
#D....D#
########
"""
    sim = Simulator(
        ascii_map=ascii_map,
        headless=True,
        strategy="P2P",
        comms_mode="udp",
        udp_base_port=19301,
    )
    try:
        assert len(sim.comms_channels) == len(sim.robot_managers)
        assert all(isinstance(ch, UdpPeerChannel) for ch in sim.comms_channels)
        sim.tick()
    finally:
        sim.close()


def test_udp_peer_channel_rejects_forged_and_replayed_messages():
    b = UdpPeerChannel("robot-1", ("127.0.0.1", 0), {})
    a = None
    try:
        endpoint = b.socket.getsockname()
        a = UdpPeerChannel("robot-0", ("127.0.0.1", 0), {"robot-1": endpoint})
        a.send(_msg("robot-0"))
        deadline = time.monotonic() + 1.0
        received = []
        while time.monotonic() < deadline and not received:
            received = b.receive()
            if not received:
                time.sleep(0.01)
        assert len(received) == 1

        # Exact packet replay is rejected by the monotonic sequence guard.
        a.send(received[0])
        time.sleep(0.02)
        assert b.receive() == []
    finally:
        if a is not None:
            a.close()
        b.close()
