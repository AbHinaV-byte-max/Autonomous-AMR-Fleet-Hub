"""
Security and Defense Hardening Layer for SIH 26123 Fleet Coordination System.
Sponsored by Bharat Electronics Limited (BEL) - Defense Sector PSU.

SECURITY CONSIDERATIONS:
In industrial and defense-grade warehouse environments, Autonomous Mobile Robots operate
over untrusted or contested physical wireless links (Wi-Fi 6 / 5G Private Networks).
To prevent rogue node injection, replay attacks, or adversarial position spoofing:
1. Message Authentication: Every telemetry and intent broadcast is signed with HMAC-SHA256
   or mutual TLS (mTLS) client certificate tokens.
2. Decentralized Peer Verification: Each recipient edge AMR verifies the sender's cryptographic
   signature before ingesting peer intent into its local conflict resolution engine.
"""

import hmac
import hashlib
import json
import time
from typing import Dict, Any, Tuple


class SecurityContext:
    """
    Cryptographic verification extension point for peer-to-peer message integrity.
    """

    def __init__(self, shared_secret_key: str = "BEL_DEFENSE_DEPOT_SECURE_HMAC_KEY_2026"):
        self.secret_key = shared_secret_key.encode("utf-8")

    def sign_payload(self, payload: Dict[str, Any]) -> str:
        """
        Generates HMAC-SHA256 signature for a message payload.
        Ensures tamper-evident transmission across peer-to-peer channels.
        """
        # Canonical JSON serialization for deterministic hashing
        canonical_str = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        signature = hmac.new(self.secret_key, canonical_str.encode("utf-8"), hashlib.sha256).hexdigest()
        return signature

    def wrap_envelope(self, sender_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        """
        Wraps message in a cryptographically signed defense envelope with timestamp.
        """
        envelope = {
            "sender_id": sender_id,
            "timestamp": time.time(),
            "payload": payload,
        }
        envelope["signature"] = self.sign_payload(envelope)
        return envelope

    def verify_envelope(self, envelope: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Validates message signature and checks for message freshness (replay defense).
        """
        if "signature" not in envelope or "payload" not in envelope:
            return False, "Missing security envelope or signature"

        received_sig = envelope["signature"]
        copy_envelope = {
            "sender_id": envelope.get("sender_id"),
            "timestamp": envelope.get("timestamp"),
            "payload": envelope.get("payload"),
        }
        expected_sig = self.sign_payload(copy_envelope)

        if not hmac.compare_digest(received_sig, expected_sig):
            return False, "Cryptographic signature mismatch — possible tampering detected"

        # Replay prevention: verify message was generated within 10 seconds
        if abs(time.time() - envelope.get("timestamp", 0)) > 10.0:
            return False, "Message timestamp expired (replay attack defense)"

        return True, "Verified"
