"""
SQLite persistence for per-tick metrics.
"""
import json
import sqlite3
import os

DB_PATH = os.path.join(os.path.dirname(__file__), "runs.db")

CREATE_TABLE = """
CREATE TABLE IF NOT EXISTS metric_snapshots (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    tick      INTEGER NOT NULL,
    snapshot  TEXT    NOT NULL,
    recorded_at REAL  NOT NULL
);
"""

_db: sqlite3.Connection = None


def init_db():
    global _db
    _db = sqlite3.connect(DB_PATH, check_same_thread=False)
    _db.execute(CREATE_TABLE)
    _db.commit()


def persist_snapshot(snapshot: dict):
    if _db is None:
        return
    tick = snapshot.get("tick", -1)
    blob = json.dumps(snapshot, default=str)
    import time
    _db.execute(
        "INSERT INTO metric_snapshots (tick, snapshot, recorded_at) VALUES (?, ?, ?)",
        (tick, blob, time.time())
    )
    _db.commit()


def close_db():
    if _db:
        _db.close()
