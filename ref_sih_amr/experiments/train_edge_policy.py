"""
Offline training script — reads logs/training_data.jsonl,
trains a DecisionTreeClassifier, exports to robot/edge_policy.onnx.

Run after generating sufficient training data:
    python experiments/train_edge_policy.py
"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import json
from pathlib import Path
import numpy as np

LOG_FILE  = Path(__file__).parent.parent / "logs" / "training_data.jsonl"
ONNX_OUT  = Path(__file__).parent.parent / "robot" / "edge_policy.onnx"


def load_dataset():
    rows = [json.loads(l) for l in LOG_FILE.read_text().splitlines() if l.strip()]
    from robot.edge_policy import ACTIONS, ACTION_INDEX
    X = np.array([r["features"] for r in rows], dtype=np.float32)
    y = np.array([ACTION_INDEX.get(r["decision"], 0) for r in rows], dtype=np.int64)
    return X, y


def train_and_export():
    from sklearn.tree import DecisionTreeClassifier
    from skl2onnx import convert_sklearn
    from skl2onnx.common.data_types import FloatTensorType
    from robot.edge_policy import ACTIONS

    X, y = load_dataset()
    print(f"Training on {len(X)} samples, {len(set(y.tolist()))} classes")

    clf = DecisionTreeClassifier(max_depth=8, random_state=42)
    clf.fit(X, y)

    # Export to ONNX
    initial_type = [("float_input", FloatTensorType([None, 9]))]
    onnx_model = convert_sklearn(clf, initial_types=initial_type)
    ONNX_OUT.write_bytes(onnx_model.SerializeToString())
    print(f"ONNX model written to {ONNX_OUT}")

    # Quick validation
    import onnxruntime as ort
    sess = ort.InferenceSession(str(ONNX_OUT))
    pred = sess.run(None, {"float_input": X[:5]})[0]
    print("Sample predictions:", [ACTIONS[int(p)] for p in pred])


def generate_training_data(n_ticks: int = 2000):
    """Run the simulator headlessly to populate training_data.jsonl."""
    from sim.simulator import Simulator

    ascii_map = """\
##########
#R..P...R#
#..####..#
#D..R...D#
##########
"""
    sim = Simulator(ascii_map=ascii_map, headless=True)
    sim.run(max_ticks=n_ticks)
    print(f"Simulation complete. Check {LOG_FILE} for training data.")


if __name__ == "__main__":
    if not LOG_FILE.exists() or LOG_FILE.stat().st_size < 100:
        print("No training data found — generating via simulation...")
        generate_training_data()

    train_and_export()
