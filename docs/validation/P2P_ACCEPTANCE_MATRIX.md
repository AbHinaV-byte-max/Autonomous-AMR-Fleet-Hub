# P2P Acceptance Matrix

## Purpose

This is the reproducible acceptance protocol for the live P2P coordination path.

The benchmark compares the same seeded workload under:
- **B2** — stop-and-wait baseline with vertex and edge-swap prevention.
- **P2P** — robot-local A* + authenticated peer intent + local reservations + auction task bidding.

## Current methodology

- **20 paired trials per scenario** by default (`BENCH_TRIALS=20`).
- Trial seeds are deterministic: `26123 + trial_index`.
- B2 and P2P receive the **same seed and generated workload** for each trial.
- Results are reported per scenario as mean ± standard deviation of makespan reduction across completed paired trials.
- The aggregate acceptance number is the **mean of per-scenario reductions**, not a raw sum of ticks.
- Vertex collisions and **edge-swap collisions** are counted separately.
- A baseline timeout is reported as resilience evidence and is not converted into an artificial percentage reduction.
- A P2P timeout makes that scenario fail acceptance.
- The acceptance script writes machine-readable evidence to `artifacts/p2p_acceptance_matrix.json`.

Run:

~~~bash
python ref_sih_amr/experiments/decentralized_acceptance.py
~~~

Useful overrides:

~~~bash
BENCH_TRIALS=20 BENCH_MAX_TICKS=3000 python ref_sih_amr/experiments/decentralized_acceptance.py
BENCH_SCENARIOS=S2_Crossing,S8_Scale python ref_sih_amr/experiments/decentralized_acceptance.py
~~~

## Acceptance scenarios

| Scenario | Workload | Evidence purpose |
|---|---:|---|
| S2_Crossing | 6 tasks | Orthogonal crossing / choke-point coordination |
| S3_Narrow | 4 tasks | Overlap-heavy narrow-aisle coordination workload |
| S4_Blocked | 6 tasks | Dynamic blocked-aisle recovery |
| S5_Failure | 6 tasks | Failure/reallocation resilience |\n| S6_CommDelay | 6 tasks | Deterministic loss + latency + jitter resilience |\n| S7_Malformed | 6 tasks | Malformed-input resilience |\n| S8_Scale | 6 tasks / 8 AMRs | Fleet scalability |

S5, S6 and S7 are included directly in the acceptance run so resilience failures cannot silently disappear from the acceptance artifact.

## Historical evidence

The previous matrix recorded one CI run with S2 30.68% reduction and S8 60.13%, while S3 was 0% and S4 was reported as baseline-timeout resilience. Those numbers are **historical evidence only** and are superseded as the acceptance record by the 20-trial paired methodology above.

They must not be presented as current performance until a fresh benchmark artifact is generated from the current commit.

## Evidence boundaries

This matrix measures simulator-level coordination behavior. It does not establish:
- Raspberry Pi / Jetson CPU, RAM, thermal or planner-latency performance.
- Real Wi-Fi/5G packet loss or physical RF behavior. S6 now injects deterministic packet loss, 2/3-tick latency and jitter in the simulator; UDP can be selected with `BENCH_COMMS_MODE=udp`.
- A shipped ONNX edge model. The live decision loop has a deterministic SafeEdgePolicy; ONNX is optional and must be supplied explicitly as a deployment artifact.
- Production UDP authentication without a shared `AMR_HMAC_MASTER_KEY` deployment secret.