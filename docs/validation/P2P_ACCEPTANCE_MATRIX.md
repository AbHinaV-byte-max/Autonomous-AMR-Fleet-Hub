# P2P Acceptance Matrix

## Purpose

This record expands the decentralized acceptance evidence beyond the original single S2_Crossing benchmark.

The matrix covers:
- S2_Crossing — orthogonal crossing / choke-point contention.
- S3_Narrow — four-AMR, one-cell-wide narrow-aisle safety case.
- S4_Blocked — dynamic blocked-aisle recovery.
- S8_Scale — eight-AMR fleet scalability.

The live P2P strategy uses robot-local planning and direct peer intent exchange. B2 is the stop-and-wait comparison baseline. The benchmark does not claim that task allocation itself is decentralized; task assignment remains fleet-level Hungarian allocation.

## Acceptance run

- Workflow: **P2P Acceptance Benchmark**
- Run: **#15**
- Run ID: **36848379383**
- Acceptance job: **110323920870**
- Evidence artifact: **p2p-acceptance-matrix**
- Artifact ID: **11154132757**
- Commit: `c9ec7b818de2d4c48a4c4bba960be619afa2627f`

The workflow uploads the machine-readable result as `p2p_acceptance_matrix.json`.

## Measured results

| Scenario | Workload | B2 makespan | P2P makespan | P2P reduction | P2P collisions | P2P completed | Result |
|---|---:|---:|---:|---:|---:|---:|---|
| S2 Crossing | 6 tasks | 1470 | 1019 | 30.68% | 0 | 6/6 | PASS |
| S3 Narrow | 1 delivery | 1315 | 1315 | 0.00% | 0 | 1/1 | PASS |
| S4 Blocked | 6 tasks | timeout (2/6) | 968 | — | 0 | 6/6 | P2P completes; B2 timeout |
| S8 Scale | 6 tasks / 8 AMRs | 2172 | 866 | 60.13% | 0 | 6/6 | PASS |

### Aggregate comparable-workload result

- B2 makespan: **4957**
- P2P makespan: **3200**
- measured reduction: **35.44%**
- P2P collisions: **0**
- P2P timeouts: **0**

The aggregate clears the project's **20% makespan-reduction** target on the comparable completed workloads.

### Blocked-aisle evidence

S4 deliberately introduces the same blocked cell after the initial routes are established.

- B2 completed only **2/6** tasks before the 3000-tick horizon.
- P2P completed **6/6** tasks in **968** ticks.
- P2P recorded **0 collisions**.

Because the baseline timed out, no artificial percentage reduction is reported for S4. It is recorded as resilience evidence rather than folded into the percentage calculation.

### Narrow-aisle evidence

S3 is intentionally a **single-delivery safety case**. The map contains four robots competing around a one-cell-wide corridor. A multi-delivery throughput workload currently causes both strategies to remain in the corridor for longer than the acceptance horizon because the map has no pull-off/service bay that can safely absorb multiple completed robots.

Therefore the matrix does **not** claim narrow-aisle throughput improvement. It records the reproducible one-delivery safety result separately:
- B2: 1/1 completed, 1315 ticks, 0 collisions.
- P2P: 1/1 completed, 1315 ticks, 0 collisions.

This is deliberately conservative evidence.

## Reproduction

Install development dependencies and run:

~~~bash
python ref_sih_amr/experiments/decentralized_acceptance.py
~~~

The default matrix runs S2, S3, S4 and S8. Results are written to `artifacts/p2p_acceptance_matrix.json`.

Environment variables can override the scenario set, task count and horizon:

~~~bash
BENCH_SCENARIOS=S2_Crossing,S8_Scale BENCH_MAX_TICKS=3000 python ref_sih_amr/experiments/decentralized_acceptance.py
~~~

## Evidence boundary

This matrix demonstrates simulator-level coordination behavior and the UDP transport integration. It is **not** evidence of Raspberry Pi / Jetson CPU, RAM, latency or thermal performance. Real edge-hardware measurements remain a separate evidence item and must be collected on the target hardware before claiming them in the SIH presentation.
