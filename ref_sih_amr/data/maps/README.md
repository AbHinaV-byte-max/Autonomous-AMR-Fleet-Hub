# Real warehouse benchmark data

This directory contains the MovingAI warehouse maps used by the SIH AMR simulator.

- `warehouse-10-20-10-2-1.map`: 161 x 63; SHA-256 `c8d1b2f24788ed6bd1ccf45065b96b4ce82d65f88c72de750e03e2758637bff0`
- `warehouse-10-20-10-2-2.map`: 170 x 84; SHA-256 `4f06e82c2b87238daa8e308086afdba701112bf023e94e740e5bf9508a6adec3`

The supplied scenario files are MovingAI start/goal workloads. They do not contain warehouse geometry. The adapter in `data/warehouse_benchmark.py` validates the supplied coordinates against warehouse 1 and uses a small, explicit 3-instance sample from each of the three supplied scenario files for the live 3-AMR workload.

The full source scenario files contained 450, 450, and 430 instances respectively. The runtime intentionally does not claim to execute all of those instances simultaneously.

MovingAI uses `T` for blocked cells and `.` for traversable cells. The simulator adapter converts `T -> #` while preserving traversable cells.
