# Edge-Class Runtime Measurement

## Current evidence status

**No Raspberry Pi / Jetson hardware result is claimed.**

The SIH requirement mentions edge hardware as an example deployment target. Since
physical target hardware is not part of the current evidence set, this project
uses a reproducible **containerized edge-class emulation** instead.

Each robot node can run as an independent Docker process with approximately:

- 1 CPU
- 1 GB RAM
- its own UDP endpoint
- its own local planner/reservation state

This reproduces process isolation and constrained resources, but it is **not
equivalent to measuring a Raspberry Pi or Jetson**.

## What the measurement helper records

`ref_sih_amr/edge/measure_runtime.py` records:

- platform/machine identity
- CPU count
- Python version
- mean and p95 robot-local policy latency
- CPU utilization when `psutil` is available
- RAM utilization when `psutil` is available

For containerized runs, report the result as **edge-class emulation**.

## Physical hardware boundary

A future physical run may use:

~~~bash
python ref_sih_amr/edge/measure_runtime.py --iterations 5000 --output docs/validation/edge_runtime_result.json
~~~

Only a result captured directly on the demonstration Raspberry Pi/Jetson may
be labeled physical edge-hardware evidence.

## Acceptance checklist

- [ ] Per-robot Docker processes use approximately 1 CPU / 1 GB RAM
- [ ] UDP peer communication verified between robot containers
- [ ] Container CPU/RAM usage recorded
- [ ] Robot-local planning/policy latency recorded
- [ ] Results labeled **edge-class emulation**
- [ ] No presentation slide calls this Raspberry Pi/Jetson evidence
- [ ] Physical target-device evidence is added separately if hardware becomes available
