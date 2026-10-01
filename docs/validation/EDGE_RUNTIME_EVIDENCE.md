# Edge Runtime Evidence

## What this measures

`ref_sih_amr/edge/measure_runtime.py` measures the robot-local decision layer on the machine where it is executed:

- machine/platform identity
- CPU count
- Python version
- mean and p95 policy-decision latency
- CPU utilization when available
- RAM utilization when available

## Required evidence boundary

Do **not** label a desktop, GitHub-hosted runner or ordinary laptop result as Raspberry Pi / Jetson evidence.
The SIH claim should only use a result whose `platform` / `machine` fields identify the actual target edge device.

## Capture command on the target device

~~~bash
python ref_sih_amr/edge/measure_runtime.py --iterations 5000 --output docs/validation/edge_runtime_result.json
~~~

Run the command directly on the Raspberry Pi or Jetson used for the demonstration. Commit the resulting JSON only after verifying that it was generated on that physical device.

## Current status

Instrumentation is ready and the JSON output option is implemented.

**Physical Pi/Jetson measurement: pending.** No desktop or CI measurement is being promoted as hardware evidence.

## Evidence checklist

- [ ] Raspberry Pi or Jetson model recorded
- [ ] CPU count recorded
- [ ] Python version recorded
- [ ] policy mean latency recorded
- [ ] policy p95 latency recorded
- [ ] CPU/RAM telemetry recorded
- [ ] result JSON committed under `docs/validation/`
- [ ] presentation claims match the measured device and workload
