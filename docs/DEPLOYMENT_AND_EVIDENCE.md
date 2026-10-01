# Deployment and evidence boundary

## Canonical runtime

The AMR simulator and live dashboard are long-running Python processes. For
judging/demo use, run them on a persistent Linux/Windows host or with Docker:

```bash
python3 -m pip install -r requirements.txt
./start_dashboard.sh
```

or:

```bash
docker compose up --build
```

The dashboard process owns the simulator lifecycle and therefore must not be
treated as durable state inside a stateless request handler.

## Vercel

`app.py` is retained as a lightweight ASGI entrypoint for preview/integration
purposes. It is not the source of truth for the robot runtime. A production
deployment should run the persistent dashboard/backend service separately from
any static/frontend hosting.

## P2P communication

The live simulator's P2P strategy uses robot-local reservation tables and peer
intent. The repository also contains `UdpPeerChannel`, a direct UDP transport
for separate robot processes. The deterministic `PubSubChannel` exists only
for lockstep tests.

## Benchmark evidence

Do not copy historical benchmark numbers into a presentation. Run:

```bash
python ref_sih_amr/experiments/decentralized_acceptance.py
```

The script compares the same fixed workload under stop-and-wait (B2) and P2P,
reports measured makespan/collisions, and exits non-zero unless both the 20%
time-reduction target and zero-collision target are met.

## Edge hardware evidence

Desktop simulation is not edge-hardware evidence. Run this on the actual
Raspberry Pi/Jetson target:

```bash
python ref_sih_amr/edge/measure_runtime.py --iterations 1000
```

The output records platform, CPU count, RAM/CPU samples, and local policy
latency. The resulting JSON should be stored with the validation run before
claiming real edge performance.
