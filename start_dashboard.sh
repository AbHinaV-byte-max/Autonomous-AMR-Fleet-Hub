#!/usr/bin/env bash
set -euo pipefail
python3 -m pip install -r requirements.txt
exec python3 -m uvicorn main:app --app-dir ref_sih_amr/dashboard/backend --host 0.0.0.0 --port 8000
