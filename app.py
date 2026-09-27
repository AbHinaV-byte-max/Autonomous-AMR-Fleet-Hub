"""Vercel entrypoint for the SIH AMR FastAPI dashboard.

The canonical application remains in
ref_sih_amr/dashboard/backend/main.py. This thin adapter lets Vercel's
FastAPI runtime discover the existing ASGI application without changing the
frozen local launcher or the simulation core.
"""

from ref_sih_amr.dashboard.backend.main import app

__all__ = ["app"]
