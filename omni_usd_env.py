"""
Central Omniverse USD Environment Initializer
Detects and configures omni.usd.libs from the Omniverse Kit release cache
to enable standalone Python execution against USD stages.
"""

import os
import sys

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
KIT_RELEASE_DIR = os.path.join(REPO_ROOT, "_build", "windows-x86_64", "release")
EXTSCACHE_DIR = os.path.join(KIT_RELEASE_DIR, "extscache")

usd_libs_dir = None
if os.path.exists(EXTSCACHE_DIR):
    for entry in os.listdir(EXTSCACHE_DIR):
        if entry.startswith("omni.usd.libs"):
            usd_libs_dir = os.path.join(EXTSCACHE_DIR, entry)
            break

if usd_libs_dir:
    bin_dir = os.path.join(usd_libs_dir, "bin")
    if hasattr(os, "add_dll_directory") and os.path.exists(bin_dir):
        try:
            os.add_dll_directory(bin_dir)
        except Exception:
            pass
    os.environ["PATH"] = bin_dir + ";" + os.environ.get("PATH", "")
    if usd_libs_dir not in sys.path:
        sys.path.insert(0, usd_libs_dir)

try:
    from pxr import Usd, UsdGeom, Gf, Sdf  # type: ignore
    USD_AVAILABLE = True
except ImportError:
    Usd = None
    UsdGeom = None
    Gf = None
    Sdf = None
    USD_AVAILABLE = False
