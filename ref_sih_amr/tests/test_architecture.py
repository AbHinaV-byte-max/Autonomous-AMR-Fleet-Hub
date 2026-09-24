"""
Architecture guard test — Section 5.2.
Greps /robot and /allocator source trees for any import of 'dashboard'.
Fails CI if any such import is found, enforcing the observer-only contract.
"""
import os
import sys
import re

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PROTECTED_DIRS = ["robot", "allocator", "sim", "comms"]
FORBIDDEN_PATTERN = re.compile(r"^\s*(import|from)\s+dashboard", re.MULTILINE)


def test_no_dashboard_imports_in_core():
    violations = []
    for d in PROTECTED_DIRS:
        dirpath = os.path.join(ROOT, d)
        for fname in os.listdir(dirpath):
            if not fname.endswith(".py"):
                continue
            fpath = os.path.join(dirpath, fname)
            code = open(fpath, encoding="utf-8").read()
            if FORBIDDEN_PATTERN.search(code):
                violations.append(fpath)

    assert not violations, (
        "ARCHITECTURE VIOLATION — /dashboard imported from core modules "
        "(Section 5.2 observer contract broken):\n" +
        "\n".join(violations)
    )
    print("Architecture guard passed — no dashboard imports in core modules.")


if __name__ == "__main__":
    test_no_dashboard_imports_in_core()
