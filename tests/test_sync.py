"""DD-DRV section 7: the copies of the shared headers in the sketch folders match firmware/common/."""
import subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def test_common_copies_identical():
    r = subprocess.run([sys.executable, str(ROOT / "tools/sync_common.py"), "--check"], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout
