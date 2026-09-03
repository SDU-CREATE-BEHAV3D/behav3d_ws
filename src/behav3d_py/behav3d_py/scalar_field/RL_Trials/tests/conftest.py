"""Local import paths shared by the isolated RL trial test suite."""

from __future__ import annotations

import sys
from pathlib import Path

SCALAR_FIELD_ROOT = Path(__file__).resolve().parents[2]
if str(SCALAR_FIELD_ROOT) not in sys.path:
    sys.path.insert(0, str(SCALAR_FIELD_ROOT))
