"""Put ``examples/roboverse`` on the import path for these tests.

The Rakuda's RoboVerse *tasks* are examples rather than API -- robopy ships the
robot and stops there, see :mod:`robopy.roboverse` -- but they are still worth
testing: the numbers in them (reach limits, mount heights, the play table's
collision boxes) are measurements, and a test is what keeps them honest when the
model is re-exported.

The example modules are written to be run as scripts, so they import each other
by plain name (``from mount import ...``).  Adding their directory here is what
makes that work from pytest, which starts somewhere else entirely.
"""

from __future__ import annotations

import sys
from pathlib import Path

EXAMPLES = Path(__file__).resolve().parents[2] / "examples" / "roboverse"

if EXAMPLES.is_dir() and str(EXAMPLES) not in sys.path:
    sys.path.insert(0, str(EXAMPLES))

try:  # registers the task names, the way an example script does
    import tasks  # noqa: F401
except ImportError:  # pragma: no cover - MetaSim is not installed here
    pass
