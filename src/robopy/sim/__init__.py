"""Simulation-side model export.

:mod:`robopy.sim.mjcf_export` turns the committed Rakuda CAD export into an
MJCF model, which is the format MuJoCo, MJX and Newton read.  It needs the
``sim`` extra (``pip install 'robopy[sim]'``); the rest of robopy does not.

The models it produces are checked in, so simply *using* the simulated robot --
see :mod:`robopy.roboverse` -- needs neither this module nor MuJoCo's presence
at import time.
"""

from __future__ import annotations
