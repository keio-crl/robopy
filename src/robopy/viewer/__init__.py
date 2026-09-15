"""Browser-based model viewer and simulator for the Rakuda dual-arm robot.

Run without any hardware::

    uv run --extra kinematics python -m robopy.viewer --urdf path/to/assembly_2.urdf \\
        --package-dir path/to/models

and open http://127.0.0.1:8765 .  Joint angles and TCP targets set in the page
are solved with the same :class:`~robopy.kinematics.urdf_model.WholeBodyModel`
and :class:`~robopy.kinematics.dual_arm_ik.DualArmIK` the control stack uses, so
what the page shows is what the controller would command.  Nothing here writes
to a serial port.
"""

from .model_bundle import ModelBundle, VisualGeometry
from .server import ViewerServer, serve

__all__ = ["ModelBundle", "ViewerServer", "VisualGeometry", "serve"]
