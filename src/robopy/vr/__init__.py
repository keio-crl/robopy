"""VR teleoperation of the Rakuda from a WebXR headset (Meta Quest).

The headset's browser opens a page served by :mod:`robopy.vr.server`; the page
streams the headset pose and both controller poses over a WebSocket and shows
the robot's head camera in front of the operator.  On the Python side:

* :mod:`robopy.vr.xr_math` converts WebXR poses (Y up, -Z forward) into the
  robot base convention (Z up, X forward) and re-centres them on the operator;
* :mod:`robopy.vr.head_tracking` turns the headset orientation into
  ``head_yaw`` / ``head_pitch`` joint targets, with the joint signs and the
  "camera looks forward" neutral configuration derived from the URDF rather
  than assumed;
* :mod:`robopy.vr.arm_teleop` turns each controller into a clutch-based
  relative Cartesian target for the corresponding arm (and the trigger into a
  gripper command when the gripper travel has been measured);
* :mod:`robopy.vr.backend` applies the resulting commands either to the
  simulated model (no hardware) or to a running
  :class:`~robopy.robots.rakuda.rakuda_control.RakudaControlSystem`;
* :mod:`robopy.vr.camera` streams JPEG frames from any camera source.

Everything except the page works without a headset: the tests drive the
WebSocket protocol directly, and the page has a desktop preview mode.
"""

from __future__ import annotations

__all__: list[str] = []
