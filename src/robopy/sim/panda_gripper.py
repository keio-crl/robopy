"""Give the simulated Rakuda a hand, borrowed from the Panda.

The Rakuda's CAD export models both grippers as *fixed frames with no geometry
at all* -- ``gripper_left_dof`` and ``gripper_right_dof`` are named ``_dof`` but
are not degrees of freedom, and nothing hangs off them.  The real machine does
have a gripper per arm, driven by an XM430-W350 (``l_arm_grip`` / ``r_arm_grip``
on the follower bus), but its shape is not in this repository.

So a hand is borrowed: the two finger meshes from CALVIN's
``panda_longer_finger``, vendored under ``models/gripper_panda/`` (Apache 2.0).
They are attached to the Rakuda's gripper frames as a one-actuator parallel jaw
per arm, which is what the real bus has.

**The result is not the real robot.**  It is a Rakuda's arms with a Panda's
fingers, and nothing about the finger shape, the stroke or the grip force
matches the hardware.  It is meant for building grasping tasks in simulation;
a policy trained on it should not be expected to transfer.  Worse, the offset
from ``gripper_*_dof`` to the real grasp point has never been measured on this
machine (it is still ``validated: false`` in
``examples/config/rakuda_control.example.yaml``), so *where* the fingers are
mounted is itself an assumption rather than a measurement.

What is faithful, and what is not
---------------------------------
Faithful to CALVIN's gripper:
    the finger mesh, and the 0 to 0.04 m per-finger stroke its geometry is
    designed around (``gripper_joint_limits: [0, 0.04]`` in
    ``calvin_env/conf/robot/panda_longer_finger.yaml``).

Faithful to the Rakuda:
    one actuator per hand rather than two independent fingers, because that is
    what the follower bus has.

Neither:
    :data:`FINGER_MASS_KG` and :data:`GRIP_FORCE_N`, both of which are
    engineering choices.  The upstream URDF gives each finger 0.1 kg, which is
    most of a Rakuda forearm (0.126 kg) and would make the hand heavier than the
    limb carrying it, so a lighter figure is used.  The grip force cannot be
    derived either: the real gripper is current-limited to raw 128, which is
    0.344 A and so roughly 0.61 N.m at an XM430-W350, but turning a servo torque
    into a jaw force needs the mechanism's lever arm and there is no mechanism
    here -- the fingers slide on a prismatic joint rather than being driven
    through the real linkage.

What it can hold
----------------
Measured on the exported model, with the pads' separation along the opening
axis: **8 mm fully closed, 88 mm fully open**.  It cannot pinch anything thinner
than 8 mm, and an object has to sit in the proximal two thirds of the fingers to
survive being swung around -- held near the tips it works loose.  A 30 mm cube
held 35 to 50 mm along the fingers stays put through a 0.9 rad wrist swing;
the same cube 65 mm along slides off the ends.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

__all__ = [
    "FINGER_STROKE_M",
    "GRIP_FORCE_N",
    "GRIP_GAIN",
    "PAD_GAP_CLOSED_M",
    "PAD_GAP_OPEN_M",
    "GRIPPER_ACTUATORS",
    "GRIPPER_JOINTS",
    "PandaGripperOptions",
    "attach_panda_fingers",
    "find_gripper_meshes",
    "gripper_targets",
    "pad_gap",
]

#: Per-finger travel, metres. From CALVIN's own ``gripper_joint_limits``; the
#: finger geometry is designed around it, so it comes with the mesh.
FINGER_STROKE_M: float = 0.04

#: Mass given to one finger.
#:
#: The upstream URDF says 0.1 kg. That is a Panda finger on a Panda, whose
#: forearm masses a couple of kilograms; on the Rakuda the whole last link is
#: 0.126 kg, so two upstream fingers would outweigh the arm carrying them and
#: the wrist servo would spend its travel holding the hand up. 0.02 kg keeps the
#: hand light relative to the limb. It is a choice, not a measurement.
FINGER_MASS_KG: float = 0.02

#: Mass of the small bracket the fingers slide on.
PALM_MASS_KG: float = 0.03

#: Force the gripper may apply, newtons.
#:
#: Not derived from anything: see the module docstring. It is high enough that
#: the force limit rather than the gain decides the squeeze, and low enough to
#: stay in the range a small servo-driven jaw could plausibly manage.
GRIP_FORCE_N: float = 20.0

#: Position gain of the gripper servo.
#:
#: Measured rather than picked: sweeping it against a 30 mm cube and then
#: swinging the wrist 0.9 rad, 80 drops the cube from every grasp depth tried,
#: 300 holds only when the cube is 35 mm along the fingers, and 800 holds from
#: 35 mm and 50 mm. Below roughly 300 the servo simply does not squeeze hard
#: enough to generate the friction that holds an object.
GRIP_GAIN: float = 800.0

#: Separation between the finger pads, metres, at each end of the travel.
#: Measured off the exported model; the closed figure is why nothing thinner
#: than 8 mm can be pinched.
PAD_GAP_CLOSED_M: float = 0.008
PAD_GAP_OPEN_M: float = 0.0876

#: Sliding friction on the finger pads. Higher than MuJoCo's 1.0 default,
#: because a parallel jaw with default friction drops anything it is not
#: squeezing hard, and the squeeze here is limited by a small servo.
FINGER_FRICTION: Tuple[float, float, float] = (1.5, 0.02, 0.001)

#: Joints the fingers add, per side, in the order they appear in the model.
GRIPPER_JOINTS: Dict[str, Tuple[str, str]] = {
    "left": ("left_finger_a_joint", "left_finger_b_joint"),
    "right": ("right_finger_a_joint", "right_finger_b_joint"),
}

#: Actuators the gripper adds, per side.
#:
#: There is one per finger joint and each is *named after the joint it drives*,
#: because MetaSim's MuJoCo handler looks an actuator up by joint name -- an
#: actuator called "left_gripper" simply cannot be commanded through it.
#:
#: Two actuators is not two motors.  The pair is tied together by an equality
#: constraint, so the hand still has the one degree of freedom the real bus has
#: (``l_arm_grip`` / ``r_arm_grip``); commanding the two differently does not buy
#: independent fingers, it just makes the constraint split the difference.  Use
#: :func:`gripper_targets` and the question does not arise.
GRIPPER_ACTUATORS: Dict[str, Tuple[str, str]] = {
    "left": ("left_finger_a_joint", "left_finger_b_joint"),
    "right": ("right_finger_a_joint", "right_finger_b_joint"),
}

#: Where the vendored meshes live, relative to the repository's ``models/``.
_MESH_DIR = "gripper_panda"
_MESH_NAME = "longer_finger_v2.obj"


@dataclass(frozen=True)
class PandaGripperOptions:
    """Knobs for :func:`attach_panda_fingers`.

    Attributes:
        stroke_m: Per-finger travel.  ``0`` is closed, ``stroke_m`` fully open.
        finger_mass_kg: Mass of one finger; see :data:`FINGER_MASS_KG`.
        palm_mass_kg: Mass of the bracket the fingers ride on.
        mount_offset_m: How far along the gripper frame's ``+z`` -- the free
            direction past the wrist -- to put the finger slides.  The wrist
            geometry ends at ``z = 0`` in that frame, so a small positive
            number keeps the fingers clear of it.
        grip_force_n: Force limit for the gripper actuator; see
            :data:`GRIP_FORCE_N`.
        gain: Position gain; see :data:`GRIP_GAIN`.
        friction: Sliding friction on the finger pads.
        contact_solref: MuJoCo contact time constant and damping for the pads.
        contact_solimp: MuJoCo contact impedance for the pads.  Together with
            ``contact_solref`` these make the pads much less squashy than
            MuJoCo's default, which is what stops a gripped object sinking into
            them and sliding free.
        pad_inner_y_m: Where the gripping face sits, taken from the finger mesh:
            its inner surface is 3.9 mm off the centreline at the tip.
        pad_thickness_m: Thickness of the pad box.
        pad_half_width_m: Half the finger's width; it is 21 mm across.
        pad_half_length_m: Half the length of finger that actually grips.
        pad_centre_z_m: Where that length is centred along the finger.
    """

    stroke_m: float = FINGER_STROKE_M
    finger_mass_kg: float = FINGER_MASS_KG
    palm_mass_kg: float = PALM_MASS_KG
    mount_offset_m: float = 0.005
    grip_force_n: float = GRIP_FORCE_N
    gain: float = GRIP_GAIN
    friction: Tuple[float, float, float] = FINGER_FRICTION
    contact_solref: Tuple[float, float] = (0.004, 1.0)
    contact_solimp: Tuple[float, float, float] = (0.98, 0.999, 0.0005)
    pad_inner_y_m: float = 0.0039
    pad_thickness_m: float = 0.005
    pad_half_width_m: float = 0.0105
    pad_half_length_m: float = 0.040
    pad_centre_z_m: float = 0.055


def find_gripper_meshes(models_dir: Path) -> Dict[str, Path]:
    """Locate the vendored finger meshes.

    Args:
        models_dir: The repository's ``models/`` directory.

    Returns:
        ``{"visual": path, "collision": path}``.

    Raises:
        FileNotFoundError: If either mesh is missing, with the path that was
            tried -- they are vendored, so absence means a broken checkout
            rather than a missing download.
    """
    found = {}
    for kind in ("visual", "collision"):
        path = Path(models_dir) / _MESH_DIR / kind / _MESH_NAME
        if not path.is_file():
            raise FileNotFoundError(
                f"the Panda finger mesh {path} is missing; it is vendored in this repository, "
                "so this is a broken checkout rather than something to download"
            )
        found[kind] = path
    return found


def _mirror_quat(sign: int) -> str:
    """Identity for one finger, 180 degrees about z for the one facing it."""
    return "1 0 0 0" if sign > 0 else "0 0 0 1"


def attach_panda_fingers(
    mjcf: ET.Element,
    meshes: Dict[str, Path],
    mesh_files: Dict[str, str],
    options: PandaGripperOptions | None = None,
) -> List[str]:
    """Hang a one-actuator parallel jaw off each of the Rakuda's gripper frames.

    The fingers are mounted on the frames the CAD calls ``gripper_left_dof`` and
    ``gripper_right_dof``, which the exporter has already turned into sites.
    They extend along each frame's ``+z``, which is the direction with nothing
    in it: the wrist's own geometry occupies ``z <= 0`` there.  They open along
    ``+-y``, where the wrist is roughly symmetric.

    Both fingers of a hand are tied to one actuator with an equality constraint,
    because the real robot has one gripper motor per arm rather than two
    independently commanded fingers.

    Args:
        mjcf: The compiled model's XML root, modified in place.
        meshes: Source meshes, from :func:`find_gripper_meshes`.
        mesh_files: What to write in each ``<mesh file=...>``, keyed the same
            way -- the caller decides whether that is a relative path or an
            absolute one.
        options: See :class:`PandaGripperOptions`.

    Returns:
        The names of the actuators that were added.

    Raises:
        ValueError: If the model does not carry the two gripper sites, which
            would mean it was exported without them.
    """
    options = options or PandaGripperOptions()

    asset = mjcf.find("asset")
    if asset is None:
        raise ValueError("the model has no <asset> section")
    for kind in ("visual", "collision"):
        ET.SubElement(asset, "mesh", {"name": f"panda_finger_{kind}", "file": mesh_files[kind]})

    hosts = _gripper_sites(mjcf)
    missing = sorted({"left", "right"} - set(hosts))
    if missing:
        raise ValueError(
            f"the model has no gripper site for {missing}; it was exported without "
            "RAKUDA_FRAME_SITES, so there is nothing to bolt a hand to"
        )

    added: List[str] = []
    equality = mjcf.find("equality")
    if equality is None:
        equality = ET.SubElement(mjcf, "equality")
    actuator = mjcf.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(mjcf, "actuator")

    for side in ("left", "right"):
        body, site = hosts[side]
        palm = ET.SubElement(
            body,
            "body",
            {
                "name": f"{side}_gripper_base",
                "pos": site.get("pos", "0 0 0"),
                "quat": site.get("quat", "1 0 0 0"),
            },
        )
        ET.SubElement(
            palm,
            "inertial",
            {
                "pos": "0 0 0.01",
                "mass": f"{options.palm_mass_kg:.6g}",
                "diaginertia": "1e-5 1e-5 1e-5",
            },
        )

        for name, sign in ((f"{side}_finger_a", 1), (f"{side}_finger_b", -1)):
            finger = ET.SubElement(
                palm, "body", {"name": name, "pos": f"0 0 {options.mount_offset_m:.6g}"}
            )
            ET.SubElement(
                finger,
                "inertial",
                {
                    "pos": "0 0 0.03",
                    "mass": f"{options.finger_mass_kg:.6g}",
                    "diaginertia": "2e-5 2e-5 2e-6",
                },
            )
            ET.SubElement(
                finger,
                "joint",
                {
                    "name": f"{name}_joint",
                    "type": "slide",
                    "axis": f"0 {sign} 0",
                    "range": f"0 {options.stroke_m:.6g}",
                    "damping": "5",
                    "armature": "0.001",
                },
            )
            quat = _mirror_quat(sign)
            ET.SubElement(
                finger,
                "geom",
                {
                    "class": "rakuda_visual",
                    "mesh": "panda_finger_visual",
                    "quat": quat,
                    "rgba": "0.9 0.9 0.9 1",
                },
            )
            # A flat pad to grip with, sitting just in front of the mesh.
            #
            # The finger's own inner face is not flat: measured off the mesh, it
            # tapers from 7.2 mm off the centreline at the base to 3.9 mm at the
            # tip. Gripping with that pinches an object between two converging
            # surfaces, which wedges it *out* of the jaw. That is why a block the
            # friction cone says is held by a hundred times its weight still slid
            # free the moment the arm lifted, and why sweeping grasp depth,
            # squeeze and lift speed never fixed it -- the geometry was the
            # problem, not the tuning.
            #
            # So the gripping surface is an explicit box, the way MuJoCo
            # Menagerie models this same gripper. The mesh stays for the finger
            # body and for looks.
            pad_half_y = options.pad_thickness_m / 2.0
            ET.SubElement(
                finger,
                "geom",
                {
                    "name": f"{name}_pad",
                    "class": "rakuda_collision",
                    "type": "box",
                    "size": (
                        f"{options.pad_half_width_m:.6g} {pad_half_y:.6g} "
                        f"{options.pad_half_length_m:.6g}"
                    ),
                    "pos": (
                        f"0 {sign * (options.pad_inner_y_m + pad_half_y):.6g} "
                        f"{options.pad_centre_z_m:.6g}"
                    ),
                    "friction": " ".join(f"{v:g}" for v in options.friction),
                    "solref": f"{options.contact_solref[0]:g} {options.contact_solref[1]:g}",
                    "solimp": " ".join(f"{v:g}" for v in options.contact_solimp),
                },
            )
            ET.SubElement(
                finger,
                "geom",
                {
                    "class": "rakuda_collision",
                    "mesh": "panda_finger_collision",
                    "quat": quat,
                    "friction": " ".join(f"{v:g}" for v in options.friction),
                    # Stiffer than MuJoCo's default contact, which lets a
                    # squeezed object sink 4 mm into each pad -- soft enough
                    # that a gripped block slides out from between the fingers
                    # while the arm lifts, even though the friction cone says it
                    # should hold a hundred times its weight.
                    "solref": f"{options.contact_solref[0]:g} {options.contact_solref[1]:g}",
                    "solimp": " ".join(f"{v:g}" for v in options.contact_solimp),
                },
            )

        first, second = GRIPPER_JOINTS[side]
        ET.SubElement(
            equality,
            "joint",
            {
                "name": f"{side}_finger_couple",
                "joint1": first,
                "joint2": second,
                "polycoef": "0 1 0 0 0",
                # Stiff on purpose. A MuJoCo equality is a soft constraint, and
                # the two finger servos are stiff enough to pull against a
                # default one: commanded to opposite ends they would settle a
                # centimetre apart instead of moving as the single degree of
                # freedom the real gripper motor gives.
                "solref": "0.001 1",
                "solimp": "0.99 0.9999 0.0001",
            },
        )
        for name in GRIPPER_ACTUATORS[side]:
            _add_gripper_actuator(actuator, name, options)
            added.append(name)
    return added


def _add_gripper_actuator(actuator: ET.Element, name: str, options: PandaGripperOptions) -> None:
    """One position servo, named after the finger joint it drives."""
    ET.SubElement(
        actuator,
        "position",
        {
            "name": name,
            "joint": name,
            # Stiff enough to actually squeeze: at a lower gain the jaw closes
            # on an object but does not generate the friction that holds it, and
            # the object works loose the moment the arm moves.
            "kp": f"{options.gain:.6g}",
            "dampratio": "1",
            "ctrlrange": f"0 {options.stroke_m:.6g}",
            "forcerange": f"{-options.grip_force_n:.6g} {options.grip_force_n:.6g}",
        },
    )


def gripper_targets(opening_m: float, side: str | None = None) -> Dict[str, float]:
    """Joint targets that open a hand by ``opening_m``, both fingers together.

    Args:
        opening_m: Travel per finger, ``0`` closed to :data:`FINGER_STROKE_M`
            open.  The gap between the pads is
            :data:`PAD_GAP_CLOSED_M` plus twice this.
        side: ``"left"``, ``"right"``, or ``None`` for both.

    Returns:
        ``{joint name: target}``, ready to merge into a ``dof_pos_target``.

    Raises:
        ValueError: If ``side`` is not a hand, or the opening is outside the
            travel -- silently clamping would hide a controller bug.
    """
    sides = ("left", "right") if side is None else (side,)
    for one in sides:
        if one not in GRIPPER_ACTUATORS:
            raise ValueError(f"side must be 'left', 'right' or None, not {side!r}")
    if not 0.0 <= opening_m <= FINGER_STROKE_M:
        raise ValueError(
            f"opening {opening_m} is outside the gripper's travel (0 to {FINGER_STROKE_M})"
        )
    return {joint: opening_m for one in sides for joint in GRIPPER_ACTUATORS[one]}


def pad_gap(opening_m: float) -> float:
    """Distance between the finger pads at a given opening, metres.

    The jaw does not close to nothing: :data:`PAD_GAP_CLOSED_M` is left at zero
    travel, which is the thinnest thing it can pinch.
    """
    return PAD_GAP_CLOSED_M + 2.0 * opening_m


def _gripper_sites(mjcf: ET.Element) -> Dict[str, Tuple[ET.Element, ET.Element]]:
    """Find the body and site for each hand, by the site names the exporter uses."""
    hosts: Dict[str, Tuple[ET.Element, ET.Element]] = {}
    for body in mjcf.iter("body"):
        for site in body.findall("site"):
            name = site.get("name", "")
            if name in ("gripper_left", "gripper_right"):
                hosts[name.rsplit("_", 1)[1]] = (body, site)
    return hosts
