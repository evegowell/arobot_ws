"""Isaac Lab 3.0 articulation config for the Alice M1 mobile humanoid.

USD source: ``~/arobot_ws/alice_m1_usd/alice_m1_sim/alice_m1_sim.usda``
(converted from ``alice_m1_sim.urdf`` with ``--merge_joints --fix_base``).

All gains, armatures and the spawn height below are ESTIMATED first-test values,
not identified from hardware.

Mimic joints: the USD carries 12 ``NewtonMimicAPI`` constraints (2 waist + 10 finger distal),
which the PhysX USD parser turns into articulation mimic joints.

The two waist followers (lower_torso_mimic, upper_torso_mimic) have no PD drive: only the mimic
constraint holds them. With a stiff drive on both leader and follower (5000 / 500, 6.5 rad/s
joint velocity limit, no armature) the waist lift went into a self-sustained oscillation even with
the arms still; removing the follower drive alone made it hold within 1e-3 rad (01_move_arms.py
experiments, 2026-10-04).

The 10 finger distal followers are still in the finger group (stiffness 2). That is consistent
while all finger targets are 0; when the fingers are actuated, command follower = coef * leader
or move them into a stiffness-0 group like the waist followers.
"""

import os

from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

ALICE_M1_USD_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "alice_m1_usd", "alice_m1_sim", "alice_m1_sim.usda")
)
"""Main USD file produced by the URDF converter."""

# Joint name groups (regex, matched against the full joint name).
ARM_JOINTS = ["[lr]_sh_[pry]", "[lr]_el_p"]
WRIST_JOINTS = ["[lr]_wr_[ypr]"]
WAIST_JOINTS = ["waist_lower_pitch", "waist_upper_pitch", "waist_y"]
WAIST_MIMIC_FOLLOWERS = ["lower_torso_mimic", "upper_torso_mimic"]
HEAD_JOINTS = ["head_[yp]"]
FINGER_JOINTS = ["(left|right)_(thumb|index|middle|ring|pinky)_.*_joint"]
WHEEL_JOINTS = ["(left|right)_wheel_joint"]


ALICE_M1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=ALICE_M1_USD_PATH,
        # The articulation root API sits on the child prim ``base_footprint``, not on the spawn prim,
        # so the fragments are mapped to descendants ("/.*"). A bare list would only target the spawn prim.
        articulation_props={
            "/.*": [
                PhysxArticulationCfg(enabled_self_collisions=False),
                NewtonArticulationCfg(self_collision_enabled=False),
            ]
        },
        # The USD already has a fixed ``root_joint`` to the world; this keeps it enabled and lets the
        # PhysX backend normalize the articulation root for a fixed base.
        fix_root_link=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # ESTIMATED: base_footprint is 0.246 m above the wheel contact plane
        # (base_link is 0.127 m below base_footprint, wheel collision radius 0.119 m).
        pos=(0.0, 0.0, 0.246),
        rot=(0.0, 0.0, 0.0, 1.0),  # (x, y, z, w) in Isaac Lab 3.0
        joint_pos={".*": 0.0},
        joint_vel={".*": 0.0},
    ),
    actuators={
        # ESTIMATED gains for all groups below.
        "arms": ImplicitActuatorCfg(
            joint_names_expr=ARM_JOINTS,
            stiffness=400.0,
            damping=40.0,
            joint_effort_limit=60.0,
            armature=0.01,
        ),
        "wrists": ImplicitActuatorCfg(
            joint_names_expr=WRIST_JOINTS,
            stiffness=100.0,
            damping=10.0,
            joint_effort_limit=20.0,
            armature=0.01,
        ),
        # Stage 1: lock the waist leaders at the zero pose.
        "waist": ImplicitActuatorCfg(
            joint_names_expr=WAIST_JOINTS,
            stiffness=5000.0,
            damping=500.0,
            joint_effort_limit=1000.0,
        ),
        # Mimic followers: no drive, the mimic constraint alone sets them (see module docstring).
        "waist_mimic_followers": ImplicitActuatorCfg(
            joint_names_expr=WAIST_MIMIC_FOLLOWERS,
            stiffness=0.0,
            damping=0.0,
            joint_effort_limit=1000.0,
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=HEAD_JOINTS,
            stiffness=50.0,
            damping=5.0,
            joint_effort_limit=10.0,
        ),
        # Effort limits stay at the USD values (0.5 / 1.1 / 2.0 N·m from the URDF).
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=FINGER_JOINTS,
            stiffness=2.0,
            damping=0.1,
            armature=0.001,
        ),
        # Velocity-damped wheels: target velocity 0, no position stiffness.
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=WHEEL_JOINTS,
            stiffness=0.0,
            damping=10.0,
        ),
    },
)
"""Alice M1 with a fixed base, self collision off, and stage-1 (waist locked) gains."""
