"""First motion test for Alice M1: swing both arms while the rest of the robot holds the zero pose.

Both shoulder pitches (sh_p) move between 0 and -1.0 rad and both elbows (el_p) between 0 and -1.2 rad
on a ~4 s sinusoid. Every other joint is commanded to 0. The torso tilt from vertical is printed every
5 s, and a waist diagnostic block every 1 s.

.. code-block:: bash

    # from ~/vla_ws/IsaacLab
    uv run --extra isaacsim isaaclab -p ~/arobot_ws/alice_lab/scripts/01_move_arms.py --viz kit

Diagnostic experiments (all off by default):

* ``--arm_scale 0``: keep the arms still, to see whether the waist moves without excitation.
* ``--waist_vel_limit V`` / ``--waist_armature A``: override velocity limit / armature on the waist joints.
* ``--solver_pos_iters N --solver_vel_iters M``: override the PhysX articulation solver iterations.

"""

"""Parse the command-line arguments first."""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description="Alice M1: swing both arms on a fixed base.")
parser.add_argument("--period", type=float, default=4.0, help="Arm swing period [s].")
parser.add_argument("--print_interval", type=float, default=5.0, help="Torso tilt print interval [s].")
parser.add_argument("--diag_interval", type=float, default=1.0, help="Waist diagnostic print interval [s].")
parser.add_argument("--arm_scale", type=float, default=1.0, help="Scale on the arm swing amplitude (0 = arms still).")
parser.add_argument(
    "--waist_vel_limit", type=float, default=None, help="Experiment: joint velocity limit on all waist joints [rad/s]."
)
parser.add_argument("--waist_armature", type=float, default=None, help="Experiment: armature on all waist joints.")
parser.add_argument("--solver_pos_iters", type=int, default=None, help="Experiment: PhysX position iterations.")
parser.add_argument("--solver_vel_iters", type=int, default=None, help="Experiment: PhysX velocity iterations.")
parser.add_argument(
    "--local_ground",
    action="store_true",
    help="Use a local collision box as the ground instead of the default ground plane asset (no network fetch).",
)
add_launcher_args(parser)
args_cli = parser.parse_args()

"""Rest everything follows."""

import math
import os
import sys

import torch
from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext

# alice_m1_cfg.py lives one directory above this script
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from alice_m1_cfg import ALICE_M1_CFG  # noqa: E402

SH_P_AMPLITUDE = -1.0 * args_cli.arm_scale  # [rad] negative = arm forward/up
EL_P_AMPLITUDE = -1.2 * args_cli.arm_scale  # [rad] negative = elbow flexion (ESTIMATED direction)
WAIST_JOINTS = ["waist_lower_pitch", "lower_torso_mimic", "upper_torso_mimic", "waist_upper_pitch", "waist_y"]
ARM_JOINTS = ["l_sh_p", "r_sh_p", "l_el_p", "r_el_p"]


def build_robot_cfg():
    """Return the robot cfg for this run, with the optional experiment overrides applied."""
    robot_cfg = ALICE_M1_CFG.replace(prim_path="/World/Robot")
    waist_overrides = {}
    if args_cli.waist_vel_limit is not None:
        waist_overrides["joint_velocity_limit"] = args_cli.waist_vel_limit
    if args_cli.waist_armature is not None:
        waist_overrides["armature"] = args_cli.waist_armature
    if waist_overrides:
        actuators = dict(robot_cfg.actuators)
        for group in ("waist", "waist_mimic_followers"):
            if group in actuators:
                actuators[group] = actuators[group].replace(**waist_overrides)
        robot_cfg = robot_cfg.replace(actuators=actuators)
    if args_cli.solver_pos_iters is not None or args_cli.solver_vel_iters is not None:
        physx_props = PhysxArticulationCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=args_cli.solver_pos_iters,
            solver_velocity_iteration_count=args_cli.solver_vel_iters,
        )
        spawn = robot_cfg.spawn.replace(
            articulation_props={"/.*": [physx_props, NewtonArticulationCfg(self_collision_enabled=False)]}
        )
        robot_cfg = robot_cfg.replace(spawn=spawn)
    return robot_cfg


def design_scene() -> Articulation:
    """Spawn the ground at z=0, a light, and one Alice M1."""
    if args_cli.local_ground:
        # 20 m x 20 m x 0.1 m static box whose top face is at z = 0
        cfg = sim_utils.CuboidCfg(
            size=(20.0, 20.0, 0.1),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.4, 0.4, 0.4)),
        )
        cfg.func("/World/ground", cfg, translation=(0.0, 0.0, -0.05))
    else:
        cfg = sim_utils.GroundPlaneCfg()
        cfg.func("/World/defaultGroundPlane", cfg)
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    return Articulation(cfg=build_robot_cfg())


def tilt_from_vertical(quat_xyzw: torch.Tensor) -> torch.Tensor:
    """Angle [rad] between the body z-axis and world z-axis, for quaternions in (x, y, z, w) order."""
    x, y = quat_xyzw[..., 0], quat_xyzw[..., 1]
    cos_tilt = 1.0 - 2.0 * (x * x + y * y)  # R[2][2]
    return torch.acos(torch.clamp(cos_tilt, -1.0, 1.0))


def print_sim_joint_properties(robot: Articulation, names: list[str]):
    """Print gains, effort and position limits as read back from the simulation."""
    ids, _ = robot.find_joints(names, preserve_order=True)
    stiffness = robot.data.joint_stiffness.torch[0]
    damping = robot.data.joint_damping.torch[0]
    effort = robot.data.joint_effort_limits.torch[0]
    pos_limits = robot.data.joint_pos_limits.torch[0]
    print("[DIAG] Joint properties read from the simulation:")
    print(f"  {'idx':>3} {'name':<20} {'stiffness':>10} {'damping':>9} {'effort_lim':>10} {'pos_limits':>18}")
    for name, i in zip(names, ids):
        lo, hi = pos_limits[i, 0].item(), pos_limits[i, 1].item()
        print(
            f"  {i:3d} {name:<20} {stiffness[i].item():10.2f} {damping[i].item():9.2f}"
            f" {effort[i].item():10.2f}   [{lo:+.3f}, {hi:+.3f}]"
        )


def print_waist_diagnostics(robot: Articulation, sim_time: float, waist_ids: list[int], torso_id: int, head_id: int):
    """Print per-joint waist state, targets and efforts, plus every joint with a non-zero target."""
    joint_pos = robot.data.joint_pos.torch[0]
    joint_vel = robot.data.joint_vel.torch[0]
    target = robot.actuators.target_command.position.torch[0]  # what the user wrote
    target_sim = robot.actuators.output_command.position.torch[0]  # what was submitted to the solver
    # For implicit actuators this is Isaac Lab's PD estimate kp*(q_des-q)+kd*(qd_des-qd), clipped;
    # the solver's own drive force is not exposed.
    applied = robot.actuators.applied_effort.torch[0]
    pos_limits = robot.data.joint_pos_limits.torch[0]
    try:
        gravity = robot.data.gravity_compensation_forces.torch[0]
        base_dofs = robot.num_base_dofs
    except Exception:  # not available on every backend
        gravity, base_dofs = None, 0

    torso_z = robot.data.body_link_pos_w.torch[0, torso_id, 2].item()
    head_z = robot.data.body_link_pos_w.torch[0, head_id, 2].item()
    print(f"[DIAG t={sim_time:6.2f}s] torso_link z={torso_z:.4f} m, head_pitch z={head_z:.4f} m")
    print(
        f"  {'joint':<18} {'q':>8} {'q_target':>9} {'q_sim':>8} {'qd':>8}"
        f" {'effort_est':>11} {'g(q)':>8}  limit"
    )
    for i in waist_ids:
        name = robot.joint_names[i]
        q = joint_pos[i].item()
        lo, hi = pos_limits[i, 0].item(), pos_limits[i, 1].item()
        flag = "OUT OF LIMIT" if (q < lo - 1e-3 or q > hi + 1e-3) else "ok"
        g = f"{gravity[base_dofs + i].item():8.2f}" if gravity is not None else "     n/a"
        print(
            f"  {name:<18} {q:+8.4f} {target[i].item():+9.4f} {target_sim[i].item():+8.4f} {joint_vel[i].item():+8.3f}"
            f" {applied[i].item():+11.2f} {g}  {flag}"
        )
    nonzero = [robot.joint_names[i] for i in torch.nonzero(target.abs() > 1e-6).flatten().tolist()]
    nonzero_sim = [robot.joint_names[i] for i in torch.nonzero(target_sim.abs() > 1e-6).flatten().tolist()]
    print(f"  non-zero targets (user): {nonzero}")
    print(f"  non-zero targets (sim):  {nonzero_sim}")


def run_simulator(sim: SimulationContext, robot: Articulation):
    """Runs the simulation loop until the window is closed."""
    print("[INFO]: Joint order (index: name):")
    for i, name in enumerate(robot.joint_names):
        print(f"  {i:2d}: {name}")
    print("[INFO]: Body names (index: name):")
    for i, name in enumerate(robot.body_names):
        print(f"  {i:2d}: {name}")
    print(
        f"[DIAG] Experiments: arm_scale={args_cli.arm_scale},"
        f" waist_vel_limit={args_cli.waist_vel_limit}, waist_armature={args_cli.waist_armature},"
        f" solver_pos_iters={args_cli.solver_pos_iters}, solver_vel_iters={args_cli.solver_vel_iters}"
    )
    print_sim_joint_properties(robot, WAIST_JOINTS + ARM_JOINTS)

    sh_p_ids, sh_p_names = robot.find_joints(["l_sh_p", "r_sh_p"], preserve_order=True)
    el_p_ids, el_p_names = robot.find_joints(["l_el_p", "r_el_p"], preserve_order=True)
    waist_ids, _ = robot.find_joints(WAIST_JOINTS, preserve_order=True)
    torso_id = robot.find_bodies("torso_link")[0][0]
    head_id = robot.find_bodies("head_pitch")[0][0]
    print("[DIAG] Sinusoid joints (index: name):")
    for i, name in zip(sh_p_ids + el_p_ids, sh_p_names + el_p_names):
        print(f"  {i:2d}: {name}")
    print(f"[INFO]: Swinging {sh_p_names} and {el_p_names}, period {args_cli.period:.1f} s")

    # start from the default (all-zero) state
    root_pose = robot.data.default_root_pose.torch.clone()
    robot.write_root_pose_to_sim_index(root_pose=root_pose)
    robot.write_root_velocity_to_sim_index(root_velocity=robot.data.default_root_vel.torch.clone())
    robot.write_joint_position_to_sim_index(position=robot.data.default_joint_pos.torch.clone())
    robot.write_joint_velocity_to_sim_index(velocity=robot.data.default_joint_vel.torch.clone())
    robot.reset()

    # all-zero position targets; only the arm columns change below (kept on the sim device)
    targets = torch.zeros((robot.num_instances, robot.num_joints), device=robot.device)

    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    next_print = 0.0
    next_diag = 0.0
    while sim.is_running():
        # smooth sinusoid from 0 to 1 and back, starting at 0 with zero velocity
        phase = 0.5 * (1.0 - math.cos(2.0 * math.pi * sim_time / args_cli.period))
        targets[:, sh_p_ids] = SH_P_AMPLITUDE * phase
        targets[:, el_p_ids] = EL_P_AMPLITUDE * phase
        robot.actuators.target_command.set_position_index(value=targets)

        robot.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        robot.update(sim_dt)

        if sim_time >= next_diag:
            next_diag += args_cli.diag_interval
            print_waist_diagnostics(robot, sim_time, waist_ids, torso_id, head_id)

        if sim_time >= next_print:
            next_print += args_cli.print_interval
            torso_quat = robot.data.body_link_quat_w.torch[:, torso_id]
            tilt_deg = math.degrees(tilt_from_vertical(torso_quat)[0].item())
            joint_pos = robot.data.joint_pos.torch[0]
            arms = ", ".join(
                f"{n}={joint_pos[i].item():+.3f}" for n, i in zip(sh_p_names + el_p_names, sh_p_ids + el_p_ids)
            )
            print(f"[t={sim_time:6.1f}s] torso tilt = {tilt_deg:6.3f} deg")
            print(f"            arms: {arms} (target sh_p={SH_P_AMPLITUDE * phase:+.3f}, el_p={EL_P_AMPLITUDE * phase:+.3f})")


def main():
    """Main function."""
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    with launch_simulation(sim_cfg, args_cli):
        sim = SimulationContext(sim_cfg)
        sim.set_camera_view([2.5, 2.0, 1.8], [0.0, 0.0, 1.0])
        robot = design_scene()
        sim.reset()
        print("[INFO]: Setup complete...")
        run_simulator(sim, robot)


if __name__ == "__main__":
    main()
