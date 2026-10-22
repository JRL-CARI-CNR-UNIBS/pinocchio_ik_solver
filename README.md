# pinocchio_ik_solver

Numerical inverse kinematics plugin for [`ik_solver`](https://github.com/JRL-CARI-CNR-UNIBS/ik_solver). It works with any serial chain described in a URDF, and computes the kinematics with [Pinocchio](https://github.com/stack-of-tasks/pinocchio).

It is a drop-in alternative to the analytic plugins (for example `ur_ik_solver`). Use it for robots that have no closed-form solver, chains with fewer or more than 6 joints, or chains that are part of a larger URDF (a mobile base, a second arm, a gripper).

## Contents

- [Installation](#installation)
- [Usage](#usage)
- [Parameters](#parameters)
- [How it works](#how-it-works)
- [Performance](#performance)
- [Testing](#testing)
- [Limitations](#limitations)
- [License](#license)

The C++ classes are documented in [docs/API.md](docs/API.md).

## Installation

Dependencies: ROS 2, `ik_solver`, and Pinocchio. Tested on Jazzy with Pinocchio 4.1.

```bash
sudo apt install ros-jazzy-pinocchio
cd ~/ros2_ws
colcon build --packages-select pinocchio_ik_solver
```

## Usage

The plugin is loaded by `ik_solver_node`, like any other `ik_solver` plugin. Set `type: ik_solver/PinocchioIkSolver` in the configuration file:

```yaml
pinocchio_ik:
  type: ik_solver/PinocchioIkSolver
  base_frame: base_link
  flange_frame: tool0
  tool_frame: tool0
  joint_names:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
  desired_solutions: 8
  min_stall_iterations: 50
  max_stall_iterations: 300
```

Then start the node with the `ik_solver` launch file:

```bash
ros2 launch ik_solver ik_solver.launch.py config:=/path/to/config.yaml
```

The node reads the URDF from the `robot_description` topic and offers the usual services (`get_ik`, `get_ik_array`, `get_fk`, `get_fk_array`, `get_bounds`, `set_bounds`, ...). They are described in the [`ik_solver` README](https://github.com/JRL-CARI-CNR-UNIBS/ik_solver#ik-solver-ros-node).

[`config/params.yaml`](config/params.yaml) is a complete example for a UR arm.

## Parameters

### Common parameters

These are read by the `ik_solver` base class:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `type` | string | required | `ik_solver/PinocchioIkSolver` |
| `base_frame` | string | required | Frame in which the flange pose is expressed. It doesn't have to be the URDF root. |
| `flange_frame` | string | required | End frame of the chain. |
| `tool_frame` | string | required | Frame of the IK target, rigidly attached to `flange_frame` (read from TF). |
| `joint_names` | string[] | required | Joints of the chain, in the order of the solutions. |
| `desired_solutions` | int | 8 | Distinct solutions (modulo 2π) to look for. |
| `min_stall_iterations` | int | 998 | Attempts in a row with no new solution before stopping, once at least one has been found. |
| `max_stall_iterations` | int | 999 | Attempts in a row with no new solution before stopping, if none has been found. |
| `limits/<joint>/lower`, `limits/<joint>/upper` | double | URDF | Narrower joint limits. |

The base-class defaults for the stall parameters are high for a numerical solver: a pose that can't be reached costs about 1000 attempts. Values around 50 and 300 work well for 6-joint arms.

### Pinocchio solver parameters

All are optional:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `translation_tolerance` | double | 1e-5 | Position error to accept a solution [m]. |
| `rotation_tolerance` | double | 1e-4 | Orientation error to accept a solution [rad]. |
| `max_iterations_per_attempt` | int | 100 | Iterations of one attempt. |
| `damping` | double | 1e-3 | Initial Levenberg–Marquardt damping. |
| `max_step` | double | 0.5 | Maximum norm of a joint step [rad or m]. |
| `duplicate_tolerance` | double | 1e-3 | Two solutions closer than this (modulo 2π for revolute joints) are the same. |
| `max_attempts` | int | 100000 | Upper bound on the attempts of one query. |
| `random_seed` | int | -1 | Seed of the random restarts (-1: random). Each solver instance (one per worker thread) uses the same seed. |
| `locked_joints/<joint>` | double | 0 | Value of a URDF joint that is not in `joint_names`. |

## How it works

1. **Model.** At configuration, the URDF is parsed with Pinocchio and reduced to `joint_names`. All other joints are locked at `locked_joints/<joint>`, or at 0 if not set. Fixed frames are kept, so `base_frame` and `flange_frame` can be any link of the URDF.
2. **Forward kinematics.** `getFK` returns the pose of `flange_frame` in `base_frame`.
3. **Single attempt.** One attempt runs Levenberg–Marquardt on the pose error `log6(T_current⁻¹ · T_target)`, expressed in the flange frame:
   - Each step is `dq = −Jᵀ (J Jᵀ + λI)⁻¹ e`, where `J` is the Jacobian of the error. It accounts for the base frame moving with the chain joints.
   - The step is limited to `max_step` and clamped to the joint limits.
   - λ shrinks after a step that reduces the error, and grows after one that doesn't.
   - The attempt succeeds when both the position and orientation errors are below the tolerances.
4. **Multi-start.** Each query starts one attempt from each seed in the request, then from random configurations within the limits.
   - A converged result is kept if it differs from those already found, comparing revolute joints modulo 2π.
   - The search stops when `desired_solutions` are found, or when the stall limits are reached.
5. **Response.** Each solution is expanded with its 2π copies that fit within the joint limits, and each copy comes with the translation and rotation residuals. So the response can contain more configurations than `desired_solutions`. For example, a UR10e with ±2π limits returns 8 distinct solutions as 256 configurations.

`ik_solver_node` creates one plugin instance per worker thread, and each instance has its own Pinocchio `Data`, so `get_ik_array` can run in parallel safely.

## Performance

Measured on a UR10e with `desired_solutions: 8`, `min_stall_iterations: 50` and `max_stall_iterations: 300`, through the `get_ik` service: 3–11 ms per pose, including the service call. All 8 solutions were found, and the residuals were below 1e-5 m and 1e-4 rad.

A numerical solver is slower than an analytic one such as `ur_ik_solver`. When an analytic plugin exists for your robot, prefer it.

## Testing

The unit tests check the kinematics core without ROS, on a UR10e URDF generated from `ur_description` at build time:
- FK at the home pose;
- convergence from nearby seeds, and seeds used first;
- distinct and valid multi-start solutions that include the original configuration;
- a base frame that isn't the URDF root;
- locked joints, and a joint order different from the URDF;
- error handling.

```bash
colcon build --packages-select pinocchio_ik_solver --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select pinocchio_ik_solver --event-handlers console_direct+
```

The tests need `ur_description` and `xacro`. If either is missing, the tests are skipped with a warning.

## Limitations

- Only 1-DOF joints (revolute, prismatic, continuous) can be in the chain.
- Continuous joints don't work well yet, because of how the `ik_solver` base class reads URDF limits. It dereferences `<limit>` (crashing if the tag is missing), gets [0, 0] for continuous joints, and doesn't mark them as revolute.
- The search is randomized. With low stall limits, some solutions may be missed. Use `random_seed` for repeatable results.
- Joint limits are handled by clamping. Near a limit, convergence can be slower than with a constrained (QP) step.
- No collision checking. Use the `ik_solver` collision-checking services for that.

## License

Apache License 2.0, see [LICENSE](LICENSE).

Dependencies:
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) is BSD-2-Clause, which is compatible with Apache-2.0. Pinocchio is linked, not distributed with this package.
- The test URDF is generated at build time from [`ur_description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) (BSD-3-Clause) and is not included in this package.
