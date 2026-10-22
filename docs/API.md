# pinocchio_ik_solver API reference

This page covers the C++ classes. The parameters and the ROS interface are described in the [README](../README.md). The services are those of `ik_solver_node`, see the [`ik_solver` README](https://github.com/JRL-CARI-CNR-UNIBS/ik_solver#ik-solver-ros-node).

## Contents

- [`include/pinocchio_ik_solver/pinocchio_ik_solver.hpp`](#includepinocchio_ik_solverpinocchio_ik_solverhpp)
  - [`ik_solver::PinocchioIkSolver`](#class-ik_solverpinocchioiksolver)
- [`include/pinocchio_ik_solver/pinocchio_kinematics.hpp`](#includepinocchio_ik_solverpinocchio_kinematicshpp)
  - [`pinocchio_ik_solver::IkOptions`](#struct-pinocchio_ik_solverikoptions)
  - [`pinocchio_ik_solver::IkResult`](#struct-pinocchio_ik_solverikresult)
  - [`pinocchio_ik_solver::PinocchioKinematics`](#class-pinocchio_ik_solverpinocchiokinematics)
- [Libraries and CMake targets](#libraries-and-cmake-targets)

## include/pinocchio_ik_solver/pinocchio_ik_solver.hpp

Numerical IK solver plugin for any URDF chain, based on Pinocchio.

### class `ik_solver::PinocchioIkSolver`

Inherits from `ik_solver::IkSolver`. Exported with pluginlib as `ik_solver/PinocchioIkSolver`.

Multi-start damped least squares within the joint limits. It is a thin wrapper around [`PinocchioKinematics`](#class-pinocchio_ik_solverpinocchiokinematics): it reads the parameters, runs the search, and adds the 2π copies of the solutions.

#### Public functions

##### `config`

```cpp
bool config(const std::string& param_ns = "") override
```

Calls `IkSolver::config()`, which reads the common parameters, the URDF and the joint limits. Then it reads the solver parameters and builds the reduced Pinocchio model.

The value of each non-fixed URDF joint that is not in `joint_names` is read from `<param_ns>/locked_joints/<joint>` (default 0).

- `param_ns`: parameter namespace.
- Returns false if the base configuration fails, or if a joint or frame is missing from the URDF.

##### `getIk`

```cpp
Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                const int& desired_solutions = -1, const int& min_stall_iterations = -1,
                const int& max_stall_iterations = -1) override
```

Computes the IK solutions of a flange pose. Seeds with the wrong size are replaced by random configurations.

- `T_base_flange`: pose of the flange in the base frame.
- `seeds`: initial guesses, tried first and in order.
- `desired_solutions`: number of distinct solutions (modulo 2π) to look for. -1 uses the parameter.
- `min_stall_iterations`: attempts in a row with no new solution before stopping, once a solution is found. -1 uses the parameter.
- `max_stall_iterations`: attempts in a row with no new solution before stopping, if none is found. -1 uses the parameter.
- Returns the solutions, each with its 2π copies within the limits, and their translation and rotation residuals. The message gives the number of distinct solutions and of configurations.

##### `getFK`

```cpp
Eigen::Affine3d getFK(const Configuration& s) override
```

- `s`: joint configuration, in the order of `joint_names`.
- Returns the pose of the flange in the base frame.

#### Protected members

| Member | Description |
|---|---|
| `pinocchio_ik_solver::PinocchioKinematics kinematics_` | Kinematics of the chain. |
| `pinocchio_ik_solver::IkOptions options_` | Solver options, from the parameters. |
| `std::mt19937 rng_` | Generator of the random restarts, seeded from `random_seed`. |

## include/pinocchio_ik_solver/pinocchio_kinematics.hpp

ROS-free kinematics of a serial chain, computed with Pinocchio. It depends only on Pinocchio and Eigen, so it can be used outside ROS and outside `ik_solver`.

### struct `pinocchio_ik_solver::IkOptions`

Options of the numerical IK.

| Field | Type | Default | Description |
|---|---|---|---|
| `translation_tolerance` | double | 1e-5 | Position error to accept a solution [m]. |
| `rotation_tolerance` | double | 1e-4 | Orientation error to accept a solution [rad]. |
| `max_iterations` | int | 100 | Iterations of a single attempt. |
| `damping` | double | 1e-3 | Initial Levenberg–Marquardt damping. |
| `max_step` | double | 0.5 | Maximum norm of a joint step [rad or m]. |
| `duplicate_tolerance` | double | 1e-3 | Distance below which two solutions are the same. |
| `max_attempts` | int | 100000 | Upper bound on the attempts of `solveMultiStart()`. |

### struct `pinocchio_ik_solver::IkResult`

Result of a single IK attempt.

| Field | Type | Description |
|---|---|---|
| `q` | `Eigen::VectorXd` | Joint configuration, in the order of the chain joints. |
| `translation_error` | double | Distance between the reached and the target position [m]. |
| `rotation_error` | double | Angle between the reached and the target orientation [rad]. |
| `converged` | bool | True if both errors are below the tolerances. |

### class `pinocchio_ik_solver::PinocchioKinematics`

Forward and inverse kinematics of the chain from `base_frame` to `flange_frame`.

The URDF is reduced to the chain joints, and every other joint is locked. Configurations are in the order of `joint_names`, which can differ from the URDF order.

An instance isn't thread-safe, because it owns its Pinocchio `Data`. Use one instance per thread.

#### Public functions

##### `init`

```cpp
bool init(const std::string& urdf_xml, const std::vector<std::string>& joint_names,
          const std::string& base_frame, const std::string& flange_frame,
          const std::map<std::string, double>& locked_joints, std::string& error)
```

Builds the model.

- `urdf_xml`: URDF.
- `joint_names`: chain joints, in solver order. Each must have one degree of freedom.
- `base_frame`: frame in which the flange pose is expressed. Links are searched first, then any frame type.
- `flange_frame`: end frame of the chain.
- `locked_joints`: values of the joints that are not in the chain. Missing joints are locked at 0. For a continuous joint, the value is an angle.
- `error`: reason of the failure (output).
- Returns false if the URDF can't be parsed, a joint or frame is missing, or a chain joint has more than one degree of freedom.

##### `dof`

```cpp
size_t dof() const
```

Number of chain joints.

##### `fk`

```cpp
Eigen::Affine3d fk(const Eigen::VectorXd& q)
```

Pose of the flange in the base frame.

##### `solve`

```cpp
IkResult solve(const Eigen::Affine3d& T_base_flange, const Eigen::VectorXd& q0,
               const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const IkOptions& opt)
```

Damped least squares (Levenberg–Marquardt) IK from one initial guess, within the bounds. The rotation of `T_base_flange` is re-orthonormalized.

- `T_base_flange`: target pose of the flange in the base frame.
- `q0`: initial guess, clamped to the bounds.
- `lb`, `ub`: joint limits.
- `opt`: options.
- Returns the last configuration and its errors. `converged` says whether the tolerances were met.

##### `solveMultiStart`

```cpp
std::vector<IkResult> solveMultiStart(const Eigen::Affine3d& T_base_flange,
                                      const std::vector<Eigen::VectorXd>& seeds,
                                      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                                      const std::vector<bool>& revolute, int desired_solutions,
                                      int min_stall_attempts, int max_stall_attempts,
                                      const IkOptions& opt, std::mt19937& rng)
```

Looks for several distinct solutions, starting from the seeds and then from random configurations drawn uniformly within the bounds.

Two solutions are the same if they differ by less than `opt.duplicate_tolerance`, with revolute joints compared modulo 2π. The 2π copies are left to the caller.

The search stops at the first of:
- `desired_solutions` solutions found;
- `min_stall_attempts` attempts in a row with no new solution, if at least one solution has been found;
- `max_stall_attempts` attempts in a row with no new solution;
- `opt.max_attempts` attempts in total.

Returns the converged distinct solutions, in the order they were found.

#### Example

```cpp
#include <pinocchio_ik_solver/pinocchio_kinematics.hpp>

pinocchio_ik_solver::PinocchioKinematics kin;
std::string error;
if (!kin.init(urdf, joint_names, "base_link", "tool0", {}, error))
  throw std::runtime_error(error);

Eigen::Affine3d target = kin.fk(q);
std::mt19937 rng(42);
auto solutions = kin.solveMultiStart(target, {}, lb, ub, std::vector<bool>(kin.dof(), true),
                                     8, 50, 300, pinocchio_ik_solver::IkOptions(), rng);
```

## Libraries and CMake targets

| Target | Description |
|---|---|
| `pinocchio_kinematics` | ROS-free kinematics (`PinocchioKinematics`). It depends on Pinocchio and Eigen. |
| `pinocchio_ik_solver` | The pluginlib plugin. It depends on `pinocchio_kinematics`, `ik_solver` and `pluginlib`. |

Both are exported as `pinocchio_ik_solver::<target>`. Use them with `find_package(pinocchio_ik_solver REQUIRED)`.
