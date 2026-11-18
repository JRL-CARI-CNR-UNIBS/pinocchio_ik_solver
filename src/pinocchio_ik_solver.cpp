/* Copyright (C) 2026 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#include "pinocchio_ik_solver/pinocchio_ik_solver.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ik_solver::PinocchioIkSolver, ik_solver::IkSolver)

namespace ik_solver
{

bool PinocchioIkSolver::config(const ParamMap& params)
{
  if (!IkSolver::config(params))
  {
    IK_FATAL(this->logger_, "Ik solver initial config FAILED");
    return false;
  }

  // Values of the URDF joints that are not in the chain
  std::map<std::string, double> locked;
  try
  {
    options_ = pinocchio_ik_solver::IkOptions();
    get_optional_param(this->logger_, params_, "translation_tolerance", options_.translation_tolerance);
    get_optional_param(this->logger_, params_, "rotation_tolerance", options_.rotation_tolerance);
    get_optional_param(this->logger_, params_, "max_iterations_per_attempt", options_.max_iterations);
    get_optional_param(this->logger_, params_, "damping", options_.damping);
    get_optional_param(this->logger_, params_, "max_step", options_.max_step);
    get_optional_param(this->logger_, params_, "duplicate_tolerance", options_.duplicate_tolerance);
    get_optional_param(this->logger_, params_, "max_attempts", options_.max_attempts);

    int seed = -1;
    get_optional_param(this->logger_, params_, "random_seed", seed);
    rng_.seed(seed < 0 ? std::random_device{}() : static_cast<unsigned int>(seed));

    for (const auto& joint : model_->joints_)
    {
      if (joint.second->type == urdf::Joint::FIXED ||
          std::find(joint_names_.begin(), joint_names_.end(), joint.first) != joint_names_.end())
        continue;
      double value = 0.0;
      get_optional_param(this->logger_, params_, "locked_joints." + joint.first, value);
      locked[joint.first] = value;
    }
  }
  catch (const std::invalid_argument&)
  {
    return false;  // wrong parameter type, already logged
  }

  std::string error;
  if (!kinematics_.init(robot_description_, joint_names_, base_frame_, flange_frame_, locked, error))
  {
    IK_FATAL(this->logger_, "Pinocchio Ik Solver: " << error);
    return false;
  }

  IK_INFO(this->logger_, "Pinocchio Ik Solver ready: " << kinematics_.dof() << " joints, " << locked.size()
                                                          << " locked, from " << base_frame_ << " to " << flange_frame_);
  return true;
}

Solutions PinocchioIkSolver::getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                                   const int& desired_solutions, const int& min_stall_iterations,
                                   const int& max_stall_iterations)
{
  const int desired = desired_solutions < 0 ? desired_solutions_ : desired_solutions;
  const int min_stall = min_stall_iterations < 0 ? min_stall_iter_ : min_stall_iterations;
  const int max_stall = max_stall_iterations < 0 ? max_stall_iter_ : max_stall_iterations;

  const std::vector<pinocchio_ik_solver::IkResult> found = kinematics_.solveMultiStart(
      T_base_flange, seeds, lb(), ub(), revolute(), desired, min_stall, max_stall, options_, rng_);

  Solutions sols;
  for (const auto& r : found)
  {
    // Add the 2π copies within the limits
    for (const Configuration& q : getMultiplicity({ r.q }, ub(), lb(), revolute()))
    {
      if (!outOfBound(q, ub(), lb()).empty())
        continue;
      sols.configurations().push_back(q);
      sols.translation_residuals().push_back(r.translation_error);
      sols.rotation_residuals().push_back(r.rotation_error);
    }
  }
  sols.message() = "Found " + std::to_string(found.size()) + " distinct solutions (" +
                   std::to_string(sols.configurations().size()) + " with 2pi copies)";
  return sols;
}

Eigen::Affine3d PinocchioIkSolver::getFK(const Configuration& s)
{
  return kinematics_.fk(s);
}

}  // namespace ik_solver
