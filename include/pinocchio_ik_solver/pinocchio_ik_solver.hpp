/* Copyright (C) 2026 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

/**
 * @file pinocchio_ik_solver.hpp
 * @brief Numerical IK solver plugin for any URDF chain, based on Pinocchio.
 */
#ifndef PINOCCHIO_IK_SOLVER__PINOCCHIO_IK_SOLVER_HPP
#define PINOCCHIO_IK_SOLVER__PINOCCHIO_IK_SOLVER_HPP

#include <random>

#include <ik_solver/ik_solver.hpp>

#include "pinocchio_ik_solver/pinocchio_kinematics.hpp"

namespace ik_solver
{
/**
 * @brief IK solver plugin based on Pinocchio: multi-start damped least squares within the joint limits.
 *
 * Parameters, in addition to the ones of IkSolverBase (all optional):
 * - @c translation_tolerance (double, default 1e-5 m), @c rotation_tolerance (double, default 1e-4 rad);
 * - @c max_iterations_per_attempt (int, default 100);
 * - @c damping (double, default 1e-3): initial Levenberg-Marquardt damping;
 * - @c max_step (double, default 0.5): maximum norm of a joint step;
 * - @c duplicate_tolerance (double, default 1e-3): two solutions closer than this (modulo 2π) are the same;
 * - @c max_attempts (int, default 100000): upper bound on the attempts of a query;
 * - @c random_seed (int, default -1 = random): seed of the random restarts;
 * - @c locked_joints/<joint> (double, default 0): value of a URDF joint that is not in @c joint_names.
 *
 * The stall iterations count consecutive attempts (seeds first, then random configurations) that
 * do not find a new solution. @c desired_solutions counts distinct solutions modulo 2π; the
 * returned list also contains their 2π copies within the joint limits.
 */
class PinocchioIkSolver : public IkSolver
{
public:
  bool config(const std::string& param_ns = "") override;

  Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                  const int& desired_solutions = -1, const int& min_stall_iterations = -1,
                  const int& max_stall_iterations = -1) override;

  Eigen::Affine3d getFK(const Configuration& s) override;

protected:
  pinocchio_ik_solver::PinocchioKinematics kinematics_;
  pinocchio_ik_solver::IkOptions options_;
  std::mt19937 rng_;
};
}  // namespace ik_solver

#endif  // PINOCCHIO_IK_SOLVER__PINOCCHIO_IK_SOLVER_HPP
