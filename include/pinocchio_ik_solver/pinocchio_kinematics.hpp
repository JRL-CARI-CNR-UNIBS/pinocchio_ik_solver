/* Copyright (C) 2026 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

/**
 * @file pinocchio_kinematics.hpp
 * @brief ROS-free kinematics of a serial chain, computed with Pinocchio.
 */
#ifndef PINOCCHIO_IK_SOLVER__PINOCCHIO_KINEMATICS_HPP
#define PINOCCHIO_IK_SOLVER__PINOCCHIO_KINEMATICS_HPP

#include <map>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace pinocchio_ik_solver
{

/**
 * @brief Options of the numerical IK.
 */
struct IkOptions
{
  double translation_tolerance = 1e-5;  ///< Position error to accept a solution [m].
  double rotation_tolerance = 1e-4;     ///< Orientation error to accept a solution [rad].
  int max_iterations = 100;             ///< Iterations of a single attempt.
  double damping = 1e-3;                ///< Initial Levenberg-Marquardt damping.
  double max_step = 0.5;                ///< Maximum norm of a joint step [rad or m].
  double duplicate_tolerance = 1e-3;    ///< Distance below which two solutions are the same.
  int max_attempts = 100000;            ///< Upper bound on the attempts of solveMultiStart().
};

/**
 * @brief Result of a single IK attempt.
 */
struct IkResult
{
  Eigen::VectorXd q;                ///< Joint configuration, in the order of the chain joints.
  double translation_error = 0.0;   ///< Distance between the reached and the target position [m].
  double rotation_error = 0.0;      ///< Angle between the reached and the target orientation [rad].
  bool converged = false;           ///< True if both errors are below the tolerances.
};

/**
 * @brief Forward and inverse kinematics of the chain from @c base_frame to @c flange_frame.
 *
 * The URDF is reduced to the chain joints: all the other joints are locked (at zero, or at
 * the values given to init()). The configurations are in the order of @c joint_names.
 *
 * An instance is not thread-safe: use one instance per thread.
 */
class PinocchioKinematics
{
public:
  /**
   * @brief Build the model.
   * @param urdf_xml URDF.
   * @param joint_names Chain joints, in solver order (each must have one degree of freedom).
   * @param base_frame Frame in which the flange pose is expressed.
   * @param flange_frame End frame of the chain.
   * @param locked_joints Values of the joints that are not in the chain (missing ones: zero).
   * @param[out] error Reason of the failure.
   * @return False if the URDF cannot be parsed, or a joint or a frame is missing.
   */
  bool init(const std::string& urdf_xml, const std::vector<std::string>& joint_names, const std::string& base_frame,
            const std::string& flange_frame, const std::map<std::string, double>& locked_joints, std::string& error);

  /**
   * @brief Number of chain joints.
   */
  size_t dof() const { return joints_.size(); }

  /**
   * @brief Pose of the flange in the base frame.
   */
  Eigen::Affine3d fk(const Eigen::VectorXd& q);

  /**
   * @brief Damped least squares (Levenberg-Marquardt) IK from one initial guess, within the bounds.
   * @param T_base_flange Target pose of the flange in the base frame.
   * @param q0 Initial guess (clamped to the bounds).
   * @param lb Lower joint limits.
   * @param ub Upper joint limits.
   * @param opt Options.
   */
  IkResult solve(const Eigen::Affine3d& T_base_flange, const Eigen::VectorXd& q0, const Eigen::VectorXd& lb,
                 const Eigen::VectorXd& ub, const IkOptions& opt);

  /**
   * @brief Look for several distinct solutions, starting from the seeds and then from random configurations.
   *
   * Two solutions are the same if they differ by less than @c opt.duplicate_tolerance, with revolute
   * joints compared modulo 2π (the 2π copies are left to the caller). The search stops when
   * @p desired_solutions are found, after @p min_stall_attempts consecutive attempts without a new
   * solution if at least one has been found, after @p max_stall_attempts otherwise, or after
   * @c opt.max_attempts attempts.
   * @return The converged distinct solutions.
   */
  std::vector<IkResult> solveMultiStart(const Eigen::Affine3d& T_base_flange, const std::vector<Eigen::VectorXd>& seeds,
                                        const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                                        const std::vector<bool>& revolute, int desired_solutions,
                                        int min_stall_attempts, int max_stall_attempts, const IkOptions& opt,
                                        std::mt19937& rng);

private:
  struct JointIndex
  {
    int idx_q;  ///< First index in the Pinocchio configuration.
    int idx_v;  ///< Index in the Pinocchio velocity.
    int nq;     ///< 1, or 2 for continuous joints (cos, sin).
  };

  void setConfiguration(const Eigen::VectorXd& q);
  /** Flange pose in the base frame, after the forward kinematics. */
  pinocchio::SE3 relativePose() const;
  /** Pose error and, if @p jacobian is not null, its Jacobian, in the flange frame. */
  void evaluate(const Eigen::VectorXd& q, const pinocchio::SE3& target, Eigen::Matrix<double, 6, 1>& err,
                Eigen::MatrixXd* jacobian);
  static void errors(const pinocchio::SE3& current, const pinocchio::SE3& target, double& translation,
                     double& rotation);

  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<JointIndex> joints_;
  pinocchio::FrameIndex base_id_ = 0;
  pinocchio::FrameIndex flange_id_ = 0;
  Eigen::VectorXd q_pin_;
  pinocchio::Data::Matrix6x J_flange_;
  pinocchio::Data::Matrix6x J_base_;
};

}  // namespace pinocchio_ik_solver

#endif  // PINOCCHIO_IK_SOLVER__PINOCCHIO_KINEMATICS_HPP
