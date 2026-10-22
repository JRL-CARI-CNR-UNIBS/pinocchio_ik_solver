/* Copyright (C) 2026 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#include "pinocchio_ik_solver/pinocchio_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace pinocchio_ik_solver
{

namespace
{
pinocchio::FrameIndex findFrame(const pinocchio::Model& model, const std::string& name)
{
  // Links are BODY frames; fall back to any frame type (e.g. a fixed joint)
  if (model.existFrame(name, pinocchio::BODY))
    return model.getFrameId(name, pinocchio::BODY);
  if (model.existFrame(name))
    return model.getFrameId(name);
  return model.frames.size();
}

pinocchio::SE3 toSE3(const Eigen::Affine3d& T)
{
  // Re-orthonormalize the rotation, which may come from a message with limited precision
  Eigen::Quaterniond quat(T.linear());
  return pinocchio::SE3(quat.normalized().toRotationMatrix(), T.translation());
}

Eigen::VectorXd clamp(const Eigen::VectorXd& q, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
{
  return q.cwiseMax(lb).cwiseMin(ub);
}

bool sameSolution(const Eigen::VectorXd& a, const Eigen::VectorXd& b, const std::vector<bool>& revolute, double tol)
{
  Eigen::VectorXd d = a - b;
  for (Eigen::Index i = 0; i < d.size(); i++)
  {
    if (revolute.at(i))
      d(i) = std::remainder(d(i), 2.0 * M_PI);
  }
  return d.norm() < tol;
}
}  // namespace

bool PinocchioKinematics::init(const std::string& urdf_xml, const std::vector<std::string>& joint_names,
                               const std::string& base_frame, const std::string& flange_frame,
                               const std::map<std::string, double>& locked_joints, std::string& error)
{
  pinocchio::Model full;
  try
  {
    pinocchio::urdf::buildModelFromXML(urdf_xml, full);
  }
  catch (const std::exception& e)
  {
    error = std::string("cannot parse the URDF: ") + e.what();
    return false;
  }

  for (const std::string& name : joint_names)
  {
    if (!full.existJointName(name))
    {
      error = "joint " + name + " is not in the URDF";
      return false;
    }
  }

  // Lock every joint that is not in the chain
  Eigen::VectorXd q_ref = pinocchio::neutral(full);
  std::vector<pinocchio::JointIndex> to_lock;
  for (pinocchio::JointIndex id = 1; id < static_cast<pinocchio::JointIndex>(full.njoints); id++)
  {
    const std::string& name = full.names[id];
    if (std::find(joint_names.begin(), joint_names.end(), name) != joint_names.end())
      continue;
    to_lock.push_back(id);

    auto it = locked_joints.find(name);
    if (it == locked_joints.end())
      continue;
    const auto& jm = full.joints[id];
    if (jm.nq() == 1)
    {
      q_ref(jm.idx_q()) = it->second;
    }
    else if (jm.nq() == 2 && jm.nv() == 1)
    {
      q_ref(jm.idx_q()) = std::cos(it->second);
      q_ref(jm.idx_q() + 1) = std::sin(it->second);
    }
  }
  model_ = to_lock.empty() ? full : pinocchio::buildReducedModel(full, to_lock, q_ref);
  data_ = pinocchio::Data(model_);

  joints_.clear();
  for (const std::string& name : joint_names)
  {
    const auto& jm = model_.joints[model_.getJointId(name)];
    if (jm.nv() != 1)
    {
      error = "joint " + name + " has " + std::to_string(jm.nv()) + " degrees of freedom (only 1 is supported)";
      return false;
    }
    joints_.push_back({ jm.idx_q(), jm.idx_v(), jm.nq() });
  }

  base_id_ = findFrame(model_, base_frame);
  flange_id_ = findFrame(model_, flange_frame);
  if (base_id_ >= model_.frames.size())
  {
    error = "frame " + base_frame + " is not in the URDF";
    return false;
  }
  if (flange_id_ >= model_.frames.size())
  {
    error = "frame " + flange_frame + " is not in the URDF";
    return false;
  }

  q_pin_ = pinocchio::neutral(model_);
  J_flange_.setZero(6, model_.nv);
  J_base_.setZero(6, model_.nv);
  return true;
}

void PinocchioKinematics::setConfiguration(const Eigen::VectorXd& q)
{
  for (size_t i = 0; i < joints_.size(); i++)
  {
    const JointIndex& j = joints_[i];
    if (j.nq == 1)
    {
      q_pin_(j.idx_q) = q(i);
    }
    else
    {
      q_pin_(j.idx_q) = std::cos(q(i));
      q_pin_(j.idx_q + 1) = std::sin(q(i));
    }
  }
}

pinocchio::SE3 PinocchioKinematics::relativePose() const
{
  return data_.oMf[base_id_].actInv(data_.oMf[flange_id_]);
}

Eigen::Affine3d PinocchioKinematics::fk(const Eigen::VectorXd& q)
{
  setConfiguration(q);
  pinocchio::forwardKinematics(model_, data_, q_pin_);
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::SE3 T = relativePose();
  Eigen::Affine3d out = Eigen::Affine3d::Identity();
  out.linear() = T.rotation();
  out.translation() = T.translation();
  return out;
}

void PinocchioKinematics::evaluate(const Eigen::VectorXd& q, const pinocchio::SE3& target,
                                   Eigen::Matrix<double, 6, 1>& err, Eigen::MatrixXd* jacobian)
{
  setConfiguration(q);
  if (jacobian)
    pinocchio::computeJointJacobians(model_, data_, q_pin_);  // also computes the forward kinematics
  else
    pinocchio::forwardKinematics(model_, data_, q_pin_);
  pinocchio::updateFramePlacements(model_, data_);

  // Error in the flange frame: err = log6(T_current^-1 * T_target)
  const pinocchio::SE3 iMd = relativePose().actInv(target);
  err = pinocchio::log6(iMd).toVector();

  if (!jacobian)
    return;

  // Jacobian of the flange motion relative to the base, in the flange frame
  J_flange_.setZero();
  J_base_.setZero();
  pinocchio::getFrameJacobian(model_, data_, flange_id_, pinocchio::LOCAL, J_flange_);
  pinocchio::getFrameJacobian(model_, data_, base_id_, pinocchio::LOCAL, J_base_);
  const pinocchio::SE3 fMb = data_.oMf[flange_id_].actInv(data_.oMf[base_id_]);
  pinocchio::Data::Matrix6x J_rel = J_flange_ - fMb.toActionMatrix() * J_base_;

  // Derivative of err with respect to the joint velocities
  pinocchio::Data::Matrix6 Jlog;
  pinocchio::Jlog6(iMd.inverse(), Jlog);
  pinocchio::Data::Matrix6x J_err = -Jlog * J_rel;

  jacobian->resize(6, joints_.size());
  for (size_t i = 0; i < joints_.size(); i++)
    jacobian->col(i) = J_err.col(joints_[i].idx_v);
}

void PinocchioKinematics::errors(const pinocchio::SE3& current, const pinocchio::SE3& target, double& translation,
                                 double& rotation)
{
  translation = (current.translation() - target.translation()).norm();
  rotation = Eigen::AngleAxisd(current.rotation().transpose() * target.rotation()).angle();
}

IkResult PinocchioKinematics::solve(const Eigen::Affine3d& T_base_flange, const Eigen::VectorXd& q0,
                                    const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const IkOptions& opt)
{
  const pinocchio::SE3 target = toSE3(T_base_flange);
  IkResult res;
  res.q = clamp(q0, lb, ub);

  Eigen::Matrix<double, 6, 1> err, err_new;
  Eigen::MatrixXd J, J_new;
  evaluate(res.q, target, err, &J);
  pinocchio::SE3 current = relativePose();
  double cost = err.squaredNorm();
  double lambda = opt.damping;

  for (int it = 0;; it++)
  {
    errors(current, target, res.translation_error, res.rotation_error);
    if (res.translation_error < opt.translation_tolerance && res.rotation_error < opt.rotation_tolerance)
    {
      res.converged = true;
      break;
    }
    if (it >= opt.max_iterations)
      break;

    Eigen::Matrix<double, 6, 6> H = J * J.transpose();
    H.diagonal().array() += lambda;
    Eigen::VectorXd dq = -J.transpose() * H.ldlt().solve(err);
    const double step = dq.norm();
    if (step > opt.max_step)
      dq *= opt.max_step / step;

    const Eigen::VectorXd q_new = clamp(res.q + dq, lb, ub);
    if ((q_new - res.q).norm() < 1e-12)
      break;  // blocked by the joint limits

    evaluate(q_new, target, err_new, &J_new);
    const double cost_new = err_new.squaredNorm();
    if (cost_new < cost)
    {
      res.q = q_new;
      err = err_new;
      J.swap(J_new);
      current = relativePose();
      cost = cost_new;
      lambda = std::max(lambda * 0.3, 1e-9);
    }
    else
    {
      lambda *= 10.0;
      if (lambda > 1e6)
        break;  // no descent direction
    }
  }
  return res;
}

std::vector<IkResult> PinocchioKinematics::solveMultiStart(
    const Eigen::Affine3d& T_base_flange, const std::vector<Eigen::VectorXd>& seeds, const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub, const std::vector<bool>& revolute, int desired_solutions, int min_stall_attempts,
    int max_stall_attempts, const IkOptions& opt, std::mt19937& rng)
{
  std::vector<IkResult> solutions;
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const size_t n = dof();
  int stall = 0;

  for (int attempt = 0; attempt < opt.max_attempts; attempt++)
  {
    Eigen::VectorXd q0(n);
    if (static_cast<size_t>(attempt) < seeds.size() && static_cast<size_t>(seeds[attempt].size()) == n)
    {
      q0 = seeds[attempt];
    }
    else
    {
      for (size_t i = 0; i < n; i++)
        q0(i) = lb(i) + unit(rng) * (ub(i) - lb(i));
    }

    IkResult r = solve(T_base_flange, q0, lb, ub, opt);
    bool is_new = false;
    if (r.converged)
    {
      is_new = std::none_of(solutions.begin(), solutions.end(), [&](const IkResult& s) {
        return sameSolution(s.q, r.q, revolute, opt.duplicate_tolerance);
      });
      if (is_new)
        solutions.push_back(r);
    }
    stall = is_new ? 0 : stall + 1;

    if (static_cast<int>(solutions.size()) >= desired_solutions)
      break;
    if (!solutions.empty() && stall >= min_stall_attempts)
      break;
    if (stall >= max_stall_attempts)
      break;
  }
  return solutions;
}

}  // namespace pinocchio_ik_solver
