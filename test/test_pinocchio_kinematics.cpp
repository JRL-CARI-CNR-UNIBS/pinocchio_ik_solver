/* Copyright (C) 2026 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#include <cmath>
#include <fstream>
#include <sstream>

#include <gtest/gtest.h>

#include "pinocchio_ik_solver/pinocchio_kinematics.hpp"

using pinocchio_ik_solver::IkOptions;
using pinocchio_ik_solver::PinocchioKinematics;

namespace
{
const std::vector<std::string> UR_JOINTS = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                             "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };

std::string loadUrdf()
{
  std::ifstream file(TEST_URDF_PATH);
  std::stringstream ss;
  ss << file.rdbuf();
  return ss.str();
}

void expectSamePose(const Eigen::Affine3d& a, const Eigen::Affine3d& b, double tol)
{
  EXPECT_LT((a.translation() - b.translation()).norm(), tol);
  EXPECT_LT(Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle(), tol);
}

double distanceModulo2Pi(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
  Eigen::VectorXd d = a - b;
  for (Eigen::Index i = 0; i < d.size(); i++)
    d(i) = std::remainder(d(i), 2.0 * M_PI);
  return d.norm();
}

class UrKinematics : public ::testing::Test
{
protected:
  void SetUp() override
  {
    urdf_ = loadUrdf();
    ASSERT_FALSE(urdf_.empty());
    std::string error;
    ASSERT_TRUE(kin_.init(urdf_, UR_JOINTS, "base_link", "tool0", {}, error)) << error;
    lb_ = Eigen::VectorXd::Constant(6, -M_PI);
    ub_ = Eigen::VectorXd::Constant(6, M_PI);
  }

  Eigen::VectorXd randomConfiguration()
  {
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    Eigen::VectorXd q(6);
    for (int i = 0; i < 6; i++)
      q(i) = 0.95 * M_PI * unit(rng_);
    return q;
  }

  std::string urdf_;
  PinocchioKinematics kin_;
  Eigen::VectorXd lb_, ub_;
  std::vector<bool> revolute_ = std::vector<bool>(6, true);
  IkOptions opt_;
  std::mt19937 rng_{ 42 };
};
}  // namespace

TEST_F(UrKinematics, HomePose)
{
  // UR10e at zero (DH: d1=0.1807, a2=-0.6127, a3=-0.57155, d4=0.17415, d5=0.11985, d6=0.11655):
  // horizontal distance hypot(a2 + a3, d4 + d6), height d1 - d5
  const Eigen::Affine3d T = kin_.fk(Eigen::VectorXd::Zero(6));
  EXPECT_NEAR(T.translation().head<2>().norm(), std::hypot(1.18425, 0.2907), 1e-4);
  EXPECT_NEAR(T.translation().z(), 0.06085, 1e-4);
}

TEST_F(UrKinematics, SingleSolveFromNearbySeed)
{
  for (int k = 0; k < 50; k++)
  {
    const Eigen::VectorXd q = randomConfiguration();
    const Eigen::VectorXd seed = q + 0.05 * Eigen::VectorXd::Ones(6);
    const auto r = kin_.solve(kin_.fk(q), seed, lb_, ub_, opt_);
    ASSERT_TRUE(r.converged);
    expectSamePose(kin_.fk(r.q), kin_.fk(q), 1e-4);
    // Same branch; near singularities (e.g. elbow close to pi) the pose tolerance allows a few mrad
    EXPECT_LT((r.q - q).norm(), 1e-2) << "q = " << q.transpose() << "\nsol = " << r.q.transpose();
  }
}

TEST_F(UrKinematics, MultiStartFindsValidDistinctSolutions)
{
  int recovered = 0;
  const int n_targets = 30;
  for (int k = 0; k < n_targets; k++)
  {
    const Eigen::VectorXd q = randomConfiguration();
    const Eigen::Affine3d T = kin_.fk(q);
    const auto sols = kin_.solveMultiStart(T, {}, lb_, ub_, revolute_, 8, 50, 200, opt_, rng_);
    ASSERT_FALSE(sols.empty());
    EXPECT_LE(sols.size(), 8u);  // a 6R UR arm has at most 8 solutions modulo 2pi

    bool found = false;
    for (size_t i = 0; i < sols.size(); i++)
    {
      EXPECT_TRUE(sols[i].converged);
      EXPECT_LT(sols[i].translation_error, opt_.translation_tolerance);
      expectSamePose(kin_.fk(sols[i].q), T, 1e-4);
      EXPECT_TRUE(((sols[i].q - lb_).array() >= 0).all() && ((ub_ - sols[i].q).array() >= 0).all());
      for (size_t j = 0; j < i; j++)
        EXPECT_GT(distanceModulo2Pi(sols[i].q, sols[j].q), opt_.duplicate_tolerance);
      found |= distanceModulo2Pi(sols[i].q, q) < 1e-3;
    }
    recovered += found;
  }
  EXPECT_GE(recovered, n_targets * 9 / 10);
}

TEST_F(UrKinematics, SeedsAreUsedFirst)
{
  const Eigen::VectorXd q = randomConfiguration();
  const Eigen::VectorXd seed = q + 0.01 * Eigen::VectorXd::Ones(6);
  const auto sols = kin_.solveMultiStart(kin_.fk(q), { seed }, lb_, ub_, revolute_, 1, 10, 10, opt_, rng_);
  ASSERT_EQ(sols.size(), 1u);
  EXPECT_LT((sols[0].q - q).norm(), 1e-3);
}

TEST_F(UrKinematics, BaseFrameNotRoot)
{
  // base_link_inertia is rotated by pi around z with respect to base_link
  PinocchioKinematics other_base, fixed;
  std::string error;
  ASSERT_TRUE(other_base.init(urdf_, UR_JOINTS, "base_link_inertia", "tool0", {}, error)) << error;
  ASSERT_TRUE(fixed.init(urdf_, UR_JOINTS, "base_link_inertia", "base_link", {}, error)) << error;
  const Eigen::Affine3d T_inertia_base = fixed.fk(Eigen::VectorXd::Zero(6));

  for (int k = 0; k < 20; k++)
  {
    const Eigen::VectorXd q = randomConfiguration();
    const Eigen::Affine3d T = other_base.fk(q);
    expectSamePose(T, T_inertia_base * kin_.fk(q), 1e-9);

    const auto r = other_base.solve(T, q + 0.05 * Eigen::VectorXd::Ones(6), lb_, ub_, opt_);
    ASSERT_TRUE(r.converged);
    EXPECT_LT((r.q - q).norm(), 1e-3);
  }
}

TEST_F(UrKinematics, LockedJoints)
{
  // 5-joint chain with wrist_3 locked
  const std::vector<std::string> joints(UR_JOINTS.begin(), UR_JOINTS.begin() + 5);
  PinocchioKinematics reduced;
  std::string error;
  ASSERT_TRUE(reduced.init(urdf_, joints, "base_link", "tool0", { { "wrist_3_joint", 0.7 } }, error)) << error;
  ASSERT_EQ(reduced.dof(), 5u);

  Eigen::VectorXd q = randomConfiguration();
  q(5) = 0.7;
  expectSamePose(reduced.fk(q.head(5)), kin_.fk(q), 1e-9);
}

TEST_F(UrKinematics, JointOrderDifferentFromUrdf)
{
  std::vector<std::string> joints = UR_JOINTS;
  std::swap(joints[0], joints[5]);
  PinocchioKinematics swapped;
  std::string error;
  ASSERT_TRUE(swapped.init(urdf_, joints, "base_link", "tool0", {}, error)) << error;

  Eigen::VectorXd q = randomConfiguration();
  Eigen::VectorXd q_swapped = q;
  std::swap(q_swapped(0), q_swapped(5));
  expectSamePose(swapped.fk(q_swapped), kin_.fk(q), 1e-9);
}

TEST_F(UrKinematics, InitErrors)
{
  PinocchioKinematics k;
  std::string error;
  EXPECT_FALSE(k.init(urdf_, { "not_a_joint" }, "base_link", "tool0", {}, error));
  EXPECT_FALSE(k.init(urdf_, UR_JOINTS, "base_link", "not_a_frame", {}, error));
  EXPECT_FALSE(k.init("not an urdf", UR_JOINTS, "base_link", "tool0", {}, error));
}
