#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

#include "common/math_utils.hpp"

namespace interceptor
{
namespace
{

constexpr double kTol = 1e-9;

// Quaternion as (w, x, y, z), the layout used by quatToRot / rotToQuat.
Eigen::Vector4d axisAngleQuat(const Eigen::Vector3d & axis, double angle)
{
  const Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis.normalized()));
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

TEST(SkewSymmetric, MatchesCrossProduct)
{
  const Eigen::Vector3d a(1.0, -2.0, 3.5);
  const Eigen::Vector3d b(-0.5, 4.0, 2.0);
  EXPECT_TRUE((skewSymmetric(a) * b).isApprox(a.cross(b), kTol));
}

TEST(SkewSymmetric, IsAntiSymmetric)
{
  const Eigen::Matrix3d m = skewSymmetric(Eigen::Vector3d(0.3, 1.2, -4.0));
  EXPECT_TRUE((m + m.transpose()).isZero(kTol));
}

TEST(QuatToRot, IdentityQuaternionGivesIdentity)
{
  const Eigen::Vector4d q(1.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(quatToRot(q).isApprox(Eigen::Matrix3d::Identity(), kTol));
}

TEST(QuatToRot, MatchesEigenForArbitraryRotation)
{
  const Eigen::Vector3d axis(1.0, 2.0, -0.5);
  const double angle = 1.1;
  const Eigen::Matrix3d expected = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  EXPECT_TRUE(quatToRot(axisAngleQuat(axis, angle)).isApprox(expected, kTol));
}

TEST(QuatToRot, ProducesOrthonormalMatrix)
{
  const Eigen::Matrix3d r = quatToRot(axisAngleQuat(Eigen::Vector3d(0.2, -1.0, 0.7), 2.3));
  EXPECT_TRUE((r * r.transpose()).isApprox(Eigen::Matrix3d::Identity(), kTol));
  EXPECT_NEAR(r.determinant(), 1.0, kTol);
}

// Exercise every branch of rotToQuat: positive trace, and the three
// negative-trace cases where x, y or z is the dominant component.
class RotQuatRoundTrip : public ::testing::TestWithParam<Eigen::Vector4d>
{
};

TEST_P(RotQuatRoundTrip, RecoversRotation)
{
  const Eigen::Vector4d q = GetParam();
  const Eigen::Matrix3d r = quatToRot(q);
  const Eigen::Vector4d q_back = rotToQuat(r);

  EXPECT_NEAR(q_back.norm(), 1.0, kTol);
  // q and -q describe the same rotation.
  EXPECT_NEAR(std::abs(q_back.dot(q)), 1.0, 1e-9);
  EXPECT_TRUE(quatToRot(q_back).isApprox(r, kTol));
}

INSTANTIATE_TEST_SUITE_P(
  AllBranches, RotQuatRoundTrip,
  ::testing::Values(
    axisAngleQuat(Eigen::Vector3d(0.3, -0.4, 0.8), 0.5),
    axisAngleQuat(Eigen::Vector3d::UnitX(), 3.0),
    axisAngleQuat(Eigen::Vector3d::UnitY(), 3.0),
    axisAngleQuat(Eigen::Vector3d::UnitZ(), 3.0)));

TEST(SaturateVector, LeavesShortVectorUnchanged)
{
  const Eigen::Vector3d v(1.0, 2.0, 2.0);  // norm 3
  EXPECT_TRUE(saturateVector(v, 5.0).isApprox(v, kTol));
}

TEST(SaturateVector, ClampsLongVectorPreservingDirection)
{
  const Eigen::Vector3d v(3.0, 0.0, 4.0);  // norm 5
  const Eigen::Vector3d out = saturateVector(v, 2.0);
  EXPECT_NEAR(out.norm(), 2.0, kTol);
  EXPECT_TRUE(out.normalized().isApprox(v.normalized(), kTol));
}

TEST(SaturateVector, ZeroVectorStaysZero)
{
  EXPECT_TRUE(saturateVector(Eigen::Vector3d::Zero(), 0.0).isZero());
}

}  // namespace
}  // namespace interceptor
