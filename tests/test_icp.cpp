#include "synthetic_terrain.hpp"
#include "terrain_icp/align_files.hpp"
#include "terrain_icp/icp.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <numbers>
#include <random>

namespace terrain_icp {
namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

bool has(const std::vector<std::string>& v, const char* s) { return std::find(v.begin(), v.end(), s) != v.end(); }

// The reference case of the specification: yaw 2 deg, 3 m translation, scale 1.005.
Similarity truth_transform(double scale = 1.005) {
  Similarity t;
  t.scale = scale;
  t.rotation = rotation_from_ypr(2.0 * kDeg, 0.0, 0.0);
  t.translation = {2.0, -2.0, 1.0};  // |t| = 3 m
  return t;
}

struct Scene {
  std::vector<Eigen::Vector3d> reference;
  std::vector<Eigen::Vector3d> normals;
  std::vector<Eigen::Vector3d> moving;
};

// DEM: 1 m grid over 200 m x 200 m. Mesh: 6000 random vertices over the central 140 m x 140 m, 2 cm noise.
Scene make_scene(const Similarity& truth, const std::vector<test::Hill>& hills = test::kHills) {
  Scene s;
  s.reference = test::grid(100.0, 1.0, hills);
  s.normals = estimate_normals(s.reference, 10);
  s.moving = test::moving_samples(6000, 70.0, 0.02, truth, 42, hills);
  return s;
}

void expect_recovered(const Similarity& estimate, const Similarity& truth, double angle_tol_deg,
                      double translation_tol_m, double scale_tol) {
  const Eigen::Vector3d error_ypr = ypr_from_rotation(truth.rotation.transpose() * estimate.rotation) / kDeg;
  EXPECT_LT(error_ypr.cwiseAbs().maxCoeff(), angle_tol_deg) << "yaw/pitch/roll error [deg] " << error_ypr.transpose();
  EXPECT_LT((estimate.translation - truth.translation).norm(), translation_tol_m)
      << "translation " << estimate.translation.transpose();
  EXPECT_NEAR(estimate.scale, truth.scale, scale_tol);
}

TEST(Rotation, YawPitchRollRoundTrip) {
  const Eigen::Matrix3d r = rotation_from_ypr(0.3, -0.2, 0.1);
  const Eigen::Vector3d ypr = ypr_from_rotation(r);
  EXPECT_NEAR(ypr.x(), 0.3, 1e-12);
  EXPECT_NEAR(ypr.y(), -0.2, 1e-12);
  EXPECT_NEAR(ypr.z(), 0.1, 1e-12);
  // Yaw is counter-clockwise about Up: East rotates towards North.
  const Eigen::Vector3d east = rotation_from_ypr(90.0 * kDeg, 0.0, 0.0) * Eigen::Vector3d::UnitX();
  EXPECT_LT((east - Eigen::Vector3d::UnitY()).norm(), 1e-12);
}

TEST(Normals, TiltedPlaneGivesItsNormalPointingUp) {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) points.emplace_back(i, j, 0.3 * i - 0.2 * j);
  }
  const Eigen::Vector3d expected = Eigen::Vector3d(-0.3, 0.2, 1.0).normalized();
  for (const auto& n : estimate_normals(points, 10)) EXPECT_LT((n - expected).norm(), 1e-9);
}

TEST(Icp, RecoversKnownSimilarity) {
  const Similarity truth = truth_transform();
  const Scene s = make_scene(truth);
  IcpOptions options;
  options.estimate_scale = true;
  const IcpResult r = align(s.reference, s.normals, s.moving, Similarity{}, options);

  EXPECT_TRUE(r.converged);
  expect_recovered(r.transform, truth, 0.01, 0.02, 1e-4);
  EXPECT_LT(r.rms, 0.025);  // the 2 cm noise (trimmed), nothing more
  EXPECT_TRUE(r.weak_parameters.empty());
  EXPECT_LT(r.condition_number, options.max_condition);
  EXPECT_EQ(r.eigenvalues.size(), 7u);
}

TEST(Icp, RigidAlignmentWithoutScale) {
  const Similarity truth = truth_transform(1.0);
  const Scene s = make_scene(truth);
  const IcpResult r = align(s.reference, s.normals, s.moving, Similarity{}, IcpOptions{});

  EXPECT_TRUE(r.converged);
  expect_recovered(r.transform, truth, 0.01, 0.02, 0.0);
  EXPECT_EQ(r.eigenvalues.size(), 6u);
}

TEST(Icp, RobustToTenPercentOutliers) {
  const Similarity truth = truth_transform();
  Scene s = make_scene(truth);
  // 10 % of the mesh vertices lifted 3-20 m (vegetation, buildings, reconstruction blobs).
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> lift(3.0, 20.0);
  for (std::size_t i = 0; i < s.moving.size(); i += 10) s.moving[i].z() += lift(rng);

  IcpOptions options;
  options.estimate_scale = true;
  const IcpResult r = align(s.reference, s.normals, s.moving, Similarity{}, options);

  EXPECT_TRUE(r.converged);
  expect_recovered(r.transform, truth, 0.01, 0.03, 1e-4);
  std::size_t outliers_kept = 0;
  for (std::size_t i = 0; i < s.moving.size(); i += 10) outliers_kept += r.inliers[i] ? 1 : 0;
  EXPECT_EQ(outliers_kept, 0u);
}

TEST(Icp, FlatTerrainIsReportedAndHorizontalDofsHeld) {
  const Similarity truth = truth_transform();
  const Scene s = make_scene(truth, /*hills=*/{});
  IcpOptions options;
  options.estimate_scale = true;
  const IcpResult r = align(s.reference, s.normals, s.moving, Similarity{}, options);

  EXPECT_TRUE(has(r.weak_parameters, "rotation_z"));
  EXPECT_TRUE(has(r.weak_parameters, "translation_x"));
  EXPECT_TRUE(has(r.weak_parameters, "translation_y"));
  EXPECT_TRUE(has(r.weak_parameters, "scale"));
  EXPECT_FALSE(has(r.weak_parameters, "translation_z"));
  EXPECT_GT(r.condition_number, options.max_condition);

  // The unobservable parameters stay at the initial guess instead of drifting; the vertical fit still works.
  EXPECT_LT(std::abs(r.transform.translation.x()), 0.01);
  EXPECT_LT(std::abs(r.transform.translation.y()), 0.01);
  EXPECT_LT(r.rms, 0.025);

  const auto warnings = make_warnings(r, options);
  ASSERT_FALSE(warnings.empty());
  EXPECT_EQ(warnings.front().rfind("flat terrain", 0), 0u) << warnings.front();
}

TEST(Icp, MaxDistanceRejectsFarPairs) {
  const Similarity truth = truth_transform(1.0);
  const Scene s = make_scene(truth);
  IcpOptions options;
  options.max_distance = 1e-3;  // nothing is that close before alignment
  EXPECT_THROW(align(s.reference, s.normals, s.moving, Similarity{}, options), std::runtime_error);
}

}  // namespace
}  // namespace terrain_icp
