// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Fabian Alejandro Ruiz Osorio (AlejoRuiz11)
#include "synthetic_terrain.hpp"
#include "terrain_icp/align_files.hpp"
#include "terrain_icp/ply.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <fstream>
#include <numbers>
#include <sstream>

namespace terrain_icp {
namespace {

// End to end through files: synthetic DEM and mesh as PLY -> align_files -> results.json and residuals.ply.
TEST(AlignFiles, WritesResultsAndResiduals) {
  const auto dir = std::filesystem::temp_directory_path() / "terrain_icp_test_align_files";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  Similarity truth;
  truth.scale = 1.005;
  truth.rotation = rotation_from_ypr(2.0 * std::numbers::pi / 180.0, 0.0, 0.0);
  truth.translation = {2.0, -2.0, 1.0};
  PointCloud reference{.points = test::grid(100.0, 1.0), .normals = {}};
  PointCloud moving{.points = test::moving_samples(4000, 70.0, 0.02, truth, 3), .normals = {}};
  write_ply(dir / "dem.ply", reference, PlyFormat::BinaryLittleEndian);
  write_ply(dir / "mesh.ply", moving, PlyFormat::Ascii);

  AlignRequest request;
  request.reference = dir / "dem.ply";
  request.moving = dir / "mesh.ply";
  request.out_dir = dir / "out";
  request.options.estimate_scale = true;
  std::ostringstream log;
  align_files(request, log);
  EXPECT_NE(log.str().find("converged"), std::string::npos) << log.str();

  std::ifstream in(request.out_dir / "results.json");
  const auto j = nlohmann::json::parse(in);
  EXPECT_NEAR(j["transform"]["rotation_deg"]["yaw"].get<double>(), 2.0, 0.01);
  EXPECT_NEAR(j["transform"]["scale"].get<double>(), 1.005, 1e-4);
  EXPECT_NEAR(j["transform"]["translation_m"][0].get<double>(), 2.0, 0.02);
  EXPECT_TRUE(j["fit"]["converged"].get<bool>());
  EXPECT_LT(j["fit"]["rms_m"].get<double>(), 0.025);
  EXPECT_EQ(j["inputs"]["reference"], "dem.ply");  // file name only, never the full path
  EXPECT_EQ(j["inputs"]["reference_normals"], "pca");
  EXPECT_TRUE(j["warnings"].empty());

  const PointCloud residuals = read_ply(request.out_dir / "residuals.ply");
  EXPECT_EQ(residuals.points.size(), moving.points.size());

  // A results.json is a valid initial guess: re-running from it starts at the solution.
  const Similarity init = read_initial_guess(request.out_dir / "results.json");
  EXPECT_NEAR(init.scale, j["transform"]["scale"].get<double>(), 1e-12);
  EXPECT_NEAR(init.translation.y(), j["transform"]["translation_m"][1].get<double>(), 1e-12);
}

}  // namespace
}  // namespace terrain_icp
