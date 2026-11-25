// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Fabian Alejandro Ruiz Osorio (AlejoRuiz11)
#include "terrain_icp/ply.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>

namespace terrain_icp {
namespace {

std::filesystem::path temp_file(const std::string& name) {
  return std::filesystem::temp_directory_path() / ("terrain_icp_test_" + name);
}

void write_text(const std::filesystem::path& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary);
  out << text;
}

PointCloud sample_cloud() {
  PointCloud c;
  c.points = {{1.25, -2.5, 3.0}, {100.125, 200.5, -7.75}, {0.0, 0.0, 0.0}};
  c.normals = {{0.0, 0.0, 1.0}, {0.6, 0.0, 0.8}, {0.0, -1.0, 0.0}};
  return c;
}

class RoundTrip : public ::testing::TestWithParam<PlyFormat> {};

TEST_P(RoundTrip, PointsNormalsAndFieldSurvive) {
  const PointCloud c = sample_cloud();
  const std::vector<double> distance = {0.5, -0.25, 0.0};
  const ScalarField fields[] = {{"distance", distance}};
  const auto path = temp_file(GetParam() == PlyFormat::Ascii ? "rt_ascii.ply" : "rt_binary.ply");
  write_ply(path, c, GetParam(), fields);

  const PointCloud r = read_ply(path);
  ASSERT_EQ(r.points.size(), c.points.size());
  ASSERT_EQ(r.normals.size(), c.normals.size());
  for (std::size_t i = 0; i < c.points.size(); ++i) {
    EXPECT_LT((r.points[i] - c.points[i]).norm(), 1e-5);  // stored as float
    EXPECT_LT((r.normals[i] - c.normals[i]).norm(), 1e-6);
  }
}

INSTANTIATE_TEST_SUITE_P(Ply, RoundTrip, ::testing::Values(PlyFormat::Ascii, PlyFormat::BinaryLittleEndian));

TEST(Ply, AsciiMeshSkipsExtraPropertiesAndFaces) {
  const auto path = temp_file("mesh_ascii.ply");
  write_text(path,
             "ply\r\nformat ascii 1.0\r\ncomment made by hand\r\nelement vertex 3\r\nproperty float x\r\n"
             "property float y\r\nproperty float z\r\nproperty uchar red\r\nproperty float quality\r\n"
             "element face 1\r\nproperty list uchar int vertex_indices\r\nend_header\r\n"
             "0 0 1 255 0.5\r\n1 0 2 128 0.5\r\n0 1 3 0 0.5\r\n3 0 1 2\r\n");
  const PointCloud c = read_ply(path);
  ASSERT_EQ(c.points.size(), 3u);
  EXPECT_TRUE(c.normals.empty());
  EXPECT_EQ(c.points[2], Eigen::Vector3d(0, 1, 3));
}

TEST(Ply, BinaryWithDoublesAndMixedTypes) {
  const auto path = temp_file("mixed_binary.ply");
  {
    std::ofstream out(path, std::ios::binary);
    out << "ply\nformat binary_little_endian 1.0\nelement vertex 2\nproperty uchar flag\nproperty double x\n"
           "property double y\nproperty double z\nproperty short id\nend_header\n";
    for (int v = 0; v < 2; ++v) {
      const std::uint8_t flag = 1;
      const double xyz[3] = {1000000.25 + v, 2000000.5, 1234.125};  // doubles keep large coordinates exact
      const std::int16_t id = static_cast<std::int16_t>(v);
      out.write(reinterpret_cast<const char*>(&flag), 1);
      out.write(reinterpret_cast<const char*>(xyz), sizeof xyz);
      out.write(reinterpret_cast<const char*>(&id), sizeof id);
    }
  }
  const PointCloud c = read_ply(path);
  ASSERT_EQ(c.points.size(), 2u);
  EXPECT_EQ(c.points[1], Eigen::Vector3d(1000001.25, 2000000.5, 1234.125));
}

TEST(Ply, RejectsUnsupportedFiles) {
  const auto big = temp_file("big_endian.ply");
  write_text(big, "ply\nformat binary_big_endian 1.0\nelement vertex 1\nproperty float x\nend_header\n");
  EXPECT_THROW(read_ply(big), std::runtime_error);

  const auto truncated = temp_file("truncated.ply");
  write_text(truncated,
             "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\n"
             "end_header\n1 2 3\n");
  EXPECT_THROW(read_ply(truncated), std::runtime_error);

  const auto no_z = temp_file("no_z.ply");
  write_text(no_z, "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nend_header\n1 2\n");
  EXPECT_THROW(read_ply(no_z), std::runtime_error);

  EXPECT_THROW(read_ply(temp_file("does_not_exist.ply")), std::runtime_error);
}

}  // namespace
}  // namespace terrain_icp
