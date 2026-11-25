// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Fabian Alejandro Ruiz Osorio (AlejoRuiz11)
#pragma once

#include <Eigen/Core>

#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace terrain_icp {

// A point cloud in metres. `normals` is either empty or has one unit vector per point.
struct PointCloud {
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> normals;
};

// Reads the `vertex` element of a PLY file (ASCII or binary little-endian): x, y, z and, when all three are present,
// nx, ny, nz. Any other scalar vertex property (colour, confidence...) is skipped, and elements after `vertex`
// (faces of a mesh) are ignored. Throws std::runtime_error on anything else (big-endian, list properties in
// `vertex`, `vertex` not being the first element, truncated data).
PointCloud read_ply(const std::filesystem::path& path);

enum class PlyFormat { Ascii, BinaryLittleEndian };

// One extra per-vertex property, written as `float <name>`.
struct ScalarField {
  std::string name;
  std::span<const double> values;  // one value per point
};

// Writes x, y, z (as float), nx, ny, nz when `cloud.normals` is not empty, then the scalar fields.
void write_ply(const std::filesystem::path& path, const PointCloud& cloud, PlyFormat format,
               std::span<const ScalarField> fields = {});

}  // namespace terrain_icp
