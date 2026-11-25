// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Fabian Alejandro Ruiz Osorio (AlejoRuiz11)
#pragma once

#include "terrain_icp/icp.hpp"

#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace terrain_icp {

// File-level driver of `terrain-icp align`.
struct AlignRequest {
  std::filesystem::path reference;  // PLY, e.g. the DEM converted to points
  std::filesystem::path moving;     // PLY, e.g. the vertices of the mesh to check
  std::filesystem::path out_dir;    // receives results.json and residuals.ply
  std::optional<std::filesystem::path> init;  // initial guess, see read_initial_guess
  IcpOptions options;
};

// Initial guess: {"scale": s, "rotation_deg": {"yaw": , "pitch": , "roll": }, "translation_m": [x, y, z]}, every key
// optional (identity by default). A results.json is accepted too: its "transform" object is read.
Similarity read_initial_guess(const std::filesystem::path& path);

// Human-readable warnings about the result (ill-conditioning, no convergence, few inliers).
std::vector<std::string> make_warnings(const IcpResult& result, const IcpOptions& options);

// Reads both clouds (reference normals from the file when present, PCA otherwise), runs the ICP, writes
// <out_dir>/results.json and <out_dir>/residuals.ply (aligned moving points with the fields distance, inlier and
// normal_z), and prints a short summary to `log`.
IcpResult align_files(const AlignRequest& request, std::ostream& log);

}  // namespace terrain_icp
