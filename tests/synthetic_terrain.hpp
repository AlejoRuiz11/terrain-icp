// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Fabian Alejandro Ruiz Osorio (AlejoRuiz11)
#pragma once

// Synthetic terrain for the tests: a sum of Gaussian hills (no real site anywhere in this repository).

#include "terrain_icp/icp.hpp"

#include <cmath>
#include <random>
#include <vector>

namespace terrain_icp::test {

struct Hill {
  double x, y, height, sigma;  // [m]
};

// Rolling terrain of ~200 m x 200 m with relief of a few tens of metres.
inline const std::vector<Hill> kHills = {
    {-40.0, 30.0, 18.0, 25.0}, {35.0, 45.0, 12.0, 18.0}, {10.0, -35.0, 22.0, 30.0},
    {-55.0, -50.0, 9.0, 15.0}, {60.0, -10.0, -10.0, 20.0}, {-5.0, 5.0, 6.0, 10.0},
};

inline double terrain_height(double x, double y, const std::vector<Hill>& hills = kHills) {
  double z = 0.0;
  for (const Hill& h : hills) {
    const double r2 = (x - h.x) * (x - h.x) + (y - h.y) * (y - h.y);
    z += h.height * std::exp(-r2 / (2.0 * h.sigma * h.sigma));
  }
  return z;
}

// Regular grid (the "DEM") over [-half, half]^2 with the given spacing.
inline std::vector<Eigen::Vector3d> grid(double half, double spacing, const std::vector<Hill>& hills = kHills) {
  std::vector<Eigen::Vector3d> points;
  const int n = static_cast<int>(std::round(2.0 * half / spacing));
  for (int i = 0; i <= n; ++i) {
    for (int j = 0; j <= n; ++j) {
      const double x = -half + spacing * i;
      const double y = -half + spacing * j;
      points.emplace_back(x, y, terrain_height(x, y, hills));
    }
  }
  return points;
}

// Random surface samples (the "mesh vertices") over [-half, half]^2, with Gaussian noise of `noise` metres along z,
// expressed in the moving frame: x_moving = truth^-1(x_surface), so that aligning them recovers `truth`.
inline std::vector<Eigen::Vector3d> moving_samples(std::size_t count, double half, double noise,
                                                   const Similarity& truth, unsigned seed,
                                                   const std::vector<Hill>& hills = kHills) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> coord(-half, half);
  std::normal_distribution<double> gauss(0.0, noise);
  std::vector<Eigen::Vector3d> points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double x = coord(rng);
    const double y = coord(rng);
    const Eigen::Vector3d surface(x, y, terrain_height(x, y, hills) + gauss(rng));
    points.push_back(truth.rotation.transpose() * (surface - truth.translation) / truth.scale);
  }
  return points;
}

}  // namespace terrain_icp::test
