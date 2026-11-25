#pragma once

#include <Eigen/Core>

#include <limits>
#include <string>
#include <vector>

namespace terrain_icp {

// Frame convention: x = East, y = North, z = Up (local metric frame, e.g. a DEM converted around a chosen origin).

// Similarity transform mapping a moving point onto the reference frame: x_ref = scale * rotation * x + translation.
struct Similarity {
  double scale = 1.0;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  Eigen::Vector3d apply(const Eigen::Vector3d& p) const { return scale * (rotation * p) + translation; }
};

// R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in radians. Yaw is counter-clockwise about Up seen from above.
Eigen::Matrix3d rotation_from_ypr(double yaw, double pitch, double roll);
// Inverse of rotation_from_ypr: returns (yaw, pitch, roll) in radians, pitch in [-pi/2, pi/2].
Eigen::Vector3d ypr_from_rotation(const Eigen::Matrix3d& rotation);

// Unit normal of each point from PCA of its k nearest neighbours (eigenvector of the smallest covariance
// eigenvalue), oriented upwards (n.z >= 0): terrain is a height field, so "up" is the outside.
std::vector<Eigen::Vector3d> estimate_normals(const std::vector<Eigen::Vector3d>& points, int neighbours);

struct IcpOptions {
  bool estimate_scale = false;  // 7-parameter similarity instead of a 6-parameter rigid transform
  // A pair is rejected when the moving point is farther than this from its nearest reference point [m].
  double max_distance = std::numeric_limits<double>::infinity();
  double trim_ratio = 0.1;  // fraction of the remaining pairs with the largest |residual| dropped every iteration
  int max_iterations = 100;
  // Directions of the normal matrix with eigenvalue < lambda_max / max_condition are not updated (held at their
  // initial value) and reported as weak: the terrain does not constrain them.
  double max_condition = 1e3;
  int normal_neighbours = 10;
  // Convergence: stop when one iteration changes the rotation, translation and scale by less than these.
  double tol_rotation = 1e-7;     // [rad]
  double tol_translation = 1e-5;  // [m]
  double tol_scale = 1e-7;        // [-]
};

struct IcpResult {
  Similarity transform;  // total moving -> reference transform (includes the initial guess)
  int iterations = 0;
  bool converged = false;
  // Final pairs (after the last update): signed point-to-plane distance of every moving point to its nearest
  // reference point's tangent plane [m], positive when the moving point is above the reference; and whether that
  // pair passed the distance and trimming filters.
  std::vector<double> signed_distances;
  std::vector<double> normal_z;  // z component of the paired reference normal: a height error dz shows as n_z * dz
  std::vector<bool> inliers;
  std::size_t inlier_count = 0;
  double rms = 0.0;  // RMS of the inlier signed distances [m]
  // Conditioning of the final normal matrix (6x6, or 7x7 with scale), normalised per pair and with the rotation and
  // scale columns divided by the cloud radius so that every unknown moves a typical point by ~1 m per unit.
  std::vector<double> eigenvalues;  // ascending
  double condition_number = 0.0;    // lambda_max / lambda_min (infinity when lambda_min <= 0)
  // Parameters lying mostly (> 50 %) in the weak subspace: "rotation_x", ..., "translation_z", "scale".
  std::vector<std::string> weak_parameters;
};

// Point-to-plane ICP of `moving` onto `reference` (with one normal per reference point), from `initial`.
IcpResult align(const std::vector<Eigen::Vector3d>& reference, const std::vector<Eigen::Vector3d>& reference_normals,
                const std::vector<Eigen::Vector3d>& moving, const Similarity& initial, const IcpOptions& options);

}  // namespace terrain_icp
