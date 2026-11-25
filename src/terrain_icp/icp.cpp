// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Fabian Alejandro Ruiz Osorio (AlejoRuiz11)
#include "terrain_icp/icp.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <nanoflann.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace terrain_icp {

namespace {

using Vec3 = Eigen::Vector3d;

// nanoflann dataset adaptor over a vector of points (the vector must outlive the tree).
struct CloudAdaptor {
  const std::vector<Vec3>& points;
  std::size_t kdtree_get_point_count() const { return points.size(); }
  double kdtree_get_pt(std::size_t index, std::size_t dim) const {
    return points[index][static_cast<Eigen::Index>(dim)];
  }
  template <class BoundingBox>
  bool kdtree_get_bbox(BoundingBox&) const {
    return false;  // let nanoflann compute it
  }
};

class KdTree {
 public:
  explicit KdTree(const std::vector<Vec3>& points)
      : adaptor_{points}, tree_(3, adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(16)) {}

  // Indices and squared distances of the k nearest points to `query` (k is clamped to the cloud size).
  std::size_t knn(const Vec3& query, std::size_t k, std::uint32_t* indices, double* squared_distances) const {
    return tree_.knnSearch(query.data(), k, indices, squared_distances);
  }

 private:
  using Tree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, CloudAdaptor>, CloudAdaptor,
                                                   3, std::uint32_t>;
  CloudAdaptor adaptor_;
  Tree tree_;
};

constexpr std::array<const char*, 7> kParameterNames = {"rotation_x",    "rotation_y",    "rotation_z", "translation_x",
                                                        "translation_y", "translation_z", "scale"};

// Nearest-neighbour pairs of every (already transformed) moving point, with the distance and trimming filters.
struct Pairs {
  std::vector<double> signed_distances;
  std::vector<std::uint32_t> nearest;
  std::vector<bool> inliers;
  std::size_t inlier_count = 0;
};

Pairs match(const KdTree& tree, const std::vector<Vec3>& reference, const std::vector<Vec3>& normals,
            const std::vector<Vec3>& moved, const IcpOptions& options) {
  const std::size_t n = moved.size();
  Pairs pairs;
  pairs.signed_distances.resize(n);
  pairs.nearest.resize(n);
  pairs.inliers.assign(n, false);

  const double max_sq = options.max_distance * options.max_distance;
  std::vector<double> candidate_abs;  // |residual| of the pairs within max_distance
  candidate_abs.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    std::uint32_t j = 0;
    double squared = 0.0;
    tree.knn(moved[i], 1, &j, &squared);
    pairs.nearest[i] = j;
    // Point-to-plane distance: projection of the offset on the reference normal (normals point up, so a positive
    // value means the moving point is above the reference surface).
    pairs.signed_distances[i] = normals[j].dot(moved[i] - reference[j]);
    if (squared <= max_sq) {
      pairs.inliers[i] = true;
      candidate_abs.push_back(std::abs(pairs.signed_distances[i]));
    }
  }
  if (candidate_abs.empty()) return pairs;

  // Trimmed ICP: keep the (1 - trim_ratio) fraction of candidates with the smallest |residual|.
  const auto keep = static_cast<std::size_t>(
      std::ceil((1.0 - options.trim_ratio) * static_cast<double>(candidate_abs.size())));
  const std::size_t last = std::clamp<std::size_t>(keep, 1, candidate_abs.size()) - 1;
  std::nth_element(candidate_abs.begin(), candidate_abs.begin() + static_cast<std::ptrdiff_t>(last),
                   candidate_abs.end());
  const double threshold = candidate_abs[last];
  for (std::size_t i = 0; i < n; ++i) {
    if (pairs.inliers[i] && std::abs(pairs.signed_distances[i]) > threshold) pairs.inliers[i] = false;
    if (pairs.inliers[i]) ++pairs.inlier_count;
  }
  return pairs;
}

// Linearised point-to-plane least squares about the centroid c of the inlier moving points. For a small update
// (rotation vector w, translation dt, relative scale ds) a point p moves to p + w x (p - c) + dt + ds (p - c), so
// its residual r = n.(p - q) changes by [(p - c) x n].w + n.dt + [n.(p - c)] ds. The rotation and scale columns
// are divided by the RMS radius L of the cloud: the unknowns become w*L, dt, ds*L, all in metres of displacement
// of a typical point, which makes the eigenvalues (and the condition number) independent of the cloud size.
struct NormalSystem {
  Eigen::MatrixXd a;  // J^T J / N
  Eigen::VectorXd b;  // -J^T r / N
  Vec3 centroid = Vec3::Zero();
  double radius = 1.0;
};

NormalSystem build_system(const std::vector<Vec3>& moved, const std::vector<Vec3>& normals, const Pairs& pairs,
                          bool estimate_scale) {
  const Eigen::Index m = estimate_scale ? 7 : 6;
  NormalSystem sys;
  sys.a = Eigen::MatrixXd::Zero(m, m);
  sys.b = Eigen::VectorXd::Zero(m);

  for (std::size_t i = 0; i < moved.size(); ++i) {
    if (pairs.inliers[i]) sys.centroid += moved[i];
  }
  const auto count = static_cast<double>(pairs.inlier_count);
  sys.centroid /= count;
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < moved.size(); ++i) {
    if (pairs.inliers[i]) sum_sq += (moved[i] - sys.centroid).squaredNorm();
  }
  sys.radius = std::max(std::sqrt(sum_sq / count), 1e-9);

  Eigen::VectorXd row(m);
  for (std::size_t i = 0; i < moved.size(); ++i) {
    if (!pairs.inliers[i]) continue;
    const Vec3& n = normals[pairs.nearest[i]];
    const Vec3 d = moved[i] - sys.centroid;
    row.head<3>() = d.cross(n) / sys.radius;
    row.segment<3>(3) = n;
    if (estimate_scale) row(6) = n.dot(d) / sys.radius;
    sys.a.noalias() += row * row.transpose();
    sys.b.noalias() -= row * pairs.signed_distances[i];
  }
  sys.a /= count;
  sys.b /= count;
  return sys;
}

// Solution of a x = b restricted to the well-conditioned eigen-directions of a (truncated eigen-decomposition):
// along a direction the terrain does not constrain the data carry no information, so it is not updated.
struct Solution {
  Eigen::VectorXd x;
  std::vector<double> eigenvalues;
  double condition_number = 0.0;
  std::vector<std::string> weak_parameters;
};

Solution solve(const NormalSystem& sys, double max_condition) {
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(sys.a);
  const Eigen::VectorXd& lambda = eig.eigenvalues();  // ascending
  const Eigen::MatrixXd& v = eig.eigenvectors();
  const Eigen::Index m = lambda.size();
  const double lambda_max = lambda(m - 1);

  Solution sol;
  sol.x = Eigen::VectorXd::Zero(m);
  sol.eigenvalues.assign(lambda.data(), lambda.data() + m);
  sol.condition_number =
      lambda(0) > 0.0 ? lambda_max / lambda(0) : std::numeric_limits<double>::infinity();

  Eigen::VectorXd weak_weight = Eigen::VectorXd::Zero(m);
  for (Eigen::Index k = 0; k < m; ++k) {
    if (lambda(k) * max_condition >= lambda_max && lambda(k) > 0.0) {
      sol.x += v.col(k) * (v.col(k).dot(sys.b) / lambda(k));
    } else {
      weak_weight += v.col(k).cwiseAbs2();  // share of each parameter in the weak subspace
    }
  }
  for (Eigen::Index j = 0; j < m; ++j) {
    if (weak_weight(j) > 0.5) sol.weak_parameters.emplace_back(kParameterNames[static_cast<std::size_t>(j)]);
  }
  return sol;
}

void transform_all(const Similarity& t, const std::vector<Vec3>& in, std::vector<Vec3>& out) {
  out.resize(in.size());
  for (std::size_t i = 0; i < in.size(); ++i) out[i] = t.apply(in[i]);
}

}  // namespace

Eigen::Matrix3d rotation_from_ypr(double yaw, double pitch, double roll) {
  return (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
          Eigen::AngleAxisd(roll, Vec3::UnitX()))
      .toRotationMatrix();
}

Eigen::Vector3d ypr_from_rotation(const Eigen::Matrix3d& r) {
  const double yaw = std::atan2(r(1, 0), r(0, 0));
  const double pitch = std::asin(std::clamp(-r(2, 0), -1.0, 1.0));
  const double roll = std::atan2(r(2, 1), r(2, 2));
  return {yaw, pitch, roll};
}

std::vector<Vec3> estimate_normals(const std::vector<Vec3>& points, int neighbours) {
  const KdTree tree(points);
  const std::size_t k = std::min<std::size_t>(static_cast<std::size_t>(std::max(neighbours, 3)), points.size());
  std::vector<std::uint32_t> indices(k);
  std::vector<double> squared(k);
  std::vector<Vec3> normals(points.size(), Vec3::UnitZ());
  if (k < 3) return normals;

  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::size_t found = tree.knn(points[i], k, indices.data(), squared.data());
    Vec3 mean = Vec3::Zero();
    for (std::size_t j = 0; j < found; ++j) mean += points[indices[j]];
    mean /= static_cast<double>(found);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (std::size_t j = 0; j < found; ++j) {
      const Vec3 d = points[indices[j]] - mean;
      cov.noalias() += d * d.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    Vec3 n = eig.eigenvectors().col(0);  // direction of least spread = surface normal
    if (n.z() < 0.0) n = -n;
    normals[i] = n;
  }
  return normals;
}

IcpResult align(const std::vector<Vec3>& reference, const std::vector<Vec3>& reference_normals,
                const std::vector<Vec3>& moving, const Similarity& initial, const IcpOptions& options) {
  if (reference.size() != reference_normals.size()) throw std::invalid_argument("align: one normal per point");
  if (reference.empty() || moving.empty()) throw std::invalid_argument("align: empty point cloud");
  const std::size_t unknowns = options.estimate_scale ? 7 : 6;

  const KdTree tree(reference);
  IcpResult result;
  result.transform = initial;
  Similarity& t = result.transform;
  std::vector<Vec3> moved;

  auto pairs_for = [&](const Similarity& current) {
    transform_all(current, moving, moved);
    Pairs pairs = match(tree, reference, reference_normals, moved, options);
    if (pairs.inlier_count < unknowns) {
      throw std::runtime_error("align: too few valid pairs (" + std::to_string(pairs.inlier_count) +
                               "); check --max-dist and the initial guess");
    }
    return pairs;
  };

  for (int it = 0; it < options.max_iterations; ++it) {
    const Pairs pairs = pairs_for(t);
    const NormalSystem sys = build_system(moved, reference_normals, pairs, options.estimate_scale);
    const Solution sol = solve(sys, options.max_condition);

    const Vec3 w = sol.x.head<3>() / sys.radius;
    const Vec3 dt = sol.x.segment<3>(3);
    const double ds = options.estimate_scale ? sol.x(6) / sys.radius : 0.0;
    const double angle = w.norm();
    const Eigen::Matrix3d dr =
        angle > 0.0 ? Eigen::AngleAxisd(angle, w / angle).toRotationMatrix() : Eigen::Matrix3d::Identity();

    // Compose the increment, applied about the centroid c: p -> c + (1 + ds) dR (p - c) + dt.
    const Vec3& c = sys.centroid;
    t.scale *= 1.0 + ds;
    t.rotation = dr * t.rotation;
    t.translation = c + (1.0 + ds) * (dr * (t.translation - c)) + dt;
    result.iterations = it + 1;

    if (angle < options.tol_rotation && dt.norm() < options.tol_translation && std::abs(ds) < options.tol_scale) {
      result.converged = true;
      break;
    }
  }

  // Report the pairs and the conditioning at the final transform.
  Pairs pairs = pairs_for(t);
  const NormalSystem sys = build_system(moved, reference_normals, pairs, options.estimate_scale);
  Solution sol = solve(sys, options.max_condition);
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < moved.size(); ++i) {
    if (pairs.inliers[i]) sum_sq += pairs.signed_distances[i] * pairs.signed_distances[i];
  }
  result.rms = std::sqrt(sum_sq / static_cast<double>(pairs.inlier_count));
  result.inlier_count = pairs.inlier_count;
  result.normal_z.reserve(moving.size());
  for (const std::uint32_t j : pairs.nearest) result.normal_z.push_back(reference_normals[j].z());
  result.signed_distances = std::move(pairs.signed_distances);
  result.inliers = std::move(pairs.inliers);
  result.eigenvalues = std::move(sol.eigenvalues);
  result.condition_number = sol.condition_number;
  result.weak_parameters = std::move(sol.weak_parameters);
  return result;
}

}  // namespace terrain_icp
