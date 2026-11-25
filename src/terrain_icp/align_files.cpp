#include "terrain_icp/align_files.hpp"

#include "terrain_icp/ply.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numbers>
#include <ostream>
#include <stdexcept>

namespace terrain_icp {

namespace {

using Json = nlohmann::ordered_json;

constexpr double kDeg = 180.0 / std::numbers::pi;

bool contains(const std::vector<std::string>& v, const char* s) { return std::find(v.begin(), v.end(), s) != v.end(); }

std::string join(const std::vector<std::string>& v) {
  std::string out;
  for (const std::string& s : v) out += (out.empty() ? "" : ", ") + s;
  return out;
}

Json vec_json(const Eigen::Vector3d& v) { return Json::array({v.x(), v.y(), v.z()}); }

// Percentile (0..1) of a copy of `values`, nearest-rank.
double percentile(std::vector<double> values, double q) {
  if (values.empty()) return 0.0;
  const auto k = static_cast<std::size_t>(std::ceil(q * static_cast<double>(values.size()))) - 1;
  const std::size_t index = std::min(k, values.size() - 1);
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
  return values[index];
}

Json results_json(const AlignRequest& request, std::size_t reference_points, bool reference_normals_from_file,
                  const std::vector<Eigen::Vector3d>& moving, const IcpResult& r,
                  const std::vector<std::string>& warnings) {
  const IcpOptions& o = request.options;
  const Similarity& t = r.transform;
  const Eigen::Vector3d ypr = ypr_from_rotation(t.rotation) * kDeg;

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& p : moving) centroid += p;
  centroid /= static_cast<double>(moving.size());

  std::vector<double> inlier_residuals;
  std::vector<double> inlier_abs;
  for (std::size_t i = 0; i < r.signed_distances.size(); ++i) {
    if (!r.inliers[i]) continue;
    inlier_residuals.push_back(r.signed_distances[i]);
    inlier_abs.push_back(std::abs(r.signed_distances[i]));
  }
  double mean = 0.0;
  for (double d : inlier_residuals) mean += d;
  mean /= static_cast<double>(std::max<std::size_t>(inlier_residuals.size(), 1));

  Json rotation_matrix = Json::array();
  for (int i = 0; i < 3; ++i) {
    rotation_matrix.push_back(Json::array({t.rotation(i, 0), t.rotation(i, 1), t.rotation(i, 2)}));
  }

  Json j;
  j["convention"] =
      "x_ref = scale * R * x_moving + translation; R = Rz(yaw) * Ry(pitch) * Rx(roll); x East, y North, z Up; "
      "yaw counter-clockwise seen from above; residual > 0 when the moving point is above the reference";
  // File names only: full paths of the user's data never end up in the output.
  j["inputs"] = {{"reference", request.reference.filename().string()},
                 {"moving", request.moving.filename().string()},
                 {"reference_points", reference_points},
                 {"moving_points", moving.size()},
                 {"reference_normals", reference_normals_from_file ? "file" : "pca"}};
  j["options"] = {{"estimate_scale", o.estimate_scale},
                  {"max_distance_m", o.max_distance},  // null = no limit
                  {"trim_ratio", o.trim_ratio},
                  {"max_iterations", o.max_iterations},
                  {"max_condition", o.max_condition},
                  {"normal_neighbours", o.normal_neighbours}};
  j["transform"] = {{"scale", t.scale},
                    {"rotation_deg", {{"yaw", ypr.x()}, {"pitch", ypr.y()}, {"roll", ypr.z()}}},
                    {"rotation_matrix", rotation_matrix},
                    {"translation_m", vec_json(t.translation)},
                    {"moving_centroid_shift_m", vec_json(t.apply(centroid) - centroid)}};
  j["fit"] = {{"converged", r.converged},
              {"iterations", r.iterations},
              {"inliers", r.inlier_count},
              {"inlier_ratio", static_cast<double>(r.inlier_count) / static_cast<double>(moving.size())},
              {"rms_m", r.rms},
              {"mean_m", mean},
              {"abs_p50_m", percentile(inlier_abs, 0.50)},
              {"abs_p95_m", percentile(inlier_abs, 0.95)}};
  j["conditioning"] = {{"condition_number", r.condition_number},  // null = singular
                       {"eigenvalues", r.eigenvalues},
                       {"weak_parameters", r.weak_parameters}};
  j["warnings"] = warnings;
  return j;
}

}  // namespace

Similarity read_initial_guess(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open initial guess " + path.string());
  const Json root = Json::parse(in);
  const Json& j = root.contains("transform") ? root.at("transform") : root;

  Similarity t;
  t.scale = j.value("scale", 1.0);
  if (j.contains("rotation_deg")) {
    const Json& r = j.at("rotation_deg");
    t.rotation = rotation_from_ypr(r.value("yaw", 0.0) / kDeg, r.value("pitch", 0.0) / kDeg, r.value("roll", 0.0) / kDeg);
  }
  if (j.contains("translation_m")) {
    const auto v = j.at("translation_m").get<std::vector<double>>();
    if (v.size() != 3) throw std::runtime_error("initial guess: translation_m needs 3 values");
    t.translation = {v[0], v[1], v[2]};
  }
  if (!(t.scale > 0.0)) throw std::runtime_error("initial guess: scale must be positive");
  return t;
}

std::vector<std::string> make_warnings(const IcpResult& r, const IcpOptions& o) {
  std::vector<std::string> warnings;
  const auto& weak = r.weak_parameters;
  if (!weak.empty()) {
    const bool flat = contains(weak, "rotation_z") && contains(weak, "translation_x") && contains(weak, "translation_y");
    std::string text = flat ? "flat terrain: yaw and horizontal translation are poorly constrained. " : "";
    text += "Ill-conditioned normal matrix (condition number " +
            (std::isfinite(r.condition_number) ? std::to_string(r.condition_number) : std::string("inf")) + " > " +
            std::to_string(o.max_condition) + "): the terrain does not constrain " + join(weak) +
            "; those parameters were held at their initial value.";
    warnings.push_back(text);
  }
  if (!r.converged) {
    warnings.push_back("did not converge in " + std::to_string(r.iterations) + " iterations");
  }
  const double ratio = static_cast<double>(r.inlier_count) / static_cast<double>(r.inliers.size());
  if (ratio < 0.5) {
    warnings.push_back("only " + std::to_string(static_cast<int>(100.0 * ratio)) +
                       " % of the moving points were used: check the overlap, --max-dist and the initial guess");
  }
  return warnings;
}

IcpResult align_files(const AlignRequest& request, std::ostream& log) {
  const PointCloud reference = read_ply(request.reference);
  const PointCloud moving = read_ply(request.moving);
  const bool normals_from_file = !reference.normals.empty();
  const std::vector<Eigen::Vector3d> normals = [&] {
    if (!normals_from_file) return estimate_normals(reference.points, request.options.normal_neighbours);
    std::vector<Eigen::Vector3d> n = reference.normals;
    for (auto& v : n) {
      if (v.z() < 0.0) v = -v;  // same orientation rule as PCA normals: up
    }
    return n;
  }();
  const Similarity initial = request.init ? read_initial_guess(*request.init) : Similarity{};

  const IcpResult result = align(reference.points, normals, moving.points, initial, request.options);
  const std::vector<std::string> warnings = make_warnings(result, request.options);

  std::filesystem::create_directories(request.out_dir);
  {
    std::ofstream out(request.out_dir / "results.json");
    out << results_json(request, reference.points.size(), normals_from_file, moving.points, result, warnings).dump(2)
        << '\n';
    if (!out) throw std::runtime_error("cannot write results.json");
  }

  PointCloud aligned;
  aligned.points.reserve(moving.points.size());
  for (const auto& p : moving.points) aligned.points.push_back(result.transform.apply(p));
  std::vector<double> inlier(result.inliers.begin(), result.inliers.end());
  const ScalarField fields[] = {
      {"distance", result.signed_distances}, {"inlier", inlier}, {"normal_z", result.normal_z}};
  write_ply(request.out_dir / "residuals.ply", aligned, PlyFormat::BinaryLittleEndian, fields);

  const Eigen::Vector3d ypr = ypr_from_rotation(result.transform.rotation) * kDeg;
  const Eigen::Vector3d& tr = result.transform.translation;
  log << "reference: " << reference.points.size() << " points, moving: " << moving.points.size() << " points\n"
      << "scale " << result.transform.scale << ", yaw/pitch/roll [deg] " << ypr.x() << ' ' << ypr.y() << ' '
      << ypr.z() << ", translation [m] " << tr.x() << ' ' << tr.y() << ' ' << tr.z() << '\n'
      << "rms " << result.rms << " m, inliers " << result.inlier_count << ", iterations " << result.iterations
      << (result.converged ? " (converged)" : " (NOT converged)") << ", condition number "
      << result.condition_number << '\n';
  for (const std::string& w : warnings) log << "WARNING: " << w << '\n';
  return result;
}

}  // namespace terrain_icp
