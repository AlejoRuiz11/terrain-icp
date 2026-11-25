// terrain-icp: align a terrain point cloud (e.g. mesh vertices) onto a reference (e.g. a DEM) with point-to-plane ICP.

#include "terrain_icp/align_files.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr const char* kUsage =
    "usage: terrain-icp align <reference.ply> <moving.ply> --out <dir> [options]\n"
    "\n"
    "Estimates x_ref = scale * R * x_moving + t by point-to-plane ICP and writes <dir>/results.json and\n"
    "<dir>/residuals.ply (aligned moving points with their signed distance to the reference).\n"
    "\n"
    "options:\n"
    "  --init <file.json>   initial guess (scale, rotation_deg {yaw, pitch, roll}, translation_m); a results.json works\n"
    "  --scale              also estimate a uniform scale (similarity, 7 parameters)\n"
    "  --max-dist <m>       reject pairs farther apart than this (default: no limit)\n"
    "  --trim <ratio>       drop this fraction of the worst pairs each iteration (default 0.1)\n"
    "  --max-iter <n>       iteration limit (default 100)\n"
    "  --max-cond <c>       condition-number limit; weaker directions are held fixed and reported (default 1000)\n";

double parse_double(std::string_view flag, const std::string& text) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size()) throw std::invalid_argument(std::string(flag) + ": not a number: " + text);
  return value;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args[0] != "align") {
    std::cerr << kUsage;
    return 2;
  }
  try {
    terrain_icp::AlignRequest request;
    std::vector<std::string> positional;
    for (std::size_t i = 1; i < args.size(); ++i) {
      const std::string& a = args[i];
      auto value = [&]() -> const std::string& {
        if (i + 1 >= args.size()) throw std::invalid_argument(a + " needs a value");
        return args[++i];
      };
      if (a == "--out") {
        request.out_dir = value();
      } else if (a == "--init") {
        request.init = value();
      } else if (a == "--scale") {
        request.options.estimate_scale = true;
      } else if (a == "--max-dist") {
        request.options.max_distance = parse_double(a, value());
      } else if (a == "--trim") {
        request.options.trim_ratio = parse_double(a, value());
      } else if (a == "--max-iter") {
        request.options.max_iterations = static_cast<int>(parse_double(a, value()));
      } else if (a == "--max-cond") {
        request.options.max_condition = parse_double(a, value());
      } else if (a.starts_with("--")) {
        throw std::invalid_argument("unknown option " + a);
      } else {
        positional.push_back(a);
      }
    }
    if (positional.size() != 2 || request.out_dir.empty()) {
      std::cerr << kUsage;
      return 2;
    }
    const auto& o = request.options;
    if (!(o.max_distance > 0.0) || o.trim_ratio < 0.0 || o.trim_ratio >= 1.0 || o.max_iterations < 0 ||
        !(o.max_condition > 1.0)) {
      throw std::invalid_argument("need --max-dist > 0, 0 <= --trim < 1, --max-iter >= 0, --max-cond > 1");
    }
    request.reference = positional[0];
    request.moving = positional[1];
    terrain_icp::align_files(request, std::cout);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "terrain-icp: " << e.what() << '\n';
    return 1;
  }
}
