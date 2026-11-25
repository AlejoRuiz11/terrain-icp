// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Fabian Alejandro Ruiz Osorio (AlejoRuiz11)
#include "terrain_icp/ply.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace terrain_icp {

// Binary PLY is read and written by copying bytes: valid only on a little-endian host (x86-64, ARM64).
static_assert(std::endian::native == std::endian::little, "binary PLY I/O assumes a little-endian host");

namespace {

enum class Type { Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Float64 };

Type parse_type(const std::string& name) {
  if (name == "char" || name == "int8") return Type::Int8;
  if (name == "uchar" || name == "uint8") return Type::UInt8;
  if (name == "short" || name == "int16") return Type::Int16;
  if (name == "ushort" || name == "uint16") return Type::UInt16;
  if (name == "int" || name == "int32") return Type::Int32;
  if (name == "uint" || name == "uint32") return Type::UInt32;
  if (name == "float" || name == "float32") return Type::Float32;
  if (name == "double" || name == "float64") return Type::Float64;
  throw std::runtime_error("PLY: unknown property type '" + name + "'");
}

std::size_t type_size(Type t) {
  switch (t) {
    case Type::Int8:
    case Type::UInt8: return 1;
    case Type::Int16:
    case Type::UInt16: return 2;
    case Type::Int32:
    case Type::UInt32:
    case Type::Float32: return 4;
    case Type::Float64: return 8;
  }
  return 0;
}

template <typename T>
double load(const char* bytes) {
  T value;
  std::memcpy(&value, bytes, sizeof(T));
  return static_cast<double>(value);
}

double decode(Type t, const char* bytes) {
  switch (t) {
    case Type::Int8: return load<std::int8_t>(bytes);
    case Type::UInt8: return load<std::uint8_t>(bytes);
    case Type::Int16: return load<std::int16_t>(bytes);
    case Type::UInt16: return load<std::uint16_t>(bytes);
    case Type::Int32: return load<std::int32_t>(bytes);
    case Type::UInt32: return load<std::uint32_t>(bytes);
    case Type::Float32: return load<float>(bytes);
    case Type::Float64: return load<double>(bytes);
  }
  return 0.0;
}

struct Property {
  std::string name;
  Type type;
};

struct Header {
  bool binary = false;
  std::size_t vertex_count = 0;
  std::vector<Property> properties;  // of the vertex element
};

Header read_header(std::istream& in) {
  std::string line;
  std::getline(in, line);
  if (line.rfind("ply", 0) != 0) throw std::runtime_error("PLY: missing 'ply' magic line");

  Header header;
  bool have_format = false;
  bool in_vertex = false;
  bool seen_element = false;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::istringstream words(line);
    std::string keyword;
    words >> keyword;
    if (keyword == "format") {
      std::string format;
      words >> format;
      if (format == "ascii") {
        header.binary = false;
      } else if (format == "binary_little_endian") {
        header.binary = true;
      } else {
        throw std::runtime_error("PLY: unsupported format '" + format + "' (ascii or binary_little_endian only)");
      }
      have_format = true;
    } else if (keyword == "element") {
      std::string name;
      std::size_t count = 0;
      words >> name >> count;
      if (!seen_element) {
        if (name != "vertex") throw std::runtime_error("PLY: the first element must be 'vertex'");
        header.vertex_count = count;
        in_vertex = true;
      } else {
        in_vertex = false;  // faces and anything else after the vertices are ignored
      }
      seen_element = true;
    } else if (keyword == "property" && in_vertex) {
      std::string type;
      words >> type;
      if (type == "list") throw std::runtime_error("PLY: list properties in 'vertex' are not supported");
      Property p{.name = {}, .type = parse_type(type)};
      words >> p.name;
      header.properties.push_back(p);
    } else if (keyword == "end_header") {
      if (!have_format) throw std::runtime_error("PLY: missing 'format' line");
      if (!seen_element) throw std::runtime_error("PLY: no 'vertex' element");
      return header;
    }
    // comment, obj_info and properties of other elements are ignored.
  }
  throw std::runtime_error("PLY: missing 'end_header'");
}

std::optional<std::size_t> find(const std::vector<Property>& props, const char* name) {
  for (std::size_t i = 0; i < props.size(); ++i) {
    if (props[i].name == name) return i;
  }
  return std::nullopt;
}

const char* type_name(PlyFormat format) {
  return format == PlyFormat::Ascii ? "ascii" : "binary_little_endian";
}

}  // namespace

PointCloud read_ply(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("PLY: cannot open " + path.string());
  const Header header = read_header(in);

  const auto ix = find(header.properties, "x");
  const auto iy = find(header.properties, "y");
  const auto iz = find(header.properties, "z");
  if (!ix || !iy || !iz) throw std::runtime_error("PLY: vertex element needs x, y and z");
  const auto inx = find(header.properties, "nx");
  const auto iny = find(header.properties, "ny");
  const auto inz = find(header.properties, "nz");
  const bool has_normals = inx && iny && inz;

  PointCloud cloud;
  cloud.points.reserve(header.vertex_count);
  if (has_normals) cloud.normals.reserve(header.vertex_count);

  std::vector<double> values(header.properties.size());
  std::vector<std::size_t> offsets;
  std::size_t stride = 0;
  for (const Property& p : header.properties) {
    offsets.push_back(stride);
    stride += type_size(p.type);
  }
  std::vector<char> record(stride);

  for (std::size_t v = 0; v < header.vertex_count; ++v) {
    if (header.binary) {
      in.read(record.data(), static_cast<std::streamsize>(stride));
      for (std::size_t k = 0; k < values.size(); ++k) {
        values[k] = decode(header.properties[k].type, record.data() + offsets[k]);
      }
    } else {
      for (double& value : values) in >> value;
    }
    if (!in) throw std::runtime_error("PLY: truncated vertex data in " + path.string());
    cloud.points.emplace_back(values[*ix], values[*iy], values[*iz]);
    if (has_normals) {
      Eigen::Vector3d n(values[*inx], values[*iny], values[*inz]);
      const double norm = n.norm();
      cloud.normals.push_back(norm > 0.0 ? Eigen::Vector3d(n / norm) : Eigen::Vector3d::UnitZ());
    }
  }
  return cloud;
}

void write_ply(const std::filesystem::path& path, const PointCloud& cloud, PlyFormat format,
               std::span<const ScalarField> fields) {
  const std::size_t n = cloud.points.size();
  const bool has_normals = !cloud.normals.empty();
  if (has_normals && cloud.normals.size() != n) throw std::invalid_argument("write_ply: normals size mismatch");
  for (const ScalarField& f : fields) {
    if (f.values.size() != n) throw std::invalid_argument("write_ply: field '" + f.name + "' size mismatch");
  }

  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("PLY: cannot create " + path.string());
  out << "ply\nformat " << type_name(format) << " 1.0\nelement vertex " << n << '\n'
      << "property float x\nproperty float y\nproperty float z\n";
  if (has_normals) out << "property float nx\nproperty float ny\nproperty float nz\n";
  for (const ScalarField& f : fields) out << "property float " << f.name << '\n';
  out << "end_header\n";
  out << std::setprecision(std::numeric_limits<float>::max_digits10);  // ASCII round-trips the float exactly

  std::vector<float> row;
  for (std::size_t i = 0; i < n; ++i) {
    row.clear();
    for (int k = 0; k < 3; ++k) row.push_back(static_cast<float>(cloud.points[i][k]));
    if (has_normals) {
      for (int k = 0; k < 3; ++k) row.push_back(static_cast<float>(cloud.normals[i][k]));
    }
    for (const ScalarField& f : fields) row.push_back(static_cast<float>(f.values[i]));

    if (format == PlyFormat::BinaryLittleEndian) {
      out.write(reinterpret_cast<const char*>(row.data()), static_cast<std::streamsize>(row.size() * sizeof(float)));
    } else {
      for (std::size_t k = 0; k < row.size(); ++k) out << (k ? " " : "") << row[k];
      out << '\n';
    }
  }
  if (!out) throw std::runtime_error("PLY: write failed for " + path.string());
}

}  // namespace terrain_icp
