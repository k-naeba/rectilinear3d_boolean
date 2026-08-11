#pragma once

#include <vector>

#include "common_geometry/bbox.hpp"
#include "common_geometry/math.hpp"

// Coordinate clustering shared by CompressXCoordinates/CompressYCoordinates/
// CompressZCoordinates. Not part of the public API. The clustering itself
// is ns_cg::ClusterCoordinates (common_geometry); this using-declaration
// keeps this project's existing unqualified call sites working.
namespace ns_r3b::detail {

using ns_cg::ClusterCoordinates;

// Collects every box's min/max along `axis` (0=x, 1=y, 2=z) from a and b
// and epsilon-clusters them. Consecutive pairs of the result define the
// elementary intervals along that axis.
inline std::vector<double> CompressAxisCoordinates(
    const std::vector<ns_cg::BBox3d>& a, const std::vector<ns_cg::BBox3d>& b,
    int axis, double epsilon) {
  std::vector<double> coords;
  coords.reserve(2 * (a.size() + b.size()));
  for (const auto& box : a) {
    coords.push_back(box.GetMin()(axis));
    coords.push_back(box.GetMax()(axis));
  }
  for (const auto& box : b) {
    coords.push_back(box.GetMin()(axis));
    coords.push_back(box.GetMax()(axis));
  }
  return ClusterCoordinates(std::move(coords), epsilon);
}

inline std::vector<double> CompressXCoordinates(
    const std::vector<ns_cg::BBox3d>& a, const std::vector<ns_cg::BBox3d>& b,
    double epsilon) {
  return CompressAxisCoordinates(a, b, 0, epsilon);
}

inline std::vector<double> CompressYCoordinates(
    const std::vector<ns_cg::BBox3d>& a, const std::vector<ns_cg::BBox3d>& b,
    double epsilon) {
  return CompressAxisCoordinates(a, b, 1, epsilon);
}

inline std::vector<double> CompressZCoordinates(
    const std::vector<ns_cg::BBox3d>& a, const std::vector<ns_cg::BBox3d>& b,
    double epsilon) {
  return CompressAxisCoordinates(a, b, 2, epsilon);
}

}  // namespace ns_r3b::detail
