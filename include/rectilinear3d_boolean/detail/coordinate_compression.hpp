#pragma once

#include <algorithm>
#include <vector>

#include "common_geometry/bbox.hpp"

// Coordinate clustering shared by CompressXCoordinates/CompressYCoordinates/
// CompressZCoordinates. Not part of the public API.
namespace ns_r3b::detail {

// Clusters sorted coordinate values so that any two values within `epsilon`
// of a common cluster representative collapse to that one representative.
// Comparison is always against the representative (the first/smallest raw
// value accepted into the cluster), not the previous raw value -- this
// bounds every cluster to a span of at most `epsilon` and avoids the
// "chained" drift a naive consecutive-gap comparison would allow.
inline std::vector<double> ClusterCoordinates(std::vector<double> values,
                                               double epsilon) {
  std::sort(values.begin(), values.end());
  std::vector<double> result;
  result.reserve(values.size());
  for (double v : values) {
    if (result.empty() || v - result.back() > epsilon) result.push_back(v);
  }
  return result;
}

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
