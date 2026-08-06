#pragma once

#include <cassert>
#include <vector>

#include "common_geometry/bbox.hpp"
#include "rectilinear3d_boolean/detail/coordinate_compression.hpp"
#include "rectilinear3d_boolean/detail/grid_classification.hpp"

// Out-of-line definition of ns_r3b::ComputeBooleanOp3d(), included from the
// bottom of boolean_op.hpp after BooleanOpResult3d/the declaration are
// already visible.
namespace ns_r3b {

inline BooleanOpResult3d ComputeBooleanOp3d(
    const std::vector<ns_cg::BBox3d>& a, const std::vector<ns_cg::BBox3d>& b,
    double epsilon) {
  assert([&] {
    for (const auto& box : a)
      if (!((box.GetMin().array() <= box.GetMax().array()).all()))
        return false;
    for (const auto& box : b)
      if (!((box.GetMin().array() <= box.GetMax().array()).all()))
        return false;
    return true;
  }());

  BooleanOpResult3d result;
  if (a.empty() && b.empty()) return result;

  const std::vector<double> x_coords =
      detail::CompressXCoordinates(a, b, epsilon);
  const std::vector<double> y_coords =
      detail::CompressYCoordinates(a, b, epsilon);
  const std::vector<double> z_coords =
      detail::CompressZCoordinates(a, b, epsilon);
  if (x_coords.size() < 2 || y_coords.size() < 2 || z_coords.size() < 2) {
    return result;  // degenerate: no volume at all
  }

  const std::size_t nx = x_coords.size() - 1;
  const std::size_t ny = y_coords.size() - 1;
  const std::size_t nz = z_coords.size() - 1;

  const std::vector<detail::Label3d> labels =
      detail::ClassifyCells(a, b, x_coords, y_coords, z_coords);

  result.common = detail::GreedyMergeCells(labels, detail::Label3d::kCommon,
                                            nx, ny, nz, x_coords, y_coords,
                                            z_coords);
  result.a_only = detail::GreedyMergeCells(labels, detail::Label3d::kAOnly,
                                            nx, ny, nz, x_coords, y_coords,
                                            z_coords);
  result.b_only = detail::GreedyMergeCells(labels, detail::Label3d::kBOnly,
                                            nx, ny, nz, x_coords, y_coords,
                                            z_coords);
  return result;
}

}  // namespace ns_r3b
