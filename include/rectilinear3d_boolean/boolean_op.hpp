#pragma once

#include <vector>

#include "common_geometry/bbox.hpp"

namespace ns_r3b {

// Default absolute tolerance for ComputeBooleanOp3d()'s epsilon parameter.
// Coordinates within this distance of each other are treated as coincident.
// Pick a value appropriate to the caller's coordinate scale -- 1e-9 assumes
// roughly unit-scale (e.g. meter) coordinates.
inline constexpr double kDefaultEpsilon = 1e-9;

struct BooleanOpResult3d {
  std::vector<ns_cg::BBox3d> common;  // A ∩ B
  std::vector<ns_cg::BBox3d> a_only;  // A \ B
  std::vector<ns_cg::BBox3d> b_only;  // B \ A
};

// Computes A∩B, A\B, B\A for two sets of axis-aligned 3D boxes.
//
// Precondition: boxes within `a` are pairwise non-overlapping (they may
// touch), and likewise within `b`; overlap across A and B is expected and
// is exactly what this function resolves. Each box must independently be
// valid, i.e. GetMin() <= GetMax() componentwise (checked via assert() in
// debug builds only; the pairwise-disjoint precondition is NOT validated --
// the caller is responsible for it).
//
// `epsilon` is an absolute tolerance: any two input coordinates within
// `epsilon` of each other are treated as coincident during coordinate
// compression, which absorbs floating-point noise between faces that are
// meant to align exactly.
//
// Unlike rectilinear2d_boolean's ComputeBooleanOp (an event-driven sweep
// producing maximal merged rectangles in O((n+m) log(n+m))), this builds
// the full coordinate-compressed elementary-cell grid, classifies each
// cell by point-containment, then greedily merges same-label cells into
// boxes (grow x, then y, then z). Simpler to get right in 3D; scales with
// the grid's cell count rather than just the input's box count, and does
// not guarantee the minimum possible number of output boxes. See the
// README's "Algorithm" section for the tradeoffs this implies.
//
// Rectangles within each of the three output vectors are pairwise
// non-overlapping. Output order is an unspecified implementation detail --
// do not depend on it.
BooleanOpResult3d ComputeBooleanOp3d(const std::vector<ns_cg::BBox3d>& a,
                                      const std::vector<ns_cg::BBox3d>& b,
                                      double epsilon = kDefaultEpsilon);

}  // namespace ns_r3b

#include "rectilinear3d_boolean/detail/boolean_op_impl.hpp"
