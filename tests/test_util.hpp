#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

#include "common_geometry/bbox.hpp"

// Test-only helpers, not part of the library's public API.
namespace ns_r3b::testutil {

inline std::tuple<double, double, double, double, double, double> AsTuple(
    const ns_cg::BBox3d& r) {
  return {r.GetMin().x(), r.GetMin().y(), r.GetMin().z(),
          r.GetMax().x(), r.GetMax().y(), r.GetMax().z()};
}

// Deterministic ordering for comparing result vectors regardless of the
// algorithm's unspecified emission order.
inline std::vector<ns_cg::BBox3d> Sorted(std::vector<ns_cg::BBox3d> boxes) {
  std::sort(boxes.begin(), boxes.end(),
            [](const ns_cg::BBox3d& a, const ns_cg::BBox3d& b) {
              return AsTuple(a) < AsTuple(b);
            });
  return boxes;
}

// BBox3d has no operator==; compares the six coordinates exactly.
inline bool Equal(const ns_cg::BBox3d& a, const ns_cg::BBox3d& b) {
  return AsTuple(a) == AsTuple(b);
}

inline bool Equal(const std::vector<ns_cg::BBox3d>& a,
                   const std::vector<ns_cg::BBox3d>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (!Equal(a[i], b[i])) return false;
  return true;
}

inline double Volume(const ns_cg::BBox3d& r) {
  const ns_cg::Vec3d size = r.size();
  return size.x() * size.y() * size.z();
}

inline double TotalVolume(const std::vector<ns_cg::BBox3d>& boxes) {
  double total = 0.0;
  for (const auto& r : boxes) total += Volume(r);
  return total;
}

// True if the two boxes' interiors share positive volume; touching faces,
// edges, or corners do not count as overlap.
inline bool VolumesOverlap(const ns_cg::BBox3d& a, const ns_cg::BBox3d& b) {
  const double x_lo = std::max(a.GetMin().x(), b.GetMin().x());
  const double x_hi = std::min(a.GetMax().x(), b.GetMax().x());
  const double y_lo = std::max(a.GetMin().y(), b.GetMin().y());
  const double y_hi = std::min(a.GetMax().y(), b.GetMax().y());
  const double z_lo = std::max(a.GetMin().z(), b.GetMin().z());
  const double z_hi = std::min(a.GetMax().z(), b.GetMax().z());
  return x_lo < x_hi && y_lo < y_hi && z_lo < z_hi;
}

// O(n^2), fine for the small fixtures used in these tests.
inline bool NoPairwiseVolumeOverlap(const std::vector<ns_cg::BBox3d>& boxes) {
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    for (std::size_t j = i + 1; j < boxes.size(); ++j) {
      if (VolumesOverlap(boxes[i], boxes[j])) return false;
    }
  }
  return true;
}

}  // namespace ns_r3b::testutil
