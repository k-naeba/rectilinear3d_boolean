#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "common_geometry/bbox.hpp"

// Elementary-cell classification and greedy voxel merging used by
// ComputeBooleanOp3d(). Not part of the public API.
//
// Unlike rectilinear2d_boolean's event-driven sweep (which classifies and
// merges in one O((n+m) log(n+m)) pass), this builds the full
// nx*ny*nz elementary-cell grid explicitly and classifies each cell by
// point-containment -- simpler to get right in 3D, at the cost of scaling
// with the grid's cell count rather than just the input's event count.
namespace ns_r3b::detail {

enum class Label3d : std::uint8_t { kNone, kAOnly, kBOnly, kCommon };

constexpr Label3d Classify3d(bool a_active, bool b_active) {
  if (a_active && b_active) return Label3d::kCommon;
  if (a_active) return Label3d::kAOnly;
  if (b_active) return Label3d::kBOnly;
  return Label3d::kNone;
}

inline std::size_t CellIndex(std::size_t i, std::size_t j, std::size_t k,
                              std::size_t nx, std::size_t ny) {
  return i + j * nx + k * nx * ny;
}

// Classifies every elementary cell of the grid spanned by x_coords x
// y_coords x z_coords (nx = x_coords.size()-1 cells in x, likewise y, z),
// flattened via CellIndex. A cell's coverage by `a` (or `b`) is constant
// throughout the cell -- that's the point of coordinate compression -- so
// testing the cell's center point against each side's boxes is exact.
inline std::vector<Label3d> ClassifyCells(const std::vector<ns_cg::BBox3d>& a,
                                           const std::vector<ns_cg::BBox3d>& b,
                                           const std::vector<double>& x_coords,
                                           const std::vector<double>& y_coords,
                                           const std::vector<double>& z_coords) {
  if (x_coords.size() < 2 || y_coords.size() < 2 || z_coords.size() < 2) {
    return {};
  }
  const std::size_t nx = x_coords.size() - 1;
  const std::size_t ny = y_coords.size() - 1;
  const std::size_t nz = z_coords.size() - 1;

  std::vector<Label3d> labels(nx * ny * nz, Label3d::kNone);
  for (std::size_t k = 0; k < nz; ++k) {
    for (std::size_t j = 0; j < ny; ++j) {
      for (std::size_t i = 0; i < nx; ++i) {
        const ns_cg::Vec3d mid(0.5 * (x_coords[i] + x_coords[i + 1]),
                                0.5 * (y_coords[j] + y_coords[j + 1]),
                                0.5 * (z_coords[k] + z_coords[k + 1]));
        bool in_a = false;
        for (const auto& box : a) {
          if (box.Contains(mid)) { in_a = true; break; }
        }
        bool in_b = false;
        for (const auto& box : b) {
          if (box.Contains(mid)) { in_b = true; break; }
        }
        labels[CellIndex(i, j, k, nx, ny)] = Classify3d(in_a, in_b);
      }
    }
  }
  return labels;
}

// Greedily partitions every cell labeled `target` into axis-aligned boxes:
// from each not-yet-visited target cell (scanned in increasing k, then j,
// then i), grow as far as possible in x, then as far as possible in y
// (checking the whole x-run at each step), then as far as possible in z
// (checking the whole x*y footprint at each step); mark the resulting box
// visited and emit it. Every target cell ends up in exactly one output
// box, so this is a valid partition -- just not necessarily the minimum
// possible number of boxes (that greedy order biases toward long x-runs
// first, then y, then z).
inline std::vector<ns_cg::BBox3d> GreedyMergeCells(
    const std::vector<Label3d>& labels, Label3d target, std::size_t nx,
    std::size_t ny, std::size_t nz, const std::vector<double>& x_coords,
    const std::vector<double>& y_coords, const std::vector<double>& z_coords) {
  std::vector<ns_cg::BBox3d> result;
  if (labels.empty()) return result;

  std::vector<bool> visited(labels.size(), false);
  for (std::size_t k = 0; k < nz; ++k) {
    for (std::size_t j = 0; j < ny; ++j) {
      for (std::size_t i = 0; i < nx; ++i) {
        const std::size_t start = CellIndex(i, j, k, nx, ny);
        if (visited[start] || labels[start] != target) continue;

        std::size_t i_hi = i + 1;
        while (i_hi < nx) {
          const std::size_t idx = CellIndex(i_hi, j, k, nx, ny);
          if (visited[idx] || labels[idx] != target) break;
          ++i_hi;
        }

        std::size_t j_hi = j + 1;
        while (j_hi < ny) {
          bool ok = true;
          for (std::size_t ii = i; ii < i_hi; ++ii) {
            const std::size_t idx = CellIndex(ii, j_hi, k, nx, ny);
            if (visited[idx] || labels[idx] != target) { ok = false; break; }
          }
          if (!ok) break;
          ++j_hi;
        }

        std::size_t k_hi = k + 1;
        while (k_hi < nz) {
          bool ok = true;
          for (std::size_t jj = j; jj < j_hi && ok; ++jj) {
            for (std::size_t ii = i; ii < i_hi; ++ii) {
              const std::size_t idx = CellIndex(ii, jj, k_hi, nx, ny);
              if (visited[idx] || labels[idx] != target) { ok = false; break; }
            }
          }
          if (!ok) break;
          ++k_hi;
        }

        for (std::size_t kk = k; kk < k_hi; ++kk)
          for (std::size_t jj = j; jj < j_hi; ++jj)
            for (std::size_t ii = i; ii < i_hi; ++ii)
              visited[CellIndex(ii, jj, kk, nx, ny)] = true;

        result.push_back(ns_cg::BBox3d(
            ns_cg::Vec3d(x_coords[i], y_coords[j], z_coords[k]),
            ns_cg::Vec3d(x_coords[i_hi], y_coords[j_hi], z_coords[k_hi])));
      }
    }
  }
  return result;
}

}  // namespace ns_r3b::detail
