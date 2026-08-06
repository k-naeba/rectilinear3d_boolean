// The 3D analog of rectilinear2d_boolean's two_circles_boolean_steps_demo:
// represents two partially-overlapping spheres A and B as sets of
// axis-aligned boxes (a nested version of the same horizontal-strip
// decomposition -- slice into z-bands, then slice each z-band's circular
// cross-section into y-strips), computes their boolean overlap via
// ComputeBooleanOp3d, and exports the result two ways:
//
//   - .obj files (both individually and combined per step) for interactive
//     inspection in obj_mesh_viewer -- there's no good 3D analog of the 2D
//     step-by-step SVG grid diagram, so this is the primary way to actually
//     look at the shapes from any angle.
//   - single-mesh static .svg snapshots (common_geometry's fixed-view
//     Mesh3d renderer), one per region, for embedding directly in the
//     README the way levelset3d_polygon's sphere.svg/torus.svg already do.

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

#include "common_geometry/bbox.hpp"
#include "common_geometry/mesh3d.hpp"
#include "common_geometry/obj.hpp"
#include "common_geometry/svg.hpp"
#include "rectilinear3d_boolean/rectilinear3d_boolean.hpp"

using ns_cg::BBox3d;
using ns_cg::Mesh3d;
using ns_cg::SvgStyle;
using ns_cg::Vec3d;

namespace {

// Sphere -> boxes: slice into num_z_slices z-bands: within each band, the
// sphere's cross-section is a disk of some smaller radius, which is itself
// sliced into num_y_slices horizontal strips (the same 2D decomposition
// circle_rectangle_demo.cpp uses), each extruded through the z-band's
// thickness.
std::vector<BBox3d> ApproximateSphere(const Vec3d& center, double radius,
                                       int num_z_slices, int num_y_slices) {
  std::vector<BBox3d> boxes;
  const double z0 = center.z() - radius;
  const double dz = (2.0 * radius) / num_z_slices;
  for (int k = 0; k < num_z_slices; ++k) {
    const double z_lo = z0 + k * dz;
    const double z_hi = z0 + (k + 1) * dz;
    const double z_mid = 0.5 * (z_lo + z_hi);
    const double dz_from_center = z_mid - center.z();
    const double r_at_z_sq = radius * radius - dz_from_center * dz_from_center;
    if (r_at_z_sq <= 0.0) continue;
    const double r_at_z = std::sqrt(r_at_z_sq);

    const double y0 = center.y() - r_at_z;
    const double dy = (2.0 * r_at_z) / num_y_slices;
    for (int j = 0; j < num_y_slices; ++j) {
      const double y_lo = y0 + j * dy;
      const double y_hi = y0 + (j + 1) * dy;
      const double y_mid = 0.5 * (y_lo + y_hi);
      const double dy_from_center = y_mid - center.y();
      const double under_sqrt = r_at_z_sq - dy_from_center * dy_from_center;
      if (under_sqrt <= 0.0) continue;
      const double half_width = std::sqrt(under_sqrt);
      boxes.push_back(BBox3d(Vec3d(center.x() - half_width, y_lo, z_lo),
                              Vec3d(center.x() + half_width, y_hi, z_hi)));
    }
  }
  return boxes;
}

// Appends one axis-aligned box's 12 triangles (outward-facing normals,
// verified against TriangleNormal = (b-a)x(c-a)) to a shared vertex/
// triangle list.
void AppendBoxTriangles(const BBox3d& box, std::vector<Vec3d>& vertices,
                         std::vector<std::array<std::size_t, 3>>& triangles) {
  const Vec3d& mn = box.GetMin();
  const Vec3d& mx = box.GetMax();
  const std::size_t base = vertices.size();
  vertices.push_back(Vec3d(mn.x(), mn.y(), mn.z()));  // 0
  vertices.push_back(Vec3d(mx.x(), mn.y(), mn.z()));  // 1
  vertices.push_back(Vec3d(mx.x(), mx.y(), mn.z()));  // 2
  vertices.push_back(Vec3d(mn.x(), mx.y(), mn.z()));  // 3
  vertices.push_back(Vec3d(mn.x(), mn.y(), mx.z()));  // 4
  vertices.push_back(Vec3d(mx.x(), mn.y(), mx.z()));  // 5
  vertices.push_back(Vec3d(mx.x(), mx.y(), mx.z()));  // 6
  vertices.push_back(Vec3d(mn.x(), mx.y(), mx.z()));  // 7

  const auto quad = [&](std::size_t a, std::size_t b, std::size_t c,
                         std::size_t d) {
    triangles.push_back({base + a, base + b, base + c});
    triangles.push_back({base + a, base + c, base + d});
  };
  quad(0, 3, 2, 1);  // bottom, z=mn, normal -z
  quad(4, 5, 6, 7);  // top,    z=mx, normal +z
  quad(0, 1, 5, 4);  // front,  y=mn, normal -y
  quad(3, 7, 6, 2);  // back,   y=mx, normal +y
  quad(0, 4, 7, 3);  // left,   x=mn, normal -x
  quad(1, 2, 6, 5);  // right,  x=mx, normal +x
}

Mesh3d BoxesToMesh(const std::vector<BBox3d>& boxes) {
  std::vector<Vec3d> vertices;
  std::vector<std::array<std::size_t, 3>> triangles;
  vertices.reserve(boxes.size() * 8);
  triangles.reserve(boxes.size() * 12);
  for (const auto& box : boxes) AppendBoxTriangles(box, vertices, triangles);
  return Mesh3d(std::move(vertices), std::move(triangles));
}

Mesh3d Combine(std::initializer_list<const std::vector<BBox3d>*> groups) {
  std::vector<BBox3d> all;
  for (const auto* g : groups) all.insert(all.end(), g->begin(), g->end());
  return BoxesToMesh(all);
}

double TotalVolume(const std::vector<BBox3d>& boxes) {
  double total = 0.0;
  for (const auto& b : boxes) {
    const Vec3d size = b.size();
    total += size.x() * size.y() * size.z();
  }
  return total;
}

}  // namespace

int main() {
  const Vec3d center_a(0.0, 0.0, 0.0);
  const Vec3d center_b(6.0, 0.0, 0.0);
  const double radius = 5.0;
  const int num_z_slices = 6;
  const int num_y_slices = 6;

  const std::vector<BBox3d> sphere_a =
      ApproximateSphere(center_a, radius, num_z_slices, num_y_slices);
  const std::vector<BBox3d> sphere_b =
      ApproximateSphere(center_b, radius, num_z_slices, num_y_slices);

  const ns_r3b::BooleanOpResult3d result =
      ns_r3b::ComputeBooleanOp3d(sphere_a, sphere_b);

  std::cout << "sphere A: center=(" << center_a.x() << "," << center_a.y()
            << "," << center_a.z() << "), radius=" << radius << ", "
            << num_z_slices << "x" << num_y_slices << " slices ("
            << sphere_a.size() << " boxes)\n";
  std::cout << "sphere B: center=(" << center_b.x() << "," << center_b.y()
            << "," << center_b.z() << "), radius=" << radius << ", "
            << num_z_slices << "x" << num_y_slices << " slices ("
            << sphere_b.size() << " boxes)\n\n";

  std::cout << "common: " << result.common.size()
            << " box(es), volume=" << TotalVolume(result.common) << "\n";
  std::cout << "a_only: " << result.a_only.size()
            << " box(es), volume=" << TotalVolume(result.a_only) << "\n";
  std::cout << "b_only: " << result.b_only.size()
            << " box(es), volume=" << TotalVolume(result.b_only) << "\n\n";

  constexpr double kPi = 3.14159265358979323846;
  const double analytic_sphere_volume = (4.0 / 3.0) * kPi * radius * radius * radius;
  std::cout << "sphere approximation volume = " << TotalVolume(sphere_a)
            << " (analytic 4/3*pi*r^3 = " << analytic_sphere_volume << ")\n";
  std::cout << "common + a_only = "
            << TotalVolume(result.common) + TotalVolume(result.a_only)
            << " (sphere A volume = " << TotalVolume(sphere_a) << ")\n";
  std::cout << "common + b_only = "
            << TotalVolume(result.common) + TotalVolume(result.b_only)
            << " (sphere B volume = " << TotalVolume(sphere_b) << ")\n";

  // .obj export -- individual pieces plus one combined file per step, for
  // interactive inspection in obj_mesh_viewer.
  WriteObjFile("sphere_a.obj", BoxesToMesh(sphere_a));
  WriteObjFile("sphere_b.obj", BoxesToMesh(sphere_b));
  WriteObjFile("two_spheres_inputs.obj", Combine({&sphere_a, &sphere_b}));
  WriteObjFile("two_spheres_common.obj", BoxesToMesh(result.common));
  WriteObjFile("two_spheres_a_only.obj", BoxesToMesh(result.a_only));
  WriteObjFile("two_spheres_b_only.obj", BoxesToMesh(result.b_only));
  WriteObjFile("two_spheres_result.obj",
               Combine({&result.common, &result.a_only, &result.b_only}));

  // Static SVG -- one single-color mesh per file, same convention as
  // levelset3d_polygon's sphere.svg/torus.svg.
  SvgStyle style_a;
  style_a.mesh_color = "#3b82f6";  // blue
  WriteSvgFile("sphere_a.svg", BoxesToMesh(sphere_a), style_a);

  SvgStyle style_b;
  style_b.mesh_color = "#f97316";  // orange
  WriteSvgFile("sphere_b.svg", BoxesToMesh(sphere_b), style_b);

  SvgStyle style_common;
  style_common.mesh_color = "#a855f7";  // purple
  WriteSvgFile("two_spheres_common.svg", BoxesToMesh(result.common), style_common);

  WriteSvgFile("two_spheres_a_only.svg", BoxesToMesh(result.a_only), style_a);
  WriteSvgFile("two_spheres_b_only.svg", BoxesToMesh(result.b_only), style_b);

  std::cout << "\nwrote sphere_a.obj, sphere_b.obj, two_spheres_inputs.obj, "
               "two_spheres_common.obj, two_spheres_a_only.obj, "
               "two_spheres_b_only.obj, two_spheres_result.obj, "
               "sphere_a.svg, sphere_b.svg, two_spheres_common.svg, "
               "two_spheres_a_only.svg, two_spheres_b_only.svg\n";
  return 0;
}
