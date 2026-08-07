// The 3D analog of rectilinear2d_boolean's two_circles_boolean_steps_demo:
// represents two partially-overlapping spheres A and B as sets of
// axis-aligned boxes (a nested version of the same horizontal-strip
// decomposition -- slice into z-bands, then slice each z-band's circular
// cross-section into y-strips), computes their boolean overlap via
// ComputeBooleanOp3d, and exports the result two ways:
//
//   - .obj files (both individually and combined per step) for interactive
//     inspection in obj_mesh_viewer -- the primary way to actually look at
//     the shapes from any angle.
//   - single-mesh static .svg snapshots (common_geometry's fixed-view
//     Mesh3d renderer), one per region, for embedding directly in the
//     README the way levelset3d_polygon's sphere.svg/torus.svg already do.
//   - a two-color combined .svg with the actual coordinate-compressed grid
//     (ns_r3b::detail::CompressXCoordinates/Y/Z -- not a reimplementation)
//     drawn as a wireframe cage on top, sharing the same camera projection
//     as the solid meshes above (RotateYawPitch/painter's-algorithm/
//     Lambertian-shading logic mirrored from common_geometry/src/svg.cpp,
//     self-contained here rather than added to common_geometry, matching
//     rectilinear2d_boolean's two_circles demo's own precedent).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "common_geometry/bbox.hpp"
#include "common_geometry/mesh3d.hpp"
#include "common_geometry/obj.hpp"
#include "common_geometry/svg.hpp"
#include "rectilinear3d_boolean/detail/coordinate_compression.hpp"
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

// A single mesh built from multiple box groups, each keeping its own
// per-triangle color (12 triangles per box) -- BoxesToMesh/Combine only
// produce one color for the whole mesh, which isn't enough to tell sphere
// A's boxes apart from sphere B's in a combined render.
struct ColoredMesh {
  Mesh3d mesh;
  std::vector<std::string> triangle_colors;
};

ColoredMesh CombineColored(
    std::initializer_list<std::pair<const std::vector<BBox3d>*, std::string>>
        groups) {
  std::vector<Vec3d> vertices;
  std::vector<std::array<std::size_t, 3>> triangles;
  std::vector<std::string> colors;
  for (const auto& group : groups) {
    for (const auto& box : *group.first) {
      const std::size_t triangles_before = triangles.size();
      AppendBoxTriangles(box, vertices, triangles);
      colors.insert(colors.end(), triangles.size() - triangles_before,
                     group.second);
    }
  }
  return ColoredMesh{Mesh3d(std::move(vertices), std::move(triangles)),
                      std::move(colors)};
}

// -- Grid-overlay SVG rendering below, mirroring common_geometry's own
// Mesh3d -> SVG pipeline (src/svg.cpp) so the wireframe grid lines up
// exactly with the solid mesh's projection. Kept self-contained in this
// example rather than folded into common_geometry, since it's a one-off
// combination (per-triangle color + wireframe overlay) not needed by the
// library's general SvgStyle-based API.

Vec3d RotateYawPitch(const Vec3d& p, double yaw_degrees,
                      double pitch_degrees) {
  const double yaw = yaw_degrees * M_PI / 180.0;
  const double pitch = pitch_degrees * M_PI / 180.0;
  const Vec3d q(p.x() * std::cos(yaw) + p.z() * std::sin(yaw), p.y(),
                -p.x() * std::sin(yaw) + p.z() * std::cos(yaw));
  return Vec3d(q.x(), q.y() * std::cos(pitch) - q.z() * std::sin(pitch),
               q.y() * std::sin(pitch) + q.z() * std::cos(pitch));
}

std::string ShadeHexColor(const std::string& hex, double intensity) {
  intensity = std::clamp(intensity, 0.2, 1.0);
  const int r = std::stoi(hex.substr(1, 2), nullptr, 16);
  const int g = std::stoi(hex.substr(3, 2), nullptr, 16);
  const int b = std::stoi(hex.substr(5, 2), nullptr, 16);
  std::ostringstream os;
  os << "#" << std::hex << std::setfill('0') << std::setw(2)
     << static_cast<int>(r * intensity) << std::setw(2)
     << static_cast<int>(g * intensity) << std::setw(2)
     << static_cast<int>(b * intensity);
  return os.str();
}

struct Pt2 {
  double x, y;
};

struct Segment3 {
  Vec3d a, b;
};

// The compressed grid's wireframe cage: x/y lines on the floor (z=z_min),
// x/z lines on the back wall (y=y_max), and y/z lines on the side wall
// (x=x_min) -- three visible faces of the grid's own bounding box, drawn
// with the real compressed coordinate values.
std::vector<Segment3> BuildGridWalls(const std::vector<double>& xs,
                                      const std::vector<double>& ys,
                                      const std::vector<double>& zs) {
  std::vector<Segment3> segs;
  const double x_min = xs.front(), x_max = xs.back();
  const double y_min = ys.front(), y_max = ys.back();
  const double z_min = zs.front(), z_max = zs.back();

  for (double x : xs)
    segs.push_back({Vec3d(x, y_min, z_min), Vec3d(x, y_max, z_min)});
  for (double y : ys)
    segs.push_back({Vec3d(x_min, y, z_min), Vec3d(x_max, y, z_min)});

  for (double x : xs)
    segs.push_back({Vec3d(x, y_max, z_min), Vec3d(x, y_max, z_max)});
  for (double z : zs)
    segs.push_back({Vec3d(x_min, y_max, z), Vec3d(x_max, y_max, z)});

  for (double y : ys)
    segs.push_back({Vec3d(x_min, y, z_min), Vec3d(x_min, y, z_max)});
  for (double z : zs)
    segs.push_back({Vec3d(x_min, y_min, z), Vec3d(x_min, y_max, z)});

  return segs;
}

// Renders `colored_mesh` (painter's algorithm + Lambertian shading,
// exactly as common_geometry's ToSvg(Mesh3d) does it) with the
// coordinate-compressed grid's wireframe cage drawn on top, sharing the
// same yaw/pitch rotation so the cage lines up with the geometry it was
// computed from. The wireframe is drawn unconditionally on top (ignoring
// the mesh's own depth order) -- an intentional simplification for
// legibility, the same choice rectilinear2d_boolean's step diagrams make.
void WriteGridOverlaySvg(const std::string& path,
                          const ColoredMesh& colored_mesh,
                          const std::vector<double>& xs,
                          const std::vector<double>& ys,
                          const std::vector<double>& zs,
                          double target_width_px) {
  constexpr double kYawDegrees = -35.0;
  constexpr double kPitchDegrees = 20.0;
  constexpr double kPadding = 10.0;

  const Mesh3d& mesh = colored_mesh.mesh;
  std::vector<Vec3d> rotated_vertices;
  rotated_vertices.reserve(mesh.GetVertices().size());
  for (const auto& v : mesh.GetVertices())
    rotated_vertices.push_back(RotateYawPitch(v, kYawDegrees, kPitchDegrees));

  const std::vector<Segment3> grid_segments = BuildGridWalls(xs, ys, zs);
  std::vector<std::array<Vec3d, 2>> rotated_grid;
  rotated_grid.reserve(grid_segments.size());
  for (const auto& seg : grid_segments)
    rotated_grid.push_back({RotateYawPitch(seg.a, kYawDegrees, kPitchDegrees),
                             RotateYawPitch(seg.b, kYawDegrees, kPitchDegrees)});

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  const auto expand = [&](const Vec3d& p) {
    min_x = std::min(min_x, p.x());
    max_x = std::max(max_x, p.x());
    min_y = std::min(min_y, p.y());
    max_y = std::max(max_y, p.y());
  };
  for (const auto& p : rotated_vertices) expand(p);
  for (const auto& seg : rotated_grid) {
    expand(seg[0]);
    expand(seg[1]);
  }

  const double view_w = (max_x - min_x) + 2.0 * kPadding;
  const double view_h = (max_y - min_y) + 2.0 * kPadding;
  const auto map = [&](const Vec3d& p) {
    return Pt2{p.x() - min_x + kPadding, (max_y - p.y()) + kPadding};
  };

  std::vector<std::size_t> order(mesh.GetTriangles().size());
  std::iota(order.begin(), order.end(), 0);
  std::vector<double> avg_z(mesh.GetTriangles().size());
  for (std::size_t t = 0; t < mesh.GetTriangles().size(); ++t) {
    const auto& tri = mesh.GetTriangles()[t];
    avg_z[t] = (rotated_vertices[tri[0]].z() + rotated_vertices[tri[1]].z() +
                rotated_vertices[tri[2]].z()) /
               3.0;
  }
  std::sort(order.begin(), order.end(),
            [&](std::size_t a, std::size_t b) { return avg_z[a] < avg_z[b]; });

  const Vec3d light_dir = Vec3d(-0.4, 0.6, 1.0).normalized();
  const double seam_stroke_width = view_w * 0.0015;

  std::ostringstream os;
  const double aspect = view_w > 0.0 ? view_h / view_w : 1.0;
  os << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << target_width_px
     << "\" height=\"" << target_width_px * aspect << "\" viewBox=\"0 0 "
     << view_w << " " << view_h << "\">\n";

  for (std::size_t t : order) {
    const auto& tri = mesh.GetTriangles()[t];
    const Vec3d& a = rotated_vertices[tri[0]];
    const Vec3d& b = rotated_vertices[tri[1]];
    const Vec3d& c = rotated_vertices[tri[2]];
    const Vec3d normal = (b - a).cross(c - a).normalized();
    const double intensity = 0.25 + 0.75 * std::max(normal.dot(light_dir), 0.0);
    const std::string color =
        ShadeHexColor(colored_mesh.triangle_colors[t], intensity);
    const Pt2 p0 = map(a), p1 = map(b), p2 = map(c);
    os << "  <polygon points=\"" << p0.x << "," << p0.y << " " << p1.x << ","
       << p1.y << " " << p2.x << "," << p2.y << "\" fill=\"" << color
       << "\" stroke=\"" << color << "\" stroke-width=\"" << seam_stroke_width
       << "\"/>\n";
  }

  const double grid_stroke_width = view_w * 0.0025;
  for (const auto& seg : rotated_grid) {
    const Pt2 p0 = map(seg[0]);
    const Pt2 p1 = map(seg[1]);
    os << "  <line x1=\"" << p0.x << "\" y1=\"" << p0.y << "\" x2=\"" << p1.x
       << "\" y2=\"" << p1.y << "\" stroke=\"#374151\" stroke-width=\""
       << grid_stroke_width << "\" stroke-opacity=\"0.55\"/>\n";
  }

  os << "</svg>\n";

  std::ofstream out(path);
  if (!out) throw std::runtime_error("Failed to open file for writing: " + path);
  out << os.str();
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

  // The real algorithm's own coordinate-compressed grid -- not a
  // reimplementation of it.
  const std::vector<double> grid_x = ns_r3b::detail::CompressXCoordinates(
      sphere_a, sphere_b, ns_r3b::kDefaultEpsilon);
  const std::vector<double> grid_y = ns_r3b::detail::CompressYCoordinates(
      sphere_a, sphere_b, ns_r3b::kDefaultEpsilon);
  const std::vector<double> grid_z = ns_r3b::detail::CompressZCoordinates(
      sphere_a, sphere_b, ns_r3b::kDefaultEpsilon);

  const auto print_axis = [](const char* label,
                              const std::vector<double>& coords) {
    std::cout << "  " << label << " (" << coords.size() << "): ";
    for (std::size_t i = 0; i < coords.size(); ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << std::fixed << std::setprecision(2) << coords[i];
    }
    std::cout << "\n";
  };
  std::cout << "compressed grid: " << grid_x.size() << " x-planes x "
            << grid_y.size() << " y-planes x " << grid_z.size()
            << " z-planes = "
            << (grid_x.size() - 1) * (grid_y.size() - 1) * (grid_z.size() - 1)
            << " elementary cells\n";
  print_axis("x", grid_x);
  print_axis("y", grid_y);
  print_axis("z", grid_z);
  std::cout << "\n";

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

  // Combined two-color mesh with the compressed grid's wireframe cage
  // drawn on top, sharing the same camera projection as the meshes above.
  const ColoredMesh inputs_colored = CombineColored(
      {{&sphere_a, style_a.mesh_color}, {&sphere_b, style_b.mesh_color}});
  WriteGridOverlaySvg("two_spheres_grid.svg", inputs_colored, grid_x, grid_y,
                       grid_z, /*target_width_px=*/800.0);

  std::cout << "\nwrote sphere_a.obj, sphere_b.obj, two_spheres_inputs.obj, "
               "two_spheres_common.obj, two_spheres_a_only.obj, "
               "two_spheres_b_only.obj, two_spheres_result.obj, "
               "sphere_a.svg, sphere_b.svg, two_spheres_common.svg, "
               "two_spheres_a_only.svg, two_spheres_b_only.svg, "
               "two_spheres_grid.svg\n";
  return 0;
}
