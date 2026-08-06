#include <iostream>
#include <vector>

#include "common_geometry/bbox.hpp"
#include "rectilinear3d_boolean/rectilinear3d_boolean.hpp"

int main() {
  using ns_cg::BBox3d;
  using ns_cg::Vec3d;

  const std::vector<BBox3d> a = {
      BBox3d(Vec3d(0.0, 0.0, 0.0), Vec3d(6.0, 4.0, 4.0))};
  const std::vector<BBox3d> b = {
      BBox3d(Vec3d(3.0, 2.0, 2.0), Vec3d(9.0, 6.0, 6.0))};

  const ns_r3b::BooleanOpResult3d result = ns_r3b::ComputeBooleanOp3d(a, b);

  auto print = [](const char* label, const std::vector<BBox3d>& boxes) {
    std::cout << label << " (" << boxes.size() << " box(es)):\n";
    for (const auto& box : boxes) {
      std::cout << "  BBox3d(" << box.GetMin().x() << ", " << box.GetMin().y()
                 << ", " << box.GetMin().z() << ", " << box.GetMax().x()
                 << ", " << box.GetMax().y() << ", " << box.GetMax().z()
                 << ")\n";
    }
  };
  print("common (A and B)", result.common);
  print("a_only (A minus B)", result.a_only);
  print("b_only (B minus A)", result.b_only);
  return 0;
}
