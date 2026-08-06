#include <gtest/gtest.h>

#include <vector>

#include "common_geometry/bbox.hpp"
#include "rectilinear3d_boolean/boolean_op.hpp"
#include "test_util.hpp"

namespace ns_r3b {
namespace {

using ns_cg::BBox3d;
using ns_cg::Vec3d;

BBox3d MakeBox(double xmin, double ymin, double zmin, double xmax,
               double ymax, double zmax) {
  return BBox3d(Vec3d(xmin, ymin, zmin), Vec3d(xmax, ymax, zmax));
}

TEST(BooleanOpTest, PartialOverlap) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 6, 4, 4)};
  const std::vector<BBox3d> b = {MakeBox(3, 2, 2, 9, 6, 6)};

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b);

  // The overlap of two single boxes is itself a single box -- unambiguous
  // regardless of merge strategy.
  const std::vector<BBox3d> expected_common = {MakeBox(3, 2, 2, 6, 4, 4)};
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.common),
                               testutil::Sorted(expected_common)));

  EXPECT_TRUE(testutil::NoPairwiseVolumeOverlap(result.a_only));
  EXPECT_TRUE(testutil::NoPairwiseVolumeOverlap(result.b_only));
  EXPECT_DOUBLE_EQ(
      testutil::TotalVolume(result.common) + testutil::TotalVolume(result.a_only),
      testutil::TotalVolume(a));
  EXPECT_DOUBLE_EQ(
      testutil::TotalVolume(result.common) + testutil::TotalVolume(result.b_only),
      testutil::TotalVolume(b));
}

TEST(BooleanOpTest, DisjointBoxes) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 2, 2, 2)};
  const std::vector<BBox3d> b = {MakeBox(5, 5, 5, 7, 7, 7)};

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b);

  EXPECT_TRUE(result.common.empty());
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.a_only),
                               testutil::Sorted(a)));
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.b_only),
                               testutil::Sorted(b)));
}

TEST(BooleanOpTest, ContainmentProducesShell) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 10, 10, 10)};
  const std::vector<BBox3d> b = {MakeBox(3, 3, 3, 7, 7, 7)};

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b);

  const std::vector<BBox3d> expected_common = {MakeBox(3, 3, 3, 7, 7, 7)};
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.common),
                               testutil::Sorted(expected_common)));
  EXPECT_TRUE(result.b_only.empty());

  EXPECT_TRUE(testutil::NoPairwiseVolumeOverlap(result.a_only));
  EXPECT_DOUBLE_EQ(testutil::TotalVolume(result.a_only), 1000.0 - 64.0);
}

TEST(BooleanOpTest, TouchingFaceNotOverlapping) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 4, 4, 4)};
  const std::vector<BBox3d> b = {MakeBox(4, 0, 0, 8, 4, 4)};

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b);

  EXPECT_TRUE(result.common.empty());
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.a_only),
                               testutil::Sorted(a)));
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.b_only),
                               testutil::Sorted(b)));
}

TEST(BooleanOpTest, TouchingEdgeNotOverlapping) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 4, 4, 4)};
  const std::vector<BBox3d> b = {MakeBox(4, 4, 0, 8, 8, 4)};

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b);

  EXPECT_TRUE(result.common.empty());
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.a_only),
                               testutil::Sorted(a)));
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.b_only),
                               testutil::Sorted(b)));
}

TEST(BooleanOpTest, TouchingCornerNotOverlapping) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 4, 4, 4)};
  const std::vector<BBox3d> b = {MakeBox(4, 4, 4, 8, 8, 8)};

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b);

  EXPECT_TRUE(result.common.empty());
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.a_only),
                               testutil::Sorted(a)));
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.b_only),
                               testutil::Sorted(b)));
}

TEST(BooleanOpTest, EmptyInputs) {
  const std::vector<BBox3d> a;
  const std::vector<BBox3d> b;

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b);

  EXPECT_TRUE(result.common.empty());
  EXPECT_TRUE(result.a_only.empty());
  EXPECT_TRUE(result.b_only.empty());
}

TEST(BooleanOpTest, OneSideEmpty) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 5, 5, 5)};
  const std::vector<BBox3d> b;

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b);

  EXPECT_TRUE(result.common.empty());
  EXPECT_TRUE(result.b_only.empty());
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.a_only),
                               testutil::Sorted(a)));
}

TEST(BooleanOpTest, MultipleBoxesPerSide) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 4, 4, 4),
                                  MakeBox(6, 0, 0, 10, 4, 4),
                                  MakeBox(0, 6, 0, 4, 10, 4)};
  const std::vector<BBox3d> b = {MakeBox(2, 2, 0, 8, 6, 4),
                                  MakeBox(8, 8, 0, 12, 12, 4)};

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b);

  EXPECT_TRUE(testutil::NoPairwiseVolumeOverlap(result.common));
  EXPECT_TRUE(testutil::NoPairwiseVolumeOverlap(result.a_only));
  EXPECT_TRUE(testutil::NoPairwiseVolumeOverlap(result.b_only));

  EXPECT_DOUBLE_EQ(
      testutil::TotalVolume(result.common) + testutil::TotalVolume(result.a_only),
      testutil::TotalVolume(a));
  EXPECT_DOUBLE_EQ(
      testutil::TotalVolume(result.common) + testutil::TotalVolume(result.b_only),
      testutil::TotalVolume(b));
}

TEST(BooleanOpTest, FractionalCoordinates) {
  const std::vector<BBox3d> a = {MakeBox(0.0, 0.0, 0.0, 6.5, 4.25, 3.5)};
  const std::vector<BBox3d> b = {MakeBox(3.5, 2.25, 1.5, 9.0, 6.0, 5.0)};

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b);

  const std::vector<BBox3d> expected_common = {
      MakeBox(3.5, 2.25, 1.5, 6.5, 4.25, 3.5)};
  EXPECT_TRUE(testutil::Equal(testutil::Sorted(result.common),
                               testutil::Sorted(expected_common)));
  EXPECT_DOUBLE_EQ(
      testutil::TotalVolume(result.common) + testutil::TotalVolume(result.a_only),
      testutil::TotalVolume(a));
}

// A gap smaller than the default epsilon is absorbed by coordinate
// compression: the two faces snap to a single canonical value and the boxes
// are treated as exactly touching, not overlapping and not gapped.
TEST(BooleanOpTest, EpsilonAbsorbsSubToleranceGap) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 4, 4, 4)};
  const std::vector<BBox3d> b = {MakeBox(4 + 1e-12, 0, 0, 8, 4, 4)};

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b, kDefaultEpsilon);

  EXPECT_TRUE(result.common.empty());
  ASSERT_EQ(result.a_only.size(), 1u);
  ASSERT_EQ(result.b_only.size(), 1u);
  EXPECT_TRUE(testutil::Equal(result.a_only[0], MakeBox(0, 0, 0, 4, 4, 4)));
  // b_only's xmin is repaired to the shared canonical face (4.0), not the
  // slightly-off literal that was passed in.
  EXPECT_TRUE(testutil::Equal(result.b_only[0], MakeBox(4, 0, 0, 8, 4, 4)));
}

// A gap larger than epsilon is preserved: the faces are kept distinct and a
// real (if tiny) gap remains between a_only and b_only.
TEST(BooleanOpTest, GapBeyondEpsilonIsPreserved) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 4, 4, 4)};
  const std::vector<BBox3d> b = {MakeBox(4 + 1e-6, 0, 0, 8, 4, 4)};

  const BooleanOpResult3d result = ComputeBooleanOp3d(a, b, kDefaultEpsilon);

  EXPECT_TRUE(result.common.empty());
  ASSERT_EQ(result.a_only.size(), 1u);
  ASSERT_EQ(result.b_only.size(), 1u);
  EXPECT_TRUE(testutil::Equal(result.a_only[0], MakeBox(0, 0, 0, 4, 4, 4)));
  EXPECT_TRUE(
      testutil::Equal(result.b_only[0], MakeBox(4 + 1e-6, 0, 0, 8, 4, 4)));
}

// A caller-supplied epsilon wider than the default absorbs a gap the
// default would have preserved.
TEST(BooleanOpTest, CustomEpsilonWidensTolerance) {
  const std::vector<BBox3d> a = {MakeBox(0, 0, 0, 4, 4, 4)};
  const std::vector<BBox3d> b = {MakeBox(4 + 1e-4, 0, 0, 8, 4, 4)};

  const BooleanOpResult3d default_result = ComputeBooleanOp3d(a, b);
  ASSERT_EQ(default_result.b_only.size(), 1u);
  EXPECT_TRUE(testutil::Equal(default_result.b_only[0],
                               MakeBox(4 + 1e-4, 0, 0, 8, 4, 4)));

  const BooleanOpResult3d widened_result = ComputeBooleanOp3d(a, b, 1e-3);
  EXPECT_TRUE(widened_result.common.empty());
  ASSERT_EQ(widened_result.b_only.size(), 1u);
  EXPECT_TRUE(
      testutil::Equal(widened_result.b_only[0], MakeBox(4, 0, 0, 8, 4, 4)));
}

}  // namespace
}  // namespace ns_r3b
