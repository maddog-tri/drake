#include "drake/automotive/maliput/api/rules/road_ruleset.h"

#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <gtest/gtest.h>

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace {



GTEST_TEST(SRangeTest, Construction) {
  EXPECT_NO_THROW(SRange());
  EXPECT_NO_THROW(SRange(23., 79.));
  EXPECT_NO_THROW(SRange(79., 23.));
}


GTEST_TEST(SRangeTest, Accessors) {
  const SRange dut0;
  EXPECT_EQ(dut0.s0(), 0.);
  EXPECT_EQ(dut0.s1(), 0.);

  const SRange dut1(10., 50.);
  EXPECT_EQ(dut1.s0(), 10.);
  EXPECT_EQ(dut1.s1(), 50.);

  SRange dut2;
  dut2.set_s0(26.);
  dut2.set_s1(-90.);
  EXPECT_EQ(dut2.s0(), 26.);
  EXPECT_EQ(dut2.s1(), -90.);
}


GTEST_TEST(SRangeTest, CopyingAndAssignment) {
  const SRange source(12., 24.);

  const SRange dut0(source);
  EXPECT_EQ(dut0.s0(), 12.);
  EXPECT_EQ(dut0.s1(), 24.);

  SRange dut1;
  dut1 = source;
  EXPECT_EQ(dut1.s0(), 12.);
  EXPECT_EQ(dut1.s1(), 24.);
}


GTEST_TEST(SRangeTest, Equality) {
}


GTEST_TEST(LaneFragmentTest, Construction) {
  EXPECT_NO_THROW(LaneFragment(LaneId("dut"), SRange(0., 50.)));
  EXPECT_NO_THROW(LaneFragment(LaneId("dut"), {0., 50.}));
}


GTEST_TEST(LaneFragmentTest, Accessors) {
  const LaneFragment dut(LaneId("dut"), SRange(34., 0.));
  EXPECT_EQ(dut.lane_id(), LaneId("dut"));
  EXPECT_EQ(dut.s_range(), SRange(34., 0.));
}


GTEST_TEST(LaneFragmentTest, CopyingAndAssignment) {
}



GTEST_TEST(LaneRouteTest, Construction) {
}


GTEST_TEST(LaneRouteTest, Accessors) {
}


GTEST_TEST(LaneRouteTest, CopyingAndAssignment) {
}



GTEST_TEST(SpeedLimitRuleTest, Construction) {
}


GTEST_TEST(SpeedLimitRuleTest, Accessors) {
}


GTEST_TEST(SpeedLimitRuleTest, CopyingAndAssignment) {
}


#if 0
GTEST_TEST(TypeSpecificIdentifierTest, Construction) {
  EXPECT_NO_THROW(CId("x"));
  EXPECT_THROW(CId(""), std::runtime_error);
}


GTEST_TEST(TypeSpecificIdentifierTest, IdentifiedType) {
  ::testing::StaticAssertTypeEq<CId::identified_type, C>();
}


GTEST_TEST(TypeSpecificIdentifierTest, Accessors) {
  const CId dut("x");
  EXPECT_EQ(dut.string(), "x");
}


GTEST_TEST(TypeSpecificIdentifierTest, Equality) {
  const CId dut_x1("x");
  const CId dut_x2("x");
  const CId dut_y("y");

  EXPECT_TRUE(dut_x1 == dut_x2);
  EXPECT_FALSE(dut_x1 != dut_x2);

  EXPECT_FALSE(dut_x1 == dut_y);
  EXPECT_TRUE(dut_x1 != dut_y);
}


GTEST_TEST(TypeSpecificIdentifierTest, CopyingAndAssignment) {
  const CId dut1("x");
  const CId dut2(dut1);
  CId dut3("y");
  dut3 = dut1;

  EXPECT_TRUE(dut1 == dut2);
  EXPECT_TRUE(dut1 == dut3);
}


// Test usage with ordered/unordered sets.
template <typename T>
class TypeSpecificIdentifierSetTest : public ::testing::Test {};

typedef ::testing::Types<std::set<CId>,
                         std::unordered_set<CId>> SetTypes;

TYPED_TEST_CASE(TypeSpecificIdentifierSetTest, SetTypes);

TYPED_TEST(TypeSpecificIdentifierSetTest, SetTypes) {
  TypeParam dut;

  dut.insert(CId("a"));
  dut.insert(CId("b"));
  dut.insert(CId("c"));
  // Insert a fresh, duplicate instance of "a".
  dut.insert(CId("a"));

  EXPECT_EQ(dut.size(), 3);
  EXPECT_EQ(dut.count(CId("a")), 1);
  EXPECT_EQ(dut.count(CId("b")), 1);
  EXPECT_EQ(dut.count(CId("c")), 1);
}


// Test usage with ordered/unordered maps.
template <typename T>
class TypeSpecificIdentifierMapTest : public ::testing::Test {};

typedef ::testing::Types<std::map<CId, int>,
                         std::unordered_map<CId, int>> MapTypes;

TYPED_TEST_CASE(TypeSpecificIdentifierMapTest, MapTypes);

TYPED_TEST(TypeSpecificIdentifierMapTest, MapTypes) {
  TypeParam dut;

  dut[CId("a")] = 1;
  dut[CId("b")] = 2;
  dut[CId("c")] = 3;
  // Insert a fresh, duplicate instance of "a".
  dut[CId("a")] = 5;

  EXPECT_EQ(dut.size(), 3);
  EXPECT_EQ(dut[CId("a")], 5);
  EXPECT_EQ(dut[CId("b")], 2);
  EXPECT_EQ(dut[CId("c")], 3);
}
#endif

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
