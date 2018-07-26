#include "drake/automotive/maliput/utility/junction_analysis.h"

#include <cmath>
#include <fstream>

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/automotive/maliput/multilane/loader.h"

namespace drake {
namespace maliput {
namespace utility {
namespace {

GTEST_TEST(JunctionAnalysisAnalyzeConfluentSegments, Xxx) {
  std::string dut_yaml = R"R(# -*- yaml -*-
---
# distances are meters; angles are degrees.
maliput_multilane_builder:
  id: dut
  lane_width: 1
  left_shoulder: 1
  right_shoulder: 1
  elevation_bounds: [0, 5]
  scale_length: 1.0
  linear_tolerance: 0.01
  angular_tolerance: 0.5
  computation_policy: prefer-speed
  points:
    origin:
      xypoint: [0, 0, 0]
      zpoint: [0, 0, 0, 0]
  connections:
    east0:
      lanes: [2, 0, 0]
      start: ["ref", "points.origin.forward"]
      length:  100
      z_end: ["ref", [0, 0, 0, 0]]
    east1:
      lanes: [2, 0, 0]
      start: ["lane.0", "connections.east0.end.0.forward"]
      length: 100
      z_end: ["lane.0", [0, 0, 0, 0]]
##    east2:
##      lanes: [2, 0, 0]
##      start: ["lane.0", "connections.east1.end.0.forward"]
##      length: 100
##      z_end: ["lane.0", [0, 0, 0, 0]]
##
##    loop:
##      lanes: [1, 0, 0]
##      start: ["lane.0", "connections.east3.end.0.forward"]
##      arc: [150, -270]
##      z_end: ["lane.0", [0, 0, 0, 0]]
##
##    north0:
##      lanes: [2, 0, 0]
##      start: ["lane.0", "connections.loop.end.0.forward"]
##      length: 100
##      z_end: ["lane.0", [0, 0, 0, 0]]
##    north1:
##      lanes: [2, 0, 0]
##      start: ["lane.0", "connections.north0.end.0.forward"]
##      length: 100
##      z_end: ["lane.0", [0, 0, 0, 0]]
##    north2:
##      lanes: [2, 0, 0]
##      start: ["lane.0", "connections.north1.end.0.forward"]
##      length: 100
##      z_end: ["lane.0", [0, 0, 0, 0]]
##
##    turn:
##      lanes: [1, 0, 0]
##      start: ["lane.0", "connections.east0.end.0.forward"]
##      arc: [50, -90]
##      explicit_end: ["lane.0", "connections.north0.end.reverse"]

  groups: {}
)R";


  std::unique_ptr<const api::RoadGeometry> rg =
      multilane::Load(multilane::BuilderFactory(), dut_yaml);

  auto errors = rg->CheckInvariants();

  std::vector<std::unordered_set<const api::Segment*>> groups =
      AnalyzeConfluentSegments(rg.get());

  for (auto& group : groups) {
    std::cerr << "GROUP:\n";
    for (auto& segment : group) {
      std::cerr << "   " << segment->id().string() << "\n";
    }
  }
  EXPECT_TRUE(false);
}



}  // anonymous namespace
}  // namespace utility
}  // namespace maliput
}  // namespace drake
