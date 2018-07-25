#include "drake/automotive/maliput/utility/junction_analysis.h"

#include <cmath>
#include <fstream>

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace maliput {
namespace utility {
namespace {


struct LaneForm {
  std::string id_string;
};

struct SegmentForm {
  std::string id_string;
  std::vector<LaneForm> lanes;
};

struct JunctionForm {
  std::string id_string;
  std::vector<SegmentForm> segments;
};

struct BranchPointForm {
  std::string id_string;
  std::vector<std::string> a_side;
  std::vector<std::string> b_side;
};


class MockRoadGeometry;
class MockJunction;
class MockSegment;


class MockBranchPoint : public api::BranchPoint {
 public:
  MockBranchPoint(const std::string& id_string, MockRoadGeometry* road_geometry)
      : id_(id_string),
        road_geometry_(road_geometry) {}

 private:
  const api::BranchPointId do_id() const final { DRAKE_ABORT(); }

  const api::RoadGeometry* do_road_geometry() const final;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd& end) const final { DRAKE_ABORT(); }

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd& end) const final { DRAKE_ABORT(); }

  optional<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd& end) const final { DRAKE_ABORT(); }

  const api::LaneEndSet* DoGetASide() const final { DRAKE_ABORT(); }

  const api::LaneEndSet* DoGetBSide() const final { DRAKE_ABORT(); }

  api::BranchPointId id_;
  MockRoadGeometry* road_geometry_{};
};


class MockLane : public api::Lane {
 public:
  MockLane(const std::string& id_string, MockSegment* segment)
      : id_(id_string),
        segment_(segment) {}

 private:
  const api::LaneId do_id() const final { return id_; }

  const api::Segment* do_segment() const final;

  int do_index() const final { return index_; }

  const api::Lane* do_to_left() const final { DRAKE_ABORT(); }

  const api::Lane* do_to_right() const final { DRAKE_ABORT(); }

  double do_length() const final { DRAKE_ABORT(); }

  api::RBounds do_lane_bounds(double s) const final { DRAKE_ABORT(); }

  api::RBounds do_driveable_bounds(double s) const final { DRAKE_ABORT(); }

  api::HBounds do_elevation_bounds(double s, double r) const final {
    DRAKE_ABORT();
  }

  api::GeoPosition DoToGeoPosition(const api::LanePosition&) const final {
    DRAKE_ABORT();
  }

  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_pos,
      api::GeoPosition* nearest_point,
      double* distance) const final { DRAKE_ABORT(); }

  api::Rotation DoGetOrientation(const api::LanePosition&) const final {
    DRAKE_ABORT();
  }

  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition&, const api::IsoLaneVelocity&) const final {
    DRAKE_ABORT();
  }

  const api::BranchPoint* DoGetBranchPoint(
      const api::LaneEnd::Which which_end) const final { DRAKE_ABORT(); }

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd::Which which_end) const final { DRAKE_ABORT(); }

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd::Which which_end) const final { DRAKE_ABORT(); }

  optional<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd::Which which_end) const final { DRAKE_ABORT(); }

  api::LaneId id_;
  MockSegment* segment_{};
  int index_{};
};


class MockSegment : public api::Segment {
 public:
  MockSegment(const std::string& id_string, MockJunction* junction)
      : id_(id_string),
        junction_(junction) {}

  MockLane* NewLane(const std::string& id_string) {
    lanes_.emplace_back(std::make_unique<MockLane>(id_string, this));
    return lanes_.back().get();
  }

  void AddLanes(const std::vector<LaneForm>& lane_forms) {
    for (auto& lane_form : lane_forms) {
      NewLane(lane_form.id_string);
    }
  }

 private:
  const api::SegmentId do_id() const final { return id_; }

  const api::Junction* do_junction() const final;

  int do_num_lanes() const final { return lanes_.size(); }

  const api::Lane* do_lane(int index) const final {
    return lanes_.at(index).get();
  }

  api::SegmentId id_;
  MockJunction* junction_;
  std::vector<std::unique_ptr<MockLane>> lanes_;
};



class MockJunction : public api::Junction {
 public:
  MockJunction(const std::string& id_string,
               MockRoadGeometry* road_geometry)
      : id_(id_string),
        road_geometry_(road_geometry) {}

  MockSegment* NewSegment(const std::string& id_string) {
    segments_.emplace_back(std::make_unique<MockSegment>(id_string, this));
    return segments_.back().get();
  }

  void AddSegments(const std::vector<SegmentForm>& segment_forms) {
    for (auto& segment_form : segment_forms) {
      MockSegment* segment = NewSegment(segment_form.id_string);
      segment->AddLanes(segment_form.lanes);
    }
  }

 private:
  const api::JunctionId do_id() const final { return id_; }

  const api::RoadGeometry* do_road_geometry() const final;

  int do_num_segments() const final { return segments_.size(); }

  const api::Segment* do_segment(int index) const final {
    return segments_.at(index).get();
  }

  api::JunctionId id_;
  MockRoadGeometry* road_geometry_{};
  std::vector<std::unique_ptr<MockSegment>> segments_;
};


class MockRoadGeometry : public api::RoadGeometry {
 public:
  MockJunction* NewJunction(const std::string& id_string) {
    junctions_.emplace_back(std::make_unique<MockJunction>(id_string, this));
    return junctions_.back().get();
  }

  MockBranchPoint* NewBranchPoint(const std::string& id_string) {
    branch_points_.emplace_back(
        std::make_unique<MockBranchPoint>(id_string, this));
    return branch_points_.back().get();
  }

  void AddJunctions(const std::vector<JunctionForm>& junction_forms) {
    for (auto& junction_form : junction_forms) {
      MockJunction* junction = NewJunction(junction_form.id_string);
      junction->AddSegments(junction_form.segments);
    }
  }

  void AddBranchPoints(
      const std::vector<BranchPointForm>& branch_point_forms) {
    for (auto& branch_point_form : branch_point_forms) {
      /////      MockBranchPoint* branch_point =
          NewBranchPoint(branch_point_form.id_string);
      /////      branch_point->AddSegments(branch_point_form.segments);
    }
  }

 private:
  const api::RoadGeometryId do_id() const final { DRAKE_ABORT(); }

  int do_num_junctions() const final { return junctions_.size(); }

  const api::Junction* do_junction(int index) const final {
    return junctions_.at(index).get();
  }

  int do_num_branch_points() const final { return branch_points_.size(); }

  const api::BranchPoint* do_branch_point(int index) const final {
    return branch_points_.at(index).get();
  }

  const api::RoadGeometry::IdIndex& DoById() const final { DRAKE_ABORT(); }

  api::RoadPosition DoToRoadPosition(const api::GeoPosition& geo_pos,
                                     const api::RoadPosition* hint,
                                     api::GeoPosition* nearest_position,
                                     double* distance) const final {
    DRAKE_ABORT();
  }

  double do_linear_tolerance() const final { DRAKE_ABORT(); }

  double do_angular_tolerance() const final { DRAKE_ABORT(); }

 private:
  std::vector<std::unique_ptr<MockJunction>> junctions_;
  std::vector<std::unique_ptr<MockBranchPoint>> branch_points_;
};



const api::RoadGeometry* MockJunction::do_road_geometry() const {
  return road_geometry_;
}

const api::RoadGeometry* MockBranchPoint::do_road_geometry() const {
  return road_geometry_;
}

const api::Junction* MockSegment::do_junction() const {
  return junction_;
}

const api::Segment* MockLane::do_segment() const {
  return segment_;
}





GTEST_TEST(JunctionAnalysisAnalyzeConfluentSegments, Xxx) {
  MockSegment s("s", nullptr);
  s.AddLanes({{"l0_0_0"}, {"l0_0_1"}});

  MockJunction j("j", nullptr);
  j.AddSegments({{"s0_0", {{"l0_0_0"},
                           {"l0_0_1"}}},
                 {"s0_1", {{"l0_1_0"},
                           {"l0_1_1"}}}, });

  MockRoadGeometry rg;
  rg.AddJunctions({
      {"j0", {{"s0_0", {{"l0_0_0"},
                        {"l0_0_1"}}},
              {"s0_1", {{"l0_1_0"},
                        {"l0_1_1"}}}, }},
      {"j1", {{"s1_0", {{"l1_0_0"},
                        {"l1_0_1"},
                        {"l1_0_2"}}}, }},
      {"j2", {{"s2_0", {{"l2_0_0"},
                        {"l2_0_1"}}}, }},
      {"j3", {{"s3_0", {{"l3_0_0"},
                        {"l3_0_1"}}}, }}, });

  rg.AddBranchPoints({
      {"bpA", {"l0_0_0", "l0_1_0"},
              {"l1_0_0", "l2_0_0"}},
      {"bpB", {"l0_0_0", "l0_1_0"},
              {"l1_0_0", "l2_0_0"}}, });


  std::vector<std::unordered_set<const api::Segment*>> groups =
      AnalyzeConfluentSegments(&rg);

  for (auto& group : groups) {
    std::cerr << "GROUP:\n";
    for (auto& segment : group) {
      std::cerr << "   " << segment->id().string() << "\n";
    }
  }
}



}  // anonymous namespace
}  // namespace utility
}  // namespace maliput
}  // namespace drake
