#include "drake/automotive/maliput/utility/junction_analysis.h"

#include <cmath>
#include <fstream>

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/automotive/maliput/api/basic_id_index.h"
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

struct LaneEndForm {
  std::string id_string;
  api::LaneEnd::Which end;
};

struct BranchPointForm {
  std::string id_string;
  std::vector<LaneEndForm> a_side;
  std::vector<LaneEndForm> b_side;
};


class MockRoadGeometry;
class MockJunction;
class MockSegment;


class MockLaneEndSet : public api::LaneEndSet {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockLaneEndSet)

  MockLaneEndSet() = default;
  ~MockLaneEndSet() override = default;

  /// Adds a LaneEnd.
  void add(const api::LaneEnd& end) { ends_.push_back(end); }

 private:
  int do_size() const override { return ends_.size(); }

  const api::LaneEnd& do_get(int index) const override { return ends_[index]; }

  std::vector<api::LaneEnd> ends_;
};


class MockBranchPoint : public api::BranchPoint {
 public:
  MockBranchPoint(const std::string& id_string, MockRoadGeometry* road_geometry)
      : id_(id_string),
        road_geometry_(road_geometry) {}

  /// Adds a LaneEnd to the "A side" of the BranchPoint.
  const api::LaneEnd& AddABranch(const api::LaneEnd& lane_end);

  /// Adds a LaneEnd to the "B side" of the BranchPoint.
  const api::LaneEnd& AddBBranch(const api::LaneEnd& lane_end);

  /// Sets the default branch for @p lane_end to @p default_branch.
  ///
  /// The specified LaneEnds must belong to opposite sides of this BranchPoint.
  void SetDefault(const api::LaneEnd& lane_end,
                  const api::LaneEnd& default_branch);

 private:
  const api::BranchPointId do_id() const final { return id_; }

  const api::RoadGeometry* do_road_geometry() const final;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd& end) const final;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd& end) const final;

  optional<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd& end) const final;

  const api::LaneEndSet* DoGetASide() const final { return &a_side_; }

  const api::LaneEndSet* DoGetBSide() const final { return &b_side_; }

  api::BranchPointId id_;
  MockRoadGeometry* road_geometry_{};
  MockLaneEndSet a_side_;
  MockLaneEndSet b_side_;

  std::map<api::LaneEnd, MockLaneEndSet*,
           api::LaneEnd::StrictOrder> confluent_branches_;
  std::map<api::LaneEnd, MockLaneEndSet*,
           api::LaneEnd::StrictOrder> ongoing_branches_;
  std::map<api::LaneEnd, api::LaneEnd,
           api::LaneEnd::StrictOrder> defaults_;
};


const api::LaneEndSet* MockBranchPoint::DoGetConfluentBranches(
    const api::LaneEnd& end) const {
  return confluent_branches_.at(end);
}

const api::LaneEndSet* MockBranchPoint::DoGetOngoingBranches(
    const api::LaneEnd& end) const {
  return ongoing_branches_.at(end);
}

optional<api::LaneEnd> MockBranchPoint::DoGetDefaultBranch(
    const api::LaneEnd& end) const {
  auto default_it = defaults_.find(end);
  if (default_it == defaults_.end()) { return nullopt; }
  return default_it->second;
}

const api::LaneEnd& MockBranchPoint::AddABranch(const api::LaneEnd& lane_end) {
  DRAKE_DEMAND(confluent_branches_.find(lane_end) == confluent_branches_.end());
  DRAKE_DEMAND(ongoing_branches_.find(lane_end) == ongoing_branches_.end());
  a_side_.add(lane_end);
  confluent_branches_[lane_end] = &a_side_;
  ongoing_branches_[lane_end] = &b_side_;
  return lane_end;
}

const api::LaneEnd& MockBranchPoint::AddBBranch(const api::LaneEnd& lane_end) {
  DRAKE_DEMAND(confluent_branches_.find(lane_end) == confluent_branches_.end());
  DRAKE_DEMAND(ongoing_branches_.find(lane_end) == ongoing_branches_.end());
  b_side_.add(lane_end);
  confluent_branches_[lane_end] = &b_side_;
  ongoing_branches_[lane_end] = &a_side_;
  return lane_end;
}

void MockBranchPoint::SetDefault(const api::LaneEnd& lane_end,
                             const api::LaneEnd& default_branch) {
  const auto& le_ongoing = ongoing_branches_.find(lane_end);
  const auto& db_confluent = confluent_branches_.find(default_branch);
  // Verify that lane_end belongs to this BranchPoint.
  DRAKE_DEMAND(le_ongoing != ongoing_branches_.end());
  // Verify that default_branch belongs to this BranchPoint.
  DRAKE_DEMAND(db_confluent != confluent_branches_.end());
  // Verify that default_branch is an ongoing lane for lane_end.
  DRAKE_DEMAND(db_confluent->second == le_ongoing->second);

  defaults_[lane_end] = default_branch;
}


class MockLane : public api::Lane {
 public:
  MockLane(const std::string& id_string, MockSegment* segment)
      : id_(id_string),
        segment_(segment) {}

  void SetStartBp(MockBranchPoint* bp) {
    DRAKE_THROW_UNLESS(start_bp_ == nullptr);
    start_bp_ = bp;
  }

  void SetEndBp(MockBranchPoint* bp) {
    DRAKE_THROW_UNLESS(end_bp_ == nullptr);
    end_bp_ = bp;
  }

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
      const api::LaneEnd::Which which_end) const final;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd::Which which_end) const final;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd::Which which_end) const final;

  optional<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd::Which which_end) const final;

  api::LaneId id_;
  MockSegment* segment_{};
  int index_{};
  MockBranchPoint* start_bp_{};
  MockBranchPoint* end_bp_{};
};


const api::BranchPoint* MockLane::DoGetBranchPoint(
    const api::LaneEnd::Which which_end) const {
  switch (which_end) {
    case api::LaneEnd::kStart:  { return start_bp_; }
    case api::LaneEnd::kFinish: { return end_bp_; }
  }
  DRAKE_ABORT();
}

const api::LaneEndSet* MockLane::DoGetConfluentBranches(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* MockLane::DoGetOngoingBranches(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetOngoingBranches({this, which_end});
}

optional<api::LaneEnd> MockLane::DoGetDefaultBranch(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetDefaultBranch({this, which_end});
}


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
  void AddJunctions(const std::vector<JunctionForm>& junction_forms) {
    for (auto& junction_form : junction_forms) {
      MockJunction* junction = NewJunction(junction_form.id_string);
      junction->AddSegments(junction_form.segments);
      id_index_.AddJunction(junction);
      for (int si = 0; si < junction->num_segments(); ++si) {
        const api::Segment* const segment = junction->segment(si);
        id_index_.AddSegment(segment);
        for (int li = 0; li < segment->num_lanes(); ++li) {
          const api::Lane* const lane = segment->lane(li);
          id_index_.AddLane(lane);
        }
      }
    }
  }

  void AddBranchPoints(
      const std::vector<BranchPointForm>& branch_point_forms) {
    for (const auto& branch_point_form : branch_point_forms) {
      MockBranchPoint* branch_point =
          NewBranchPoint(branch_point_form.id_string);
      id_index_.AddBranchPoint(branch_point);
      for (const auto& a_end : branch_point_form.a_side) {
        MockLane* const a_lane =
            dynamic_cast<MockLane*>(
                const_cast<api::Lane*>(
                    ById().GetLane(api::LaneId(a_end.id_string))));
        DRAKE_THROW_UNLESS(a_lane != nullptr);
        branch_point->AddABranch({a_lane, a_end.end});
        a_end.end == api::LaneEnd::kStart ?
            a_lane->SetStartBp(branch_point) : a_lane->SetEndBp(branch_point);
      }
      for (const auto& b_end : branch_point_form.b_side) {
        MockLane* const b_lane =
            dynamic_cast<MockLane*>(
                const_cast<api::Lane*>(
                    ById().GetLane(api::LaneId(b_end.id_string))));
        DRAKE_THROW_UNLESS(b_lane != nullptr);
        branch_point->AddBBranch({b_lane, b_end.end});
        b_end.end == api::LaneEnd::kStart ?
            b_lane->SetStartBp(branch_point) : b_lane->SetEndBp(branch_point);
      }
    }
  }

 private:
  MockJunction* NewJunction(const std::string& id_string) {
    junctions_.emplace_back(std::make_unique<MockJunction>(id_string, this));
    return junctions_.back().get();
  }

  MockBranchPoint* NewBranchPoint(const std::string& id_string) {
    branch_points_.emplace_back(
        std::make_unique<MockBranchPoint>(id_string, this));
    return branch_points_.back().get();
  }

  const api::RoadGeometryId do_id() const final { DRAKE_ABORT(); }

  int do_num_junctions() const final { return junctions_.size(); }

  const api::Junction* do_junction(int index) const final {
    return junctions_.at(index).get();
  }

  int do_num_branch_points() const final { return branch_points_.size(); }

  const api::BranchPoint* do_branch_point(int index) const final {
    return branch_points_.at(index).get();
  }

  const api::RoadGeometry::IdIndex& DoById() const final { return id_index_; }

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
  api::BasicIdIndex id_index_;
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

  constexpr api::LaneEnd::Which kStart = api::LaneEnd::Which::kStart;
  constexpr api::LaneEnd::Which kFinish = api::LaneEnd::Which::kFinish;
  rg.AddBranchPoints({
      {"bpA", {{"l0_0_0", kStart}, {"l0_1_0", kStart}},
              {{"l1_0_0", kStart}, {"l2_0_0", kStart}}},
      {"bpB", {{"l0_0_0", kFinish}, {"l0_1_0", kFinish}},
              {{"l1_0_0", kFinish}, {"l2_0_0", kFinish}}}, });

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
