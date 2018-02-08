#pragma once

#include <vector>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {


/// Directional range from s0 to s1.
class SRange {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SRange)

  // Default constructor
  SRange() = default;

  SRange(double s0, double s1) : s0_(s0), s1_(s1) {}

  double s0() const { return s0_; }

  double s1() const { return s1_; }

  void set_s0(double s0) { s0_ = s0; }

  void set_s1(double s1) { s1_ = s1; }

  bool operator==(const SRange& rhs) const {
    return (s0_ == rhs.s0_) && (s1_ == rhs.s1_);
  }

  bool operator!=(const SRange& rhs) const {
    return !(*this == rhs);
  }

 private:
  double s0_{0.};
  double s1_{0.};
};


class LaneFragment {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LaneFragment)

  LaneFragment(const LaneId& lane_id, const SRange& s_range)
      : lane_id_(lane_id), s_range_(s_range) {}

  const LaneId& lane_id() const { return lane_id_; }

  optional<SRange> s_range() const { return s_range_; }

 private:
  LaneId lane_id_;
  SRange s_range_;
};


/// contiguous longitudinal path, possibly over multiple lanes
class LaneRoute {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LaneRoute)

  void CheckInvariants(const RoadGeometry& road_geometry);

 private:
  std::vector<LaneFragment> fragments_;
};


}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
