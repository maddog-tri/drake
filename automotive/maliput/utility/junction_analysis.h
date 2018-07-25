#pragma once

#include <unordered_set>
#include <vector>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace maliput {
namespace utility {


std::vector<std::unordered_set<const api::Segment*>>
AnalyzeConfluentSegments(const api::RoadGeometry* road_geometry);


}  // namespace utility
}  // namespace maliput
}  // namespace drake
