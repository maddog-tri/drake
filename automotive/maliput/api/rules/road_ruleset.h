#pragma once

#include <vector>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

class RoadRuleset {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadRuleset)

  virtual ~RoadRuleset() = default;

  struct QueryResults {
    std::vector<RightOfWayRule> right_of_way;
    std::vector<SpeedLimitRule> speed_limit;
  };

  QueryResults FindRules(
      const std::vector<LaneFragment>& affected_fragments) const {
    return DoFindRules(affected_fragments);
  }

 protected:
  RoadRuleset() = default;

 private:
  virtual QueryResults DoFindRules(
      const std::vector<LaneFragment>& affected_fragments) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
