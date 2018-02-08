#pragma once

#include <string>

#include "drake/automotive/maliput/api/rules/bits.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

class RightOfWayRule {
 public:
  using Id = TypeSpecificIdentifier<class RightOfWayRule>;

  enum class Type {
    kProceedWithCaution = 0,
    kYield,
    kStopThenGo,
    kDynamic
  };

  enum class DynamicState {
    kUncontrolled = 0,  // e.g., "dark mode"
    kGo,
    kPrepareToStop,
    kStop,
    kPrepareToGo,
    kProceedWithCaution,
    kYield,
    kStopThenGo
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RightOfWayRule)

  /// Returns the persistent identifier.
  Id id() const { return id_; }

  const LaneRoute& controlled_zone() const { return controlled_zone_; }

  Type type() const { return type_; }

 private:
  Id id_;
  LaneRoute controlled_zone_;
  Type type_;
  // TODO(maddog) add bool field for "stopping is excluded in zone"?
  // TODO(maddog) add explicit yield-to-whom semantics
};


// Abstract interface
class RightOfWayStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWayStateProvider)

  virtual ~RightOfWayStateProvider() = default;

  RightOfWayRule::DynamicState GetState(const RightOfWayRule::Id& id) const {
    return DoGetState(id); }

 protected:
  RightOfWayStateProvider() = default;

 private:
  virtual RightOfWayRule::DynamicState DoGetState(
      const RightOfWayRule::Id& id) const = 0;
};


}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
