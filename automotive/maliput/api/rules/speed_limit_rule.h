#pragma once

#include "drake/automotive/maliput/api/rules/bits.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

class SpeedLimitRule {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpeedLimitRule)

  using Id = TypeSpecificIdentifier<class SpeedLimitRule>;

  enum Type {
    kStrict = 0,
    kAdvisory
  };

  /// Returns the persistent identifier.
  Id id() const { return id_; }

  const LaneFragment& fragment() const { return fragment_; }

  Type type() const { return type_; }

  double limit() const { return limit_; }

 private:
  Id id_;
  LaneFragment fragment_;
  Type type_;
  double limit_;
  // TODO(maddog@tri.global)  add field: applicable vehicle types
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
