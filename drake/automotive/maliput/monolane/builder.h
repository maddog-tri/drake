#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/monolane/junction.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {

namespace api {
class RoadGeometry;
}

namespace monolane {

/// @file
/// Builder for monolane road networks.
///
/// monolane is a simple road-network implementation:
///  - single lane per segment;
///  - constant lane_bounds and driveable_bounds, same for all lanes;
///  - only linear and constant-curvature-arc primitives in XY-plane;
///  - cubic polynomials (parameterized on XY-arc-length) for elevation
///    and superelevation;
///  - superelevation (bank of road) rotates around the reference line (r = 0)
///    of the path.


/// XY-plane-only parameters for an endpoint of a connection, specified in
/// the world frame.
///
/// The three components are:
///  - x: x position
///  - y: y position
///  - heading: heading of reference path (radians, zero == x-direction)
class EndpointXy {
 public:
  EndpointXy() {}

  EndpointXy(double x, double y, double heading)
      :x_(x), y_(y), heading_(heading) {}

  /// Returns the parameters for an endpoint with reversed direction.
  EndpointXy reverse() const {
    return EndpointXy(x_, y_,
                      std::atan2(-std::sin(heading_), -std::cos(heading_)));
  }

  double x() const { return x_; }

  double y() const { return y_; }

  double heading() const { return heading_; }

 private:
  double x_{};
  double y_{};
  double heading_{};
};


/// Out-of-plane parameters for an endpoint of a connection, specified in
/// the world frame.
///
/// The four components are:
///  - z: elevation
///  - z_dot: grade (rate of change of elevation with respect to
///           arc length of the reference path)
///  - theta: superelevation (rotation of road surface around r = 0 centerline;
///           theta > 0 --> elevation at r > 0 is above elevation at r < 0)
///  - theta_dot: rate of change of superelevation with respect to arc length
///               of the reference path
class EndpointZ {
 public:
  EndpointZ() {}

  EndpointZ(double z, double z_dot, double theta, double theta_dot)
      : z_(z), z_dot_(z_dot), theta_(theta), theta_dot_(theta_dot) {}

  /// Returns the parameters for an endpoint with reversed direction.
  EndpointZ reverse() const {
    return EndpointZ(z_, -z_dot_, -theta_, -theta_dot_);
  }

  double z() const { return z_; }

  double z_dot() const { return z_dot_; }

  double theta() const { return theta_; }

  double theta_dot() const { return theta_dot_; }

 private:
  double z_{};
  double z_dot_{};

  double theta_{};
  double theta_dot_{};
};


/// Complete set of parameters for an endpoint of a connection,
/// specified in the world frame.  It comprises two subsets of parameters:
/// those pertaining only to the xy ground-plane, and those pertaining to
/// out-of-plane aspects of an endpoint.
class Endpoint {
 public:
  Endpoint() {}

  Endpoint(const EndpointXy& xy, const EndpointZ& z) : xy_(xy), z_(z) {}

  /// Returns the parameters for an endpoint with reversed direction.
  Endpoint reverse() const {
    return Endpoint(xy_.reverse(), z_.reverse());
  }

  /// Returns the subset of parameters pertaining to the xy ground-plane.
  const EndpointXy& xy() const { return xy_; }

  /// Returns the subset of parameters pertaining to out-of-ground-plane
  /// aspects.
  const EndpointZ& z() const { return z_; }

 private:
  EndpointXy xy_;
  EndpointZ z_;
};


/// Specification for path offset along a circular arc.
///  - radius: radius of the arc, which must be non-negative
///  - d_theta:  angle of arc segment (Δθ)
///              d_theta > 0 is counterclockwise ('veer to left')
///              d_theta < 0 is clockwise ('veer to right')
struct ArcOffset {
  ArcOffset() {}

  ArcOffset(const double aradius, const double ad_theta)
      : radius(aradius), d_theta(ad_theta) {
    DRAKE_DEMAND(radius > 0.);
  }

  double radius{};
  double d_theta{};
};


/// Representation of a reference path connecting two endpoints.
///
/// Upon building the RoadGeometry, a Connection yields a Segment
/// bearing a single Lane with the specified reference path.  The
/// Segment will belong to its own Junction, unless the Connection was
/// grouped with other Connections into a Group.
///
/// Two connection geometries are supported: line and arc.  These
/// primitives determine the projection of the reference path onto the
/// (locally-flat) plane of the earth.  The out-of-plane shape of
/// the path will be determined by the EndpointZ (elevation) parameters
/// of the endpoints.
class Connection {
 public:
  /// Possible connection geometries:  line- or arc-segment.
  enum Type { kLine, kArc };

  /// Construct a line-segment connection joining @p start to @p end.
  Connection(const std::string& id,
             const Endpoint& start, const Endpoint& end)
      : type_(kLine), id_(id), start_(start), end_(end) {}

  /// Constructs an arc-segment connection joining @p start to @p end.
  ///
  /// @p cx, @p cy specify the center of the arc. @p radius is the radius,
  /// and @p d_theta is the angle of arc.
  ///
  /// @p radius must be non-negative.  @p d_theta > 0 indicates a
  /// counterclockwise arc from start to end.
  Connection(const std::string& id,
             const Endpoint& start, const Endpoint& end,
             double cx, double cy, double radius, double d_theta)
      : type_(kArc), id_(id), start_(start), end_(end),
        cx_(cx), cy_(cy), radius_(radius), d_theta_(d_theta) {}

  /// Returns the geometric type of the path.
  Type type() const { return type_; }

  /// Returns the ID string.
  const std::string& id() const { return id_; }

  /// Returns the parameters of the start point.
  const Endpoint& start() const { return start_; }

  /// Returns the parameters of the endpoint.
  const Endpoint& end() const { return end_; }

  /// Returns the x-component of the arc center (for arc connections only).
  double cx() const { return cx_; }

  /// Returns the y-component of the arc center (for arc connections only).
  double cy() const { return cy_; }

  /// Returns the radius of the arc (for arc connections only).
  double radius() const { return radius_; }

  /// Returns the angle of the arc (for arc connections only).
  double d_theta() const { return d_theta_; }

  /// @name Deleted Copy/Move Operations
  /// Connection is neither copyable nor moveable.
  ///@{
  explicit Connection(const Connection&) = delete;
  Connection& operator=(const Connection&) = delete;
  ///@}

 private:
  Type type_{};
  std::string id_;
  Endpoint start_;
  Endpoint end_;

  // Bits specific to type_ == kArc:
  double cx_{};
  double cy_{};
  double radius_{};
  double d_theta_{};
};


/// A group of Connections.
///
/// Upon building the RoadGeometry, a Group yields a Junction containing the
/// corresponding Segments specified by all the Connections in the Group.
class Group {
 public:
  /// Constructs an empty Group with the specified @p id.
  explicit Group(const std::string& id) : id_(id) {}

  /// Constructs a Group with @p id, populated by @p connections.
  Group(const std::string& id,
        const std::vector<const Connection*>& connections)
      : id_(id), connections_(connections.begin(), connections.end()) {}

  /// Adds a Connection.
  void Add(const Connection* connection) {
    auto result = connections_.insert(connection);
    DRAKE_DEMAND(result.second);
  }

  /// Returns the ID string.
  const std::string& id() const { return id_; }

  /// Returns the grouped Connections.
  const std::set<const Connection*>& connections() const {
    return connections_;
  }

  /// @name Deleted Copy/Move Operations
  /// Group is neither copyable nor moveable.
  ///@{
  explicit Group(const Group&) = delete;
  Group& operator=(const Group&) = delete;
  ///@}

 private:
  std::string id_;
  std::set<const Connection*> connections_;
};


/// Convenient builder class which makes it easy to construct a
/// monolane road network.
class Builder {
 public:
  /// Construct a Builder which can be used to specify and assemble an
  /// instance of an api::RoadGeometry.
  ///
  /// Bounds @p lane_bounds and @p driveable_bounds are applied uniformly
  /// to the single lanes of every segment. @p linear_tolerance and
  /// @p angular_tolerance specify the respective tolerances for the
  /// resulting RoadGeometry.
  Builder(const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds,
          const double linear_tolerance,
          const double angular_tolerance);

  /// Connect @p start to an end-point linearly displaced from @p start.
  /// @p length specifies the length of displacement (in the direction of the
  /// heading of @p start).  @p z_end specifies the elevation characteristics
  /// at the end-point.
  const Connection* Connect(
      const std::string& id,
      const Endpoint& start,
      const double length,
      const EndpointZ& z_end);

  /// Connect @p start to an end-point displaced from @p start via an arc.
  /// @p arc specifies the shape of the arc.  @p z_end specifies the
  /// elevation characteristics at the end-point.
  const Connection* Connect(
      const std::string& id,
      const Endpoint& start,
      const ArcOffset& arc,
      const EndpointZ& z_end);

  /// Set the default branch for one end of a connection.
  ///
  /// The default branch for the @p in_end of connection @p in will set to be
  /// @p out_end of connection @p out.  The specified connections must
  /// actually be joined at the specified ends (i.e., the Endpoint's for
  /// those ends must be coincident and (anti)parallel within the tolerances
  /// for the Builder).
  void SetDefaultBranch(
      const Connection* in, const api::LaneEnd::Which in_end,
      const Connection* out, const api::LaneEnd::Which out_end);

  /// Create a new empty connection group with ID string @p id.
  Group* MakeGroup(const std::string& id);

  /// Create a new connection group with ID @p id, populated with the
  /// given @p connections.
  Group* MakeGroup(const std::string& id,
                   const std::vector<const Connection*>& connections);

  /// Produce a RoadGeometry, with the ID @p id.
  std::unique_ptr<const api::RoadGeometry> Build(
      const api::RoadGeometryId& id) const;

  /// @name Deleted Copy/Move Operations
  /// Builder is neither copyable nor moveable.
  ///@{
  explicit Builder(const Builder&) = delete;
  Builder& operator=(const Builder&) = delete;
  ///@}

 private:
  class EndpointFuzzyOrder {
   public:
    explicit EndpointFuzzyOrder(const double linear_tolerance)
        : lin_tol_(linear_tolerance) {}

    bool operator()(const Endpoint& lhs, const Endpoint& rhs) const {
      switch (fuzzy_compare(rhs.xy().x(), lhs.xy().x())) {
        case -1: { return true; }
        case 1: { return false; }
        case 0: {
          switch (fuzzy_compare(rhs.xy().y(), lhs.xy().y())) {
            case -1: { return true; }
            case 1: { return false; }
            case 0: {
              switch (fuzzy_compare(rhs.z().z(), lhs.z().z())) {
                case -1: { return true; }
                case 1: { return false; }
                case 0: { return false; }
                default: { DRAKE_ABORT(); }
              }
            }
            default: { DRAKE_ABORT(); }
          }
        }
        default: { DRAKE_ABORT(); }
      }
    }

   private:
    int fuzzy_compare(const double a, const double b) const {
      if (a < (b - lin_tol_)) {
        return -1;
      } else if (a > (b + lin_tol_)) {
        return 1;
      } else {
        return 0;
      }
    }

    const double lin_tol_{};
  };

  struct DefaultBranch {
    DefaultBranch() {}

    DefaultBranch(
        const Connection* ain, const api::LaneEnd::Which ain_end,
        const Connection* aout, const api::LaneEnd::Which aout_end)
        : in(ain), in_end(ain_end), out(aout), out_end(aout_end) {}

    const Connection* in{};
    api::LaneEnd::Which in_end{};
    const Connection* out{};
    api::LaneEnd::Which out_end{};
  };

  Lane* BuildConnection(
      const Connection* const cnx,
      Junction* const junction,
      RoadGeometry* const rg,
      std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* const bp_map) const;

  BranchPoint* FindOrCreateBranchPoint(
      const Endpoint& point,
      RoadGeometry* rg,
      std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* const bp_map) const;

  void AttachBranchPoint(
      const Endpoint& point, Lane* const lane, const api::LaneEnd::Which end,
      RoadGeometry* rg,
      std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* bp_map) const;

  api::RBounds lane_bounds_;
  api::RBounds driveable_bounds_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  std::vector<std::unique_ptr<Connection>> connections_;
  std::vector<DefaultBranch> default_branches_;
  std::vector<std::unique_ptr<Group>> groups_;
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
