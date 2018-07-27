#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace automotive {


/// A non-physical car that operates in LANE-space on an InfiniteCircuitRoad,
/// i.e., a maliput road network that only has a single Lane and infinite
/// longitudinal extent.  The car is constrained to operating within the
/// driveable-bounds of the road network:  a "Magic Guard Rail" feature will
/// clamp lateral (`r`) derivatives to zero if they would push the car's
/// position past the driveable-bounds.
///
/// (Elevation-above-road 'h' is implicitly zero, too.)
///
/// configuration:  EndlessRoadCarConfig
///
/// state vector:  EndlessRoadCarState
/// * planar LANE-space position:  (s, r)
/// * planar heading and speed (isometric LANE-space forward velocity)
///
/// input vector:  Depends on @p control_type specified at construction;
/// see EndlessRoadCar() constructor for details.
///
/// output vector: EndlessRoadCarState (same as state vector)
///
/// @tparam T must support certain arithmetic operations.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
/// @ingroup automotive_systems


/// Dynamical representation of the idealized hybrid dynamics
/// of a ball dropping from a height and bouncing on a surface.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
//XXX /// - drake::AutoDiffXd
//XXX /// - symbolic::Expression
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// They are already available to link against in the containing library.
///
/// Inputs:
///   * kappa (double, 1/m, curvature)
///   * a (double, m/s^2, longitudinal acceleration)
///   * next_lane_id (maliput::api::LaneId)
///
/// States: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
///   * lane_id  (maliput::api::LaneId)
///   * s  (double, m)
///   * r  (double, m)
///   * heading  (double, rad)
///   * speed  (double, m/s)
///
/// Outputs:
///   * state
///   ??? reflected inputs?
// TODO(maddog@tri.global) Let some other System do the "rendering" conversion
//                         of lane-frame to world-frame PoseBundle/etc.
template <typename T>
class LaneFrameKinematicPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LaneFrameKinematicPlant);

  LaneFrameKinematicPlant();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit LaneFrameKinematicPlant(const LaneFrameKinematicPlant<U>&)
      : LaneFrameKinematicPlant<T>() {}



 private:
  // A witness function to determine when the bouncing ball crosses the
  // boundary q = 0 from q > 0. Note that the witness function only triggers
  // when the signed distance is positive at the left hand side of an interval
  // (via WitnessFunctionDirection::kPositiveThenNonPositive). An "unrestricted
  // update" event is necessary to change the velocity of the system
  // discontinuously.

  T CheckLongitudinalLaneBounds(const systems::Context<T>& context) const;

  T CheckLateralLaneBounds(const systems::Context<T>& context) const;

  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const {
    output->get_mutable_value() =
        context.get_continuous_state().CopyToVector();
  }

  void CopyOutAbstractState(
      const systems::Context<T>& context, api::LaneId* output) const;

  void CopyOutContinuousState(
      const systems::Context<T>& context,
      LaneFrameKinematicPlantContinuousState<T>* output) const;



  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override {
    // Obtain the state.
    const systems::VectorBase<T>& state = context.get_continuous_state_vector();

    // Obtain the structure we need to write into.
    DRAKE_ASSERT(derivatives != nullptr);
    systems::VectorBase<T>& derivatives_vec = derivatives->get_mutable_vector();

    // Time derivative of position (state index 0) is velocity.
    derivatives_vec.SetAtIndex(0, state.GetAtIndex(1));

    // Time derivative of velocity (state index 1) is acceleration.
    derivatives_vec.SetAtIndex(1, T(get_gravitational_acceleration()));
  }

  void SetDefaultState(const systems::Context<T>&,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    Vector2<T> x0;
    x0 << 10.0, 0.0;  // initial state values.
    state->get_mutable_continuous_state().SetFromVector(x0);
  }

  // Updates the velocity discontinuously to reverse direction. This method
  // is called by the Simulator when the signed distance witness function
  // triggers.
  void DoCalcUnrestrictedUpdate(const systems::Context<T>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<T>*>&,
      systems::State<T>* next_state) const override {
    systems::VectorBase<T>& next_cstate =
        next_state->get_mutable_continuous_state().get_mutable_vector();

    // Get present state.
    const systems::VectorBase<T>& cstate =
        context.get_continuous_state().get_vector();

    // Copy the present state to the new one.
    next_state->CopyFrom(context.get_state());

    // Verify that velocity is non-positive.
    DRAKE_DEMAND(cstate.GetAtIndex(1) <= 0.0);

    // Update the velocity using Newtonian restitution (note that Newtonian
    // restitution can lead to unphysical energy gains, as described in
    // [Stronge 1991]). For this reason, other impact models are generally
    // preferable.
    //
    // [Stronge 1991]  W. J. Stronge. Unraveling paradoxical theories for rigid
    //                 body collisions. J. Appl. Mech., 58:1049-1055, 1991.
    next_cstate.SetAtIndex(
        1, cstate.GetAtIndex(1) * restitution_coef_ * -1.);
  }

  // The signed distance witness function is always active and, hence, always
  // returned.
  void DoGetWitnessFunctions(
      const systems::Context<T>&,
      std::vector<const systems::WitnessFunction<T>*>* witnesses)
      const override {
    witnesses->push_back(signed_distance_witness_.get());
  }

  const double restitution_coef_ = 1.0;  // Coefficient of restitution.

  // The witness function for computing the signed distance between the ball
  // and the ground.
  std::unique_ptr<systems::WitnessFunction<T>> signed_distance_witness_;
};

}  // namespace bouncing_ball
}  // namespace drake
