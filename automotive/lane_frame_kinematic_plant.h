#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/gen/lane_frame_kinematic_plant_continuous_input.h"
#include "drake/automotive/gen/lane_frame_kinematic_plant_continuous_state.h"
#include "drake/automotive/maliput/api/lane.h"
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


/// Instantiated templates for the following scalar types @p T are provided:
/// - double
// XXX /// - drake::AutoDiffXd
// XXX /// - symbolic::Expression
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// They are already available to link against in the containing library.
///
/// Input:
///   * Abstract
///     * LaneFrameKinematicPlant::AbstractInput:
///       * next_lane  (const maliput::api::Lane*)
///   * Continuous:
///       * LaneFrameKinematicPlantContinuousInput:
///         * kappa (double, 1/m, curvature)
///         * a (double, m/s^2, longitudinal acceleration)
///
/// State:
///   * Abstract:
///     * LaneFrameKinematicPlant::AbstractState:
///       * lane  (const maliput::api::Lane*)
///   * Continuous:
///     * LaneFrameKinematicPlantContinuousState:
///       * s  (double, m)
///       * r  (double, m)
///       * heading  (double, rad)
///       * speed  (double, m/s)
///
/// Outputs:  (mirror of state)
///   * Abstract:
///     * LaneFrameKinematicPlant::AbstractState:
///       * lane  (const maliput::api::Lane*)
///   * Continuous:
///     * LaneFrameKinematicPlantContinuousState:
///       * s  (double, m)
///       * r  (double, m)
///       * heading  (double, rad)
///       * speed  (double, m/s)
///
// TODO(maddog@tri.global) Let some other System do the "rendering" conversion
//                         of lane-frame to world-frame PoseBundle/etc.
template <typename T>
class LaneFrameKinematicPlant final : public systems::LeafSystem<T> {
 public:
  struct AbstractState {
    const maliput::api::Lane* lane{};
  };

  struct AbstractInput {
    const maliput::api::Lane* next_lane{};
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LaneFrameKinematicPlant);

  LaneFrameKinematicPlant();

// XXX/// Scalar-converting copy constructor. See @ref system_scalar_conversion.
// XXX   template <typename U>
// XXX   explicit LaneFrameKinematicPlant(const LaneFrameKinematicPlant<U>&)
// XXX       : LaneFrameKinematicPlant<T>() {}


  /// Getter methods for input and output ports.
  /// @{
  const systems::InputPort<T>& continuous_input_port() const;
  const systems::InputPort<T>& abstract_input_port() const;
  const systems::OutputPort<T>& continuous_output_port() const;
  const systems::OutputPort<T>& abstract_output_port() const;
  /// @}



 private:
  // A witness function to determine when the bouncing ball crosses the
  // boundary q = 0 from q > 0. Note that the witness function only triggers
  // when the signed distance is positive at the left hand side of an interval
  // (via WitnessFunctionDirection::kPositiveThenNonPositive). An "unrestricted
  // update" event is necessary to change the velocity of the system
  // discontinuously.

  T CheckLongitudinalLaneBounds(const systems::Context<T>& context) const;

  T CheckLateralLaneBounds(const systems::Context<T>& context) const;

  void CopyOutAbstractState(const systems::Context<T>& context,
                            AbstractState* output) const;

  void CopyOutContinuousState(
      const systems::Context<T>& context,
      LaneFrameKinematicPlantContinuousState<T>* output) const;



  static const LaneFrameKinematicPlantContinuousState<T>&
      get_continuous_state(const systems::ContinuousState<T>& state) {
    return dynamic_cast<const LaneFrameKinematicPlantContinuousState<T>&>(
        state.get_vector());
  }

  static const LaneFrameKinematicPlantContinuousState<T>&
      get_continuous_state(const systems::Context<T>& context) {
    return get_continuous_state(context.get_continuous_state());
  }


  static LaneFrameKinematicPlantContinuousState<T>&
      get_mutable_continuous_state(systems::ContinuousState<T>* state) {
    return dynamic_cast<LaneFrameKinematicPlantContinuousState<T>&>(
        state->get_mutable_vector());
  }

  static LaneFrameKinematicPlantContinuousState<T>&
      get_mutable_continuous_state(systems::Context<T>* context) {
    return get_mutable_continuous_state(
        &context->get_mutable_continuous_state());
  }




  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

// XXX   void SetDefaultState(const systems::Context<T>&,
// XXX                        systems::State<T>* state) const override;

  // Updates the velocity discontinuously to reverse direction. This method
  // is called by the Simulator when the signed distance witness function
  // triggers.
  void DoCalcUnrestrictedUpdate(
      const systems::Context<T>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<T>*>&,
      systems::State<T>* next_state) const override;

  // The signed distance witness function is always active and, hence, always
  // returned.
  void DoGetWitnessFunctions(
      const systems::Context<T>&,
      std::vector<const systems::WitnessFunction<T>*>* witnesses)
      const override;


  int abstract_state_index_{-1};

  int continuous_input_port_index_{-1};
  int abstract_input_port_index_{-1};

  int continuous_output_port_index_{-1};
  int abstract_output_port_index_{-1};

  std::unique_ptr<systems::WitnessFunction<T>> longitudinal_bounds_witness_;
  std::unique_ptr<systems::WitnessFunction<T>> lateral_bounds_witness_;
};

}  // namespace automotive
}  // namespace drake
