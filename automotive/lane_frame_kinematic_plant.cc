#include "drake/automotive/lane_frame_kinematic_plant.h"

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace automotive {


template <typename T>
LaneFrameKinematicPlant<T>::LaneFrameKinematicPlant()
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::LaneFrameKinematicPlant>{}) {
  // Model for LaneId for port declarations.
  const maliput::api::Lane* kModelLanePointer{};

  // Declare state.
  this->DeclareAbstractState(systems::AbstractValue::Make(kModelLanePointer));
  this->DeclareContinuousState(LaneFrameKinematicPlantContinuousState<T>(),
                               2, 2, 0);

  // Declare input.
  /* xxx_ = */
  this->DeclareVectorInputPort(LaneFrameKinematicPlantContinuousInput<T>());
  /* xxx_ = */
  this->DeclareAbstractInputPort(
      systems::Value<const maliput::api::Lane*>());

  // Declare output.
  /* xxx_ = */
  this->DeclareAbstractOutputPort(
      kModelLanePointer,
      &LaneFrameKinematicPlant::CopyOutAbstractState);
  /* xxx_ = */
  this->DeclareVectorOutputPort(
      LaneFrameKinematicPlantContinuousState<T>(),
      &LaneFrameKinematicPlant::CopyOutContinuousState);

  // Declare witness functions.
  /* xxx_ = */
  this->DeclareWitnessFunction(
      "Longitudinal bounds check",
      systems::WitnessFunctionDirection::kNegativeThenNonNegative,
      &LaneFrameKinematicPlant::CheckLongitudinalLaneBounds,
      systems::UnrestrictedUpdateEvent<T>());
  /* xxx_ = */
  this->DeclareWitnessFunction(
      "Longitudinal bounds check",
      systems::WitnessFunctionDirection::kNegativeThenNonNegative,
      &LaneFrameKinematicPlant::CheckLateralLaneBounds,
      systems::UnrestrictedUpdateEvent<T>());
}


template <typename T>
T LaneFrameKinematicPlant<T>::CheckLongitudinalLaneBounds(
    const systems::Context<T>& context) const {

  // xxxxxxxxxxxx;
  const systems::VectorBase<T>& xc = context.get_continuous_state_vector();
  return xc.GetAtIndex(0);
}


template <typename T>
T LaneFrameKinematicPlant<T>::CheckLateralLaneBounds(
    const systems::Context<T>& context) const {

  //  xxxxxxxxxxxx;
  const systems::VectorBase<T>& xc = context.get_continuous_state_vector();
  return xc.GetAtIndex(0);
}


template <typename T>
void LaneFrameKinematicPlant<T>::CopyOutAbstractState(
    const systems::Context<T>& context,
    const maliput::api::Lane** output) const {
  // TODO(maddog@tri.global)  0 index is a magic value.
  *output = context.template get_abstract_state<const maliput::api::Lane*>(0);
}


template <typename T>
void LaneFrameKinematicPlant<T>::CopyOutContinuousState(
    const systems::Context<T>& context,
    LaneFrameKinematicPlantContinuousState<T>* output) const {
  output->SetFromVector(context.get_continuous_state().CopyToVector());
}


template <typename T>
void LaneFrameKinematicPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* raw_derivatives) const {
  // Obtain the current state.
  const LaneFrameKinematicPlantContinuousState<T>& cstate =
      context.get_continuous_state_vector();
  const maliput::api::Lane* lane = context.get_abstract_state();

  // Obtain the parameters.
// XXX   const EndlessRoadCarConfig<T>& config =
// XXX     this->template GetNumericParameter<EndlessRoadCarConfig>(context, 0);
// XXX   const LaneFrameKinematicPlantParameters<T>& params =
// XXX       context.get_parameters();

  // Obtain the inputs.
  const LaneFrameKinematicPlantContinuousInput<T>* const input =
      EvalVectorInput(context, continuous_input_port_.get_index());

  // Recall:  centripetal acceleration = v^2 / r.
  // TODO(maddog@tri.global)  Correct with lane s-path curvature.
  const T lateral_acceleration =
      state.speed() * state.speed() * input.curvature();

  // Position + velocity ---> position derivatives.
  const maliput::api::LanePosition lane_position(state.s(), state.r(), 0.);
  const maliput::api::IsoLaneVelocity lane_velocity(
      state.speed() * cos(state.heading()),
      state.speed() * sin(state.heading()),
      0.);
  const maliput::api::LanePosition iso_derivatives =
      lane->EvalMotionDerivatives(lane_position, lane_velocity);

  const T heading_dot =
      (cstate.speed() == 0.) ? 0. : (lateral_acceleration / state.speed());
  const T speed_dot = input.forward_acceleration();

  // Return the state's derivatives.
  LaneFrameKinematicPlantContinuousState<T>& derivatives =
      raw_derivatives.get_mutable_continuous_state();
  derivatives->set_s(iso_derivatives.s());
  derivatives->set_r(iso_derivatives.r());
  // Ignore iso_derivatives.h_, which should be zero anyhow.
  derivatives->set_heading(heading_dot);
  derivatives->set_speed(speed_dot);

// XXX  // Magic Guard Rail:  If car is at driveable bounds, clamp r-derivative.
// XXXmaliput::api::RBounds bounds = road_->lane()->driveable_bounds(state.s());
// XXX   if (state.r() <= bounds.r_min) {
// XXX     derivatives.r = std::max(0., derivatives.r);
// XXX   } else if (state.r() >= bounds.r_max) {
// XXX     derivatives.r = std::min(0., derivatives.r);
// XXX   }
}


#if 0
template <typename T>
void LaneFrameKinematicPlant<T>::SetDefaultState(
    const systems::Context<T>&,
    systems::State<T>* state) const override {
  DRAKE_DEMAND(state != nullptr);
  Vector2<T> x0;
  x0 << 10.0, 0.0;  // initial state values.
  state->get_mutable_continuous_state().SetFromVector(x0);
}


// Updates the velocity discontinuously to reverse direction. This method
// is called by the Simulator when the signed distance witness function
// triggers.
template <typename T>
void LaneFrameKinematicPlant<T>::DoCalcUnrestrictedUpdate(
    const systems::Context<T>& context,
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
template <typename T>
void LaneFrameKinematicPlant<T>::DoGetWitnessFunctions(
    const systems::Context<T>&,
    std::vector<const systems::WitnessFunction<T>*>* witnesses)
    const override {
  witnesses->push_back(signed_distance_witness_.get());
}
#endif

}  // namespace automotive
}  // namespace drake


DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::automotive::LaneFrameKinematicPlant)
