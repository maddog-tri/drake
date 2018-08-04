#include "drake/automotive/lane_frame_kinematic_plant.h"

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/default_scalars.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace automotive {


template <typename T>
LaneFrameKinematicPlant<T>::LaneFrameKinematicPlant()
    : systems::LeafSystem<T>(
// XXX          systems::SystemTypeTag<automotive::LaneFrameKinematicPlant>{}
) {
  // Model for AbstractState for abstract state declarations.
  const AbstractState kModelAbstractState{};

  // Declare state.
  abstract_state_index_ = this->DeclareAbstractState(
      systems::AbstractValue::Make(kModelAbstractState));
  this->DeclareContinuousState(LaneFrameKinematicPlantContinuousState<T>(),
                               2, 2, 0);

  // Declare input.
  abstract_input_port_index_ =
      this->DeclareAbstractInputPort(
          systems::Value<AbstractInput>()).get_index();
  continuous_input_port_index_ =
      this->DeclareVectorInputPort(
          LaneFrameKinematicPlantContinuousInput<T>()).get_index();

  // Declare output.
  abstract_output_port_index_ =
      this->DeclareAbstractOutputPort(
          kModelAbstractState,
          &LaneFrameKinematicPlant::CopyOutAbstractState).get_index();
  continuous_output_port_index_ =
      this->DeclareVectorOutputPort(
          LaneFrameKinematicPlantContinuousState<T>(),
          &LaneFrameKinematicPlant::CopyOutContinuousState).get_index();

  // Declare witness functions.
  longitudinal_bounds_witness_ =
      this->DeclareWitnessFunction(
          "Longitudinal bounds check",
          systems::WitnessFunctionDirection::kNegativeThenNonNegative,
          &LaneFrameKinematicPlant::CheckLongitudinalLaneBounds,
          systems::UnrestrictedUpdateEvent<T>());
  lateral_bounds_witness_ =
      this->DeclareWitnessFunction(
          "Lateral bounds check",
          systems::WitnessFunctionDirection::kNegativeThenNonNegative,
          &LaneFrameKinematicPlant::CheckLateralLaneBounds,
          systems::UnrestrictedUpdateEvent<T>());
}



template <typename T>
const systems::InputPort<T>&
LaneFrameKinematicPlant<T>::continuous_input_port() const {
  return this->get_input_port(continuous_input_port_index_);
}


template <typename T>
const systems::InputPort<T>&
LaneFrameKinematicPlant<T>::abstract_input_port() const {
  return this->get_input_port(abstract_input_port_index_);
}


template <typename T>
const systems::OutputPort<T>&
LaneFrameKinematicPlant<T>::continuous_output_port() const {
  return this->get_output_port(continuous_output_port_index_);
}


template <typename T>
const systems::OutputPort<T>&
LaneFrameKinematicPlant<T>::abstract_output_port() const {
  return this->get_output_port(abstract_output_port_index_);
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
    AbstractState* output) const {
  *output = context.template get_abstract_state<AbstractState>(
      abstract_state_index_);
}


template <typename T>
void LaneFrameKinematicPlant<T>::CopyOutContinuousState(
    const systems::Context<T>& context,
    LaneFrameKinematicPlantContinuousState<T>* output) const {
  // TODO(maddog@tri.global)  Three ways (at least) of copying the vector...
  //                          Which way is the best?
  // output->SetFromVector(context.get_continuous_state().CopyToVector());
  // output->set_value(context.get_continuous_state().CopyToVector());
  output->get_mutable_value() = context.get_continuous_state().CopyToVector();
}


template <typename T>
void LaneFrameKinematicPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* raw_derivatives) const {
  // Obtain the current state.
  const LaneFrameKinematicPlantContinuousState<T>& cstate =
      get_continuous_state(context);
  AbstractState astate =
  context.template get_abstract_state<AbstractState>(
      abstract_state_index_);

  // Obtain the parameters.
// XXX   const EndlessRoadCarConfig<T>& config =
// XXX     this->template GetNumericParameter<EndlessRoadCarConfig>(context, 0);
// XXX   const LaneFrameKinematicPlantParameters<T>& params =
// XXX       context.get_parameters();

  // Obtain the inputs.
  const LaneFrameKinematicPlantContinuousInput<T>* const input =
      this->template EvalVectorInput<LaneFrameKinematicPlantContinuousInput>(
          context, continuous_input_port_index_);


  // Recall:  centripetal acceleration = v^2 / r.
  // TODO(maddog@tri.global)  Correct with lane's s-path curvature.
  const T lateral_acceleration =
      cstate.speed() * cstate.speed() * input->curvature();
  input->curvature();

  // Position + velocity ---> position derivatives.
  const maliput::api::LanePosition lane_position(cstate.s(), cstate.r(), 0.);
  const maliput::api::IsoLaneVelocity lane_velocity(
      cstate.speed() * cos(cstate.heading()),
      cstate.speed() * sin(cstate.heading()),
      0.);
  DRAKE_THROW_UNLESS(astate.lane != nullptr);
  const maliput::api::LanePosition iso_derivatives =
      astate.lane->EvalMotionDerivatives(lane_position, lane_velocity);

  const T heading_dot =
      (cstate.speed() == 0.) ? 0. : (lateral_acceleration / cstate.speed());
  const T speed_dot = input->forward_acceleration();

  // Return the state's derivatives.
  LaneFrameKinematicPlantContinuousState<T>& derivatives =
      get_mutable_continuous_state(raw_derivatives);
  derivatives.set_s(iso_derivatives.s());
  derivatives.set_r(iso_derivatives.r());
  // Ignore iso_derivatives.h_, which should be zero anyhow.
  derivatives.set_heading(heading_dot);
  derivatives.set_speed(speed_dot);

// XXX  // Magic Guard Rail:  If car is at driveable bounds, clamp r-derivative.
// XXXmaliput::api::RBounds bounds = road_->lane()->driveable_bounds(state.s());
// XXX   if (state.r() <= bounds.r_min) {
// XXX     derivatives.r = std::max(0., derivatives.r);
// XXX   } else if (state.r() >= bounds.r_max) {
// XXX     derivatives.r = std::min(0., derivatives.r);
// XXX   }
}


// XXX template <typename T>
// XXX void LaneFrameKinematicPlant<T>::SetDefaultState(
// XXX     const systems::Context<T>&,
// XXX     systems::State<T>* state) const override {
// XXX   DRAKE_DEMAND(state != nullptr);
// XXX   Vector2<T> x0;
// XXX   x0 << 10.0, 0.0;  // initial state values.
// XXX   state->get_mutable_continuous_state().SetFromVector(x0);
// XXX }


template <typename T>
void LaneFrameKinematicPlant<T>::DoCalcUnrestrictedUpdate(
    const systems::Context<T>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<T>*>&,
    systems::State<T>* next_state) const {
#if 0
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
#endif
}


template <typename T>
void LaneFrameKinematicPlant<T>::DoGetWitnessFunctions(
    const systems::Context<T>&,
    std::vector<const systems::WitnessFunction<T>*>* witnesses) const {
  // Both witness functions are always active.
  witnesses->push_back(longitudinal_bounds_witness_.get());
  witnesses->push_back(lateral_bounds_witness_.get());
}


}  // namespace automotive
}  // namespace drake

// TODO(maddog@tri.global)  Maybe later, maybe never?
// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//     class ::drake::automotive::LaneFrameKinematicPlant)

template class ::drake::automotive::LaneFrameKinematicPlant<double>;
