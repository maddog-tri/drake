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
  const maliput::api::LaneId kModelLaneId("*model*lane*id*");

  // Declare state.
  DeclareAbstractState(systems::AbstractValue::Make(kModelLaneId));
  DeclareContinuousState(LaneFrameKinematicPlantContinuousState<T>(),
                         2, 2, 0);

  // Declare input.
  xxx_ = DeclareVectorInputPort(LaneFrameKinematicPlantContinuousInput<T>());
  xxx_ = DeclareAbstractInputPort(systems::AbstractValue::Make(kModelLaneId));

  // Declare output.
  xxx_ = DeclareAbstractOutputPort(
      systems::AbstractValue::Make(kModelLaneId),
      &LaneFrameKinematicPlant::CopyOutLaneIdState);
  xxx_ = DeclareVectorOutputPort(
      LaneFrameKinematicPlantContinuousState<T>(),
      &LaneFrameKinematicPlant::CopyOutContinuousState);

  // Declare witness functions.
  xxx_ = DeclareWitnessFunction(
      "Longitudinal bounds check",
      systems::WitnessFunctionDirection::kNegativeThenNonNegative,
      @LaneFrameKinematicPlant::CheckLongitudinalLaneBounds;
      systems::UnrestrictedUpdateEvent<T>());
  xxx_ = DeclareWitnessFunction(
      "Longitudinal bounds check",
      systems::WitnessFunctionDirection::kNegativeThenNonNegative,
      @LaneFrameKinematicPlant::CheckLateralLaneBounds;
      systems::UnrestrictedUpdateEvent<T>());
}


template <typename T>
T LaneFrameKinematicPlant<T>::CheckLongitudinalLaneBounds(
    const systems::Context<T>& context) const {

  xxxxxxxxxxxx;
  const systems::VectorBase<T>& xc = context.get_continuous_state_vector();
  return xc.GetAtIndex(0);
}


template <typename T>
T LaneFrameKinematicPlant<T>::CheckLateralLaneBounds(
    const systems::Context<T>& context) const {

  xxxxxxxxxxxx;
  const systems::VectorBase<T>& xc = context.get_continuous_state_vector();
  return xc.GetAtIndex(0);
}


void LaneFrameKinematicPlant<T>::CopyOutAbstractState(
    const systems::Context<T>& context, api::LaneId* output) const {
  *output = context.template get_abstract_state<api::LaneId>();
}


void LaneFrameKinematicPlant<T>::CopyOutContinuousState(
    const systems::Context<T>& context,
    LaneFrameKinematicPlantContinuousState<T>* output) const {
  output->SetFromVector(context.get_continuous_state().CopyToVector());
}




void LaneFrameKinematicPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* raw_derivatives) const override {


  // Obtain the state.
  const LaneFrameKinematicPlantContinuousState<T>& cstate =
      context.get_continuous_state_vector();
  const api::Lane* lane = context.get_abstract_state();

  const LaneFrameKinematicPlantParameters<T>& params =
      context.get_parameters();

  LaneFrameKinematicPlantContinuousState<T>& derivatives =
      raw_derivatives.get_mutable_continuous_state();




  // Obtain the parameters.
  const EndlessRoadCarConfig<T>& config =
      this->template GetNumericParameter<EndlessRoadCarConfig>(context, 0);


  // Obtain the inputs.
  const LaneFrameKinematicPlantContinuousInput<T>* const input =
      EvalVectorInput(context, continuous_input_port_.get_index());

  // Recall:  centripetal acceleration = v^2 / r.
  const T lateral_acceleration =
      state.speed() * state.speed() * input.curvature();


  // Position + velocity ---> position derivatives.
  maliput::api::LanePosition lane_position(state.s(), state.r(), 0.);
  maliput::api::IsoLaneVelocity lane_velocity(
      state.speed() * cos(state.heading()),
      state.speed() * sin(state.heading()),
      0.);
  maliput::api::LanePosition iso_derivatives =
      lane->EvalMotionDerivatives(lane_position, lane_velocity);

  // Magic Guard Rail:  If car is at driveable bounds, clamp r-derivative.
  maliput::api::RBounds bounds = road_->lane()->driveable_bounds(state.s());
  if (state.r() <= bounds.r_min) {
    derivatives.r = std::max(0., derivatives.r);
  } else if (state.r() >= bounds.r_max) {
    derivatives.r = std::min(0., derivatives.r);
  }

  derivatives->set_s(iso_derivatives.s());
  derivatives->set_r(iso_derivatives.r());
  // Ignore iso_derivatives.h_, which should be zero anyhow.

  const double heading_dot =
      (cstate.speed() == 0.) ? 0. : (lateral_acceleration / state.speed());
  double speed_dot = input.forward_acceleration();

  derivatives->set_heading(heading_dot);
  derivatives->set_speed(speed_dot);
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


  // The witness function for computing the signed distance between the ball
  // and the ground.
  std::unique_ptr<systems::WitnessFunction<T>> signed_distance_witness_;
};

}  // namespace bouncing_ball
}  // namespace drake


DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::automotive::LaneFrameKinematicPlant)
