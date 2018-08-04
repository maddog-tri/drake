#include "drake/automotive/lane_frame_kinematic_plant.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/multilane/builder.h"

namespace drake {

using maliput::api::LaneEnd;

using systems::BasicVector;
using systems::LeafContext;
using systems::Parameters;

namespace api = maliput::api;
namespace multilane = maliput::multilane;

namespace automotive {
namespace {


class LaneFrameKinematicPlantTest : public::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<LaneFrameKinematicPlant<double>>();
    context_ = dut_->CreateDefaultContext();

    abstract_input_ = &context_->FixInputPort(
        dut_->abstract_input_port().get_index(),
        dut_->AllocateInputAbstract(dut_->abstract_input_port()));
    continuous_input_ = &context_->FixInputPort(
        dut_->continuous_input_port().get_index(),
        dut_->AllocateInputVector(dut_->continuous_input_port()));
  }

  std::unique_ptr<const api::RoadGeometry> MakeBasicRoad() {
    const double kStraightLength = 100.;
    const double kCurveRadius = 50.;
    const double kLeftShoulder = 1.;
    const double kRightShoulder = 1.;
    const double kLaneWidth = 5.;
    const api::HBounds kElevationBounds(0., 10.);
    const double kLinearTolerance = 0.01;
    const double kAngularTolerance = 0.01;
    const double kScaleLength = 1.0;
    const multilane::LaneLayout kThreeLaneLayout(
        kLeftShoulder, kRightShoulder, 3, 0, 0.);
    const double kStartHeading = 0.;  // x direction
    const multilane::EndpointZ kLowFlatZ {0., 0., 0., {}};
    const multilane::Endpoint kOrigin {{0., 0., kStartHeading}, kLowFlatZ};

    using LaneEnd = api::LaneEnd;
    using Direction = multilane::Direction;

    multilane::Builder b(kLaneWidth, kElevationBounds,
                         kLinearTolerance, kAngularTolerance, kScaleLength,
                         multilane::ComputationPolicy::kPreferAccuracy);

    auto straight = b.Connect(
        "straight", kThreeLaneLayout,
        multilane::StartLane(0).at(kOrigin, Direction::kForward),
        multilane::LineOffset(kStraightLength),
        multilane::EndLane(0).z_at(kLowFlatZ, Direction::kForward));
    b.Connect(
        "curve", kThreeLaneLayout,
        multilane::StartLane(0).at(
            *straight, 0, LaneEnd::kFinish, Direction::kForward),
        multilane::ArcOffset(kCurveRadius, M_PI),
        multilane::EndLane(0).z_at(kLowFlatZ, Direction::kForward));

    return b.Build(api::RoadGeometryId("basic-road"));
  }


  LaneFrameKinematicPlantContinuousState<double>& continuous_state() {
    auto result = dynamic_cast<LaneFrameKinematicPlantContinuousState<double>*>(
        &context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return *result;
  }


  LaneFrameKinematicPlant<double>::AbstractState& abstract_state() {
    // TODO(maddog@tri.global)  Zero index is magic.
    return context_->template get_mutable_abstract_state<
      LaneFrameKinematicPlant<double>::AbstractState>(0);
  }


  const LaneFrameKinematicPlantContinuousState<double>&
  continuous_output(systems::SystemOutput<double>* output) const {
    auto result =
        dynamic_cast<const LaneFrameKinematicPlantContinuousState<double>*>(
            output->get_vector_data(
                dut_->continuous_output_port().get_index()));
    if (result == nullptr) { throw std::bad_cast(); }
    return *result;
  }

  const LaneFrameKinematicPlant<double>::AbstractState&
  abstract_output(systems::SystemOutput<double>* output) const {
    return output->get_data(dut_->abstract_output_port().get_index())->
        GetValueOrThrow<LaneFrameKinematicPlant<double>::AbstractState>();
  }

  void set_abstract_input(
      const LaneFrameKinematicPlant<double>::AbstractInput& value) const {
    abstract_input_->GetMutableData()->GetMutableValueOrThrow<
      LaneFrameKinematicPlant<double>::AbstractInput>() = value;
  }

  void set_continuous_input(
      const LaneFrameKinematicPlantContinuousInput<double>& value) const {
    continuous_input_->GetMutableVectorData<double>()->set_value(
          value.get_value());
  }


  std::unique_ptr<LaneFrameKinematicPlant<double>> dut_;
  std::unique_ptr<systems::Context<double>> context_;
  systems::FixedInputPortValue* abstract_input_;
  systems::FixedInputPortValue* continuous_input_;
};


TEST_F(LaneFrameKinematicPlantTest, SystemTopology) {
  // Check composition of input ports.
  EXPECT_EQ(dut_->get_num_input_ports(), 2);

  const auto& abstract_input = dut_->abstract_input_port();
  EXPECT_EQ(abstract_input.get_data_type(), systems::kAbstractValued);

  const auto& continuous_input = dut_->continuous_input_port();
  EXPECT_EQ(continuous_input.get_data_type(), systems::kVectorValued);
  EXPECT_EQ(continuous_input.size(),
            LaneFrameKinematicPlantContinuousInputIndices::kNumCoordinates);

  // Check composition of output ports.
  EXPECT_EQ(dut_->get_num_output_ports(), 2);

  const auto& abstract_output = dut_->abstract_output_port();
  EXPECT_EQ(abstract_output.get_data_type(), systems::kAbstractValued);

  const auto& continuous_output = dut_->continuous_output_port();
  EXPECT_EQ(continuous_output.get_data_type(), systems::kVectorValued);
  EXPECT_EQ(continuous_output.size(),
            LaneFrameKinematicPlantContinuousStateIndices::kNumCoordinates);

  // Check composition of context's state.
  EXPECT_EQ(context_->get_num_abstract_states(), 1);
  EXPECT_EQ(context_->get_continuous_state().size(),
            LaneFrameKinematicPlantContinuousStateIndices::kNumCoordinates);
  EXPECT_EQ(context_->get_continuous_state().num_q(), 2);
  EXPECT_EQ(context_->get_continuous_state().num_v(), 2);

  // TODO(maddog@tri.global)  Check composition of context's parameters.
}


TEST_F(LaneFrameKinematicPlantTest, OutputCopiesState) {
  // Set up state in context_.
  api::Lane* const kExpectedLane = reinterpret_cast<api::Lane*>(0xDeadBeef);
  constexpr double kExpectedS = 99.9;
  constexpr double kExpectedR = -2.3;
  constexpr double kExpectedHeading = 0.32;
  constexpr double kExpectedSpeed = 500.7;
  abstract_state().lane = kExpectedLane;
  continuous_state().set_s(kExpectedS);
  continuous_state().set_r(kExpectedR);
  continuous_state().set_heading(kExpectedHeading);
  continuous_state().set_speed(kExpectedSpeed);

  // Run dut_.
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut_->AllocateOutput();
  dut_->CalcOutput(*context_, output.get());

  // Verify results.
  EXPECT_EQ(abstract_output(output.get()).lane, kExpectedLane);
  EXPECT_EQ(continuous_output(output.get()).s(), kExpectedS);
  EXPECT_EQ(continuous_output(output.get()).r(), kExpectedR);
  EXPECT_EQ(continuous_output(output.get()).heading(), kExpectedHeading);
  EXPECT_EQ(continuous_output(output.get()).speed(), kExpectedSpeed);
}


struct DerivativesTestParameters {
  struct State {
    std::string lane_id;
    double s_fraction{};  // fraction of lane's length
    double r{};
    double heading{};
    double speed{};
  } state;
  struct Input {
    double forward_acceleration{};
    double curvature{};
  } input;
  struct expected {
    double ds{};
    double dr{};
    double dheading{};
    double dspeed{};
  } expected;
};

std::ostream& operator<<(std::ostream& os, const DerivativesTestParameters& p) {
  return os << fmt::format(
      "DerivativesTestParameters( State( lane_id '{}', s_fraction {}, r {}, "
      "heading {}, speed {}), Input( forward_acceleration {}, curvature {}), "
      "Expected( ds {}, dr {}, dheading {}, dspeed {}))",
      p.state.lane_id,
      p.state.r, p.state.s_fraction, p.state.heading, p.state.speed,
      p.input.forward_acceleration, p.input.curvature,
      p.expected.ds, p.expected.dr, p.expected.dheading, p.expected.dspeed);
}

class LaneFrameKinematicPlantDerivativesTest
    : public LaneFrameKinematicPlantTest,
      public testing::WithParamInterface<DerivativesTestParameters> {};



TEST_P(LaneFrameKinematicPlantDerivativesTest, Derivatives) {
  const DerivativesTestParameters& p = GetParam();

  // Set up state in context_...
  const std::unique_ptr<const api::RoadGeometry> road = MakeBasicRoad();
  const api::Lane* const lane =
      road->ById().GetLane(api::LaneId(p.state.lane_id));
  ASSERT_TRUE(lane != nullptr);
  abstract_state().lane = lane;
  continuous_state().set_s(p.state.s_fraction * lane->length());
  continuous_state().set_r(p.state.r);
  continuous_state().set_heading(p.state.heading);
  continuous_state().set_speed(p.state.speed);

  // Set up input...
  set_abstract_input({lane});
  LaneFrameKinematicPlantContinuousInput<double> input;
  input.set_forward_acceleration(p.input.forward_acceleration);
  input.set_curvature(p.input.curvature);
  set_continuous_input(input);

  // Run dut_.
  const std::unique_ptr<systems::ContinuousState<double>> derivatives_state =
      dut_->AllocateTimeDerivatives();
  dut_->CalcTimeDerivatives(*context_, derivatives_state.get());

  // Verify results.
  LaneFrameKinematicPlantContinuousState<double>& derivatives =
      dynamic_cast<LaneFrameKinematicPlantContinuousState<double>&>(
          derivatives_state->get_mutable_vector());
  EXPECT_EQ(derivatives.s(), p.expected.ds);
  EXPECT_EQ(derivatives.r(), p.expected.dr);
  EXPECT_EQ(derivatives.heading(), p.expected.dheading);
  EXPECT_EQ(derivatives.speed(), p.expected.dspeed);
}

INSTANTIATE_TEST_CASE_P(
    Derivatives, LaneFrameKinematicPlantDerivativesTest,
    testing::ValuesIn(std::vector<DerivativesTestParameters>{
        {{"l:straight_0", 0.5, 0., 0., 2.}, {10., 0.}, {2., 0., 0., 10.}}
      }));


}  // namespace
}  // namespace automotive
}  // namespace drake
