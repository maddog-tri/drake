#include "drake/automotive/lane_frame_kinematic_plant.h"

#include <gtest/gtest.h>

namespace drake {

using maliput::api::LaneEnd;

using systems::BasicVector;
using systems::LeafContext;
using systems::Parameters;

namespace automotive {
namespace {


class LaneFrameKinematicPlantTest : public::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<LaneFrameKinematicPlant<double>>();
  }

  std::unique_ptr<LaneFrameKinematicPlant<double>> dut_;
  std::unique_ptr<systems::Context<double>> context_;
};


TEST_F(LaneFrameKinematicPlantTest, SystemTopology) {
  // Check composition of input ports.
  ASSERT_EQ(dut_->get_num_input_ports(), 2);

  const auto& abstract_input = dut_->abstract_input();
  EXPECT_EQ(systems::kAbstractValued, abstract_input.get_data_type());

  const auto& continuous_input = dut_->continuous_input();
  EXPECT_EQ(systems::kVectorValued, continuous_input.get_data_type());
  EXPECT_EQ(LaneFrameKinematicPlantContinuousInputIndices::kNumCoordinates,
            continuous_input.size());

  // Check composition of output ports.
  ASSERT_EQ(dut_->get_num_output_ports(), 2);

  const auto& abstract_output = dut_->abstract_output();
  EXPECT_EQ(systems::kAbstractValued, abstract_output.get_data_type());

  const auto& continuous_output = dut_->continuous_output();
  EXPECT_EQ(systems::kVectorValued, continuous_output.get_data_type());
  EXPECT_EQ(LaneFrameKinematicPlantContinuousStateIndices::kNumCoordinates,
            continuous_output.size());
}


}  // namespace
}  // namespace automotive
}  // namespace drake
