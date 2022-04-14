// Copyright 2022 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nao_phase_provider/nao_phase_provider.hpp"

using std::placeholders::_1;

namespace nao_phase_provider
{

NaoPhaseProvider::NaoPhaseProvider(const rclcpp::NodeOptions & options)
: rclcpp::Node{"nao_phase_provider", options}
{
  // Create subscription
  fsrSub =
    create_subscription<nao_sensor_msgs::msg::FSR>(
    "fsr", 10, std::bind(&NaoPhaseProvider::fsrCallback, this, _1));

  // Create publisher
  phasePub = create_publisher<biped_interfaces::msg::Phase>("phase", 10);
}

void NaoPhaseProvider::fsrCallback(const nao_sensor_msgs::msg::FSR::SharedPtr fsr)
{
  auto sumLeftFoot =
    fsr->l_foot_front_left +
    fsr->l_foot_front_right +
    fsr->l_foot_back_left +
    fsr->l_foot_back_right;

  auto sumRightFoot =
    fsr->r_foot_front_left +
    fsr->r_foot_front_right +
    fsr->r_foot_back_left +
    fsr->r_foot_back_right;

  biped_interfaces::msg::Phase phase;
  if (sumLeftFoot >= sumRightFoot) {
    phase.phase = phase.LEFT_STANCE;
  } else {
    phase.phase = phase.RIGHT_STANCE;
  }

  phasePub->publish(phase);
}


}  // namespace nao_phase_provider
