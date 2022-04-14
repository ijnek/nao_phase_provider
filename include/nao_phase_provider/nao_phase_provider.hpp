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

#ifndef NAO_PHASE_PROVIDER__NAO_PHASE_PROVIDER_HPP_
#define NAO_PHASE_PROVIDER__NAO_PHASE_PROVIDER_HPP_

#include "rclcpp/node.hpp"
#include "nao_sensor_msgs/msg/fsr.hpp"
#include "biped_interfaces/msg/phase.hpp"

namespace nao_phase_provider
{

class NaoPhaseProvider : public rclcpp::Node
{
public:
  explicit NaoPhaseProvider(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  rclcpp::Subscription<nao_sensor_msgs::msg::FSR>::SharedPtr fsrSub;
  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr phasePub;

  void fsrCallback(const nao_sensor_msgs::msg::FSR::SharedPtr fsr);
};

}  // namespace nao_phase_provider

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nao_phase_provider::NaoPhaseProvider)

#endif  // NAO_PHASE_PROVIDER__NAO_PHASE_PROVIDER_HPP_
