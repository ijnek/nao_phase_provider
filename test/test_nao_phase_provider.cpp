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

#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nao_phase_provider/nao_phase_provider.hpp"

class TestNaoPhaseProvider : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    naoPhaseProvider = std::make_shared<nao_phase_provider::NaoPhaseProvider>();

    node = std::make_shared<rclcpp::Node>("test");
    fsrPub = node->create_publisher<nao_sensor_msgs::msg::FSR>("fsr", 10);
    phaseSub = node->create_subscription<biped_interfaces::msg::Phase>(
      "phase", 10,
      [this](biped_interfaces::msg::Phase::SharedPtr phase) {
        receivedPhase = phase;
      });
  }

  nao_phase_provider::NaoPhaseProvider::SharedPtr naoPhaseProvider;
  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<nao_sensor_msgs::msg::FSR>::SharedPtr fsrPub;
  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr phaseSub;

  void phaseCallback(const biped_interfaces::msg::Phase::SharedPtr phase);
  biped_interfaces::msg::Phase::SharedPtr receivedPhase;
};

TEST_F(TestNaoPhaseProvider, l_foot_front_left)
{
  nao_sensor_msgs::msg::FSR fsr;
  fsr.l_foot_front_left = 10.0;
  fsrPub->publish(fsr);

  rclcpp::spin_some(naoPhaseProvider);
  rclcpp::spin_some(node);
  ASSERT_TRUE(receivedPhase);
  EXPECT_EQ(receivedPhase->phase, biped_interfaces::msg::Phase::LEFT_STANCE);
}

TEST_F(TestNaoPhaseProvider, l_foot_front_right)
{
  nao_sensor_msgs::msg::FSR fsr;
  fsr.l_foot_front_right = 10.0;
  fsrPub->publish(fsr);

  rclcpp::spin_some(naoPhaseProvider);
  rclcpp::spin_some(node);
  ASSERT_TRUE(receivedPhase);
  EXPECT_EQ(receivedPhase->phase, biped_interfaces::msg::Phase::LEFT_STANCE);
}

TEST_F(TestNaoPhaseProvider, l_foot_back_left)
{
  nao_sensor_msgs::msg::FSR fsr;
  fsr.l_foot_back_left = 10.0;
  fsrPub->publish(fsr);

  rclcpp::spin_some(naoPhaseProvider);
  rclcpp::spin_some(node);
  ASSERT_TRUE(receivedPhase);
  EXPECT_EQ(receivedPhase->phase, biped_interfaces::msg::Phase::LEFT_STANCE);
}

TEST_F(TestNaoPhaseProvider, l_foot_back_right)
{
  nao_sensor_msgs::msg::FSR fsr;
  fsr.l_foot_back_right = 10.0;
  fsrPub->publish(fsr);

  rclcpp::spin_some(naoPhaseProvider);
  rclcpp::spin_some(node);
  ASSERT_TRUE(receivedPhase);
  EXPECT_EQ(receivedPhase->phase, biped_interfaces::msg::Phase::LEFT_STANCE);
}

TEST_F(TestNaoPhaseProvider, r_foot_front_left)
{
  nao_sensor_msgs::msg::FSR fsr;
  fsr.r_foot_front_left = 10.0;
  fsrPub->publish(fsr);

  rclcpp::spin_some(naoPhaseProvider);
  rclcpp::spin_some(node);
  ASSERT_TRUE(receivedPhase);
  EXPECT_EQ(receivedPhase->phase, biped_interfaces::msg::Phase::RIGHT_STANCE);
}

TEST_F(TestNaoPhaseProvider, r_foot_front_right)
{
  nao_sensor_msgs::msg::FSR fsr;
  fsr.r_foot_front_right = 10.0;
  fsrPub->publish(fsr);

  rclcpp::spin_some(naoPhaseProvider);
  rclcpp::spin_some(node);
  ASSERT_TRUE(receivedPhase);
  EXPECT_EQ(receivedPhase->phase, biped_interfaces::msg::Phase::RIGHT_STANCE);
}

TEST_F(TestNaoPhaseProvider, r_foot_back_left)
{
  nao_sensor_msgs::msg::FSR fsr;
  fsr.r_foot_back_left = 10.0;
  fsrPub->publish(fsr);

  rclcpp::spin_some(naoPhaseProvider);
  rclcpp::spin_some(node);
  ASSERT_TRUE(receivedPhase);
  EXPECT_EQ(receivedPhase->phase, biped_interfaces::msg::Phase::RIGHT_STANCE);
}

TEST_F(TestNaoPhaseProvider, r_foot_back_right)
{
  nao_sensor_msgs::msg::FSR fsr;
  fsr.r_foot_back_right = 10.0;
  fsrPub->publish(fsr);

  rclcpp::spin_some(naoPhaseProvider);
  rclcpp::spin_some(node);
  ASSERT_TRUE(receivedPhase);
  EXPECT_EQ(receivedPhase->phase, biped_interfaces::msg::Phase::RIGHT_STANCE);
}
