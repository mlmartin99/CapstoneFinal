// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "state_interface/msg/state.hpp"
// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class UARTBridge : public rclcpp::Node
{
public:
  UARTBridge();


private:
  void init_uart();
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_pub;
  std::shared_ptr<rclcpp::Subscription<state_interface::msg::State>> state_sub_;
  rclcpp::TimerBase::SharedPtr recv_timer;
  void twist_callback(const geometry_msgs::msg::Twist & msg);
  void state_callback(const std::shared_ptr<state_interface::msg::State> msg);
  void recv_timer_callback();
  size_t count_;
  struct termios tty;
  int serial_port;
  float speeds[2];
  uint8_t current_state_;
};

