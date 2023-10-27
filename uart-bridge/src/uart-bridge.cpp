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
#include "uart-bridge/uart-bridge.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

UARTBridge::UARTBridge()
: Node("uart_bridge"), count_(0)
{
  init_uart();
  RCLCPP_INFO(this->get_logger(), "UART init");
  twist_sub = create_subscription<geometry_msgs::msg::Twist>(
    "speed", 10, std::bind(&UARTBridge::twist_callback, this, _1));

  recv_timer = this->create_wall_timer(
    500ms, std::bind(&UARTBridge::recv_timer_callback, this));

  state_sub_ = create_subscription<state_interface::msg::State>(
    "state", 3,
    [this] (const std::shared_ptr<state_interface::msg::State> msg) {state_callback(msg);}
    );

  left_pub = this->create_publisher<std_msgs::msg::Float32>("left_speed", 10);

  right_pub = this->create_publisher<std_msgs::msg::Float32>("right_speed", 10);
}


void UARTBridge::twist_callback(const geometry_msgs::msg::Twist & msg)
{

  uint8_t data[5];
  uint8_t linear = (uint8_t)msg.linear.x;
  if (linear == 129) {
    linear = 0;
  }

  uint8_t angular = (uint8_t)msg.angular.z;
  if (angular == 129) {
    angular = 0;
  }

  uint8_t header = (uint8_t)(129);

  uint8_t collection = 0;
  if(current_state_ != 0){
    collection = 1;
  }

  data[0] = header;
  data[1] = header;
  data[2] = linear;
  data[3] = angular;
  data[4] = collection;
  write(serial_port, &data, sizeof(uint8_t)*5);
  // RCLCPP_INFO_STREAM(get_logger(), "Wrote: " << std::to_string(data[0]) << "and" << std::to_string(data[1]) << "and" << std::to_string(data[2]) << " and " << std::to_string(data[3]) << std::endl);
}

void UARTBridge::init_uart()
{
  serial_port = open("/dev/ttyACM0", O_RDWR);

  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      // return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag |= CRTSCTS; // Enavle RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      // return 1;
  }
}

void UARTBridge::state_callback(const std::shared_ptr<state_interface::msg::State> msg)
{
  current_state_ = msg->state;
}

void UARTBridge::recv_timer_callback()
{
 if(read(serial_port, &speeds, sizeof(speeds)) != 0){

  //  RCLCPP_INFO_STREAM(get_logger(), "Read: " << speeds[0] << " and " << speeds[1] << std::endl);

    auto leftVelocity = std_msgs::msg::Float32();
    auto rightVelocity = std_msgs::msg::Float32();

    leftVelocity.data = speeds[0];
    rightVelocity.data = speeds[1];

   left_pub->publish(leftVelocity);
    right_pub->publish(rightVelocity);

  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UARTBridge>());
  rclcpp::shutdown();

  return 0;
}
