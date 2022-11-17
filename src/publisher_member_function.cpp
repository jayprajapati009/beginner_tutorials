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

/**
 * @file publisher_member_function.cpp
 * @author (Edited by) Jay Prajapati (jayp@umd.edu)
 * @brief Publisher defination file
 * @version 0.2
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "cpp_pubsub/srv/modify_msg.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using sharedFuture = rclcpp::Client<cpp_pubsub::srv::ModifyMsg>::SharedFuture;

/**
 * @brief MinimalPublisher class, defines the publisher, service client and the
 * associated function
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

    client = this->create_client<cpp_pubsub::srv::ModifyMsg>("modify_msg");
    RCLCPP_DEBUG(this->get_logger(), "Client created");
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Interrupted");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Service unavailable");
    }
  }

 private:
  std::string Message;
  rclcpp::Client<cpp_pubsub::srv::ModifyMsg>::SharedPtr client;

  /**
   * @brief timer_callback function, sets the message data and publishes the
   * message and also calls the service at every 10 counts
   *
   */
  void timer_callback() {
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Node setup");
    auto message = std_msgs::msg::String();
    message.data = "The count is " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    if (count_ % 10 == 0) {
      call_service();
    }
    auto steady_clock = rclcpp::Clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), steady_clock, 10000,
                                 "Node running successfully");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  /**
   * @brief call_service function, defnies the service parameters and calls the
   * response
   *
   * @return int
   */
  int call_service() {
    auto request = std::make_shared<cpp_pubsub::srv::ModifyMsg::Request>();
    request->a = "String1";
    request->b = " String2";
    RCLCPP_INFO(this->get_logger(), "Calling Service to Modify string");
    auto callbackPtr =
        std::bind(&MinimalPublisher::response_callback, this, _1);
    client->async_send_request(request, callbackPtr);
    return 1;
  }

  /**
   * @brief response_callback function, calls the response for the call_service
   * function
   *
   * @param future
   */
  void response_callback(sharedFuture future) {
    // Process the response
    RCLCPP_INFO(this->get_logger(), "Got String: %s", future.get()->c.c_str());
    Message = future.get()->c.c_str();
  }
};

/**
 * @brief main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
