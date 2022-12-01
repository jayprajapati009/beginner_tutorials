/**
 * @file modify_msg_server.cpp
 * @author Jay Prajapati (jayp@umd.edu)
 * @brief Service server defination file
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <string>
#include <vector>
#include <cstdlib>
#include <iterator>
#include <memory>

#include <cpp_pubsub/srv/modify_msg.hpp>
#include <rclcpp/rclcpp.hpp>


using ModifyMsg = cpp_pubsub::srv::ModifyMsg;

/**
 * @brief Function to process the request, i.e., to promt a msg that the string
 * is modified.
 *
 * @param request
 * @param response
 */
void add(const std::shared_ptr<ModifyMsg::Request> request,
         std::shared_ptr<ModifyMsg::Response> response) {
  response->c = request->a + " " + request->b + " have been added.";
}

/**
 * @brief main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("modify_msg_server");

  rclcpp::Service<ModifyMsg>::SharedPtr service =
      node->create_service<ModifyMsg>("modify_msg", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Modifying Msg");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
