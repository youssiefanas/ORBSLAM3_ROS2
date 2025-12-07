#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-inertial-node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MonocularInertialSlamNode>();
  std::cout << "============================ " << std::endl;

  rclcpp::spin(node);

  std::cout << "Exiting..." << std::endl;

  // Node destructor is called here when node goes out of scope or is reset
  node.reset();
  std::cout << "Node destroyed." << std::endl;

  rclcpp::shutdown();

  return 0;
}