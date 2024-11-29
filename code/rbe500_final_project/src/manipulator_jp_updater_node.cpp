#include <rbe500_final_project/manipulator_jp_updater_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char **argv)
{
  //init the rclcpp
  rclcpp::init(argc, argv);

  // create a susbcriber node using Manipulator class
  auto node = std::make_shared<manipulator::ManipulatorJPUpdaterROS>();

  RCLCPP_INFO(node->get_logger(), "Manipulator Joint Position updater Node started!");

  //spin the node to get the msgs over the subscriber and wait here
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}