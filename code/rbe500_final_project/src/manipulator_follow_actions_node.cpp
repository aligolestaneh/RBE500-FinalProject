#include <rbe500_final_project/manipulator_follow_actions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char **argv)
{
  //init the rclcpp
  rclcpp::init(argc, argv);

  // create a susbcriber node using Manipulator class
  auto node = std::make_shared<ManipulatorFollowActions>();

  RCLCPP_INFO(node->get_logger(), "Manipulator Follow Node started!");
  node->executeActionSequence();
  //spin the node to get the msgs over the subscriber and wait here
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}