#include <rbe500_final_project/manipulator_pd_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char **argv)
{
  // init the rclcpp
  rclcpp::init(argc, argv);

  // create a susbcriber node using Manipulator class
  auto node = std::make_shared<manipulator::ManipulatorPDROS>();

  RCLCPP_INFO(node->get_logger(), "Manipulator PD ROS Node started!");
  rclcpp::Rate loop_rate(20);
  while (rclcpp::ok())
  {
    //move manipualtor here
    node->moveManipulator();
    rclcpp::spin_some(node); // Process any pending callbacks
    loop_rate.sleep();       // Sleep to control the loop rate
  }
  rclcpp::shutdown();

  return 0;
}