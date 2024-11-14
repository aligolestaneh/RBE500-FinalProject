// manipulator_control.hpp
#ifndef MANIPULATOR_FOLLOW_ACTIONS_HPP
#define MANIPULATOR_FOLLOW_ACTIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <rbe500_final_project_msgs/srv/get_joint_angles.hpp>
#include <open_manipulator_msgs/srv/set_joint_position.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>
#include <map>


struct ActionSequence {
    std::string type;
    std::string position_name;
};

class ManipulatorFollowActions : public rclcpp::Node
{
public:
    ManipulatorFollowActions();

private:
    // Service clients
    rclcpp::Client<rbe500_final_project_msgs::srv::GetJointAngles>::SharedPtr ik_client_;
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr joint_position_client_;
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr gripper_client_;

    // Configuration storage
    std::map<std::string, geometry_msgs::msg::Pose> pose_configs_;
    std::vector<ActionSequence> action_sequence_;

    // Methods
    void loadConfig(const std::string& config_path);
    void executeActionSequence();
    void goToPosition(const std::string& position_name);
    void openGripper();
    void closeGripper();
    bool callIKService(const geometry_msgs::msg::Pose& pose, std::vector<double>& joint_angles);
    bool moveToJointPosition(const std::vector<double>& joint_angles);
    bool controlGripper(double position);

    const double POSITION_WAIT_TIME = 2.0;  // seconds
    const double GRIPPER_WAIT_TIME = 2.0;   // seconds
    std::vector<double> joint_angles_;
};

#endif