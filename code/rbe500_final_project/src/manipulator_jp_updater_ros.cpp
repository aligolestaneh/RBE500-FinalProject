#include "rbe500_final_project/manipulator_jp_updater_ros.hpp"

using namespace manipulator;

ManipulatorJPUpdaterROS::ManipulatorJPUpdaterROS() : rclcpp::Node("manipulator_jp_updater")
{
    last_msg_time_ = this->get_clock()->now();
    first_msg_ = true;
    target_joint_angles_ = std::vector<double>(4, 0.0);
    current_joint_angles_ = std::vector<double>(4, 0.0);

    initServiceClients();
    initSubscribers();
}

void ManipulatorJPUpdaterROS::initSubscribers()
{
    target_joint_sub_ = this->create_subscription<rbe500_final_project_msgs::msg::JointVelocity>(
        "target_joint_velocities", 1, std::bind(&ManipulatorJPUpdaterROS::onSubscriberTargetJointVelocityCB, this, std::placeholders::_1));
    current_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, std::bind(&ManipulatorJPUpdaterROS::onSubscriberJointAngleCB, this, std::placeholders::_1));
}

void ManipulatorJPUpdaterROS::initServiceClients()
{
    joint_position_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("/goal_joint_space_path");
}
void ManipulatorJPUpdaterROS::onSubscriberJointAngleCB(sensor_msgs::msg::JointState::ConstSharedPtr input_msg)
{
    // TODO this section
    current_joint_angles_[0] = input_msg->position.at(0);
    current_joint_angles_[1] = input_msg->position.at(1);
    current_joint_angles_[2] = input_msg->position.at(2);
    current_joint_angles_[3] = input_msg->position.at(3);
}
void ManipulatorJPUpdaterROS::onSubscriberTargetJointVelocityCB(rbe500_final_project_msgs::msg::JointVelocity::ConstSharedPtr input_msg)
{
    // rclcpp::Time current_time(input_msg->header.stamp);
    rclcpp::Time current_time(input_msg->header.stamp);

    if (first_msg_)
    {
        last_msg_time_ = current_time; // input_msg->header.stamp;
        first_msg_ = false;
        return;
    }
    if (input_msg->velocity.size() != 4)
    {
        RCLCPP_ERROR(this->get_logger(), "Insufficient Joint velocity or Position published");
        return;
    }

    rclcpp::Duration sample_time = current_time - last_msg_time_;

    for (size_t i = 0; i < input_msg->velocity.size(); i++)
        target_joint_angles_[i] = current_joint_angles_.at(i) + input_msg->velocity.at(i) * sample_time.seconds();

    this->moveToJointPosition(target_joint_angles_, sample_time.seconds());
    last_msg_time_ = input_msg->header.stamp;
}

bool ManipulatorJPUpdaterROS::moveToJointPosition(const std::vector<double> &joint_angles, const double &duration)
{
    auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    request->planning_group = "";
    request->joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4"};
    request->joint_position.position = joint_angles;
    request->path_time = duration; // 2.0;

    auto future = joint_position_client_->async_send_request(request);
    return rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
           rclcpp::FutureReturnCode::SUCCESS;
}