#include "rbe500_final_project/manipulator_jp_updater_ros.hpp"

using namespace manipulator;

ManipulatorJPUpdaterROS::ManipulatorJPUpdaterROS() : rclcpp::Node("manipulator_jp_updater")
{
    last_msg_time_ = this->get_clock()->now();
    first_msg_ = true;
    move_manipulator_ = false;
    target_joint_angles_ = std::vector<double>(4, 0.0);
    current_joint_angles_ = std::vector<double>(4, 0.0);

    initNodeParams();
    initServiceClients();
    initSubscribers();
}
void ManipulatorJPUpdaterROS::initNodeParams()
{
    joint_position_limits_ = this->declare_parameter("joint_position_limits", std::vector<double>{3.11, 1.53, 1.53, 1.53});
    this->get_parameter("joint_position_limits", joint_position_limits_);

    RCLCPP_INFO(this->get_logger(), "joint_position_limits_ Size: %ld", joint_position_limits_.size());

    RCLCPP_INFO(this->get_logger(), "All Node Parameters Loaded");
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
    // Wait for service to be available
    while (!joint_position_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
    }
}
void ManipulatorJPUpdaterROS::onSubscriberJointAngleCB(sensor_msgs::msg::JointState::ConstSharedPtr input_msg)
{
    std::lock_guard<std::mutex> lock(joint_angles_mutex_);
    current_joint_angles_[0] = input_msg->position.at(0);
    current_joint_angles_[1] = input_msg->position.at(1);
    current_joint_angles_[2] = input_msg->position.at(2);
    current_joint_angles_[3] = input_msg->position.at(3);
    // RCLCPP_INFO(this->get_logger(), "Got Cirrent Pose data!!");
}
void ManipulatorJPUpdaterROS::onSubscriberTargetJointVelocityCB(rbe500_final_project_msgs::msg::JointVelocity::ConstSharedPtr input_msg)
{
    rclcpp::Time current_time(input_msg->header.stamp);

    if (first_msg_)
    {
        last_msg_time_ = current_time;
        first_msg_ = false;
        RCLCPP_INFO(this->get_logger(), "Got My first vel data!!");
        return;
    }
    if (input_msg->velocity.size() != 4)
    {
        RCLCPP_ERROR(this->get_logger(), "Insufficient Joint velocity or Position published");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(joint_angles_mutex_);
        sample_time_ = (current_time - last_msg_time_).seconds();
        for (size_t i = 0; i < input_msg->velocity.size(); i++)
        {
            target_joint_angles_[i] = helpers::clamp(current_joint_angles_.at(i) + input_msg->velocity.at(i) * sample_time_, joint_position_limits_[i]);
            
        }
        move_manipulator_ = true;
        last_msg_time_ = current_time; // input_msg->header.stamp;
    }
}

bool ManipulatorJPUpdaterROS::moveToJointPosition(const std::vector<double> &joint_angles, const double &duration)
{
    auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    request->planning_group = "";
    request->joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4"};
    request->joint_position.position = joint_angles;
    request->path_time = duration; // 2.0;

    // Use a timeout and check service readiness
    if (!joint_position_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(this->get_logger(), "Service not available!");
        return false;
    }

    auto future = joint_position_client_->async_send_request(request);
    return rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
           rclcpp::FutureReturnCode::SUCCESS;
    
}

void ManipulatorJPUpdaterROS::moveManipulator()
{
    bool move = false;
    std::vector<double> target_angles;
    double sample_time;
    {
        std::lock_guard<std::mutex> lock(joint_angles_mutex_);
        move = move_manipulator_;
        move_manipulator_ = false;
        target_angles = target_joint_angles_;
        sample_time = sample_time_;
    }
    if (move)
    {
        RCLCPP_INFO(this->get_logger(), "Moving manipulator data!! Delta: %f | Moved: %d", sample_time, move);
        this->moveToJointPosition(target_angles, sample_time);
    }
}