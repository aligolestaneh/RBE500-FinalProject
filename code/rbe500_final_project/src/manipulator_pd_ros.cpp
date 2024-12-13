#include "rbe500_final_project/manipulator_pd_ros.hpp"
using namespace manipulator;
ManipulatorPDROS::ManipulatorPDROS() : Node("manipulator_pd_ros")
{

    first_loop_ = true;
    // init pd
    pd_controller_ = std::make_shared<ManipulatorPD>();
    // Load configuration
    initNodeParams();
    initPublishers();
    initClients();

    while (!joint_position_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /joint_position_client_ service");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for /joint_position_client_ service... ");
    }
    while (!joint_current_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /joint_current_client_ service");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for /joint_current_client_ service... ");
    }

    RCLCPP_INFO(this->get_logger(), "All clients setup! Ready to move Joint4!");
}
void ManipulatorPDROS::initPublishers()
{
    target_current_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetCurrent>("/set_current", 10);
    output_viz_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/pd_output_viz", 10);
}
void ManipulatorPDROS::initClients()
{
    joint_current_client_ = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetCurrent>("/get_current");
    joint_position_client_ = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("/get_position");
}
void ManipulatorPDROS::initNodeParams()
{
    joint_id_ = this->declare_parameter("joint_id", 14);
    this->get_parameter("joint_id", joint_id_);

    target_joint_angle_ = this->declare_parameter("target_joint_angle", 2000);
    this->get_parameter("target_joint_angle", target_joint_angle_);

    kp_ = this->declare_parameter("kp", 0.0);
    this->get_parameter("kp", kp_);

    kd_ = this->declare_parameter("kd", 0.0);
    this->get_parameter("kd", kd_);

    std::vector<long int> current_limits = this->declare_parameter("current_limits", std::vector<long int>{-50, 50});
    this->get_parameter("current_limits", current_limits);

    // target_angles_ = this->declare_parameter("target_joint_angles", std::vector<long int>{1050, 1500, 2000, 1050});
    // this->get_parameter("target_joint_angles", target_angles_);

    min_current_limit_ = current_limits[0];
    max_current_limit_ = current_limits[1];

    pd_controller_->setup(kp_, kd_);
    double setpoint = static_cast<double>(target_joint_angle_);
    pd_controller_->setReference(setpoint);

    RCLCPP_INFO(this->get_logger(), "All Node Parameters Loaded");
}

bool ManipulatorPDROS::callJointPositionService(int &joint_angle)
{
    auto request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
    request->id = joint_id_; // static_cast<uint8_t>(joint_id_);

    auto future = joint_position_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();

        joint_angle = result->position;
        // RCLCPP_INFO(this->get_logger(), "ID[%d] JP CurrentPosition: %d", joint_id_,current_joint_angle_);
        RCLCPP_INFO_STREAM(this->get_logger(), "ID: [" << joint_id_ << "] JP CurrentPosition: " << joint_angle);
        return true;
    }
    return false;
}

bool ManipulatorPDROS::callCurrentService(int &current)
{
    auto request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetCurrent::Request>();
    request->id = joint_id_; // static_cast<uint8_t>(joint_id_);

    auto future = joint_current_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();

        current = result->current;
        // RCLCPP_INFO(this->get_logger(), "ID[%d] JP CurrentPosition: %d", joint_id_,current_joint_angle_);
        RCLCPP_INFO_STREAM(this->get_logger(), "ID: [" << joint_id_ << "] JP CurrentAMP: " << current);
        return true;
    }
    return false;
}

void ManipulatorPDROS::moveManipulator()
{
    if (first_loop_)
    {
        last_time_ = this->get_clock()->now();
        first_loop_ = false;
        return;
    }
    // get current positomn
    if (callJointPositionService(current_joint_angle_))
    {
        sample_time_ = (this->get_clock()->now() - last_time_).seconds();
        last_time_ = this->get_clock()->now();
        // apply pid
        double current_joint = static_cast<double>(current_joint_angle_);
        if (pd_controller_->updateJoint(current_joint, sample_time_))
        {
            // get output curent
            dynamixel_sdk_custom_interfaces::msg::SetCurrent control_;
            control_.id = joint_id_;
            control_.current = static_cast<int32_t>(pd_controller_->getControl());

            // publihs the current
            geometry_msgs::msg::PointStamped viz_output;
            viz_output.header.stamp = last_time_;
            viz_output.point.x = current_joint_angle_;
            viz_output.point.y = pd_controller_->getControl();
            viz_output.point.z = target_joint_angle_;
            // if (callCurrentService(current_amp_))
            // {
            //     viz_output.point.z = static_cast<double>(current_amp_);
            // }
            // else
            //     viz_output.point.z = 0.0;

            target_current_pub_->publish(control_);
            output_viz_pub_->publish(viz_output);

            RCLCPP_INFO(this->get_logger(), "Publishied data!");
        }
    }
}
