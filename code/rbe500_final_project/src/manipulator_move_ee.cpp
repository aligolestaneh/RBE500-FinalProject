#include "rbe500_final_project/manipulator_move_ee.hpp"

ManipulatorMoveEE::ManipulatorMoveEE() : Node("manipulator_move_ee")
{
    // Initialize publisher
    // marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/twist_linear_marker", 10);
    // Load configuration
    initNodeParams();
    initPublishers();
    initClients();

    while (!get_joint_velocity_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /get_joint_velocities service");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for /get_joint_velocities service... ");
    }
    while (!joint_position_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /goal_joint_space_path service");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for /goal_joint_space_path service... ");
    }

    RCLCPP_INFO(this->get_logger(), "All clients setup! Ready to move End Effector!");
}
void ManipulatorMoveEE::initPublishers()
{
    target_joint_vel_pub_ = this->create_publisher<rbe500_final_project_msgs::msg::JointVelocity>("/target_joint_velocities", 10);
}
void ManipulatorMoveEE::initClients()
{
    get_joint_velocity_client_ = this->create_client<rbe500_final_project_msgs::srv::GetJointVelocities>("/get_joint_velocities");
    joint_position_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("/goal_joint_space_path");
}
void ManipulatorMoveEE::initNodeParams()
{
    end_effector_linear_twist_ = this->declare_parameter("end_effectot_twist_linear", std::vector<double>{0.0, 0.1, 0.0});
    this->get_parameter("end_effectot_twist_linear", end_effector_linear_twist_);
     

    home_joint_pos_ = this->declare_parameter("home_joint_pos", std::vector<double>{1.57, -1.57, 0.0,1.57});
    this->get_parameter("home_joint_pos", home_joint_pos_);

    publish_frequency_ = this->declare_parameter("update_frequency", 20.0);
    this->get_parameter("update_frequency", publish_frequency_);

    move_duration_ = this->declare_parameter("move_duration", 5.0);
    this->get_parameter("move_duration", move_duration_);

    RCLCPP_INFO(this->get_logger(), "end_effector_linear_twist_ Size: %ld | Freq: %f | Duration: %f", end_effector_linear_twist_.size(), publish_frequency_, move_duration_);

    RCLCPP_INFO(this->get_logger(), "All Node Parameters Loaded");
}

bool ManipulatorMoveEE::callGetJVService(const geometry_msgs::msg::Twist &ee_twist, std::vector<double> &joint_velocities)
{
    auto request = std::make_shared<rbe500_final_project_msgs::srv::GetJointVelocities::Request>();
    request->end_effector_twist = ee_twist;
    // std_msgs::msg::Header header;
    // header.stamp = this->get_clock()->now();
    // header.frame_id = "end_effector_link";
    // visualization_msgs::msg::Marker marker;
    // geometry_msgs::msg::Pose pose;
    // marker = helpers::convertPoseToMarker(pose, header,"goal_marker");
    // marker_pub_->publish(marker);

    auto future = get_joint_velocity_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->success)
            joint_velocities = result->joint_velocities;
        RCLCPP_INFO(this->get_logger(), "JV Resp: %s", result->msg.c_str());
        return result->success;
    }
    return false;
}

bool ManipulatorMoveEE::moveToJointPosition(const std::vector<double> &joint_angles)
{
    auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    request->planning_group = "";
    request->joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4"};
    request->joint_position.position = joint_angles;
    request->path_time = 2.0;
    RCLCPP_INFO(this->get_logger(), "Sent Joint Com: [%f,%f,%f,%f]", joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3]);
    auto future = joint_position_client_->async_send_request(request);
    return rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
           rclcpp::FutureReturnCode::SUCCESS;
}

void ManipulatorMoveEE::moveEndEffector()
{
    // std::vector<double> joint_angles = std::vector<double>(4, 0.0);
    std::vector<double> joint_velcoities = std::vector<double>(4, 0.0);
    rbe500_final_project_msgs::msg::JointVelocity joint_vel;

    // joint_angles[0] = 1.57;
    // joint_angles[1] = -1.57;//-1.1;//-1.57;
    // joint_angles[2] = 0.0; //0.0;
    // joint_angles[3] = 1.57;//1.1; //1.57;

    

    RCLCPP_INFO(this->get_logger(), "Going to home positon!");
    moveToJointPosition(home_joint_pos_);//(joint_angles);
    rclcpp::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "Going to Calc JOint velocities!");

    if (callGetJVService(helpers::convertStdVectorToTwist(end_effector_linear_twist_), joint_velcoities))
    {
        joint_vel.header.stamp = this->get_clock()->now();
        joint_vel.velocity = joint_velcoities;
        RCLCPP_INFO(this->get_logger(), "Going to Publish JOint velocities for %f seconds!", move_duration_);
        rclcpp::Rate loop_rate(publish_frequency_);

        auto initial_time = this->get_clock()->now();                                 // Capture the initial time
        while ((this->get_clock()->now() - initial_time).seconds() <= move_duration_) // Corrected condition
        {
            if (callGetJVService(helpers::convertStdVectorToTwist(end_effector_linear_twist_), joint_velcoities))
            {
                joint_vel.header.stamp = this->get_clock()->now();
                joint_vel.velocity = joint_velcoities;
            }
            // joint_vel.velocity[3] = -1.0*joint_vel.velocity[3];
            target_joint_vel_pub_->publish(joint_vel);
            // joint_vel.header.stamp = this->get_clock()->now();
            loop_rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Going Back to home pose now!");
        rclcpp::sleep_for(std::chrono::seconds(1));
        moveToJointPosition(home_joint_pos_);//(joint_angles);
    }
    else
        RCLCPP_ERROR(this->get_logger(), "Unable to get the JOint velocities!");
}
