#include "rbe500_final_project/manipulator_follow_actions.hpp"
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>

ManipulatorFollowActions::ManipulatorFollowActions() : Node("manipulator_follow_actions")
{
    // Initialize publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);

    // Initialize service clients
    ik_client_ = this->create_client<rbe500_final_project_msgs::srv::GetJointAngles>("/get_joint_angles_ik");
    joint_position_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("/goal_joint_space_path");
    gripper_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("/goal_tool_control");

    // Load configuration
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("rbe500_final_project");
    joint_angles_ = std::vector<double>(4, 0.0);
    // Construct the path to the config file

    std::filesystem::path config_path = std::filesystem::path(package_share_dir) / "params" / "manipulator_actions.yaml";
    if (!std::filesystem::exists(config_path))
    {
        RCLCPP_ERROR(this->get_logger(), "YAML file does not exist at path: %s", config_path.string().c_str());
        return;
    }
    loadConfig(config_path.string());

    while (!ik_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /get_joint_angles_ik service");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for /get_joint_angles_ik service... ");
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
    while (!gripper_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /goal_tool_control service");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for /goal_tool_control service... ");
    }

    RCLCPP_INFO(this->get_logger(), "All clients setup! Ready to execute the commands!");
}

void ManipulatorFollowActions::loadConfig(const std::string &config_path)
{
    YAML::Node config = YAML::LoadFile(config_path);

    // Load poses
    YAML::Node poses = config["poses"];
    for (const auto &pose : poses)
    {
        std::string name = pose.first.as<std::string>();
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = pose.second["position"]["x"].as<double>();
        pose_msg.position.y = pose.second["position"]["y"].as<double>();
        pose_msg.position.z = pose.second["position"]["z"].as<double>();
        pose_msg.orientation.x = pose.second["orientation"]["x"].as<double>();
        pose_msg.orientation.y = pose.second["orientation"]["y"].as<double>();
        pose_msg.orientation.z = pose.second["orientation"]["z"].as<double>();
        pose_msg.orientation.w = pose.second["orientation"]["w"].as<double>();
        pose_configs_[name] = pose_msg;

        RCLCPP_INFO(this->get_logger(), "Got Pose: %s", name.c_str());
    }

    // Load action sequence
    YAML::Node actions = config["actions"];
    // for (const auto &action : actions)
    // {
    //     ActionSequence seq;
    //     seq.type = action["type"].as<std::string>();
    //     if (seq.type == "go_to_position")
    //     {
    //         seq.position_name = action["position"].as<std::string>();
    //         RCLCPP_INFO_STREAM(this->get_logger(), "Action Pos:" << seq.position_name);
    //     }

    //     action_sequence_.push_back(seq);
    //     RCLCPP_INFO_STREAM(this->get_logger(), "Action type:" << seq.type<<"\n-----");
    // }
    for (const auto &action : actions)
    {
        ActionSequence seq;
        if (action["type"] && action["type"].IsDefined())
        {
            seq.type = action["type"].as<std::string>();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Action 'type' is missing, skipping this action.");
            continue; // Skip this action if "type" is missing
        }

        // If "type" is "go_to_position", check if "position" is present
        if (seq.type == "go_to_position")
        {
            if (action["position"] && action["position"].IsDefined())
            {
                seq.position_name = action["position"].as<std::string>();
                RCLCPP_INFO_STREAM(this->get_logger(), "Action Pos:" << seq.position_name);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Action 'position' is missing for type 'go_to_position'.");
                continue; // Skip this action if "position" is missing
            }
        }
        if (action["wait"] && action["wait"].IsDefined())
        {
            seq.wait_time = action["wait"].as<int>();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Action 'wait' is missing, using default wait %f secs", DEFAULT_WAIT_TIME);
        }
        action_sequence_.push_back(seq);
        RCLCPP_INFO_STREAM(this->get_logger(), "Added Action type:" << seq.type << "\n-----");
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Done reading data:  " << action_sequence_.size());
}

void ManipulatorFollowActions::executeActionSequence()
{
    for (const auto &action : action_sequence_)
    {
        if (action.type == "go_to_position")
        {
            RCLCPP_INFO(this->get_logger(), "Executing go_to_position: %s", action.position_name.c_str());
            goToPosition(action.position_name);
        }
        else if (action.type == "open_gripper")
        {
            RCLCPP_INFO(this->get_logger(), "Executing open_gripper");
            openGripper();
        }
        else if (action.type == "close_gripper")
        {
            RCLCPP_INFO(this->get_logger(), "Executing close_gripper");
            closeGripper();
        }
        // rclcpp::sleep_for(std::chrono::duration<double>(2.0));
        rclcpp::sleep_for(std::chrono::seconds(action.wait_time));
    }
}

void ManipulatorFollowActions::goToPosition(const std::string &position_name)
{
    if (pose_configs_.find(position_name) == pose_configs_.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Position %s not found in configuration", position_name.c_str());
        return;
    }

    if (callIKService(pose_configs_[position_name], joint_angles_))
    {
        RCLCPP_INFO(this->get_logger(), "Got Joint Angles as: [%f, %f, %f, %f]", joint_angles_[0], joint_angles_[1], joint_angles_[2], joint_angles_[3]);
        moveToJointPosition(joint_angles_);
    }
}

bool ManipulatorFollowActions::callIKService(const geometry_msgs::msg::Pose &pose, std::vector<double> &joint_angles)
{
    auto request = std::make_shared<rbe500_final_project_msgs::srv::GetJointAngles::Request>();
    request->end_effector_pose = pose;
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "link1";
    visualization_msgs::msg::Marker marker;
    marker = helpers::convertPoseToMarker(pose, header,"goal_marker");
    marker_pub_->publish(marker);

    auto future = ik_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        joint_angles = result->joint_angles;
        return true;
    }
    return false;
}

bool ManipulatorFollowActions::moveToJointPosition(const std::vector<double> &joint_angles)
{
    auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    request->planning_group = "";
    request->joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4"};
    request->joint_position.position = joint_angles;
    request->path_time = 2.0;

    auto future = joint_position_client_->async_send_request(request);
    return rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
           rclcpp::FutureReturnCode::SUCCESS;
}

void ManipulatorFollowActions::openGripper()
{
    controlGripper(0.01);
}

void ManipulatorFollowActions::closeGripper()
{
    controlGripper(-0.01);
}

bool ManipulatorFollowActions::controlGripper(double position)
{
    auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    request->planning_group = "";
    request->joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4", "gripper"};
    request->joint_position.position = {joint_angles_[0], joint_angles_[1], joint_angles_[2], joint_angles_[3], position};
    request->path_time = 5.0;

    auto future = gripper_client_->async_send_request(request);
    return rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
           rclcpp::FutureReturnCode::SUCCESS;
}
