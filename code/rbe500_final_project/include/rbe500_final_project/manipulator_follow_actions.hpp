/**
 * @file manipulator_follow_actions.hpp
 * @brief Header file for manipulator action sequence control
 * @details Provides functionality to control a robotic manipulator through predefined
 *          action sequences loaded from YAML configuration files
 * @authors Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
 */

#ifndef MANIPULATOR_FOLLOW_ACTIONS_HPP
#define MANIPULATOR_FOLLOW_ACTIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <rbe500_final_project_msgs/srv/get_joint_angles.hpp>
#include <open_manipulator_msgs/srv/set_joint_position.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>
#include <map>
#include "rbe500_final_project/helpers.hpp"

/**
 * @struct ActionSequence
 * @brief Structure to represent a single action in the manipulation sequence
 */
struct ActionSequence {
    std::string type;           /**< Type of action to perform (e.g., "move", "grip") */
    std::string position_name;  /**< Name of the target position from configuration */
    int wait_time;             /**< Time to wait after executing the action (in seconds) */
};

/**
 * @class ManipulatorFollowActions
 * @brief Class to control the manipulator through a sequence of predefined actions
 * @details This class provides functionality to load and execute a sequence of 
 *          manipulation actions defined in a YAML configuration file
 */
class ManipulatorFollowActions : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for ManipulatorFollowActions
     */
    ManipulatorFollowActions();

    /**
     * @brief Execute the sequence of actions defined in the YAML configuration
     */
    void executeActionSequence();

private:
    /** @brief Visualization marker publisher */
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    /** @brief Service client for IK service */
    rclcpp::Client<rbe500_final_project_msgs::srv::GetJointAngles>::SharedPtr ik_client_;
    /** @brief Service client for joint position service */
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr joint_position_client_;
    /** @brief Service client for gripper service */
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr gripper_client_;

    
    /**
     * @brief Loads configuration from a YAML file
     * @param config_path Path to the YAML configuration file
     */
    void loadConfig(const std::string& config_path);

    /**
     * @brief Moves the manipulator to a specified position
     * @param position_name Name of the target position from configuration
     */
    void goToPosition(const std::string& position_name);

    /**
     * @brief Opens the gripper
     */
    void openGripper();

    /**
     * @brief Closes the gripper
     */
    void closeGripper();

    /**
     * @brief Calls the IK service to get the joint angles for a given pose
     * @param pose The target pose
     * @param[out] joint_angles The resulting joint angles
     * @return true if the service call was successful, false otherwise
     */
    bool callIKService(const geometry_msgs::msg::Pose& pose, std::vector<double>& joint_angles);

    /**
     * @brief Moves the manipulator to a specified joint position
     * @param joint_angles The target joint angles
     * @return true if the service call was successful, false otherwise
     */
    bool moveToJointPosition(const std::vector<double>& joint_angles);

    /**
     * @brief Controls the gripper to a specified position
     * @param position The target gripper position
     * @return true if the service call was successful, false otherwise
     */
    bool controlGripper(double position);

    const double DEFAULT_WAIT_TIME=2.0; /**< Default wait time for actions (seconds) */
    std::vector<double> joint_angles_;  /**< Storage for joint angles */
    std::map<std::string, geometry_msgs::msg::Pose> pose_configs_; /**< Storage for pose configurations */
    std::vector<ActionSequence> action_sequence_; /**< Storage for action sequences */
};

#endif