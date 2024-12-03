/**
 * @file manipulator_move_ee.hpp
 * @brief Header file for moving  the manipulator's end effector with a constant twist
 * @details Provides functionality to control openManipualtorX end effector with a
 *          constant twist and for a specific duration, loaded from YAML configuration files.
 *          It calculates the required joint velocities for the given twist angle
 * @authors Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
 */

#ifndef MANIPULATOR_MOVE_EE_HPP
#define MANIPULATOR_MOVE_EE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rbe500_final_project_msgs/srv/get_joint_velocities.hpp>
#include <open_manipulator_msgs/srv/set_joint_position.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <rbe500_final_project_msgs/msg/joint_velocity.hpp>
#include "rbe500_final_project/helpers.hpp"

/**
 * @class ManipulatorMoveEE
 * @brief Class to control the manipulator end effector through a constant twist
 * @details This class provides functionality to publish the joint velocities as per given twist
 */
class ManipulatorMoveEE : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for ManipulatorMoveEE
     */
    ManipulatorMoveEE();

    /**
     * @brief Moves the end effector with a constant velocity
     */
    void moveEndEffector();

private:
    /**
     * @brief Initializes the node parameters
     */
    void initNodeParams();

    /**
     * @brief Initializes the node cSevice client
     */
    void initClients();

    /**
     * @brief Initializes the node publishers
     */
    void initPublishers();

    // /** @brief Visualization marker publisher for linear_twist*/
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    /** @brief Target JointVelocity publisher for each joints*/
    rclcpp::Publisher<rbe500_final_project_msgs::msg::JointVelocity>::SharedPtr target_joint_vel_pub_;

    /** @brief Service client for get_joint_velocity service */
    rclcpp::Client<rbe500_final_project_msgs::srv::GetJointVelocities>::SharedPtr get_joint_velocity_client_;

    /** @brief Service client for joint position service */
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr joint_position_client_;

    /**
     * @brief Calls the get joint velocity service to get the joint velocities for a given ee twist
     * @param pose The target end effector Twist
     * @param[out] joint_velocities The resulting joint velocities
     * @return true if the service call was successful, false otherwise
     */
    bool callGetJVService(const geometry_msgs::msg::Twist &ee_twist, std::vector<double> &joint_velocities);

    /**
     * @brief Moves the manipulator to a specified joint position
     * @param joint_angles The target joint angles
     * @return true if the service call was successful, false otherwise
     */
    bool moveToJointPosition(const std::vector<double> &joint_angles);

    std::vector<double> end_effector_linear_twist_; /**< End effector Twist linear components from the yaml file */
    double publish_frequency_;                      /**< Target velocity publish frequency */
    double move_duration_;                          /**< Time duration for which we need to publish the target velocity */
};

#endif