/**
 * @file manipulator_jp_updater_ros.hpp
 * @brief ROS 2 wrapper for updatting joint angles are per joint velocity mapping
 * @details This node subscribes to a reference joint velocities and calculated target joint angles.
 *          It calls the OpenManipualtorX joint space service
 *          to move the manipulator to the target joint angles.
 *          It will clamp the updated the joint_position based on join position limits, to avoid
 *          conflicts with OpenManipulatorX.
 * @authors Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
 */

#ifndef RBE500_FINAL_PROJECT_PKG_MANIPULATOR_JP_UPDATER_ROS_HPP_
#define RBE500_FINAL_PROJECT_PKG_MANIPULATOR_JP_UPDATER_ROS_HPP_

// Standard Core C++ libs
#include <memory>
#include <string>
#include <cmath>
#include <mutex>
#include <chrono>

// ROS 2 core inlcudes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <open_manipulator_msgs/srv/set_joint_position.hpp>
#include <rbe500_final_project_msgs/msg/joint_velocity.hpp>
#include <rbe500_final_project/helpers.hpp>

namespace manipulator
{
    class ManipulatorJPUpdaterROS : public rclcpp::Node
    {

    public:
        /**
         * @brief Constructor for the ManipulatorJPUpdaterROS class
         * @details Initializes the ROS 2 node, sets up parameters, subscribers, and publishers
         */
        ManipulatorJPUpdaterROS();
        /**
         * @brief Default destructor for the ManipulatorJPUpdaterROS class
         */
        ~ManipulatorJPUpdaterROS() = default;

        /**
         * @brief Moves the manipulator to calculated target joint positions
         * Its timer callback which runs at a frequency provided in the params
         */
        void moveManipulator();

    private:
        /**
         * @brief Initializes the node parameters
         */
        void initNodeParams();

        /**
         * @brief Initializes the node subscribers
         */
        void initSubscribers();

        /**
         * @brief Initializes the node service clients
         */
        void initServiceClients();

        /**
         * @brief Target Joint velocity subscriber
         * @details Subscribes to /target_joint_velocities topic and receives joint velociteis in radians/second
         */
        rclcpp::Subscription<rbe500_final_project_msgs::msg::JointVelocity>::SharedPtr target_joint_sub_;

        /**
         * @brief Joint angle subscriber
         * @details Subscribes to /joint_angles topic and receives joint angles in radians
         */
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_sub_;

        /** @brief Service client for joint position service */
        rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr joint_position_client_;

        /**
         * @brief Callback for target joint velocity subscriber to update joint angles
         * @param input_msg The incoming joint velocity message
         */
        void onSubscriberTargetJointVelocityCB(rbe500_final_project_msgs::msg::JointVelocity::ConstSharedPtr input_msg);

        /**
         * @brief Callback for current joint angle subscriber to update joint angles
         * @param input_msg The incoming joint state message
         */
        void onSubscriberJointAngleCB(sensor_msgs::msg::JointState::ConstSharedPtr input_msg);

        /**
         * @brief Moves the manipulator to a specified joint position
         * @param joint_angles The target joint angles
         * @return true if the service call was successful, false otherwise
         */
        bool moveToJointPosition(const std::vector<double> &joint_angles, const double &duration);

        /**
         * @brief Timestamp of the last received joint state
         */
        rclcpp::Time last_msg_time_;

        /**
         * @brief Flag indicating if this is the first joint state received
         */
        bool first_msg_;

        /**
         * @brief Target joint angles for the manipulator
         */
        std::vector<double> target_joint_angles_;

        /**
         * @brief Current joint angles of the manipulator
         */
        std::vector<double> current_joint_angles_;

        /**
         * @brief Joint position limits for the manipulator
         */
        std::vector<double> joint_position_limits_;

        /**
         * @brief Mutex for protecting access to joint angles
         */
        std::mutex joint_angles_mutex_;

        /**
         * @brief Flag indicating whether to move the manipulator
         */
        bool move_manipulator_;

        /**
         * @brief Sample time for the manipulator's movement
         */
        double sample_time_;

    }; // ManipulatorJjoinPositionUpdater
}
#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_JP_UPDATER_ROS_HPP_
