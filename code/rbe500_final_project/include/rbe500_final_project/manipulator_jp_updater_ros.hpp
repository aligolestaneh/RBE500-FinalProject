/**
 * @file manipulator_jp_updater_ros.hpp
 * @brief ROS 2 wrapper for updatting joint angles are per joint velocity mapping
 * @details This node subscribes to a reference joint velocities and calculated target joint angles.
 *          It calls the OpenManipualtorX joint space service
 *          to move the manipulator to the target joint angles.
 * @authors Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
 */

#ifndef RBE500_FINAL_PROJECT_PKG_MANIPULATOR_JP_UPDATER_ROS_HPP_
#define RBE500_FINAL_PROJECT_PKG_MANIPULATOR_JP_UPDATER_ROS_HPP_

// Standard Core C++ libs
#include <memory>
#include <string>
#include <cmath>

// ROS 2 core inlcudes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <open_manipulator_msgs/srv/set_joint_position.hpp>

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

    private:
        /**
         * @brief Initializes the node subscribers
         */
        void initSubscribers();

        /**
         * @brief Initializes the node service clients
         */
        void initServiceClients();

        /**
         * @brief Joint angle subscriber
         * @details Subscribes to /joint_angles topic and receives joint angles in radians
         */
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

        /** @brief Service client for joint position service */
        rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr joint_position_client_;

        /**
         * @brief Callback for joint velocity subscriber to update joint angles
         * @param input_msg The incoming joint velocity message
         */
        void onSubscriberJointVelocityCB(sensor_msgs::msg::JointState::ConstSharedPtr input_msg);

        /**
         * @brief Moves the manipulator to a specified joint position
         * @param joint_angles The target joint angles
         * @return true if the service call was successful, false otherwise
         */
        bool moveToJointPosition(const std::vector<double> &joint_angles, const double &duration);

        rclcpp::Time last_msg_time_;
        bool first_msg_;
        std::vector<double> joint_angles_;
    }; // ManipulatorJjoinPositionUpdater
}
#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_JP_UPDATER_ROS_HPP_
