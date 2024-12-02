/**
 * @file manipulator_vlk_ros.hpp
 * @brief ROS 2 wrapper for manipulator velocity level kinematics
 * @details This node provides a ROS 2 service interface for forward and inverse velocity kinematics calculations.
 *          It uses the manipulator_core library to compute joint velocities for a given
 *          end-effector twist received through service calls and also computes end-effector twist for a given joint velocity
 *          received through service calls
 * @authors Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
 */

#ifndef RBE500_FINAL_PROJECT_PKG_MANIPULATOR_VLK_ROS_HPP_
#define RBE500_FINAL_PROJECT_PKG_MANIPULATOR_VLK_ROS_HPP_

// Standard Core C++ libs
#include <memory>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
// Manipulator core
#include <rbe500_final_project/manipulator_core.hpp>
// ROS 2 core inlcudes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// Custom server msg
#include "rbe500_final_project_msgs/srv/get_end_effector_twist.hpp"
#include "rbe500_final_project_msgs/srv/get_joint_velocities.hpp"

namespace manipulator
{
    /**
     * @class ManipulatorVLKROS
     * @brief ROS 2 node for Velocity level kinematics service
     * @details Provides a service interface to compute joint veloctities and  end-effector twist
     */
    class ManipulatorVLKROS : public rclcpp::Node
    {

    public:
        /**
         * @brief Constructor for ManipulatorIKROS
         */
        ManipulatorVLKROS();
        /**
         * @brief Default destructor
         */
        ~ManipulatorVLKROS() = default;

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
         * @brief Initializes the node services
         */
        void initServices();

        /**
         * @brief Joint angle subscriber
         * @details Subscribes to /joint_angles topic and receives joint angles in radians
         */
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

        /** @brief Service server for Enf Effector twist calculations */
        rclcpp::Service<rbe500_final_project_msgs::srv::GetEndEffectorTwist>::SharedPtr ee_twist_server_;

        /** @brief Service server for Joint velocities calculations */
        rclcpp::Service<rbe500_final_project_msgs::srv::GetJointVelocities>::SharedPtr joint_vel_server_;

        /**
         * @brief Callback for joint angle subscriber to process end effector pose
         * @param input_msg The incoming joint state message
         */
        void onSubscriberJointAngleCB(sensor_msgs::msg::JointState::ConstSharedPtr input_msg);
        
        /**
         * @brief Callback function for the End effector twist service
         * @param request The service request containing the target joint_velocities
         * @param response The service response containing the computed End Effector twist
         */
        void onEETwistServiceCB(const std::shared_ptr<rbe500_final_project_msgs::srv::GetEndEffectorTwist::Request> request,
                                std::shared_ptr<rbe500_final_project_msgs::srv::GetEndEffectorTwist::Response> response);

        /**
         * @brief Callback function for the Joint Velocities service
         * @param request The service request containing the target end-effector twist
         * @param response The service response containing the computed joint velocities
         */
        void onJointVelServiceCB(const std::shared_ptr<rbe500_final_project_msgs::srv::GetJointVelocities::Request> request,
                                 std::shared_ptr<rbe500_final_project_msgs::srv::GetJointVelocities::Response> response);

        /** @brief Pointer to the manipulator core library instance */
        std::shared_ptr<manipulator::ManipulatorCore> manipulator_;

        /** @brief Vector containing link lengths of the manipulator */
        Eigen::VectorXd link_length_;

        /** @brief Vector containing joint angles */
        Eigen::Vector4d joint_angles_;

        // /** @brief Matrix representing end-effector pose */
        // Eigen::Isometry3d end_effector_pose_;

    }; // ManipulatorIKROS
} // namespace manipulator

#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_ROS_HPP_
