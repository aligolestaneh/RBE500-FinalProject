/**
 * @file manipulator_ik_ros.hpp
 * @brief ROS 2 wrapper for manipulator inverse kinematics calculations
 * @details This node provides a ROS 2 service interface for inverse kinematics calculations.
 *          It uses the manipulator_core library to compute joint angles for a given 
 *          end-effector pose received through service calls.
 * @authors Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
 */

#ifndef RBE500_FINAL_PROJECT_PKG_MANIPULATOR_ROS_HPP_
#define RBE500_FINAL_PROJECT_PKG_MANIPULATOR_ROS_HPP_

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
//Custom server msg
#include "rbe500_final_project_msgs/srv/get_joint_angles.hpp"


namespace manipulator
{
    /**
     * @class ManipulatorIKROS
     * @brief ROS 2 node for inverse kinematics service
     * @details Provides a service interface to compute joint angles for a given end-effector pose
     */
    class ManipulatorIKROS : public rclcpp::Node
    {

    public:
        /**
         * @brief Constructor for ManipulatorIKROS
         */
        ManipulatorIKROS();
        /**
         * @brief Default destructor
         */
        ~ManipulatorIKROS() = default;

    private:
        /**
         * @brief Initializes the node parameters
         */
        void initNodeParams();
       
        /**
         * @brief Initializes the node services
         */
        void initServices();

        /** @brief Service server for inverse kinematics calculations */
        rclcpp::Service<rbe500_final_project_msgs::srv::GetJointAngles>::SharedPtr ik_server_;

        /**
         * @brief Callback function for the inverse kinematics service
         * @param request The service request containing the target end-effector pose
         * @param response The service response containing the computed joint angles
         */
        void onServiceCB(const std::shared_ptr<rbe500_final_project_msgs::srv::GetJointAngles::Request> request,
                         std::shared_ptr<rbe500_final_project_msgs::srv::GetJointAngles::Response> response);

        /** @brief Pointer to the manipulator core library instance */
        std::shared_ptr<manipulator::ManipulatorCore> manipulator_;

        /** @brief Vector containing link lengths of the manipulator */
        Eigen::VectorXd link_length_;

        /** @brief Vector containing joint angles */
        Eigen::Vector4d joint_angles_;

        /** @brief Matrix representing end-effector pose */
        Eigen::Isometry3d end_effector_pose_;

        /** @brief Maximum iterations for inverse kinematics calculation */
        int ik_max_iteration_;

        /** @brief Tolerance threshold for inverse kinematics convergence */
        double ik_tolerance_;

        /** @brief Flag to use Newton-Raphson method for inverse kinematics */
        bool use_newton_raphson_ik_;

    }; // ManipulatorIKROS
} // namespace manipulator

#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_ROS_HPP_
