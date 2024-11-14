///////////////////////////////////////
// RBE 500
// Final Project
// Authors: Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
//////////////////////////////////////
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
    class ManipulatorIKROS : public rclcpp::Node
    {

    public:
        // constructor
        ManipulatorIKROS();
        // destructor
        ~ManipulatorIKROS() = default;

    private:
        /** @brief to initlize the node parameters */
        void initNodeParams();
       
        /** @brief to initlize the node services */
        void initServices();

       
        /** @brief TODO: Declare the Services */
        // rclcpp::Service<rbe500_final_project_msgs::srv::GetJointAngles>

        rclcpp::Service<rbe500_final_project_msgs::srv::GetJointAngles>::SharedPtr ik_server_;
        void onServiceCB(const std::shared_ptr<rbe500_final_project_msgs::srv::GetJointAngles::Request> request,
                         std::shared_ptr<rbe500_final_project_msgs::srv::GetJointAngles::Response> response);

        /**@brief manipulator_core object */
        std::shared_ptr<manipulator::ManipulatorCore> manipulator_;

        Eigen::VectorXd link_length_;
        Eigen::Vector4d joint_angles_;
        Eigen::Isometry3d end_effector_pose_;

        int ik_max_iteration_;
        double ik_tolerance_;
        bool use_newton_raphson_ik_;

    }; // Manipulator
}

#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_ROS_HPP_
