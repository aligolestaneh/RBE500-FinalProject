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
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace manipulator
{
    class ManipulatorROS : public rclcpp::Node
    {

    public:
        // constructor
        ManipulatorROS();
        // destructor
        ~ManipulatorROS() = default;

    private:
        /** @brief to initlize the node parameters */
        void initNodeParams();
        /** @brief to initlize the node subscribers */
        void initSubscribers();
        /** @brief to initlize the node publishers */
        void initPublishers();
        /** @brief to initlize the node services */
        void initServices();

        /** @brief Joint angle subscriber TODO: Update type
         * @brief Subscribes to /joint_angles topic of type Vector3
         * @brief q1,q2,q3 joint angles are mapped with x,y,z args of this topic
         * @brief Gets joint angle in degrees
         */
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr joint_sub_;

        /** @brief End Effector Pose Publisher
         * @brief Publishes the /end_effector_pose topic of type geometry_msgs/Pose
         */
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;

        /** @brief End Effector Twist subscriber
         * @brief Subscribes to /twist topic of type geometry_msgs/Twist
         */
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

        /** @brief Joint Velocities subscriber
         * @brief Subscribes to /joint_velocities topic of type std_msgs/Float32MultiArray
         */
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_vel_sub_;

        /** @brief Joint angle Subscriber callback to process End effector pose
         */
        void onSubscriberJointAngleCB(geometry_msgs::msg::Vector3::ConstSharedPtr input_msg);

        /** @brief TODO: Declare the Services */
        // rclcpp::Service<manipulator_msgs::srv::AddThreeInts>::SharedPtr template_service_server_;

        
        // void onServiceCB(const std::shared_ptr<manipulator_msgs::srv::AddThreeInts::Request> request,
        //                  std::shared_ptr<manipulator_msgs::srv::AddThreeInts::Response> response);

        /**@brief manipulator_core object */
        std::shared_ptr<manipulator::ManipulatorCore> manipulator_;

        Eigen::VectorXd link_length_;

        int ik_max_iteration_;
        double ik_tolerance_;
        bool use_newton_raphson_ik_;

    }; // Manipulator
}

#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_ROS_HPP_
