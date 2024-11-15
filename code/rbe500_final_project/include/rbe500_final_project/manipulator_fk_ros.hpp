/**
 * @file manipulator_fk_ros.hpp
 * @brief ROS 2 wrapper for manipulator forward kinematics calculations
 * @details This node provides a ROS 2 interface for forward kinematics calculations.
 *          It subscribes to joint angles, uses the manipulator_core library to compute
 *          forward kinematics, and publishes the end-effector pose.
 * @authors Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
 */

#ifndef RBE500_FINAL_PROJECT_PKG_MANIPULATOR_FKROS_HPP_
#define RBE500_FINAL_PROJECT_PKG_MANIPULATOR_FKROS_HPP_

// Standard Core C++ libs
#include <memory>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
// Manipulator core
#include <rbe500_final_project/manipulator_core.hpp>
// ROS 2 core inlcudes
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


namespace manipulator
{
    class ManipulatorFKROS : public rclcpp::Node
    {

    public:
        /**
         * @brief Constructor for the ManipulatorFKROS class
         * @details Initializes the ROS 2 node, sets up parameters, subscribers, and publishers
         */
        ManipulatorFKROS();
        /**
         * @brief Default destructor for the ManipulatorFKROS class
         */
        ~ManipulatorFKROS() = default;

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
         * @brief Initializes the node publishers
         */
        void initPublishers();
     

        /**
         * @brief Joint angle subscriber
         * @details Subscribes to /joint_angles topic and receives joint angles in radians
         */
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

        /**
         * @brief End Effector Pose Publisher
         * @details Publishes to the /end_effector_pose topic of type geometry_msgs/PoseStamped
         */
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pose_pub_;

        /**
         * @brief Callback for joint angle subscriber to process end effector pose
         * @param input_msg The incoming joint state message
         */
        void onSubscriberJointAngleCB(sensor_msgs::msg::JointState::ConstSharedPtr input_msg);

        /** @brief Manipulator core object for kinematics calculations */
        std::shared_ptr<manipulator::ManipulatorCore> manipulator_;

        /** @brief Link length vector used to initialize the manipulator_ object */
        Eigen::VectorXd link_length_;

        /** @brief Current joint angles vector used for FK calculations */
        Eigen::Vector4d joint_angles_;
        
        /** @brief Current end effector pose */
        geometry_msgs::msg::PoseStamped end_effector_pose_;

    }; // ManipulatorForwardKinematics
}
#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_FKROS_HPP_
