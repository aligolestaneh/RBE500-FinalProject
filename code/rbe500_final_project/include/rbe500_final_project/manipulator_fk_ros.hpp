///////////////////////////////////////
// RBE 500
// Final Project
// Authors: Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
//////////////////////////////////////
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
        // constructor
        ManipulatorFKROS();
        // destructor
        ~ManipulatorFKROS() = default;

    private:
        /** @brief to initlize the node parameters */
        void initNodeParams();
        /** @brief to initlize the node subscribers */
        void initSubscribers();
        /** @brief to initlize the node publishers */
        void initPublishers();
     

        /** @brief Joint angle subscriber
         * @brief Subscribes to /joint_angles topic
         * @brief Gets joint angle in radians
         */
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

        /** @brief End Effector Pose Publisher
         * @brief Publishes the /end_effector_pose topic of type geometry_msgs/Pose
         */
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pose_pub_;

        /** @brief Joint angle Subscriber callback to process End effector pose
         */
        void onSubscriberJointAngleCB(sensor_msgs::msg::JointState::ConstSharedPtr input_msg);

        /**@brief manipulator_core object */
        std::shared_ptr<manipulator::ManipulatorCore> manipulator_;

        Eigen::VectorXd link_length_;
        Eigen::Vector4d joint_angles_;
        // Eigen::Isometry3d end_effector_pose_;
        geometry_msgs::msg::PoseStamped end_effector_pose_;

    }; // ManipulatorForwardKinematics
}
#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_FKROS_HPP_
