/**
 * @file helpers.hpp
 * @brief Helper functions for angle conversions and geometric transformations
 */

// Helper functions
#ifndef RBE500_FINAL_PROJECT_PKG_HELPERS_HPP_
#define RBE500_FINAL_PROJECT_PKG_HELPERS_HPP_

// Standard Core C++ libs
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/**
 * @namespace helpers
 * @brief Namespace containing utility functions for geometric and angular transformations
 */
namespace helpers
{
    /** @brief Function to normalize angle to  make it between -pi to +pi range
     * @param rad: Angle in radians
     * @return Normalized angle in radians
     */
    template <typename T>
    inline T normalize(const T &rad)
    {
        return atan2(sin(rad), cos(rad));
    }

    /** @brief Function to convert degrees to radians
     * @param def: Angle in degrees
     * @return Normalized Angle in radians (-pi to pi range)
     */
    template <typename T>
    inline T getRad(const T &deg)
    {
        return normalize((deg * M_PI) / (180.0));
    }

    /** @brief Function to convert radians to degress
     * @param def: Angle in radians
     * @return Normalized Angle in degrees (-pi to pi range)
     */
    template <typename T>
    inline T getDegrees(const T &rad)
    {
        return ((normalize(rad) * 180.0) / (M_PI));
    }

    /**
     * @brief Converts an Eigen rotation matrix to Roll-Pitch-Yaw angles
     * @details Code has been adapted from OpenManipulatorX_ROS2 project
     * @param rotation Eigen 3x3 rotation matrix
     * @return Eigen::Vector3d containing RPY angles [roll, pitch, yaw]
     * @see https://github.com/hylander2126/OpenManipulatorX_ROS2/blob/main/robotis_manipulator/src/robotis_manipulator/robotis_manipulator_math.cpp
     */
    inline Eigen::Vector3d convertRotationMatrixToRPY(const Eigen::Matrix3d &rotation)
    {
        Eigen::Vector3d rpy; // = Eigen::MatrixXd::Zero(3,1);
        rpy(0) = atan2(rotation.coeff(2, 1), rotation.coeff(2, 2));
        rpy(1) = atan2(-rotation.coeff(2, 0), sqrt(pow(rotation.coeff(2, 1), 2) + pow(rotation.coeff(2, 2), 2)));
        rpy(2) = atan2(rotation.coeff(1, 0), rotation.coeff(0, 0));

        return rpy;
    }

    /**
     * @brief Converts an Eigen Isometry3d transform to a ROS PoseStamped message
     * @param transform The Eigen Isometry3d transform to convert
     * @param header The header information to be included in the PoseStamped message
     * @return geometry_msgs::msg::PoseStamped The converted pose message
     */
    inline geometry_msgs::msg::PoseStamped convertIsometry3dToPoseStamped(const Eigen::Isometry3d &transform, const std_msgs::msg::Header &header)
    {
        // Initialize PoseStamped message
        geometry_msgs::msg::PoseStamped pose_stamped;

        // Set the header with a frame ID and the current time
        pose_stamped.header = header;

        // Set the translation
        pose_stamped.pose.position.x = transform.translation().x();
        pose_stamped.pose.position.y = transform.translation().y();
        pose_stamped.pose.position.z = transform.translation().z();

        // Set the rotation (quaternion)
        Eigen::Quaterniond quat(transform.rotation());
        pose_stamped.pose.orientation.x = quat.x();
        pose_stamped.pose.orientation.y = quat.y();
        pose_stamped.pose.orientation.z = quat.z();
        pose_stamped.pose.orientation.w = quat.w();

        return pose_stamped;
    }

    /**
     * @brief Converts a ROS Pose message to an Eigen Isometry3d transform
     * @param pose The input ROS Pose message to convert
     * @return Eigen::Isometry3d The converted Eigen transform
     */
    inline Eigen::Isometry3d convertPosetoIsometry3d(const geometry_msgs::msg::Pose &pose)
    {
        Eigen::Isometry3d e_pose = Eigen::Isometry3d::Identity();
        // create transaltion vector
        Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);

        // Step 2: Create the rotation quaternion
        Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

        e_pose.translation() = translation;
        e_pose.linear() = quaternion.toRotationMatrix();
        
        return e_pose;
    }
}

#endif