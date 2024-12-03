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
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
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

    /**@brief Converts a ROS Pose message to a ROS Marker message
     * @param pose The input ROS Pose message to convert
     * @param ns The namespace for the marker
     * @param id The ID for the marker
     * @param marker_type The type of marker to create ("arrow", "sphere", "cube")
     * @return visualization_msgs::msg::Marker The converted marker message
     */
    inline visualization_msgs::msg::Marker convertPoseToMarker(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header, std::string ns = "pose_marker", int id = 0, std::string marker_type = "sphere")
    {
        // Create the marker
        visualization_msgs::msg::Marker marker;

        // Set frame ID and namespace
        marker.header = header;
        marker.ns = ns;
        marker.id = id;

        // Set the marker type
        if (marker_type == "arrow")
        {
            marker.type = visualization_msgs::msg::Marker::ARROW;
        }
        else if (marker_type == "sphere")
        {
            marker.type = visualization_msgs::msg::Marker::SPHERE;
        }
        else if (marker_type == "cube")
        {
            marker.type = visualization_msgs::msg::Marker::CUBE;
        }
        else
        {
            throw std::invalid_argument("Unsupported marker type");
        }

        // Set pose (position and orientation)
        marker.pose = pose;

        // Set default scale (size of the marker)
        marker.scale.x = 0.03; // For arrows, x defines shaft diameter
        marker.scale.y = 0.03; // For arrows, y defines head diameter
        marker.scale.z = 0.03; // For arrows, z defines head length

        // Set default color (Green, RGBA)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Fully opaque

        // Set marker action
        marker.action = visualization_msgs::msg::Marker::ADD;

        return marker;
    }

    /**
     * @brief Converts a std::vector to an Eigen::VectorXd
     * @param vec The input std::vector to convert
     * @return Eigen::VectorXd The converted Eigen vector
     */
    inline Eigen::VectorXd convertStdVectorToEigenVector(const std::vector<double> &vec)
    {
        Eigen::VectorXd eigen_vec(vec.size());
        for (size_t i = 0; i < vec.size(); i++)
        {
            eigen_vec(i) = vec[i];
        }
        return eigen_vec;
    }

    /**
     * @brief Converts an Eigen::Vector6d to a geometry_msgs::msg::Twist message
     * @param vec The input Eigen::Vector6d containing [vx, vy, vz, roll, pitch, yaw]
     * @return geometry_msgs::msg::Twist The converted Twist message
     */
    inline geometry_msgs::msg::Twist convertEigenVector6dToTwist(const Eigen::VectorXd &vec)
    {
        geometry_msgs::msg::Twist twist_msg;
        // vec  = Eigen::VectorXd(6);
        // Set linear velocity (vx, vy, vz)
        twist_msg.linear.x = vec(0); // vx
        twist_msg.linear.y = vec(1); // vy
        twist_msg.linear.z = vec(2); // vz

        // Set angular velocity (roll, pitch, yaw)
        twist_msg.angular.x = vec(3); // roll
        twist_msg.angular.y = vec(4); // pitch
        twist_msg.angular.z = vec(5); // yaw

        return twist_msg;
    }

    /**
     * @brief Converts a geometry_msgs::msg::Twist message to an Eigen::Vector6d
     * @param twist_msg The input Twist message containing linear and angular velocities
     * @return Eigen::Vector6d The converted Eigen vector
     */
    inline Eigen::VectorXd convertTwistToEigenVector6d(const geometry_msgs::msg::Twist &twist_msg)
    {
        Eigen::VectorXd vec = Eigen::VectorXd(6);

        // Set linear velocity (vx, vy, vz)
        vec(0) = twist_msg.linear.x; // vx
        vec(1) = twist_msg.linear.y; // vy
        vec(2) = twist_msg.linear.z; // vz

        // Set angular velocity (roll, pitch, yaw)
        vec(3) = twist_msg.angular.x; // roll
        vec(4) = twist_msg.angular.y; // pitch
        vec(5) = twist_msg.angular.z; // yaw

        return vec;
    }

    /**
     * @brief Converts an Eigen::VectorXd to a std::vector<double>
     * @param eigen_vec The input Eigen::VectorXd to convert
     * @return std::vector<double> The converted std::vector
     */
    inline std::vector<double> convertEigenVectorToStdVector(const Eigen::VectorXd &eigen_vec)
    {
        std::vector<double> vec(eigen_vec.size());
        for (long int i = 0; i < eigen_vec.size(); i++)
        {
            vec[i] = eigen_vec(i);
        }
        return vec;
    }

    /**
     * @brief Clamps the input as per the limit
     * @param value: value to be clamped
     * @param limit: absolute value of the limit
     * @return clamped_value
     */
    inline double clamp(const double &value, const double &limit)
    {
        double clamped_value = value;

        if (value >= std::fabs(limit))
            clamped_value = std::fabs(limit);
        else if (value <= -1.0 * std::fabs(limit))
            clamped_value = -1.0 * std::fabs(limit);

        return clamped_value;
    }
}

#endif