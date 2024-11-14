#include "gtest/gtest.h"
#include "rbe500_final_project/manipulator_core.hpp"

TEST(BasicEigenOperation, vector_sum_test)
{
    Eigen::Vector4d link_length;
    link_length << 1, 2, 3, 4;
    double first_two_elem_sum = link_length.head<2>().sum();
    ASSERT_DOUBLE_EQ(3.0, first_two_elem_sum);
}

TEST(BasicEigenOperation, isometry_pose_xy_dist_test)
{
    Eigen::Isometry3d end_effector_pose;

    // Example: Initialize with some translation and rotation (for demonstration purposes)
    end_effector_pose.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);           // Set translation
    end_effector_pose.rotate(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY())); // Set rotation about Y-axis

    double sqrt_xy = std::hypot(end_effector_pose.translation().x(), end_effector_pose.translation().y());
    double norm_xy = end_effector_pose.translation().head<2>().norm();

    EXPECT_DOUBLE_EQ(sqrt_xy, norm_xy) << "The computed x-y norms do not match! Sqrt value: "
                                       << sqrt_xy << " , Norm xy: " << norm_xy;
    ASSERT_NEAR(sqrt_xy, norm_xy, 1e-3);
}

TEST(BasicEigenOperation, vector_segment_test)
{
    Eigen::Vector4d complete_q_value;
    complete_q_value << 1, 2, 3, 4;
    Eigen::Vector3d sub_q_value;
    sub_q_value << 5, 6, 7;
    complete_q_value.segment<3>(1) = sub_q_value;

    Eigen::Vector4d result_q;
    result_q << 1, 5, 6, 7;

    ASSERT_EQ(result_q, complete_q_value);
}

TEST(BasicEigenOperation, angle_test)
{
    Eigen::Isometry3d pose;
    //     - Translation: [0.040, 0.148, 0.004]
    // - Rotation: in Quaternion [-0.640, 0.300, 0.697, -0.122]
    //             in RPY (radian) [1.569, 0.959, -1.838]
    //             in RPY (degree) [89.921, 54.942, -105.286]

    // Found these values from simualtions
    Eigen::Vector3d translation(0.040, 0.148, 0.004);
    Eigen::Vector3d expected_euler(1.569, 0.959, -1.838);
    // Step 2: Create the rotation quaternion
    Eigen::Quaterniond quaternion(-0.122, -0.640, 0.300, 0.697);

    pose.translation() = translation;
    pose.rotate(quaternion.toRotationMatrix());
    // Eigen::MatrixXd rot = pose.rotation();
    Eigen::Vector3d euler_angles = helpers::convertRotationMatrixToRPY(quaternion.toRotationMatrix());
    
    Eigen::Vector3d euler_diff;
    euler_diff = expected_euler - euler_angles;
    double tolerance = 1e-3;
    for (int i = 0; i < euler_diff.size(); i++)
    {

        EXPECT_LE(std::fabs(euler_diff(i)), tolerance) << "Euler angles are not under tolerance";
    }
    // ASSERT_TRUE(true);
}