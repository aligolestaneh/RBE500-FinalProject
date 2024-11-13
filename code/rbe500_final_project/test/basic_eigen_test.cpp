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

// TEST(BasicEigenOperation, angle_test)
// {
//     Eigen::Isometry3d pose;
//     //     - Translation: [0.040, 0.148, 0.004]
//     // - Rotation: in Quaternion [-0.640, 0.300, 0.697, -0.122]
//     //             in RPY (radian) [1.569, 0.959, -1.838]
//     //             in RPY (degree) [89.921, 54.942, -105.286]

//     // Found these values from simualtions
//     Eigen::Vector3d translation(0.040, 0.148, 0.004);
//     Eigen::Vector3d expected_euler(-1.838, 0.959, 1.569);
//     // Step 2: Create the rotation quaternion
//     Eigen::Quaterniond quaternion(-0.122, -0.640, 0.300, 0.697);
//     quaternion.normalize();
//     Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
//     // Convert to euler angles in different sequences to find the matching one
//     Eigen::Vector3d euler_angles_xyz = rotation_matrix.eulerAngles(0, 1, 2); // XYZ
//     Eigen::Vector3d euler_angles_zyx = rotation_matrix.eulerAngles(2, 1, 0); // ZYX
//     Eigen::Vector3d euler_angles_zxy = rotation_matrix.eulerAngles(2, 0, 1); // ZXY
//     std::cout<<"E1:\n"<<euler_angles_xyz<<std::endl;
//     std::cout<<"E2:\n"<<euler_angles_zyx<<std::endl;
//     std::cout<<"E3:\n"<<euler_angles_zxy<<std::endl;

//     double pitch2 = atan2(-rotation_matrix(2,0), 
//                      sqrt(rotation_matrix(2,1)*rotation_matrix(2,1) + rotation_matrix(2,2)*rotation_matrix(2,2)));

//     std::cout<<"Pitch angle[0.959]:  "<<pitch2<<std::endl;
//     pose.translation() = translation;
//     pose.rotate(quaternion.toRotationMatrix());

//     Eigen::Vector3d euler_angles = pose.rotation().eulerAngles(2, 1, 0); // ZYX order: yaw, pitch, roll
//     Eigen::Vector3d euler_angles2 = pose.rotation().eulerAngles(0, 1, 2);

//     double yaw = euler_angles[0];   // Z rotation
//     double pitch = euler_angles[1]; // Y rotation
//     double roll = euler_angles[2];  // X rotation

//     std::cout << "Expected angleY : " << 0.959 << std::endl;
//     std::cout << "Calc Euler: \n"
//               << euler_angles << std::endl;
//     std::cout << "Expected Euler: \n"
//               << expected_euler << std::endl;

//     std::cout << "Expected Euler2: \n"
//               << euler_angles2 << std::endl;

//     EXPECT_DOUBLE_EQ(-1.838, euler_angles[2]) << "Yaw angle should be at index 0";
//     EXPECT_DOUBLE_EQ(0.959, euler_angles[1]) << "Pitch angle should be at index 1";
//     EXPECT_DOUBLE_EQ(1.569, euler_angles[0]) << "Roll angle should be at index 1";
// }