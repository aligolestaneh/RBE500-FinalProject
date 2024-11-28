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

TEST(BasicEigenOperation, cross_product)
{

    Eigen::Vector3d v1(1.0, 2.0, 3.0);
    Eigen::Vector3d v2(4.0, 5.0, 6.0);

    // Compute the cross product
    Eigen::Vector3d result = v1.cross(v2);

    Eigen::Vector3d expected_result(-3.0, 6.0, -3.0);
    double tolerance = 1e-3;

    ASSERT_TRUE(expected_result.isApprox(result, tolerance)) << " Cross product is not equal";
}

TEST(BasicEigenOperation, pseudo_inverse)
{

    Eigen::MatrixXd jacobian = Eigen::MatrixXd(6, 4);

    jacobian << 1.0, 2.0, 3.0, 4.0,
        5.0, 6.0, 7.0, 8.0,
        9.0, 10.0, 11.0, 12.0,
        13.0, 14.0, 15.0, 16.0,
        17.0, 18.0, 19.0, 20.0,
        21.0, 22.0, 23.0, 24.0;

    Eigen::MatrixXd pseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();

    EXPECT_EQ(pseudoInverse.rows(), 4);
    EXPECT_EQ(pseudoInverse.cols(), 6);

    Eigen::MatrixXd expected_pseudoInverse = Eigen::MatrixXd(4, 6);
    expected_pseudoInverse << -0.1929, -0.1357, -0.0786, -0.0214, 0.0357, 0.0929,
        -0.0702, -0.0488, -0.0274, -0.0060, 0.0155, 0.0369,
        0.0524, 0.0381, 0.0238, 0.0095, -0.0048, -0.0190,
        0.1750, 0.1250, 0.0750, 0.0250, -0.0250, -0.0750;

    double epsilon = 1e-6;
    double tolerance = 1e-3;
    EXPECT_TRUE(expected_pseudoInverse.isApprox(pseudoInverse, tolerance)) << " Pseudo Inverse is not correct";
    Eigen::MatrixXd reconstructed = pseudoInverse * jacobian * pseudoInverse;
    EXPECT_NEAR((reconstructed - pseudoInverse).norm(), 0.0, epsilon);
}
TEST(BasicEigenOperation, pseudo_inverse_sanity)
{

    Eigen::MatrixXd jacobian = Eigen::MatrixXd(6, 4);

    jacobian << 1.0, 2.0, 3.0, 4.0,
        5.0, 6.0, 7.0, 8.0,
        9.0, 10.0, 11.0, 12.0,
        13.0, 14.0, 15.0, 16.0,
        17.0, 18.0, 19.0, 20.0,
        21.0, 22.0, 23.0, 24.0;

    Eigen::MatrixXd pseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();

    // EXPECT_EQ(pseudoInverse.rows(), 4);
    // EXPECT_EQ(pseudoInverse.cols(), 6);

    Eigen::MatrixXd expected_pseudoInverse = Eigen::MatrixXd(4, 6);
    expected_pseudoInverse << -0.1929, -0.1357, -0.0786, -0.0214, 0.0357, 0.0929,
        -0.0702, -0.0488, -0.0274, -0.0060, 0.0155, 0.0369,
        0.0524, 0.0381, 0.0238, 0.0095, -0.0048, -0.0190,
        0.1750, 0.1250, 0.0750, 0.0250, -0.0250, -0.0750;

    double epsilon = 1e-6;
    double tolerance = 1e-3;
    // EXPECT_TRUE(expected_pseudoInverse.isApprox(pseudoInverse, tolerance)) << " Pseudo Inverse is not correct";
    
    Eigen::Vector4d q(1,2,3,4);
    Eigen::VectorXd twist(6);
    Eigen::VectorXd expected_twist(6);
    expected_twist<< 30,70,110,150,190,230;
    twist = jacobian*q;

    EXPECT_TRUE(expected_twist.isApprox(twist,epsilon))<<"Calculated Twist is not correct!";

    Eigen::Vector4d new_q;
    new_q = pseudoInverse*twist;
    
    EXPECT_TRUE(q.isApprox(new_q,epsilon))<<"Calculated New Q is not correct!";
}
TEST(BasicEigenOperation, isometry_to_rotation)
{

    Eigen::Matrix4d h_mat;
    h_mat << 1, 2, 3, 10,
        4, 5, 6, 11,
        7, 8, 9, 12,
        0, 0, 0, 1;

    Eigen::Isometry3d iso_pose = Eigen::Isometry3d(h_mat);

    Eigen::Matrix3d expected_rotation_mat;
    expected_rotation_mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    Eigen::Vector3d expected_z1(3, 6, 9);

    Eigen::Vector3d expected_o_vec;
    expected_o_vec << 10, 11, 12;

    Eigen::Matrix3d iso_rot_mat = iso_pose.rotation();

    Eigen::Vector3d iso_o_vec = iso_pose.translation();

    Eigen::Vector3d iso_z_1 = iso_rot_mat * Eigen::Vector3d::UnitZ();

    // std::cout<<" Expected Z: \n"<<expected_z1<<std::endl;
    // std::cout<<" Iso Z: \n"<<iso_z_1<<std::endl;
    double tolerance = 1e-3;

    EXPECT_TRUE(expected_rotation_mat.isApprox(iso_rot_mat, tolerance)) << " Rotation matrix is not Correct";
    EXPECT_TRUE(expected_o_vec.isApprox(iso_o_vec, tolerance)) << " Origin Vector is not Correct";
    EXPECT_TRUE(expected_z1.isApprox(iso_z_1, tolerance)) << " Rotation Z Vector is not Correct";
}

TEST(BasicEigenOperation, jacobian_cell_initialization)
{
    Eigen::MatrixXd expected_jacobian = Eigen::MatrixXd(6, 4);
    Eigen::MatrixXd velocity_jacobian = Eigen::MatrixXd(6, 4);
    expected_jacobian << 1.0, 2.0, 3.0, 4.0,
        5.0, 6.0, 7.0, 8.0,
        9.0, 10.0, 11.0, 12.0,
        13.0, 14.0, 15.0, 16.0,
        17.0, 18.0, 19.0, 20.0,
        21.0, 22.0, 23.0, 24.0;

    // Extract z-axis for each joint (z-axis of each joint's coordinate frame)
    Eigen::Vector3d z_0(13, 17, 21);
    Eigen::Vector3d z_1(14, 18, 22);
    Eigen::Vector3d z_2(15, 19, 23);
    Eigen::Vector3d z_3(16, 20, 24);

    // Calculate Jacobian columns
    Eigen::Vector3d col1(1, 5, 9);

    Eigen::Vector3d col2(2, 6, 10);

    Eigen::Vector3d col3(3, 7, 11);

    Eigen::Vector3d col4(4, 8, 12);

    // Populate velocity Jacobian
    // First 3 rows are linear velocities (cross products)
    velocity_jacobian.block<3, 1>(0, 0) = col1;
    velocity_jacobian.block<3, 1>(0, 1) = col2;
    velocity_jacobian.block<3, 1>(0, 2) = col3;
    velocity_jacobian.block<3, 1>(0, 3) = col4;

    // Last 3 rows are rotational axes
    velocity_jacobian.block<3, 1>(3, 0) = z_0;
    velocity_jacobian.block<3, 1>(3, 1) = z_1;
    velocity_jacobian.block<3, 1>(3, 2) = z_2;
    velocity_jacobian.block<3, 1>(3, 3) = z_3;

    // std::cout << "Expected J: \n"
    //           << expected_jacobian << std::endl;
    // std::cout << "Velcoity J: \n"
    //           << velocity_jacobian << std::endl;

    double tolerance = 1e-3;
    EXPECT_TRUE(expected_jacobian.isApprox(velocity_jacobian, tolerance)) << " Jacobian matrix initalization is not Correct";
}