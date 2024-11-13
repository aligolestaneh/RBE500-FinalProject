#include "gtest/gtest.h"
#include "rbe500_final_project/manipulator_core.hpp"

// Test case for ManipulatorCore object creation and initialization
TEST(ManipulatorCore, ObjectCreation)
{
    std::shared_ptr<manipulator::ManipulatorCore> manipulator = std::make_shared<manipulator::ManipulatorCore>();
    ASSERT_NE(manipulator, nullptr) << "ManipulatorCore object should be created successfully";
}

// Test case for setup with valid link lengths and default IK solver
TEST(ManipulatorCore, SetupWithValidLinkLengths)
{
    manipulator::ManipulatorCore manipulator;
    Eigen::VectorXd link_lengths(6);
    link_lengths << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    bool setup_success = manipulator.setup(link_lengths, true);
    ASSERT_TRUE(setup_success) << "Setup should succeed with valid link lengths";
}

// Test case for setup with invalid link lengths (e.g., too few elements)
TEST(ManipulatorCore, SetupWithInvalidLinkLengths)
{
    manipulator::ManipulatorCore manipulator;
    Eigen::VectorXd link_lengths(3); // Only 3 elements instead of 6
    link_lengths << 1.0, 1.0, 1.0;
    bool setup_success = manipulator.setup(link_lengths, true);
    ASSERT_FALSE(setup_success) << "Setup should fail with invalid link lengths";
}

// Test case for updating and getting joint angles wihtout setup
TEST(ManipulatorCore, UpdateAndGetJointAngles)
{
    manipulator::ManipulatorCore manipulator;
    Eigen::VectorXd joint_angles(4);
    joint_angles << -0.32, 0.58, -0.84, -0.76;

    bool update_success = manipulator.updateJointAngles(joint_angles);
    EXPECT_FALSE(update_success) << "Updating joint angles should not succeed before setup";

    Eigen::VectorXd link_lengths(6);
    link_lengths << 0.036076, 0.06025, 0.128, 0.024, 0.124, 0.1334;
    bool setup_success = manipulator.setup(link_lengths, true);
    std::cout << "---------- setup: " << setup_success << std::endl;

    update_success = manipulator.updateJointAngles(joint_angles);

    EXPECT_TRUE(setup_success) << "Setup should succeed with valid link lengths";
    EXPECT_TRUE(update_success) << "Updating joint angles should succeed";
    EXPECT_TRUE(manipulator.getJointAngles().isApprox(joint_angles, 1e-5)) << "Returned joint angles should match the set values";
}

// Test case for updating joint angles with an incorrect size
TEST(ManipulatorCore, UpdateJointAnglesIncorrectSize)
{
    manipulator::ManipulatorCore manipulator;
    Eigen::VectorXd link_lengths(6);
    link_lengths << 0.036076, 0.06025, 0.128, 0.024, 0.124, 0.1334;
    manipulator.setup(link_lengths, true);

    Eigen::VectorXd joint_angles(3); // Only 3 angles instead of 4
    joint_angles << 0.5, 1.0, 0.75;
    bool update_success = manipulator.updateJointAngles(joint_angles);
    EXPECT_FALSE(update_success) << "Updating joint angles should fail with incorrect size";
}

// Test case for calculating and retrieving the end effector pose after setting joint angles
TEST(ManipulatorCore, FKHomePose)
{
    manipulator::ManipulatorCore manipulator;
    Eigen::VectorXd link_lengths(6);
    link_lengths << 0.036076, 0.06025, 0.128, 0.024, 0.124, 0.1334;
    bool setup_success = manipulator.setup(link_lengths, true);

    Eigen::VectorXd joint_angles(4);

    // TEST HOME
    //     - Translation: [0.281, 0.000, 0.224]
    // - Rotation: in Quaternion [-0.707, 0.000, 0.000, 0.707]

    // joint angles
    // q0 : 0 , q1: 0, q2: 0, q3: 0
    joint_angles << 0.0, 0.0, 0.0, 0.0;
    bool updated = manipulator.updateJointAngles(joint_angles);
    Eigen::Isometry3d pose = manipulator.getEndEffectorPose();

    // Found these values from simualtions
    Eigen::Vector3d translation(0.281, 0.0, 0.224);

    // Step 2: Create the rotation quaternion [w,x,y,z]
    Eigen::Quaterniond quaternion(0.707, -0.707, 0.0, 0.0);

    // Step 3: Create the Isometry3d transformation
    Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
    expected_pose.translation() = translation;
    expected_pose.rotate(quaternion.normalized().toRotationMatrix());

    EXPECT_TRUE(pose.isApprox(expected_pose, 1e-3)) << "End effector pose should match expected Home pose";
}

TEST(ManipulatorCore, FKRandomPose1)
{
    manipulator::ManipulatorCore manipulator;
    Eigen::VectorXd link_lengths(6);
    link_lengths << 0.036076, 0.06025, 0.128, 0.024, 0.124, 0.1334;
    bool setup_success = manipulator.setup(link_lengths, true);

    Eigen::VectorXd joint_angles(4);

    // Test1
    //     - Translation: [0.095, 0.189, -0.003]
    // - Rotation: in Quaternion [0.706, -0.026, -0.643, -0.294]
    // joint angles
    // q0 : 1.10 , q1: 0.39, q2:0.37, q3: 0.42

    joint_angles << 1.10, 0.39, 0.37, 0.42;
    manipulator.updateJointAngles(joint_angles);
    Eigen::Isometry3d pose = manipulator.getEndEffectorPose();

    // Found these values from simualtions
    Eigen::Vector3d translation(0.095, 0.189, -0.003);

    // Step 2: Create the rotation quaternion
    Eigen::Quaterniond quaternion(-0.294, 0.706, -0.026, -0.643);

    // Step 3: Create the Isometry3d transformation
    Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
    expected_pose.translation() = translation;
    expected_pose.linear() = quaternion.toRotationMatrix();

    EXPECT_TRUE(pose.isApprox(expected_pose, 1e-2)) << "End effector pose should match expected Random pose";
}

TEST(ManipulatorCore, FKRandomPose2)
{
    manipulator::ManipulatorCore manipulator;
    Eigen::VectorXd link_lengths(6);
    link_lengths << 0.036076, 0.06025, 0.128, 0.024, 0.124, 0.1334;
    bool setup_success = manipulator.setup(link_lengths, true);

    Eigen::VectorXd joint_angles(4);

    // Test2
    //  Translation: [0.006, 0.108, 0.266]
    // - Rotation: in Quaternion [0.549, 0.445, -0.524, -0.476]
    //             in RPY (radian) [0.000, 0.984, -2.604]
    //             in RPY (degree) [0.000, 56.391, -149.170]
    // joint angles
    // q0 : 1.51 , q1: -1.01, q2: 0.05, q3: 1.10

    joint_angles << 1.51, -1.01, 0.05, 1.10;
    manipulator.updateJointAngles(joint_angles);
    Eigen::Isometry3d pose = manipulator.getEndEffectorPose();

    // Found these values from simualtions
    Eigen::Vector3d translation(0.006, 0.108, 0.266);

    // Step 2: Create the rotation quaternion
    Eigen::Quaterniond quaternion(-0.476, 0.549, 0.445, -0.524);

    // Step 3: Create the Isometry3d transformation
    Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
    expected_pose.translation() = translation;
    expected_pose.linear() = quaternion.toRotationMatrix();

    EXPECT_TRUE(pose.isApprox(expected_pose, 1e-2)) << "End effector pose should match expected Random pose";
}

// Test case for calculating and retrieving the end effector pose after setting joint angles
TEST(ManipulatorCore, GetHomeJointAngles)
{
    manipulator::ManipulatorCore manipulator;
    Eigen::VectorXd link_lengths(6);
    link_lengths << 0.036076, 0.06025, 0.128, 0.024, 0.124, 0.1334;
    bool setup_success = manipulator.setup(link_lengths, true);

    Eigen::VectorXd expected_joint_angles(4);

    // TEST HOME
    //     - Translation: [0.281, 0.000, 0.224]
    // - Rotation: in Quaternion [-0.707, 0.000, 0.000, 0.707]

    // joint angles
    // q0 : 0 , q1: 0, q2: 0, q3: 0
    expected_joint_angles << 0.0, 0.0, 0.0, 0.0;

    // Found these values from simualtions
    Eigen::Vector3d translation(0.281, 0.0, 0.224);

    // Step 2: Create the rotation quaternion
    Eigen::Quaterniond quaternion(0.707, -0.707, 0.0, 0.0);

    // Step 3: Create the Isometry3d transformation
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = translation;
    pose.linear() = quaternion.toRotationMatrix();
    bool updated = manipulator.updateEndEffectorPose(pose);
    // std::cout<<"updated: "<<updated<<std::endl;
    EXPECT_TRUE(updated) << " Newton raphson method should find the solution";

    Eigen::VectorXd joint_angles = manipulator.getJointAngles();
    std::cout << "Calculated J: \n"
              << joint_angles << std::endl;
    std::cout << "Expected J: \n"
              << expected_joint_angles << std::endl;
    double tolerance = 1e-2;
    Eigen::VectorXd joint_angle_diff;
    joint_angle_diff = expected_joint_angles - joint_angles;

    for (int i = 0; i < joint_angle_diff.size(); i++)
    {

        EXPECT_LE(std::fabs(joint_angle_diff(i)), tolerance) << "Joint angles are not under tolerance";
    }
}

// Test case for calculating and retrieving the end effector pose after setting joint angles
TEST(ManipulatorCore, IKJointAngles1)
{
    manipulator::ManipulatorCore manipulator;
    Eigen::VectorXd link_lengths(6);
    link_lengths << 0.036076, 0.06025, 0.128, 0.024, 0.124, 0.1334;
    bool setup_success = manipulator.setup(link_lengths, true);

    Eigen::VectorXd expected_joint_angles(4);

    // Test1
    //     - Translation: [0.095, 0.189, -0.003]
    // - Rotation: in Quaternion [0.706, -0.026, -0.643, -0.294]
    // joint angles
    // q0 : 1.10 , q1: 0.39, q2:0.37, q3: 0.42

    expected_joint_angles << 1.10, 0.39, 0.37, 0.42;

    // Found these values from simualtions
    Eigen::Vector3d translation(0.095, 0.189, -0.003);

    // Step 2: Create the rotation quaternion
    Eigen::Quaterniond quaternion(-0.294, 0.706, -0.026, -0.643);

    // Step 3: Create the Isometry3d transformation
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = translation;
    pose.linear() = quaternion.toRotationMatrix();

    bool updated = manipulator.updateEndEffectorPose(pose);
    EXPECT_TRUE(updated) << " Newton raphson method should find the solution";

    Eigen::VectorXd joint_angles = manipulator.getJointAngles();
    std::cout << "Calculated J: \n"
              << joint_angles << std::endl;
    std::cout << "Expected J: \n"
              << expected_joint_angles << std::endl;
    double tolerance = 1e-2;
    Eigen::VectorXd joint_angle_diff;
    joint_angle_diff = expected_joint_angles - joint_angles;

    for (int i = 0; i < joint_angle_diff.size(); i++)
    {

        EXPECT_LE(std::fabs(joint_angle_diff(i)), tolerance) << "Joint angles are not under tolerance";
    }
}

// Test case for calculating and retrieving the end effector pose after setting joint angles
TEST(ManipulatorCore, IKJointAngles2)
{
    manipulator::ManipulatorCore manipulator;
    Eigen::VectorXd link_lengths(6);
    link_lengths << 0.036076, 0.06025, 0.128, 0.024, 0.124, 0.1334;
    bool setup_success = manipulator.setup(link_lengths, true);

    Eigen::VectorXd expected_joint_angles(4);

    // Test2
    //  Translation: [0.006, 0.108, 0.266]
    // - Rotation: in Quaternion [0.549, 0.445, -0.524, -0.476]
    //             in RPY (radian) [0.000, 0.984, -2.604]
    //             in RPY (degree) [0.000, 56.391, -149.170]
    // joint angles
    // q0 : 1.51 , q1: -1.01, q2: 0.05, q3: 1.10

    expected_joint_angles << 1.51, -1.01, 0.05, 1.10;

    // Found these values from simualtions
    Eigen::Vector3d translation(0.006, 0.108, 0.266);

    // Step 2: Create the rotation quaternion
    Eigen::Quaterniond quaternion(-0.476, 0.549, 0.445, -0.524);

    // Step 3: Create the Isometry3d transformation
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = translation;
    pose.linear() = quaternion.toRotationMatrix();

    bool updated = manipulator.updateEndEffectorPose(pose);
    EXPECT_TRUE(updated) << " Newton raphson method should find the solution";

    Eigen::VectorXd joint_angles = manipulator.getJointAngles();
    std::cout << "Calculated J: \n"
              << joint_angles << std::endl;
    std::cout << "Expected J: \n"
              << expected_joint_angles << std::endl;
    double tolerance = 1e-2;
    Eigen::VectorXd joint_angle_diff;
    joint_angle_diff = expected_joint_angles - joint_angles;

    for (int i = 0; i < joint_angle_diff.size(); i++)
    {

        EXPECT_LE(std::fabs(joint_angle_diff(i)), tolerance) << "Joint angles are not under tolerance";
    }
}