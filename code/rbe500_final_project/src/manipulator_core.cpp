#include <rbe500_final_project/manipulator_core.hpp>

using namespace manipulator;

ManipulatorCore::ManipulatorCore()
{
    //TODO:
    // setup DH parameters here
    //initialize link_lengths vector
    //joint anfgles
    //dh_params_
    //end effector pose
    end_effector_pose_ = Eigen::Isometry3d::Identity();
}

Eigen::Isometry3d ManipulatorCore::getEndEffectorPose() const
{
    //TODO:
     // Get A_1
    // Eigen::Matrix4d A_1 = this->getHomogeneousMat(1);
    // // Get A_2
    // Eigen::Matrix4d A_2 = this->getHomogeneousMat(2);
    // // Get A_3
    // Eigen::Matrix4d A_3 = this->getHomogeneousMat(3);

    // // Get end effector pose as A1 x A2 x A3
    // Eigen::Matrix4d pose_mat = A_1 * A_2 * A_3;
    // end_effector_pose_ = Eigen::Isometry3d(pose_mat);
    return end_effector_pose_;
}

void ManipulatorCore::setup(const Eigen::VectorXd &link_length)
{
    link_lengths_ = link_length;
    is_intialized_ = true;
}

void ManipulatorCore::updateJointAngles(const Eigen::VectorXd &joint_angles)
{
    //TODO:
    // as all joints angle is on 2nd col of the DH param matrix , so directly updating the column
    // dh_params_.col(1) = joint_angles;
}

Eigen::VectorXd ManipulatorCore::getJointAngles() const
{
    return joint_angles_;
}

Eigen::VectorXd ManipulatorCore::getJointAnglesUsingIK(const Eigen::Isometry3d &end_effector_pose) const
{
    //TODO: Calc join angles using IK
    //UPdate joint Angles
    return joint_angles_;
}

void ManipulatorCore::setupDHParams(const Eigen::VectorXd &link_lengths, const Eigen::VectorXd &joint_angles)
{
    //TODO:

    // update the param matrix as per given join angle and link length
    // for link1
    // dh_params_.row(static_cast<int>(LINK::ONE)) << 0.0, joint_angles(static_cast<int>(LINK::ONE)), link_lengths(static_cast<int>(LINK::ONE)), -1.0 * M_PI_2;
    // updated
    // dh_params_.row(static_cast<int>(LINK::ONE)) << 0.0, M_PI_2 + joint_angles(static_cast<int>(LINK::ONE)), link_lengths(static_cast<int>(LINK::ONE)), M_PI_2;
    // // for link2
    // dh_params_.row(static_cast<int>(LINK::TWO)) << link_lengths(static_cast<int>(LINK::TWO)), joint_angles(static_cast<int>(LINK::TWO)), 0.0, 0.0;
    // // for link3
    // dh_params_.row(static_cast<int>(LINK::THREE)) << link_lengths(static_cast<int>(LINK::THREE)), joint_angles(static_cast<int>(LINK::THREE)), 0.0, 0.0;
}

Eigen::Matrix4d ManipulatorCore::getHomogeneousMat(int link_number) const
{

    Eigen::Matrix4d A_i;
    // get the DH param from the matrix, cols store the dh param for a particular link
    Eigen::Vector4d dh_param = dh_params_.row(link_number);
    double a = dh_param(0);
    double theta = dh_param(1);
    double d = dh_param(2);
    double alpha = dh_param(3);
    // Construct a homogenous matrix using DH params
    A_i << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;

    return A_i;
}
Eigen::Matrix4d ManipulatorCore::calcEndEffectorPose() const
{
     Eigen::Matrix4d pose_mat;
     //TODO:
    // // Get A_1
    // Eigen::Matrix4d A_1 = this->getHomogeneousMat(1);
    // // Get A_2
    // Eigen::Matrix4d A_2 = this->getHomogeneousMat(2);
    // // Get A_3
    // Eigen::Matrix4d A_3 = this->getHomogeneousMat(3);

    // // Get end effector pose as A1 x A2 x A3
    // Eigen::Matrix4d pose_mat = A_1 * A_2 * A_3;

    return pose_mat;
}
