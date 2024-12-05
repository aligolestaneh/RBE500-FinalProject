#include <rbe500_final_project/manipulator_core.hpp>
#include <iostream>
using namespace manipulator;

ManipulatorCore::ManipulatorCore()
{
    // Initialize vectors with default values
    link_lengths_ = Eigen::VectorXd::Ones(6);              // Default link lengths
    joint_angles_ = Eigen::VectorXd::Zero(4);              // Initial joint angles set to zero
    dh_params_ = Eigen::MatrixXd(link_lengths_.size(), 4); // DH parameter matrix (6x4)
    end_effector_pose_ = Eigen::Isometry3d::Identity();    // Initial pose as identity

    joint_velocities_ = Eigen::VectorXd::Zero(4);
    ee_twist_ = Eigen::VectorXd::Zero(6);

    velocity_jacobian_ = Eigen::MatrixXd(6, 4);
    inverse_velocity_jacobian_ = Eigen::MatrixXd(4, 6);

    // Set IK solver parameters
    max_iteration_ = 100;
    tolerance_ = 1e-7;
    is_intialized_ = false;
    use_newtonRapshon_IK_ = true; // Default to Newton-Raphson method
}

bool ManipulatorCore::setup(const Eigen::VectorXd &link_length, bool use_newtonRapshon_IK)
{
    if (link_lengths_.size() != link_length.size())
        return false;

    link_lengths_ = link_length;
    // setupDHParams
    this->setupDHParams(link_lengths_, joint_angles_);
    // set init true
    use_newtonRapshon_IK_ = use_newtonRapshon_IK;
    is_intialized_ = true;

    return true;
}
void ManipulatorCore::setIKParams(const int &max_iter, const double &eps)
{
    max_iteration_ = max_iter;
    tolerance_ = eps;
}
bool ManipulatorCore::updateEndEffectorPose(const Eigen::Isometry3d &pose)
{
    if (!is_intialized_)
        return false;
    // std::cout << "Gogin to calculated using Newton: " << use_newtonRapshon_IK_ << std::endl;
    if (use_newtonRapshon_IK_)
        return calcNewtonRaphsonIK(pose, joint_angles_);
    else
        joint_angles_ = calcGeometricIK(pose);

    end_effector_pose_ = pose;
    return true;
}
Eigen::VectorXd ManipulatorCore::getJointAngles() const
{
    return joint_angles_;
}

bool ManipulatorCore::updateJointAngles(const Eigen::VectorXd &joint_angles)
{
    if (!is_intialized_ || joint_angles.size() != joint_angles_.size())
        return false;

    // Update DH parameters matrix with new joint angles
    // Note: Some joints have offsets (e.g., -π/2 for link3x)
    dh_params_.row(1)(2) = joint_angles(0);          // Link2 joint angle
    dh_params_.row(2)(2) = joint_angles(1) - M_PI_2; // Link3x joint angle (with offset)
    dh_params_.row(4)(2) = joint_angles(2);          // Link4 joint angle
    dh_params_.row(5)(2) = joint_angles(3);          // Link5 joint angle

    // Update cached values
    end_effector_pose_ = calcEndEffectorPose();
    joint_angles_ = joint_angles;
    velocity_jacobian_ = this->calcVelocityJacobian();
    return true;
}

Eigen::Isometry3d ManipulatorCore::getEndEffectorPose() const
{
    return end_effector_pose_;
}

Eigen::Isometry3d ManipulatorCore::calcEndEffectorPose() const
{
    // Calculate forward kinematics by multiplying transformation matrices
    Eigen::Matrix4d A_i = this->getHomogeneousMat(0); // Start with first link

    // Multiply subsequent transformation matrices
    for (int i = 1; i < link_lengths_.size(); i++)
    {
        A_i *= this->getHomogeneousMat(i);
    }

    return Eigen::Isometry3d(A_i);
}

bool ManipulatorCore::calcNewtonRaphsonIK(const Eigen::Isometry3d &end_effector_pose, Eigen::VectorXd &result_q) const
{
    // Initialize vectors and matrices for Newton-Raphson iteration
    Eigen::Vector3d F;                                   // Residual vector
    Eigen::Matrix3d J;                                   // Jacobian matrix
    Eigen::Vector3d delta_q;                             // Change in joint angles
    Eigen::Vector3d current_q = Eigen::Vector3d::Zero(); // Initial guess

    // Extract coefficients for residual calculation [l2x,l2y,l3,l4]
    Eigen::Vector4d coeffs = link_lengths_.segment<4>(2);

    // Calculate constants needed for IK equations
    Eigen::Vector3d constants;
    Eigen::MatrixXd rotation_matrix = end_effector_pose.rotation();

    // Extract pitch angle from rotation matrix
    double phi = helpers::convertRotationMatrixToRPY(end_effector_pose.rotation())[1];
    constants(0) = phi;
    constants(2) = end_effector_pose.translation().head<2>().norm();                    // Planar distance from base
    constants(1) = end_effector_pose.translation().z() - link_lengths_.head<2>().sum(); // Height from link2

    // Calculate base rotation (first joint angle)
    result_q(0) = atan2(end_effector_pose.translation().y(), end_effector_pose.translation().x());

    // Newton-Raphson iteration
    for (int i = 0; i < max_iteration_; i++)
    {
        // Compute residual for current joint angles
        F = this->calcResidual(current_q, coeffs, constants);

        // Check if solution is within tolerance
        if (F.norm() < tolerance_)
        {
            result_q.segment<3>(1) = current_q;
            return true;
        }

        // Calculate Jacobian and solve for joint angle updates
        J = this->calcJacobian(current_q, coeffs);
        delta_q = J.fullPivLu().solve(-F);

        // Update joint angles
        current_q += delta_q;

        // Check if change in joint angles is within tolerance
        if (delta_q.norm() < tolerance_)
        {
            result_q.segment<3>(1) = current_q;
            return true;
        }
    }
    return false; // Failed to converge
}

Eigen::VectorXd ManipulatorCore::calcGeometricIK(const Eigen::Isometry3d & /*end_effector_pose*/) const
{
    // TODO: Calc join angles using IK
    // UPdate joint Angles
    return joint_angles_;
}

void ManipulatorCore::setupDHParams(const Eigen::VectorXd &link_lengths, const Eigen::VectorXd &joint_angles)
{
    // Set up DH parameters matrix for each link
    // Format: [a, d, theta, alpha]
    dh_params_.row(0) << 0, link_lengths(0), 0, 0;                        // Link1 (fixed)
    dh_params_.row(1) << 0, link_lengths(1), joint_angles(0), -M_PI_2;    // Link2
    dh_params_.row(2) << link_lengths(2), 0, joint_angles(1) - M_PI_2, 0; // Link3x
    dh_params_.row(3) << link_lengths(3), 0, M_PI_2, 0;                   // Link3y (fixed)
    dh_params_.row(4) << link_lengths(4), 0, joint_angles(2), 0;          // Link4
    dh_params_.row(5) << link_lengths(5), 0, joint_angles(3), 0;          // Link5
}

Eigen::Matrix4d ManipulatorCore::getHomogeneousMat(int link_number) const
{
    // Extract DH parameters for the specified link
    Eigen::Vector4d dh_param = dh_params_.row(link_number);
    double a = dh_param(0);     // Link length
    double d = dh_param(1);     // Link offset
    double theta = dh_param(2); // Joint angle
    double alpha = dh_param(3); // Link twist

    // Construct homogeneous transformation matrix using DH parameters
    Eigen::Matrix4d A_i;
    A_i << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;

    return A_i;
}

Eigen::Vector3d ManipulatorCore::calcResidual(const Eigen::Vector3d &q_values, const Eigen::Vector4d &coeffs, const Eigen::Vector3d &constants) const
{
    Eigen::Vector3d F;
    double q1 = q_values(0);
    double q12 = q_values(0) + q_values(1);
    double q123 = q_values(0) + q_values(1) + q_values(2);

    // Equation 1: q1 + q2 + q3 - theta = 0
    F(0) = q_values(0) + q_values(1) + q_values(2) - constants(0);

    // Equation 2: l1*cos(q1) - l2*sin(q1) - l3*sin(q1+q2) - l4*sin(q1+q2+q3) - z = 0
    F(1) = coeffs(0) * cos(q1) - coeffs(1) * sin(q1) - coeffs(2) * sin(q12) - coeffs(3) * sin(q123) - constants(1);

    // Equation 3: l1*sin(q1) + l2*cos(q1) + l3*cos(q1+q2) + l4*cos(q1+q2+q3) - r = 0
    F(2) = coeffs(0) * sin(q1) + coeffs(1) * cos(q1) + coeffs(2) * cos(q12) + coeffs(3) * cos(q123) - constants(2);

    return F;
}

Eigen::Matrix3d ManipulatorCore::calcJacobian(const Eigen::Vector3d &q_values, const Eigen::Vector4d &coeffs) const
{
    Eigen::Matrix3d J;

    // Derivatives for F1 = q1 + q2 + q3 - theta
    J(0, 0) = 1;
    J(0, 1) = 1;
    J(0, 2) = 1;

    // Derivatives for F2 = l1*cos(q1) - l2*sin(q1) - l3*sin(q1+q2) - l4*sin(q1+q2+q3) - z
    J(1, 0) = -coeffs(0) * sin(q_values(0)) - coeffs(1) * cos(q_values(0)) - coeffs(2) * cos(q_values(0) + q_values(1)) - coeffs(3) * cos(q_values(0) + q_values(1) + q_values(2));
    J(1, 1) = -coeffs(2) * cos(q_values(0) + q_values(1)) - coeffs(3) * cos(q_values(0) + q_values(1) + q_values(2));
    J(1, 2) = -coeffs(3) * cos(q_values(0) + q_values(1) + q_values(2));

    // Derivatives for F3 = l1*sin(q1) + l2*cos(q1) + l3*cos(q1+q2) + l4*cos(q1+q2+q3) - r
    J(2, 0) = coeffs(0) * cos(q_values(0)) - coeffs(1) * sin(q_values(0)) - coeffs(2) * sin(q_values(0) + q_values(1)) - coeffs(3) * sin(q_values(0) + q_values(1) + q_values(2));
    J(2, 1) = -coeffs(2) * sin(q_values(0) + q_values(1)) - coeffs(3) * sin(q_values(0) + q_values(1) + q_values(2));
    J(2, 2) = -coeffs(3) * sin(q_values(0) + q_values(1) + q_values(2));

    return J;
}

Eigen::MatrixXd ManipulatorCore::calcVelocityJacobian() const
{
    Eigen::MatrixXd velocity_jacobian = Eigen::MatrixXd(6, 4);

    // Calculate homogeneous transformations
    Eigen::Isometry3d H_1 = Eigen::Isometry3d(this->getHomogeneousMat(0));

    Eigen::Isometry3d H_2 = Eigen::Isometry3d(this->getHomogeneousMat(0) *
                                              this->getHomogeneousMat(1));

    Eigen::Isometry3d H_3 = Eigen::Isometry3d(this->getHomogeneousMat(0) *
                                              this->getHomogeneousMat(1) *
                                              this->getHomogeneousMat(2) *
                                              this->getHomogeneousMat(3));

    Eigen::Isometry3d H_4 = Eigen::Isometry3d(this->getHomogeneousMat(0) *
                                              this->getHomogeneousMat(1) *
                                              this->getHomogeneousMat(2) *
                                              this->getHomogeneousMat(3) *
                                              this->getHomogeneousMat(4));

    Eigen::Isometry3d H_5 = this->calcEndEffectorPose();

    // Extract origins
    Eigen::Vector3d O_1 = H_1.translation();
    Eigen::Vector3d O_2 = H_2.translation();
    Eigen::Vector3d O_3 = H_3.translation();
    Eigen::Vector3d O_4 = H_4.translation();
    Eigen::Vector3d O_5 = H_5.translation();

    // Extract z-axis for each joint (z-axis of each joint's coordinate frame)
    Eigen::Vector3d z_1 = H_1.rotation() * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d z_2 = H_2.rotation() * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d z_3 = H_3.rotation() * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d z_4 = H_4.rotation() * Eigen::Vector3d::UnitZ();

    // Calculate Jacobian columns
    Eigen::Vector3d col1 = z_1.cross(O_5 - O_1);
    Eigen::Vector3d col2 = z_2.cross(O_5 - O_2);
    Eigen::Vector3d col3 = z_3.cross(O_5 - O_3);
    Eigen::Vector3d col4 = z_4.cross(O_5 - O_4);

    // Populate velocity Jacobian
    velocity_jacobian.block<3, 1>(0, 0) = col1;
    velocity_jacobian.block<3, 1>(0, 1) = col2;
    velocity_jacobian.block<3, 1>(0, 2) = col3;
    velocity_jacobian.block<3, 1>(0, 3) = col4;

    // Last 3 rows are rotational axes
    velocity_jacobian.block<3, 1>(3, 0) = z_1;
    velocity_jacobian.block<3, 1>(3, 1) = z_2;
    velocity_jacobian.block<3, 1>(3, 2) = z_3;
    velocity_jacobian.block<3, 1>(3, 3) = z_4;
    return velocity_jacobian;
}

Eigen::MatrixXd ManipulatorCore::calcPseudoInverseVelocityJacobian(const Eigen::MatrixXd &velocity_jacobian) const
{
    // calc psueod inverse of velocity_jacobian
    return velocity_jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

bool ManipulatorCore::updateJointVelocities(const Eigen::VectorXd &joint_velocities)
{
    if (!is_intialized_ || joint_velocities.size() != joint_velocities_.size())
        return false;

    velocity_jacobian_ = this->calcVelocityJacobian();
    ee_twist_ = velocity_jacobian_ * joint_velocities;
    joint_velocities_ = joint_velocities;

    return true;
}

bool ManipulatorCore::updateEndEffectorTwist(const Eigen::VectorXd &ee_twist)
{
    if (!is_intialized_ || ee_twist.size() != ee_twist_.size())
        return false;

    velocity_jacobian_ = this->calcVelocityJacobian();
    inverse_velocity_jacobian_ = this->calcPseudoInverseVelocityJacobian(velocity_jacobian_);
    joint_velocities_ = inverse_velocity_jacobian_ * ee_twist;
    ee_twist_ = ee_twist;

    return true;
}

Eigen::VectorXd ManipulatorCore::getJointVelocities() const
{
    return joint_velocities_;
}

Eigen::VectorXd ManipulatorCore::getEndEffectorTwist() const
{
    return ee_twist_;
}