#include <rbe500_final_project/manipulator_core.hpp>

using namespace manipulator;

ManipulatorCore::ManipulatorCore()
{
    // TODO:
    // initialize link_lengths vector
    link_lengths_ = Eigen::VectorXd::Ones(6);
    // joint anfgles
    joint_angles_ = Eigen::VectorXd::Zero(4);
    //  setup DH parameters here
    dh_params_ = Eigen::MatrixXd(link_lengths_.size(), 4);
    // end effector pose
    end_effector_pose_ = Eigen::Isometry3d::Identity();
    max_iteration_ = 100;
    tolerance_ = 1e-7;
    is_intialized_ = false;
    use_newtonRapshon_IK_ = true;
}

bool ManipulatorCore::setup(const Eigen::VectorXd &link_length, bool use_newtonRapshon_IK)
{
    if (link_lengths_.size() != link_length.size())
        return false;

    
    link_lengths_ = link_length;
    // std::cout<<" Link Lengths are: \n"<<link_lengths_<<std::endl;
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
    // TODO:
    //  as all joints angle is on 2nd col of the DH param matrix , so directly updating the column
    if (!is_intialized_)
        return false;
    if (joint_angles.size() != joint_angles_.size())
        return false;

    // Update dh_params_ with q_val

    dh_params_.row(1)(2) = joint_angles(0);          // For link2 joint angle
    dh_params_.row(2)(2) = joint_angles(1) - M_PI_2; // For link3x joint angle (with offset)
    dh_params_.row(4)(2) = joint_angles(2);          // For link4 joint angle
    dh_params_.row(5)(2) = joint_angles(3);          // For link5 joint angle

    end_effector_pose_ = calcEndEffectorPose();
    joint_angles_ = joint_angles;

    return true;
}

Eigen::Isometry3d ManipulatorCore::getEndEffectorPose() const
{
    return end_effector_pose_;
}

Eigen::Isometry3d ManipulatorCore::calcEndEffectorPose() const
{
    // Eigen::Matrix4d pose_mat = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d A_i = this->getHomogeneousMat(0); // Get the first link's transformation matrix

    for (int i = 1; i < link_lengths_.size(); i++)
    {
        A_i *= this->getHomogeneousMat(i);
    }
    Eigen::Isometry3d end_effector_pose = Eigen::Isometry3d(A_i);

    // end_effector_pose.matrix() = A_i;//pose_mat;

    return end_effector_pose;
}

bool ManipulatorCore::calcNewtonRaphsonIK(const Eigen::Isometry3d &end_effector_pose, Eigen::VectorXd &result_q) const
{
    // TODO: Calc join angles using IK
    // inverse kinematic equations
    Eigen::Vector3d F;
    // Jacobian for Inverse kinematic
    Eigen::Matrix3d J;
    // delta in joint angles
    Eigen::Vector3d delta_q;
    // intial joint angle guess as zero
    Eigen::Vector3d current_q = Eigen::Vector3d::Zero();

    // Eigen::Vector3d joint_angles;

    // coefficitans for calculating residual [l2x,l2y,l3,l4]
    Eigen::Vector4d coeffs = link_lengths_.segment<4>(2);

    // constants for the equations [theta,r,z_from_link2x]
    Eigen::Vector3d constants;
    Eigen::MatrixXd rotation_matrix = end_effector_pose.rotation();

    // double phi = atan2(-rotation_matrix(2, 0),
    //                    sqrt(rotation_matrix(2, 1) * rotation_matrix(2, 1) + rotation_matrix(2, 2) * rotation_matrix(2, 2)));
    double phi = helpers::convertRotationMatrixToRPY(end_effector_pose.rotation())[1];
    constants(0) = phi;
    constants(2) = end_effector_pose.translation().head<2>().norm();                    // sqrt(x^2+y^2)
    constants(1) = end_effector_pose.translation().z() - link_lengths_.head<2>().sum(); // z - (link0+link1)

    // updating q(0)
    result_q(0) = atan2(end_effector_pose.translation().y(), end_effector_pose.translation().x());
    
    for (int i = 0; i < max_iteration_; i++)
    {
        // compute function value as per current q values

        F = this->calcResidual(current_q, coeffs, constants);
        
        // check if current_q satisfies tolerance
        if (F.norm() < tolerance_)
        {
            result_q.segment<3>(1) = current_q;
            return true;
        }

        // get jacobian
        J = this->calcJacobian(current_q, coeffs);

        // get delta_q
        delta_q = J.fullPivLu().solve(-F);//J.colPivHouseholderQr().solve(-F); //J.fullPivLu().solve(-F);

        // update current_q
        current_q += delta_q;
        
        // check if current_q satisfies tolerance
        if (delta_q.norm() < tolerance_)
        {
            result_q.segment<3>(1) = current_q;
            return true;
        }
    }
    return false;
}

Eigen::VectorXd ManipulatorCore::calcGeometricIK(const Eigen::Isometry3d &end_effector_pose) const
{
    // TODO: Calc join angles using IK
    // UPdate joint Angles
    return joint_angles_;
}

void ManipulatorCore::setupDHParams(const Eigen::VectorXd &link_lengths, const Eigen::VectorXd &joint_angles)
{
    // TODO:

    // update the param matrix as per given join angle and link length
    // for link1
    dh_params_.row(0) << 0, link_lengths(0), 0, 0;
    // for link2
    dh_params_.row(1) << 0, link_lengths(1), joint_angles(0), -M_PI_2;
    // for link3x
    dh_params_.row(2) << link_lengths(2), 0, joint_angles(1) - M_PI_2, 0;
    // for link3y
    dh_params_.row(3) << link_lengths(3), 0, M_PI_2, 0;

    // for link4
    dh_params_.row(4) << link_lengths(4), 0, joint_angles(2), 0;

    // for link5
    dh_params_.row(5) << link_lengths(5), 0, joint_angles(3), 0;
}

Eigen::Matrix4d ManipulatorCore::getHomogeneousMat(int link_number) const
{

    Eigen::Matrix4d A_i;
    // get the DH param from the matrix, cols store the dh param for a particular link
    Eigen::Vector4d dh_param = dh_params_.row(link_number);
    double a = dh_param(0);
    double theta = dh_param(2);
    double d = dh_param(1);
    double alpha = dh_param(3);
    // Construct a homogenous matrix using DH params
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
