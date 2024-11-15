/**
 * @file manipulator_core.hpp
 * @brief Core C++ library for OpenX Manipulator kinematics
 * @details This library implements forward and inverse kinematics calculations for the OpenX Manipulator.
 *          It provides functionality for:
 *          - Forward kinematics using DH parameters
 *          - Inverse kinematics using both geometric and Newton-Raphson methods
 *          - End-effector pose calculations and joint angle computations
 * @authors Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
 */

#ifndef RBE500_FINAL_PROJECT_PKG_MANIPULATOR_CORE_HPP_
#define RBE500_FINAL_PROJECT_PKG_MANIPULATOR_CORE_HPP_

#include "rbe500_final_project/helpers.hpp"
// EigenLib for matrix handling
#include <eigen3/Eigen/Dense>

/** @namespace manipulator
 *  @brief Namespace containing manipulator control functionality
 */
namespace manipulator
{
    /** @class ManipulatorCore
     *  @brief Core class implementing manipulator kinematics and control
     */
    class ManipulatorCore
    {

    public:
        /** @brief Constructor */
        ManipulatorCore();
        /** @brief Destructor */
        ~ManipulatorCore() = default;

        /** 
         * @brief Sets the initial DH parameters and IK solver
         * @param link_length Link Lengths of the Manipulator
         * @param use_newtonRapshon_IK True for NewtonRaphson IK, False to use Geometric IK
         * @return True if link length dimension is correct as per this Lib size
         */
        bool setup(const Eigen::VectorXd &link_length, bool use_newtonRapshon_IK = true);

        /**
         * @brief Sets the NewtonRaphson IK solver parameters
         * @param max_iter Maximum loop iteration value
         * @param eps Epsilon value to check convergence of IK solver
         */
        void setIKParams(const int &max_iter, const double &eps);

        /** @brief function to update the end_effector_pose
         * @brief it finds the given joint angles IK solver,
         * @brief use getJointAngles() function after this function call to get updated angles
         * @param pose: An Isometry Pose of End effector
         * @return True: If its able to find the joint angles using IK
         * @return False: Otherwise
         */
        bool updateEndEffectorPose(const Eigen::Isometry3d &pose);

        /** @brief Function to get the current set joint angles
         * @return join_angle VEctor: A 4x1 Matrix having all four joint angles,
         */
        Eigen::VectorXd getJointAngles() const;

        /** @brief function to update the DH param matrix for the given joint angles
         * @brief It will calc the end_effector_pose_ using FK
         * @brief use getEndEffectorPose() function after this function call to get end-effector pose
         * @param joint_angle: A 4x1 column vector having joint angles as [q1,q2,q3,q4]^T
         * @return True: If joint angles are reasonable and its able to find the End effector pose
         * @return False: otherwise
         */
        bool updateJointAngles(const Eigen::VectorXd &joint_angles);

        /** @brief Function to get the end effector pose
         * @return pose_mat: An isometry  Matrix having both Rotational ad Positional values
         */
        Eigen::Isometry3d getEndEffectorPose() const;

    private:
        /**
         * @brief Sets the DH param matrix for the OpenX Manipulator
         * 
         * Each row will have [a,d,theta,alpha] params, it will be link_number x 4 matrix
         * @param link_lengths Column vector having link lengths as [l1,l2,l3_x,l3_y,l4,l5]^T
         * @param joint_angle Column vector having initial joint angles as [q1,q2,q3,q4]^T
         */
        void setupDHParams(const Eigen::VectorXd &link_lengths, const Eigen::VectorXd &joint_angles);

        /**
         * @brief Gets homogeneous transformation matrix from Link number
         * 
         * It directly fetches DH param a,theta,d,alpha for the given from the dh_params matrix.
         * Make sure to update the joint angles before calling this function.
         * Homogenous Matrix A_i for transformation between frame i to (i-1) as:
         * [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta)]
         * [sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)]
         * [0           sin(alpha)             cos(alpha)                 d      ]
         * [0              0                      0                       1      ]
         * 
         * @param link_number Link number for which Homogenous matrix required
         * @return A 4x4 Homogeneous Transformation matrix A_i for the given DH param
         */
        Eigen::Matrix4d getHomogeneousMat(int link_number) const;

        /** @brief A function to calc the end effector pose using FK
         * @brief it calculates using the current value of DH param
         * @return end_effector_pose
         */
        Eigen::Isometry3d calcEndEffectorPose() const;

        /** @brief Function to get the joint angles as per given end effector pose
         * @brief We can det 4 possible soultions of join_angle1,join_angle2,join_angle3
         * @param end_effector_pose: pose of the end effector as recieved by the subscriebr callback
         * @return join_angle VEctor: A 4x1 Matrix having all four joint angles,
         */
        Eigen::VectorXd calcGeometricIK(const Eigen::Isometry3d &end_effector_pose) const;

        /** @brief Function to get the joint angles as per given end effector pose
         * @brief We can det 4 possible soultions of join_angle1,join_angle2,join_angle3
         * @param end_effector_pose: pose of the end effector as recieved by the subscriebr callback
         * @param result_q: resultant values after solution converges
         * @return True: If its able to find the solution
         * @return False: otherwise
         */
        bool calcNewtonRaphsonIK(const Eigen::Isometry3d &end_effector_pose, Eigen::VectorXd &result_q) const;

        /** @brief Function to calculate inverse kinematic equations which will beused in newton raphson method
         * @param q_values: values of joint vars q1, q2, q3 at index 0,1,2 respectively
         * @param coeffs: values of link lenths l2x,l2y,l3,l4 at index 0,1,2,3 respectively
         * @param constants: values of pitch angle theta[rad] from end effector pose,
         * r = sqrt(x^2+y^2), x and y from end effector pose,
         * z_from_link2x  = z - (link0+link1), where z is from end effector pose
         * constants = [theta,r,z_from_link2x]
         * @return inverse kinematic equation
         */
        Eigen::Vector3d calcResidual(const Eigen::Vector3d &q_values, const Eigen::Vector4d &coeffs, const Eigen::Vector3d &constants) const;

        /** @brief Function to calculate Jacobian of inverse kinematic equations which will be used in newton raphson method
         * @param q_values: values of joint vars q1, q2, q3 at index 0,1,2 respectively
         * @param coeffs: values of link lenths l2x,l2y,l3,l4 at index 0,1,2,3 respectively
         * @return Jacobian of inverse kinematic equation
         */
        Eigen::Matrix3d calcJacobian(const Eigen::Vector3d &q_values, const Eigen::Vector4d &coeffs) const;

        /** @brief Column vector to store the link lengths as [l1,l2,l3,l4]^T */
        Eigen::VectorXd link_lengths_;

        /** @brief Column vector to store the joint angles as [q1,q2,q3,q4]^T */
        Eigen::VectorXd joint_angles_; // Function to compute the residual vector

        /** 
         * @brief 5x4 matrix to store the DH params:
         * 
         * A    D   Theta    Alpha
         * [0   l0   0          0]
         * [0   l1   q1     -pi/2]
         * [l2x  0   q2-pi/2    0]
         * [l2y  0   pi/2       0]
         * [l3   0   q3         0]
         * [l4   0   q4         0]
         */
        Eigen::MatrixXd dh_params_;

        /** @brief A Eigen Isometry var to store the homogenous matrix of the end effector pose */
        Eigen::Isometry3d end_effector_pose_;

        /** @brief A Eigen Quaternion var to store the end effector orientation as a quaternion */
        Eigen::Quaterniond end_effector_orientation_;

        /** @brief A Eigen MatrixXd var to store the Jacobian  matrix for the end effector pose */
        Eigen::MatrixXd jacobian_;

        /** @brief A boolean flag to che if Dh param intialization  */
        bool is_intialized_;

        /** @brief A boolean flag to set to use NewtonRaphson IK */
        bool use_newtonRapshon_IK_;

        /** @brief Max interation for the NewtonRaphson IK  loop*/
        int max_iteration_;
        
        /** @brief  Epsilon value to check convergence of IK solver */
        double tolerance_;

    }; // ManipulatorCore
} // namespace manipulator

#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_CORE_HPP_
