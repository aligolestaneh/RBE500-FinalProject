///////////////////////////////////////
// RBE 500
// Final Project
// Authors: Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
//////////////////////////////////////
#ifndef RBE500_FINAL_PROJECT_PKG_MANIPULATOR_CORE_HPP_
#define RBE500_FINAL_PROJECT_PKG_MANIPULATOR_CORE_HPP_

// Standard Core C++ libs
#include <iostream>
#include <memory>
#include <string>
#include <cmath>
// EigenLib for matrix handling
#include <eigen3/Eigen/Dense>

namespace manipulator
{   
    class ManipulatorCore
    {

    public:
        // constructor
        ManipulatorCore();
        // destructor
        ~ManipulatorCore() = default;
        
        /** @brief Function to set the initial DH parameters
         */
        void setup(const Eigen::VectorXd& link_length);
        
        /** @brief Function to get the end effector pose
         * @brief using Homogenous matrix of link1, link2 and link3
         * @return pose_mat: An isometry  Matrix having both Rotational ad Positional values
         */
        Eigen::Isometry3d getEndEffectorPose() const;
        
        /** @brief function to update the DH param matrix for the given joint angles
         * @param joint_angle: A 4x1 column vector having joint angles as [q1,q2,q3,q4]^T
         */
        void updateJointAngles(const Eigen::VectorXd &joint_angles);
        
        /** @brief Function to get the current set joint angles
         * @return join_angle VEctor: A 4x1 Matrix having all four joint angles,
         */
        Eigen::VectorXd getJointAngles() const;
        
        /** @brief Function to get the joint angles as per given end effector pose
         * @brief We can det 4 possible soultions of join_angle1,join_angle2,join_angle3
         * @param end_effector_pose: pose of the end effector as recieved by the subscriebr callback
         * @return join_angle VEctor: A 4x1 Matrix having all four joint angles,
         */
        Eigen::VectorXd getJointAnglesUsingIK(const Eigen::Isometry3d &end_effector_pose) const;

    private:

        /** @brief function to set the DH param matrix (dh_params_ var)  for the given link lengths and joint angles
         * @param link_lengths: A column vector having link lengths as [l1,l2,l3]^T
         * @param joint_angle: A column vector having joint angles as [q1,q2,q3]^T
         */
        void setupDHParams(const Eigen::VectorXd &link_lengths, const Eigen::VectorXd &joint_angles);

        // /** @brief Function to get the Jacobian based on joint angles as 0 degrees
        //  * @brief Jacobian matrix will be 6x3 matrix as it has 3 joints:
        //  *  [z_0x(O_3 - O_0)  z_1x(O_3 - O_1) z_2x(O_3 - O_2)]
        //  *  [     z_0              z_1             z_2       ]
        //  * @return jacobian matrix: A 6x3 Matrix having jacobian matrix
        //  */
        // Eigen::MatrixXd getJacobian(void) const;

        /** @brief A function to get homogenoous trasnformation matrix from Link number
         * @brief It directly fetch DH param a,theta,d,alpha for the given from the dh_params matrix
         * @brief make sure to update the joint angles before calling this function
         *  @brief Homogenous Matrix A_i for transformation between frame i to (i-1) as:
         * [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta)]
         * [sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)]
         * [0           sin(alpha)             cos(alpha)                 d      ]
         * [0              0                      0                       1      ]
         * @param link_number: Link number for which Homogenous matrix required
         * @return A_i:  a 4x4 Homgenous Transformation matrix A_i for the given DH param
         */
        Eigen::Matrix4d getHomogeneousMat(int link_number) const;

        /** @brief A function to get the end effector pose by multiplying all the transformation matrixes
         * @return A homogenous transformation matrix 
         */
        Eigen::Matrix4d calcEndEffectorPose() const;


        /** @brief  A col vector to store the link lengths as [l1,l2,l3,l4]^T*/
        Eigen::VectorXd link_lengths_;

        /** @brief  A col vector to store the joint angels as [q1,q2,q3,q4]^T*/
        Eigen::VectorXd joint_angles_;

        /** @brief  A  5x4 matrix to store the DH params as derived in the report:
         * [0   q1  l2 -pi/2]
         * [l2  q2  0   0   ]
         * [l3  q3  0  -pi/2]
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
        // Helper functions

        /** @brief Function to normalize angle to  make it between -pi to +pi range
         * @param rad: Angle in radians
         * @return Normalized angle in radians
         */
        template <typename T>
        inline T normalize(const T &rad) const
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
        inline T getDegrees(const T &rad) const
        {
            return ((normalize(rad) * 180.0) / (M_PI));
        }

    }; // ManipulatorCore
}//namespace manipulator
#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_CORE_HPP_
