/**
 * @file manipulator_pid.hpp
 * @brief Core C++ library for OpenX Manipulator PD controller for a single joint
 * @details This library implements PD controller for the OpenX Manipulator.
 *          It provides functionality for:
 *          - Calculating Current for a given single joint position
 * @authors Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
 */

#ifndef RBE500_FINAL_PROJECT_PKG_MANIPULATOR_PD_HPP_
#define RBE500_FINAL_PROJECT_PKG_MANIPULATOR_PD_HPP_

#include "rbe500_final_project/helpers.hpp"

/** @namespace manipulator
 *  @brief Namespace containing manipulator PD Control functionality for Joint4
 */
namespace manipulator
{
    /** @class ManipulatorPD
     *  @brief Core class implementing manipulator PD control
     */
    // template <class double>
    class ManipulatorPD
    {

    public:
        /** @brief Constructor */
        ManipulatorPD();

        /** @brief Constructor */
        ManipulatorPD(const double &kp, const double &kd);

        /** @brief Destructor */
        ~ManipulatorPD() = default;

        /**
         * @brief Sets the Kp and Kd params
         * @param Kp Kp gain
         * @param Kd Kd gain
         */
        void setup(const double &Kp, const double &Kd);

        /**
         * @brief Sets the reference code
         * @param ref Reference value
         */
        void setReference(const double &ref);

        /** @brief function to Calculate controll effor using PD
         * @brief It skips the first iteration to update the prev_error_
         * @param current:Current data
         * @param dt: Delta time
         * @return True: If PD control calcs data
         */
        bool updateJoint(const double &current, const double &dt);

        /**
         * @brief Get the control data
         * @return Control
         */
        double getControl() const;

        /** @brief Function to reset all vars except Kp_, Kd_
         */
        void reset();

    private:
        /** @brief Kp gain*/
        double Kp_;

        /** @brief Kd gain */
        double Kd_;

        /** @brief Reference Signal*/
        double ref_;

        /** @brief Previous error for Derivative calculations */
        double prev_error_;

        /** @brief Control output from the PD controller*/
        double control_;

        /** @brief Flag to check the first data*/
        bool is_first_data_;

    }; // ManipulatorPD
} // namespace manipulator

#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_PD_HPP_
