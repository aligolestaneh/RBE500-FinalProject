#include <rbe500_final_project/manipulator_pd.hpp>
#include <iostream>
// using namespace manipulator;
namespace manipulator
{
    
    ManipulatorPD::ManipulatorPD() : is_first_data_(true) {}

    
    ManipulatorPD::ManipulatorPD(const double &kp, const double &kd) : Kp_(kp), Kd_(kd), is_first_data_(true) {}

    
    void ManipulatorPD::setup(const double &Kp, const double &Kd)
    {
        Kp_ = Kp;
        Kd_ = Kd;
    }

    
    void ManipulatorPD::setReference(const double &ref)
    {
        ref_ = ref;
    }

    
    bool ManipulatorPD::updateJoint(const double &current, const double &dt)
    {
        if(dt<1e-6)
            return false;
        double error = ref_ - current;
        // error = helpers::normalize_angle(error);
        if (is_first_data_)
        {
            is_first_data_ = false;
            prev_error_ = error;
            return false;
        }

        

        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        control_ = Kp_ * error + Kd_ * derivative;
        // std::cout<<"Error: "<<error<<" | output: "<<control_<<std::endl;
        return true;
    }
    
    void ManipulatorPD::reset() 
    {
        prev_error_ = 0;
        is_first_data_ = true;
    }
    
    double ManipulatorPD::getControl() const
    {
        return control_;
    }
}