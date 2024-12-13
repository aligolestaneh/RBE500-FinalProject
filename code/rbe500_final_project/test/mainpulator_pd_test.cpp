#include "gtest/gtest.h"
#include "rbe500_final_project/manipulator_pd.hpp"



// Test default constructor
TEST(ManipulatorPDTest, DefaultConstructor) {
    std::shared_ptr<manipulator::ManipulatorPD> defaultController = std::make_shared<manipulator::ManipulatorPD>();
    double ref = 10.0;
    double current  = 5.0;
    double dt = 0.01;
    defaultController->setReference(ref);
    EXPECT_FALSE(defaultController->updateJoint(current, dt))<<" Did not skipped first loop";
}

TEST(ManipulatorPDTest, zeroDTTest) {
    std::shared_ptr<manipulator::ManipulatorPD> defaultController = std::make_shared<manipulator::ManipulatorPD>();
    double ref = 10.0;
    double current  = 5.0;
    double dt = 0.01;
    double dt_zero = 0.0;
    defaultController->setReference(ref);
    defaultController->updateJoint(current, dt);
    
    EXPECT_FALSE(defaultController->updateJoint(current, dt_zero))<<"  Did not skipped zero dt";
}

TEST(CPPTest, intToDoubleChange)
{
    uint32_t input_angle = 245;
    double d_input_angle = static_cast<double>(input_angle);
    uint32_t n_input = static_cast<uint32_t>(d_input_angle);
    std::cout<<"INput angle: "<<input_angle<<std::endl;
    std::cout<<"Doubl eINput angle: "<<d_input_angle<<std::endl;
    EXPECT_EQ(input_angle,n_input);
    ASSERT_TRUE(true);
}

TEST(CPPTest, negaitveToUintChange)
{
    int neg_input = -10;
    uint32_t input_angle = 65526;
    uint32_t n_input = static_cast<uint16_t>(neg_input);

    std::cout<<"INput angle: "<<input_angle<<std::endl;
    std::cout<<"New eINput angle: "<<n_input<<std::endl;
    EXPECT_EQ(input_angle,n_input);
    ASSERT_TRUE(true);
}