#include "gtest/gtest.h"
#include "rbe500_final_project/manipulator_core.hpp"

TEST(ManipulatorCore, core_object_test)
{
    std::shared_ptr<manipulator::ManipulatorCore> mymanip = std::make_shared<manipulator::ManipulatorCore>();
    
    ASSERT_EQ(4,2+2);
}
