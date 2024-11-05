#include <rbe500_final_project/manipulator_ros.hpp>

using namespace manipulator;

ManipulatorROS::ManipulatorROS() : Node("manipulator_ros_node")
{
    //read parameters for the link lenghts
    initNodeParams();
    
    //Define publishers
    initPublishers();
    
    //Define Services
    initServices();
    //  Define Subscribers
    initSubscribers();
}
void ManipulatorROS::initNodeParams()
{
    //TODO THis section
    
}
void ManipulatorROS::initPublishers()
{
    //TODO THis section
    
}
void ManipulatorROS::initServices()
{
    //TODO THis section
    
}
void ManipulatorROS::initSubscribers()
{
    //TODO THis section
    
}
void ManipulatorROS::onSubscriberJointAngleCB(geometry_msgs::msg::Vector3::ConstSharedPtr input_msg)
{
    //TODO this section
}
