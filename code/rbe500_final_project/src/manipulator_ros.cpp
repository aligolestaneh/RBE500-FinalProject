#include "rbe500_final_project/manipulator_ros.hpp"

// using namespace manipulator;
namespace manipulator
{
    ManipulatorROS::ManipulatorROS() : Node("manipulator_ros_node")
    {

        // read parameters for the link lenghts
        initNodeParams();

        // Define publishers
        initPublishers();

        // Define Services
        initServices();
        //  Define Subscribers
        initSubscribers();
    }

    void ManipulatorROS::initNodeParams()
    {
        // TODO THis section
        auto link_names = this->declare_parameter("manipulator_links", std::vector<std::string>{"link_0, link_1, link_2x, link_2y, link_3, link_4"});
        this->get_parameter("manipulator_links", link_names);

        link_length_ = Eigen::VectorXd(link_names.size());

        for (unsigned long int i = 0; i < link_names.size(); i++)
        {
            this->declare_parameter("link_lengths." + link_names[i], 0.1); // using default value of 0.1m
            this->get_parameter("link_lengths." + link_names[i], link_length_[i]);
        }

        RCLCPP_INFO(this->get_logger(), "All Node Parameters Loaded");
    }
    void ManipulatorROS::initPublishers()
    {
        // TODO THis section
    }
    void ManipulatorROS::initServices()
    {
        // TODO THis section
    }
    void ManipulatorROS::initSubscribers()
    {
        // TODO THis section
    }
    void ManipulatorROS::onSubscriberJointAngleCB(geometry_msgs::msg::Vector3::ConstSharedPtr input_msg)
    {
        // TODO this section
    }
}
