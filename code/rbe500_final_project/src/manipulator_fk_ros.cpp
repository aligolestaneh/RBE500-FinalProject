#include "rbe500_final_project/manipulator_fk_ros.hpp"

// using namespace manipulator;
namespace manipulator
{
    ManipulatorFKROS::ManipulatorFKROS() : Node("manipulator_fk_ros_node")
    {

        // init manipulator core
        manipulator_ = std::make_shared<manipulator::ManipulatorCore>();
        //init priavte vars
        joint_angles_ = Eigen::Vector4d::Zero();
        // end_effector_pose_ = Eigen::Isometry3d::Identity();
        // read parameters for the link lenghts
        initNodeParams();
        // setup manipulator

        if (!manipulator_->setup(link_length_, true))
        {
            RCLCPP_FATAL(this->get_logger(), "Unable to setup  manipulatorCore, check your params!");
            exit(0);
        }
        RCLCPP_INFO(this->get_logger(), "ManipulatorCore setup done!");
        // Define publishers
        initPublishers();

        //  Define Subscribers
        initSubscribers();
    }

    void ManipulatorFKROS::initNodeParams()
    {
        // TODO THis section
        auto link_names = this->declare_parameter("manipulator_links", std::vector<std::string>{"link_0, link_1, link_2x, link_2y, link_3, link_4"});
        this->get_parameter("manipulator_links", link_names);

        link_length_ = Eigen::VectorXd(link_names.size());

        for (unsigned long int i = 0; i < link_names.size(); i++)
        {
            this->declare_parameter("link_lengths." + link_names[i], 0.1); // using default value of 0.1m
            this->get_parameter("link_lengths." + link_names[i], link_length_(i));
        }
    }
    void ManipulatorFKROS::initPublishers()
    {
        // TODO THis section
        rclcpp::QoS durable_qos(1);
        durable_qos.transient_local(); // option for latching

        end_effector_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/end_effector_pose", durable_qos);
    }
    void ManipulatorFKROS::initSubscribers()
    {
        // TODO THis section
        
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, std::bind(&ManipulatorFKROS::onSubscriberJointAngleCB, this, std::placeholders::_1));
    }
    void ManipulatorFKROS::onSubscriberJointAngleCB(sensor_msgs::msg::JointState::ConstSharedPtr input_msg)
    {
        // TODO this section
        joint_angles_(0) = input_msg->position.at(0);
        joint_angles_(1) = input_msg->position.at(1);
        joint_angles_(2) = input_msg->position.at(2);
        joint_angles_(3) = input_msg->position.at(3);
        manipulator_->updateJointAngles(joint_angles_);
        
        end_effector_pose_ = helpers::convertIsometry3dToPoseStamped(manipulator_->getEndEffectorPose(), input_msg->header);

        end_effector_pose_.header.frame_id = "link1";
        end_effector_pose_pub_->publish(end_effector_pose_);
    }
}
