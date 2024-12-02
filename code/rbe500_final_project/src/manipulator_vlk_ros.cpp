#include "rbe500_final_project/manipulator_vlk_ros.hpp"

// using namespace manipulator;
namespace manipulator
{
    ManipulatorVLKROS::ManipulatorVLKROS() : Node("manipulator_vlk_ros_node")
    {

        // init manipulator core
        manipulator_ = std::make_shared<manipulator::ManipulatorCore>();

        joint_angles_ = Eigen::Vector4d::Zero();
        // read parameters for the link lenghts
        initNodeParams();
        // setup manipulator

        if (!manipulator_->setup(link_length_, true))
        {
            RCLCPP_FATAL(this->get_logger(), "Unable to setup  manipulatorCore, check your params!");
            exit(0);
        }
        RCLCPP_INFO(this->get_logger(), "ManipulatorCore setup done!");

        //  Define Subscribers
        initSubscribers();
        // Define Services
        initServices();
    }

    void ManipulatorVLKROS::initNodeParams()
    {
        auto link_names = this->declare_parameter("manipulator_links", std::vector<std::string>{"link_0, link_1, link_2x, link_2y, link_3, link_4"});
        this->get_parameter("manipulator_links", link_names);

        link_length_ = Eigen::VectorXd(link_names.size());

        for (unsigned long int i = 0; i < link_names.size(); i++)
        {
            this->declare_parameter("link_lengths." + link_names[i], 0.1); // using default value of 0.1m
            this->get_parameter("link_lengths." + link_names[i], link_length_(i));
        }

        RCLCPP_INFO(this->get_logger(), "link_names Size: %ld", link_names.size());

        RCLCPP_INFO(this->get_logger(), "All Node Parameters Loaded");
    }

    void ManipulatorVLKROS::initServices()
    {
        ee_twist_server_ = this->create_service<rbe500_final_project_msgs::srv::GetEndEffectorTwist>("get_end_effector_twist",
                                                                                          std::bind(&ManipulatorVLKROS::onEETwistServiceCB,
                                                                                                    this,
                                                                                                    std::placeholders::_1,
                                                                                                    std::placeholders::_2));
        joint_vel_server_ = this->create_service<rbe500_final_project_msgs::srv::GetJointVelocities>("get_joint_velocities",
                                                                                          std::bind(&ManipulatorVLKROS::onJointVelServiceCB,
                                                                                                    this,
                                                                                                    std::placeholders::_1,
                                                                                                    std::placeholders::_2));                                                                                                    
    }
    void ManipulatorVLKROS::initSubscribers()
    {
        // TODO THis section
        
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, std::bind(&ManipulatorVLKROS::onSubscriberJointAngleCB, this, std::placeholders::_1));
    }
    void ManipulatorVLKROS::onSubscriberJointAngleCB(sensor_msgs::msg::JointState::ConstSharedPtr input_msg)
    {
        // TODO this section
        joint_angles_(0) = input_msg->position.at(0);
        joint_angles_(1) = input_msg->position.at(1);
        joint_angles_(2) = input_msg->position.at(2);
        joint_angles_(3) = input_msg->position.at(3);
        manipulator_->updateJointAngles(joint_angles_);

    }
    void ManipulatorVLKROS::onEETwistServiceCB(const std::shared_ptr<rbe500_final_project_msgs::srv::GetEndEffectorTwist::Request> request,
                                        std::shared_ptr<rbe500_final_project_msgs::srv::GetEndEffectorTwist::Response> response)
    {
        Eigen::VectorXd joint_velocities = helpers::convertStdVectorToEigenVector(request->joint_velocities);
        Eigen::VectorXd ee_twist = Eigen::VectorXd(6);
        
        response->success = manipulator_->updateJointVelocities(joint_velocities);
        if (response->success)
        {
            ee_twist = manipulator_->getEndEffectorTwist();

            response->end_effector_twist = helpers::convertEigenVector6dToTwist(ee_twist);
            response->msg = "End effector Twist Calculated";
        }
        else
            response->msg = "Unable to update given joint velocities";
    }

    void ManipulatorVLKROS::onJointVelServiceCB(const std::shared_ptr<rbe500_final_project_msgs::srv::GetJointVelocities::Request> request,
                                        std::shared_ptr<rbe500_final_project_msgs::srv::GetJointVelocities::Response> response)
    {
        Eigen::VectorXd ee_twist  = helpers::convertTwistToEigenVector6d(request->end_effector_twist);
        Eigen::VectorXd joint_velocities = Eigen::VectorXd(4);
        
        response->success = manipulator_->updateEndEffectorTwist(ee_twist);
        if (response->success)
        {
            joint_velocities = manipulator_->getJointVelocities();

            response->joint_velocities = helpers::convertEigenVectorToStdVector(joint_velocities);
            response->msg = "Joint Velocities Calculated";
        }
        else
            response->msg = "Unable to update End effector twist";
    }

}
