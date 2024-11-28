#include "rbe500_final_project/manipulator_ik_ros.hpp"

// using namespace manipulator;
namespace manipulator
{
    ManipulatorIKROS::ManipulatorIKROS() : Node("manipulator_ik_ros_node")
    {

        // init manipulator core
        manipulator_ = std::make_shared<manipulator::ManipulatorCore>();
        end_effector_pose_ = Eigen::Isometry3d::Identity();
        joint_angles_ = Eigen::Vector4d::Zero();
        // read parameters for the link lenghts
        initNodeParams();
        // setup manipulator

        if (use_newton_raphson_ik_)
            manipulator_->setIKParams(ik_max_iteration_, ik_tolerance_);

        if (!manipulator_->setup(link_length_, use_newton_raphson_ik_))
        {
            RCLCPP_FATAL(this->get_logger(), "Unable to setup  manipulatorCore, check your params!");
            exit(0);
        }
        RCLCPP_INFO(this->get_logger(), "ManipulatorCore setup done!");
        // Define publishers

        // Define Services
        initServices();
    }

    void ManipulatorIKROS::initNodeParams()
    {
        // TODO THis section
        ik_max_iteration_ = this->declare_parameter("ik_max_iteration", 100);
        this->get_parameter("ik_max_iteration", ik_max_iteration_);

        ik_tolerance_ = this->declare_parameter("ik_tolerance", 1e-6);
        this->get_parameter("ik_tolerance", ik_tolerance_);

        use_newton_raphson_ik_ = this->declare_parameter("use_newton_raphson_ik", true);
        this->get_parameter("use_newton_raphson_ik", use_newton_raphson_ik_);

        auto link_names = this->declare_parameter("manipulator_links", std::vector<std::string>{"link_0, link_1, link_2x, link_2y, link_3, link_4"});
        this->get_parameter("manipulator_links", link_names);

        link_length_ = Eigen::VectorXd(link_names.size());

        for (unsigned long int i = 0; i < link_names.size(); i++)
        {
            this->declare_parameter("link_lengths." + link_names[i], 0.1); // using default value of 0.1m
            this->get_parameter("link_lengths." + link_names[i], link_length_(i));

        }

        RCLCPP_INFO(this->get_logger(), "link_names Size: %ld", link_names.size());

        RCLCPP_INFO(this->get_logger(), "ik_max_iteration: %d", ik_max_iteration_);
        RCLCPP_INFO(this->get_logger(), "ik_tolerance: %f", ik_tolerance_);
        RCLCPP_INFO(this->get_logger(), "use_newton_raphson_ik: %s", use_newton_raphson_ik_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "All Node Parameters Loaded");
    }

    void ManipulatorIKROS::initServices()
    {
        ik_server_ = this->create_service<rbe500_final_project_msgs::srv::GetJointAngles>("get_joint_angles_ik",
                                                                                          std::bind(&ManipulatorIKROS::onServiceCB,
                                                                                                    this,
                                                                                                    std::placeholders::_1,
                                                                                                    std::placeholders::_2));
    }

    void ManipulatorIKROS::onServiceCB(const std::shared_ptr<rbe500_final_project_msgs::srv::GetJointAngles::Request> request,
                                       std::shared_ptr<rbe500_final_project_msgs::srv::GetJointAngles::Response> response)
    {
        
        end_effector_pose_ = helpers::convertPosetoIsometry3d(request->end_effector_pose);
        response->success = manipulator_->updateEndEffectorPose(end_effector_pose_);
        if (response->success)
        {
            joint_angles_ = manipulator_->getJointAngles();
            
            for (long int i = 0; i < joint_angles_.size(); i++)
                response->joint_angles.push_back(joint_angles_[i]);

            response->msg = "IK solved";
        }
        else
            response->msg = "unable to get the joint angles";
    }

}
