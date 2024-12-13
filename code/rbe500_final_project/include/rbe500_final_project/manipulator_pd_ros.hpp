/**
 * @file manipulator_pd_ros.hpp
 * @brief ROS 2 wrapper for updatting joint angles with PD controller
 * @details This node calls service to get current joint angle and uses it as a feedback for PD controller
 * @details Tht output of PD is current which is then published to move the manipulator
 * @authors Kashif Khurshid Noori, Ali Golestaneh, Hrishikesh Nirgude
 */

#ifndef RBE500_FINAL_PROJECT_PKG_MANIPULATOR_PD_ROS_HPP_
#define RBE500_FINAL_PROJECT_PKG_MANIPULATOR_PD_ROS_HPP_

// Standard Core C++ libs
#include <memory>
#include <string>
#include <cmath>
#include <mutex>
#include <chrono>

// ROS 2 core inlcudes
#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk_custom_interfaces/srv/get_position.hpp>
#include <dynamixel_sdk_custom_interfaces/srv/get_current.hpp>
#include <dynamixel_sdk_custom_interfaces/msg/set_current.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rbe500_final_project/manipulator_pd.hpp>
#include <rbe500_final_project/helpers.hpp>


namespace manipulator
{
    class ManipulatorPDROS : public rclcpp::Node
    {

    public:
        /**
         * @brief Constructor for the ManipulatorPDROS class
         * @details Initializes the ROS 2 node, sets up parameters, subscribers, and publishers
         */
        ManipulatorPDROS();
        /**
         * @brief Default destructor for the ManipulatorPDROS class
         */
        ~ManipulatorPDROS() = default;

        /**
         * @brief Moves the manipulator by publihsing current based on PD controller outut
         */
        void moveManipulator();

    private:
        /**
         * @brief Initializes the node parameters
         */
        void initNodeParams();

        /**
         * @brief Initializes the node subscribers
         */
        void initPublishers();

        /**
         * @brief Initializes the node service clients
         */
        void initClients();

        /**
         * @brief Joint Current Publisher
         * @details Publishes to /set_current topic with a joint_id_
         */
        rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetCurrent>::SharedPtr target_current_pub_;

        /**
         * @brief PD controller debug output
         * @details Publishes to /pd_output_viz topic for PD data plotting
         */
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr output_viz_pub_;

        /** @brief Service client for getting joint position*/
        rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr joint_position_client_;

        /** @brief Service client for getting joint current*/
        rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetCurrent>::SharedPtr joint_current_client_;
        
        /**
        * @brief Timestamp of the last PD control loop
        */
        rclcpp::Time last_time_;

        /**
         * @brief Flag indicating if this is the first PD controll loop
         * @details It helps to initialize the prev_error and last_time vars
         */
        bool first_loop_;

        /**
         * @brief Target joint angle for the manipulator
         */
        int target_joint_angle_;

        

        /**
         * @brief Current joint angle of the manipulator
         */
        int current_joint_angle_;

        /**
         * @brief Sample time for the PD control
         */
        double sample_time_;

        /**
         * @brief Kp gain for the PD control
         */
        double kp_;

        /**
         * @brief Kf gain for the PD control
         */
        double kd_;

        /**
         * @brief Min Limit of PD controller ouput
         * @details Used in clamping PD controller current output
         */
        int min_current_limit_;
        /**
         * @brief Max Limit of PD controller ouput
         * @details Used in clamping PD controller current output
         */
        int max_current_limit_;
        /**
         * @brief Joint id of the OpenX Manipulator joint4
         * @details Used to interface with dynamixel sdk
         */
        int joint_id_;

        /**
         * @brief Current AMP of the joint4 for the openX Manipulator
         */
        int current_amp_;

        /**
         * @brief Function to call the /get_position client and gets the current joint angle
         * @param joint_angle: Recieved joint angle from the service call
         * @return True: If service call is successfull
         * @return False: Otherwise
         */
        bool callJointPositionService(int & joint_angle);

        /**
         * @brief Function to call the /get_current client and gets the current joint ampere
         * @param current: Recieved joint amp from the service call
         * @return True: If service call is successfull
         * @return False: Otherwise
         */
        bool callCurrentService(int & current);

          /**
         * @brief PD controller library object for getting control output
         */
        std::shared_ptr<ManipulatorPD> pd_controller_;


    }; // ManipulatorPDROS
}
#endif // RBE500_FINAL_PROJECT_PKG_MANIPULATOR_PD_ROS_HPP_
