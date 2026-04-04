#pragma once

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <string>
#include <sstream>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"

//  IGNITION TRANSPORT LIBRARY TO SEND INFO BY DIRECT IGNITION NODE
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <cmath>


namespace floating_controller {

    class FloatingController : public controller_interface::ControllerInterface {
    public:
        FloatingController();
        controller_interface::CallbackReturn on_init() override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        bool send_command_to_simulator(ignition::msgs::Pose & req, ignition::msgs::Boolean & rep,
            bool & result, double timeout, double _x, double _y, double _theta);
        
        void send_tfs_to_rviz(geometry_msgs::msg::TransformStamped & tf_msg, const rclcpp::Time & time,
            double _x, double _y, double _theta);

    protected:
        std::vector<std::string> joint_names;
        std::string interface_name;

        double x;
        double y;
        double theta;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        geometry_msgs::msg::Twist twist_command;

        ignition::transport::Node ign_node;
    };

} // namespace floating_controller