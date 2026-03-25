#pragma once

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

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

    protected:
        std::vector<std::string> joint_names;
        std::string interface_name;

        double x;
        double y;
        double theta;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
        geometry_msgs::msg::Twist twist_command;

        void teleport_robot();
    
    };

} // namespace floating_controller