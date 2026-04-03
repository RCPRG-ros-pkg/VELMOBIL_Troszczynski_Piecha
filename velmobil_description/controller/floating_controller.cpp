#include "floating_controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include "pluginlib/class_list_macros.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/twist.hpp"


// IT PUBLISHES VX, VY, OMEGA TO IGNITION DIRECTLY

namespace floating_controller {

    FloatingController::FloatingController(): controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn FloatingController::on_init() {
        try {
            joint_names = auto_declare<std::vector<std::string>>("joints", {});
            interface_name = auto_declare<std::string>("interface_name", "velocity");

            RCLCPP_INFO(get_node()->get_logger(), "FloatingController initialized");

        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FloatingController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {

        auto callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
            // Store the latest Twist command
            twist_command = *msg;
            RCLCPP_DEBUG(get_node()->get_logger(), "Received Twist: linear.x=%.4f, linear.y=%.4f,, angular.z=%.4f", twist_command.linear.x, twist_command.linear.y, twist_command.angular.z);
        };

        // Create the subscriber for Twist messages
        cmd_vel_subscriber = get_node()->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, callback); 
        RCLCPP_INFO(get_node()->get_logger(), "FloatingController configured. Subscribed to /cmd_vel.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration FloatingController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names.size());

        for (const auto & joint_name : joint_names){
            config.names.push_back(joint_name + "/" + interface_name);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration FloatingController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names.size() * 2); // Velocity and Position
        
        for (const auto & joint_name : joint_names) {
            config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
            config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        }
        return config;
    }

    controller_interface::CallbackReturn FloatingController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        twist_command = geometry_msgs::msg::Twist();
        RCLCPP_INFO(get_node()->get_logger(), "FloatingController activated.");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FloatingController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (auto & cmd_if : command_interfaces_)
            cmd_if.set_value(0.0);
        RCLCPP_INFO(get_node()->get_logger(), "FloatingController deactivated. Motors set to zero.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type FloatingController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        double vx_linear  = twist_command.linear.x; // Vx
        double vy_linear  = twist_command.linear.y; // Vy
        double omega = twist_command.angular.z; // omega
        double dt    = period.seconds();

        theta += omega * dt;
        
        x += (vx_linear * std::cos(theta) - vy_linear * std::sin(theta)) * dt;
        y += (vx_linear * std::sin(theta) + vy_linear * std::cos(theta)) * dt;

        for (auto & cmd_if : command_interfaces_)
            cmd_if.set_value(0.0);

        ignition::msgs::Pose request;
        ignition::msgs::Boolean response;
        bool result;

        request.set_name("velmobil");
        request.mutable_position()->set_x(x);
        request.mutable_position()->set_y(y);
        request.mutable_position()->set_z(0.0);
        request.mutable_orientation()->set_x(0.0);
        request.mutable_orientation()->set_y(0.0);
        request.mutable_orientation()->set_z(std::sin(theta / 2.0));
        request.mutable_orientation()->set_w(std::cos(theta / 2.0));

        bool executed = ign_node.Request("/world/empty/set_pose", request, 100, response, result);

        if (!executed || !result) {
            RCLCPP_WARN_THROTTLE(get_node() -> get_logger(), *get_node() -> get_clock(), 2000, "set_pose request failed");
        }

        return controller_interface::return_type::OK;
    }

} // namespace floating_controller

PLUGINLIB_EXPORT_CLASS(floating_controller::FloatingController, controller_interface::ControllerInterface)