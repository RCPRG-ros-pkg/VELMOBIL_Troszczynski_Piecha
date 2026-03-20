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


namespace floating_controller {

    FloatingController::FloatingController(): controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn FloatingController::on_init() {
        try {
            joint_names = auto_declare<std::vector<std::string>>("joints", {});
            interface_name = auto_declare<std::string>("interface_name", "velocity");

            RCLCPP_INFO(get_node()->get_logger(), "FloatingController initialized");

        } catch (const std::exception & e) {
            fprintf(stderr, "Exception thrown during init: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FloatingController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        if (joint_names.size() != 3) {
            RCLCPP_ERROR(get_node()->get_logger(),
                        "Expected exactly 3 joint names, but got %zu", joint_names.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        x_joint = joint_names[0];
        y_joint = joint_names[1];
        theta_joint = joint_names[2];

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

        for (const auto& joint_name : joint_names) {
            config.names.push_back(joint_name + "/" + interface_name);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration FloatingController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names.size() * 2); // Velocity and Position

        for (const auto& joint_name : joint_names) {
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

    controller_interface::return_type FloatingController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
        // Extract linear and angular velocity commands from Twist message
        double vx_linear  = twist_command.linear.x; // Vx
        double vy_linear  = twist_command.linear.x; // Vy
        double omega = twist_command.angular.z; // omega

        const double rad_per_sec_to_rpm = 60.0 / (2.0 * M_PI);

        double dt = period.seconds();
        // Convert linear velocity from mm/s to m/s
        theta += omega * rad_per_sec_to_rpm; // theta state

        vx_linear = vx_linear / 1000.0;
        vy_linear = vy_linear / 1000.0;

        x += (vx_linear*cos(theta) - vy_linear*sin(theta)) * dt; // x state
        y += (vx_linear*sin(theta) + vy_linear*cos(theta)) * dt; // y state


        RCLCPP_DEBUG(get_node()->get_logger(), "Calculated State: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);


        if (command_interfaces_.size() >= 3) {
            command_interfaces_[0].set_value(vx_linear);
            command_interfaces_[1].set_value(vy_linear);
            command_interfaces_[2].set_value(omega);

            RCLCPP_DEBUG(get_node()->get_logger(),
                        "Sent commands: vx=%.3f, vy=%.3f, omega=%.3f",
                        vx_linear, vy_linear, omega);
        } else {
            RCLCPP_ERROR(get_node()->get_logger(),
                        "Insufficient command interfaces");
            return controller_interface::return_type::ERROR;
        }

            return controller_interface::return_type::OK;
        }


    controller_interface::CallbackReturn FloatingController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        if (command_interfaces_.size() >= 3) {
            command_interfaces_[0].set_value(0.0);
            command_interfaces_[1].set_value(0.0);
            command_interfaces_[2].set_value(0.0);
        }

        RCLCPP_INFO(get_node()->get_logger(), "FloatingController deactivated. Motors set to zero.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

} // namespace floating_controller

PLUGINLIB_EXPORT_CLASS(floating_controller::FloatingController, controller_interface::ControllerInterface)