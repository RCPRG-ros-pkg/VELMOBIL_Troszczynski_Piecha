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


/*
BEST TUTORIAL EVER:
    https://github.com/masum919/my_custom_controller/tree/main

NEED TO PUT THIS STUFF INTO SOME DOCS... instead of writing comments in code
*/


// WHAT IT DOES: it publisher vx, vy, omega, BUT gz cannot handle that planar kinematics. So it actualy publishes Kinematic TF.

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
        double vy_linear  = twist_command.linear.y; // Vy
        double omega = twist_command.angular.z; // omega

        const double rad_per_sec_to_rpm = 60.0 / (2.0 * M_PI);

        double dt = period.seconds();
        // Convert linear velocity from mm/s to m/s
        theta += omega * rad_per_sec_to_rpm; // theta state

        x += (vx_linear*cos(theta) - vy_linear*sin(theta)) * dt; // x state
        y += (vx_linear*sin(theta) + vy_linear*cos(theta)) * dt; // y state


        RCLCPP_DEBUG(get_node()->get_logger(), "Calculated State: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);


        if (command_interfaces_.size() >= 0) {

            RCLCPP_DEBUG(get_node()->get_logger(),
                        "Sent commands: vx=%.3f, vy=%.3f, omega=%.3f",
                        vx_linear, vy_linear, omega);
        } else {
            RCLCPP_ERROR(get_node()->get_logger(),
                        "Insufficient command interfaces");
            return controller_interface::return_type::ERROR;
        }

        teleport_robot();

        return controller_interface::return_type::OK;
    }


    controller_interface::CallbackReturn FloatingController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(get_node()->get_logger(), "FloatingController deactivated. Motors set to zero.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void FloatingController::teleport_robot() {
        std::stringstream cmd;

        cmd << "ign service "
            << "-s /world/empty/set_pose "
            << "--reqtype ignition.msgs.Pose "
            << "--reptype ignition.msgs.Boolean "
            << "--timeout 2000 "
            << "--req 'name: \"velmobil\", position: {x: " << x << ", y: " << y <<", z: 0.0}'";


        std::system((cmd.str() + " > /dev/null").c_str());
    }


} // namespace floating_controller

PLUGINLIB_EXPORT_CLASS(floating_controller::FloatingController, controller_interface::ControllerInterface)