#include "roll_yaw_controller.hpp"
#include <algorithm>
#include <cstdlib>
#include <numbers>
#include <cmath>
#include <limits>
#include <memory>
#include "pluginlib/class_list_macros.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/twist.hpp"



namespace roll_yaw_controller {


    RollYawController::RollYawController(): controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn RollYawController::on_init() {
        try {
            wheel_joint_names = auto_declare<std::vector<std::string>>("wheel_joints", {});
            yaw_joint_names = auto_declare<std::vector<std::string>>("yaw_joints", {});
            wheel_radius = auto_declare<double>("wheel_radius", 0.1016);

            RCLCPP_INFO(get_node()->get_logger(), "RollYawController initialized");

        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RollYawController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {

        auto callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
            twist_command = *msg;
            RCLCPP_DEBUG(get_node()->get_logger(), "Received Twist: linear.x=%.4f, linear.y=%.4f,, angular.z=%.4f", twist_command.linear.x, twist_command.linear.y, twist_command.angular.z);
        };

        cmd_vel_subscriber = get_node()->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, callback); 
        odom_publisher = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());
        RCLCPP_INFO(get_node()->get_logger(), "RollYawController configured. Subscribed to /cmd_vel, and publishing /odom.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void RollYawController::publishOdom() {
        auto odometry_msg = nav_msgs::msg::Odometry();
        odometry_msg.header.frame_id = "/odom";
        odometry_msg.header.stamp = get_node()->now();
        odometry_msg.child_frame_id = "/base_footprint";
        odometry_msg.pose.pose.position.x = x;
        odometry_msg.pose.pose.position.y = y;
        odometry_msg.pose.pose.position.z = 0.0;
        odometry_msg.pose.pose.orientation.x = 0.0;
        odometry_msg.pose.pose.orientation.y = 0.0;
        odometry_msg.pose.pose.orientation.z = std::sin(theta / 2.0);
        odometry_msg.pose.pose.orientation.w = std::cos(theta / 2.0);
        odometry_msg.twist.twist = twist_command;
        odom_publisher->publish(odometry_msg);
    }

    controller_interface::InterfaceConfiguration RollYawController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(wheel_joint_names.size() + yaw_joint_names.size());


        for (const auto & joint_name : wheel_joint_names) {
            config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        for (const auto & joint_name : yaw_joint_names) {
            config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration RollYawController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(wheel_joint_names.size() * 2 + yaw_joint_names.size());

        for (const auto & joint_name : wheel_joint_names) {
            config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
            config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        for (const auto & joint_name : yaw_joint_names) {
            config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        }
        return config;
    }

    controller_interface::CallbackReturn RollYawController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        twist_command = geometry_msgs::msg::Twist();
        x = 0.0; 
        y = 0.0; 
        theta = 0.0;
        
        for (size_t i = 0; i < wheel_joint_names.size() * 2; i++) {
            command_interfaces_[i].set_value(0.0);
        }
        
        RCLCPP_INFO(get_node()->get_logger(), "RollYawController activated.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RollYawController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        for (size_t i = 0; i < wheel_joint_names.size(); i++) {
            command_interfaces_[i].set_value(0.0);
        }
        RCLCPP_INFO(get_node()->get_logger(), "RollYawController deactivated. Motors set to zero.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RollYawController::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void)time; //useless for now...
        double vx_linear  = twist_command.linear.x; // Vx
        double vy_linear  = twist_command.linear.y; // Vy
        double omega = twist_command.angular.z; // omega
        double vy_front = vy_linear + omega;
        double vy_back = vy_linear - omega;
        double dt = period.seconds();
        
        double v_linear_front = std::sqrt(std::pow(vx_linear,2) + std::pow(vy_front,2));
        double angle_front = std::atan2(vy_front, vx_linear);
        
        double v_linear_back = std::sqrt(std::pow(vx_linear,2) + std::pow(vy_back,2));
        double angle_back = std::atan2(vy_back, vx_linear);

        command_interfaces_[0].set_value(v_linear_front / wheel_radius);
        command_interfaces_[1].set_value(v_linear_front / wheel_radius);
        command_interfaces_[2].set_value(v_linear_back / wheel_radius);
        command_interfaces_[3].set_value(v_linear_back / wheel_radius);
        command_interfaces_[4].set_value(angle_front);
        command_interfaces_[5].set_value(angle_front);
        command_interfaces_[6].set_value(angle_back);
        command_interfaces_[7].set_value(angle_back);

        theta += omega * dt;
        
        x += (vx_linear * std::cos(theta) - vy_linear * std::sin(theta)) * dt;
        y += (vx_linear * std::sin(theta) + vy_linear * std::cos(theta)) * dt;

        publishOdom();

        return controller_interface::return_type::OK;
    }


} // namespace roll_yaw_controller

PLUGINLIB_EXPORT_CLASS(roll_yaw_controller::RollYawController, controller_interface::ControllerInterface)