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
            wheel_x_offsets = auto_declare<std::vector<double>>("wheel_x_offsets", {});
            wheel_y_offsets = auto_declare<std::vector<double>>("wheel_y_offsets", {});

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

    void RollYawController::publishOdom(const rclcpp::Time & time) {
        double orientation_z = std::sin(theta / 2.0);
        double orientation_w = std::cos(theta / 2.0);
        auto odometry_msg = nav_msgs::msg::Odometry();
        odometry_msg.header.frame_id = "/odom";
        odometry_msg.header.stamp = get_node()->now();
        odometry_msg.child_frame_id = "/base_footprint";
        odometry_msg.pose.pose.position.x = x;
        odometry_msg.pose.pose.position.y = y;
        odometry_msg.pose.pose.position.z = 0.0;
        odometry_msg.pose.pose.orientation.x = 0.0;
        odometry_msg.pose.pose.orientation.y = 0.0;
        odometry_msg.pose.pose.orientation.z = orientation_z;
        odometry_msg.pose.pose.orientation.w = orientation_w;
        odometry_msg.twist.twist = twist_command;
        odom_publisher->publish(odometry_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = time;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id  = "base_footprint";

        tf_msg.transform.translation.x = x;
        tf_msg.transform.translation.y = y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = orientation_z;
        tf_msg.transform.rotation.w = orientation_w;

        tf_broadcaster -> sendTransform(tf_msg);
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
        double vx_linear = twist_command.linear.x;
        double vy_linear = twist_command.linear.y;
        double omega = twist_command.angular.z;
        double dt = period.seconds();

        for (size_t i = 0; i < wheel_joint_names.size(); i++) {
            double vx_wheel = vx_linear - omega * wheel_y_offsets[i];
            double vy_wheel = vy_linear + omega * wheel_x_offsets[i];

            double v_wheel = std::sqrt(std::pow(vx_wheel, 2) + std::pow(vy_wheel, 2));
            double angle_wheel = std::atan2(vy_wheel, vx_wheel);

            command_interfaces_[i].set_value(v_wheel / wheel_radius);
            command_interfaces_[wheel_joint_names.size() + i].set_value(angle_wheel);
        }

        theta += omega * dt;
        
        x += (vx_linear * std::cos(theta) - vy_linear * std::sin(theta)) * dt;
        y += (vx_linear * std::sin(theta) + vy_linear * std::cos(theta)) * dt;

        publishOdom(time);

        return controller_interface::return_type::OK;
    }


} // namespace roll_yaw_controller

PLUGINLIB_EXPORT_CLASS(roll_yaw_controller::RollYawController, controller_interface::ControllerInterface)