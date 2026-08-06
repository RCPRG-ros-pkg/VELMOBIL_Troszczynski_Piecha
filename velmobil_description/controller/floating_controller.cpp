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
        odom_publisher = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());
        RCLCPP_INFO(get_node()->get_logger(), "FloatingController configured. Subscribed to /cmd_vel, and publishing /odom.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void FloatingController::odomTimerCallback() {
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
        // odometry_msg.twist.twist.linear.x
        // odometry_msg.twist.twist.linear.y
        // odometry_msg.twist.twist.angular.z
        odom_publisher->publish(odometry_msg);
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
        
        // SET INITIAL POSE IN ORDER TO AVOID SIMULATOR CRASH 
        x = 0.0; 
        y = 0.0; 
        theta = 0.0;
        ignition::msgs::Pose request;
        ignition::msgs::Boolean response;
        bool result;

        bool executed = send_command_to_simulator(request, response, result, 1000, x, y, theta);

        if (executed && result){
            RCLCPP_INFO(get_node() -> get_logger(), "FloatingController activated");
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FloatingController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "FloatingController deactivated. Motors set to zero.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type FloatingController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        double vx_linear  = twist_command.linear.x; // Vx
        double vy_linear  = twist_command.linear.y; // Vy
        double omega = twist_command.angular.z; // omega
        double dt = period.seconds();

        theta += omega * dt;
        
        x += (vx_linear * std::cos(theta) - vy_linear * std::sin(theta)) * dt;
        y += (vx_linear * std::sin(theta) + vy_linear * std::cos(theta)) * dt;

        geometry_msgs::msg::TransformStamped tf_msg;

        send_tfs_to_rviz(tf_msg, time, x, y, theta);

        ignition::msgs::Pose request;
        ignition::msgs::Boolean response;
        bool result;

        auto odometry_msg = nav_msgs::msg::Odometry();
        odometry_msg.header.frame_id = "odom";
        odometry_msg.header.stamp = get_node()->now();
        odometry_msg.child_frame_id = "base_footprint";
        odometry_msg.pose.pose.position.x = x;
        odometry_msg.pose.pose.position.y = y;
        odometry_msg.pose.pose.position.z = 0.0;
        odometry_msg.pose.pose.orientation.x = 0.0;
        odometry_msg.pose.pose.orientation.y = 0.0;
        odometry_msg.pose.pose.orientation.z = std::sin(theta / 2.0);
        odometry_msg.pose.pose.orientation.w = std::cos(theta / 2.0);
        odometry_msg.twist.twist.linear.x = twist_command.linear.x;
        odometry_msg.twist.twist.linear.y = twist_command.linear.y;
        odometry_msg.twist.twist.angular.z = twist_command.angular.z;
        odom_publisher->publish(odometry_msg);


        bool executed = send_command_to_simulator(request, response, result, 100, x, y, theta);

        if (!executed || !result) {
            RCLCPP_WARN_THROTTLE(get_node() -> get_logger(), *get_node() -> get_clock(), 2000, "set_pose request failed");
        }

        return controller_interface::return_type::OK;
    }

    bool FloatingController::send_command_to_simulator(ignition::msgs::Pose & req, ignition::msgs::Boolean & rep,
            bool & result, double timeout, double _x, double _y, double _theta){

        req.set_name("velmobil");
        req.mutable_position()->set_x(_x);
        req.mutable_position()->set_y(_y);
        req.mutable_position()->set_z(0.0);
        req.mutable_orientation()->set_x(0.0);
        req.mutable_orientation()->set_y(0.0);
        req.mutable_orientation()->set_z(std::sin(_theta / 2.0));
        req.mutable_orientation()->set_w(std::cos(_theta / 2.0));

        bool executed = ign_node.Request("/world/empty/set_pose", req, timeout, rep, result);
        return executed;
    }

    void FloatingController::send_tfs_to_rviz(geometry_msgs::msg::TransformStamped & tf_msg, const rclcpp::Time & time,
            double _x, double _y, double _theta){

        tf_msg.header.stamp = time;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id  = "base_footprint";

        tf_msg.transform.translation.x = _x;
        tf_msg.transform.translation.y = _y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = std::sin(_theta / 2.0);
        tf_msg.transform.rotation.w = std::cos(_theta / 2.0);

        tf_broadcaster -> sendTransform(tf_msg);
    }

} // namespace floating_controller

PLUGINLIB_EXPORT_CLASS(floating_controller::FloatingController, controller_interface::ControllerInterface)