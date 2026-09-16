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
#include "nav_msgs/msg/odometry.hpp"


namespace roll_yaw_controller {

    class RollYawController : public controller_interface::ControllerInterface {
    public:
        RollYawController();
        controller_interface::CallbackReturn on_init() override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;


    protected:
        std::vector<std::string> wheel_joint_names;
        std::vector<std::string> yaw_joint_names;
        double wheel_radius;


        // odometry
        double x;
        double y;
        double theta;
        geometry_msgs::msg::Twist twist_command;


        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        void publishOdom();
        void broadcastTransform(const rclcpp::Time & time);
    };

} // namespace roll_yaw_controller