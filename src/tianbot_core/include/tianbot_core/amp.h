#ifndef __AMP_H__
#define __AMP_H__

#include <rclcpp/rclcpp.hpp>
#include "chassis.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tianbot_core/msg/haitai_ctrl.hpp"
#include "tianbot_core/msg/signal_light.hpp"

class TianbotAmp : public TianbotChasis
{
public:
    TianbotAmp(const std::shared_ptr<rclcpp::Node> &node);

private:
    rclcpp::Subscription<tianbot_core::msg::SignalLight>::SharedPtr stack_light_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr lift_actuator_sub_;
    rclcpp::Subscription<tianbot_core::msg::HaitaiCtrl>::SharedPtr spindle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    void stacklightCallback(const tianbot_core::msg::SignalLight::ConstSharedPtr &msg);
    void liftactuatorCallback(const std_msgs::msg::UInt8::ConstSharedPtr &msg);
    void spindleCallback(const tianbot_core::msg::HaitaiCtrl::ConstSharedPtr &msg);
    void velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);
};

#endif
