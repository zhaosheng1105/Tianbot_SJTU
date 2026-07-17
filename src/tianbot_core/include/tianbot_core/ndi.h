#ifndef __NDI_H__
#define __NDI_H__

#include <rclcpp/rclcpp.hpp>
#include "chassis.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tianbot_core/msg/emm_v5_ctrl.hpp"
#include "tianbot_core/msg/signal_light.hpp"

class TianbotNdi : public TianbotChasis
{
public:
    TianbotNdi(const std::shared_ptr<rclcpp::Node> &node);

private:
    rclcpp::Subscription<tianbot_core::msg::SignalLight>::SharedPtr stack_light_sub_;
    rclcpp::Subscription<tianbot_core::msg::EmmV5Ctrl>::SharedPtr lift_actuator_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr line_opto_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    void stacklightCallback(const tianbot_core::msg::SignalLight::ConstSharedPtr &msg);
    void liftActuatorCallback(const tianbot_core::msg::EmmV5Ctrl::ConstSharedPtr &msg);
    void lineOptoCallback(const std_msgs::msg::UInt32::ConstSharedPtr &msg);
    void velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);
};

#endif
