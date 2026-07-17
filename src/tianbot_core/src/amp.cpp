#include "amp.h"
#include "protocol.h"

void TianbotAmp::stacklightCallback(const tianbot_core::msg::SignalLight::ConstSharedPtr &msg)
{
    signal_status signal_ctrl;
    std::vector<uint8_t> buf;

    signal_ctrl.red = msg->red;
    signal_ctrl.yellow = msg->yellow;
    signal_ctrl.green = msg->green;
    signal_ctrl.buzzer = msg->buzzer;

    buildCmd(buf, PACK_TYPE_SIGNAL_CTRL, reinterpret_cast<uint8_t *>(&signal_ctrl), sizeof(signal_ctrl));
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        RCLCPP_ERROR(this->node->get_logger(), "communication failed, reopen the device");
        heartbeat_timer_->cancel();
        communication_timer_->cancel();
        open();
        communication_timer_->reset();
    }
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
}

void TianbotAmp::liftactuatorCallback(const std_msgs::msg::UInt8::ConstSharedPtr &msg)
{
    actuator_status actuator;
    std::vector<uint8_t> buf;

    actuator.state = msg->data;

    buildCmd(buf, PACK_TYPE_ACTUATOR_CTRL, reinterpret_cast<uint8_t *>(&actuator), sizeof(actuator));
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        RCLCPP_ERROR(this->node->get_logger(), "communication failed, reopen the device");
        heartbeat_timer_->cancel();
        communication_timer_->cancel();
        open();
        communication_timer_->reset();
    }
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
}

void TianbotAmp::spindleCallback(const tianbot_core::msg::HaitaiCtrl::ConstSharedPtr &msg)
{
    HaitaiCtrl_t haitai_ctrl;
    std::vector<uint8_t> buf;

    haitai_ctrl.position = msg->position;
    haitai_ctrl.velocity = msg->velocity;
    haitai_ctrl.torque = msg->torque;
    haitai_ctrl.kp = msg->kp;
    haitai_ctrl.kd = msg->kd;

    buildCmd(buf, PACK_TYPE_HAITAI_CTRL, reinterpret_cast<uint8_t *>(&haitai_ctrl), sizeof(haitai_ctrl));
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        RCLCPP_ERROR(this->node->get_logger(), "communication failed, reopen the device");
        heartbeat_timer_->cancel();
        communication_timer_->cancel();
        open();
        communication_timer_->reset();
    }
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
}

void TianbotAmp::velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
{
    std::vector<uint8_t> buf;
    twist twist_cmd;

    twist_cmd.linear.x = msg->linear.x;
    twist_cmd.linear.y = msg->linear.y;
    twist_cmd.linear.z = msg->linear.z;
    twist_cmd.angular.x = msg->angular.x;
    twist_cmd.angular.y = msg->angular.y;
    twist_cmd.angular.z = msg->angular.z;

    buildCmd(buf, PACK_TYPE_CMD_VEL, reinterpret_cast<uint8_t *>(&twist_cmd), sizeof(twist_cmd));
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        RCLCPP_ERROR(this->node->get_logger(), "communication failed, reopen the device");
        heartbeat_timer_->cancel();
        communication_timer_->cancel();
        open();
        communication_timer_->reset();
    }
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
}

TianbotAmp::TianbotAmp(const std::shared_ptr<rclcpp::Node> &node) : TianbotChasis(node)
{
    stack_light_pub_ = node->create_publisher<std_msgs::msg::UInt8MultiArray>("stack_light_state", 1);
    lift_actuator_state_pub_ = node->create_publisher<std_msgs::msg::UInt8>("lift_actuator_state", 1);
    spindle_vel_pub_ = node->create_publisher<std_msgs::msg::Float32>("spindle_state", 1);

    stack_light_sub_ = node->create_subscription<tianbot_core::msg::SignalLight>(
        "stack_light_ctrl", 1, std::bind(&TianbotAmp::stacklightCallback, this, std::placeholders::_1));
    lift_actuator_sub_ = node->create_subscription<std_msgs::msg::UInt8>(
        "lift_actuator_ctrl", 1, std::bind(&TianbotAmp::liftactuatorCallback, this, std::placeholders::_1));
    spindle_sub_ = node->create_subscription<tianbot_core::msg::HaitaiCtrl>(
        "spindle_ctrl", 1, std::bind(&TianbotAmp::spindleCallback, this, std::placeholders::_1));
    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&TianbotAmp::velocityCallback, this, std::placeholders::_1));

    initDone_ = true;
}
