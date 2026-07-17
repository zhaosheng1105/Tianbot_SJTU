#include "ndi.h"
#include "protocol.h"

void TianbotNdi::stacklightCallback(const tianbot_core::msg::SignalLight::ConstSharedPtr &msg)
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

void TianbotNdi::liftActuatorCallback(const tianbot_core::msg::EmmV5Ctrl::ConstSharedPtr &msg)
{
    EmmV5Ctrl_t emm_ctrl;
    std::vector<uint8_t> buf;

    emm_ctrl.distance_mm = msg->distance_mm;
    emm_ctrl.vel = msg->vel;
    emm_ctrl.acc = msg->acc;

    buildCmd(buf, PACK_TYPE_EMM_V5_CTRL, reinterpret_cast<uint8_t *>(&emm_ctrl), sizeof(emm_ctrl));
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

void TianbotNdi::lineOptoCallback(const std_msgs::msg::UInt32::ConstSharedPtr &msg)
{
    line_opto_cmd cmd;
    std::vector<uint8_t> buf;

    cmd.pulse_ms = msg->data;

    buildCmd(buf, PACK_TYPE_LINE_OPTO_CTRL, reinterpret_cast<uint8_t *>(&cmd), sizeof(cmd));
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        RCLCPP_ERROR(this->node->get_logger(), "communication failed when sending line_opto_ctrl, reopen the device");
        heartbeat_timer_->cancel();
        communication_timer_->cancel();
        open();
        communication_timer_->reset();
    }
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
}

void TianbotNdi::velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
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

TianbotNdi::TianbotNdi(const std::shared_ptr<rclcpp::Node> &node) : TianbotChasis(node)
{
    stack_light_pub_ = node->create_publisher<std_msgs::msg::UInt8MultiArray>("stack_light_state", 1);
    lift_actuator_position_pub_ = node->create_publisher<std_msgs::msg::Float32>("lift_actuator_state", 1);

    stack_light_sub_ = node->create_subscription<tianbot_core::msg::SignalLight>(
        "stack_light_ctrl", 1, std::bind(&TianbotNdi::stacklightCallback, this, std::placeholders::_1));
    lift_actuator_sub_ = node->create_subscription<tianbot_core::msg::EmmV5Ctrl>(
        "lift_actuator_ctrl", 1, std::bind(&TianbotNdi::liftActuatorCallback, this, std::placeholders::_1));
    line_opto_sub_ = node->create_subscription<std_msgs::msg::UInt32>(
        "line_opto_ctrl", 1, std::bind(&TianbotNdi::lineOptoCallback, this, std::placeholders::_1));
    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&TianbotNdi::velocityCallback, this, std::placeholders::_1));

    initDone_ = true;
}
