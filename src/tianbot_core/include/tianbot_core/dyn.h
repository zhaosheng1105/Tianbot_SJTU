#ifndef __DYN_H__
#define __DYN_H__

#include <array>
#include <chrono>
#include <condition_variable>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "chassis.h"
#include "geometry_msgs/msg/twist.hpp"
#include "tianbot_core/msg/dm_mit_command.hpp"
#include "tianbot_core/msg/dm_wheel_speed_command.hpp"
#include "tianbot_core/srv/set_control_mode.hpp"

class TianbotDyn : public TianbotChasis
{
public:
    explicit TianbotDyn(const std::shared_ptr<rclcpp::Node> &node);

protected:
    void onMotionModeStatus(const struct rover_motion_mode_status &status) override;

private:
    enum ControlMode : uint32_t
    {
        MODE_UNKNOWN = 0xFFFFFFFF,
        PC_SPEED = 0,
        PC_MIT = 1,
        MODE_SWITCHING = 0xFFFFFFFE
    };

    enum SpeedSource : uint8_t
    {
        SPEED_SOURCE_NONE = 0,
        SPEED_SOURCE_CMD_VEL = 1,
        SPEED_SOURCE_WHEEL = 2
    };

    struct MotionModeCache
    {
        uint32_t chassis_mode = MODE_UNKNOWN;
        uint32_t pc_control_mode = MODE_UNKNOWN;
        uint32_t dm_mode = MODE_UNKNOWN;
        bool ready = false;
        uint8_t ctrl_source = 0;
        std::array<uint8_t, 4> motor_ctrl_mode{};
        std::array<uint8_t, 4> motor_state{};
        std::chrono::steady_clock::time_point updated_at{};
        bool has_status = false;
    };

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<tianbot_core::msg::DmWheelSpeedCommand>::SharedPtr wheel_speed_sub_;
    rclcpp::Subscription<tianbot_core::msg::DmMitCommand>::SharedPtr wheel_mit_sub_;
    rclcpp::Service<tianbot_core::srv::SetControlMode>::SharedPtr set_control_mode_srv_;

    std::mutex mode_mutex_;
    std::condition_variable mode_cv_;
    MotionModeCache mode_cache_;
    uint32_t param_pc_control_mode_;
    bool has_param_pc_control_mode_;
    uint32_t current_mode_;
    SpeedSource active_speed_source_;
    std::chrono::steady_clock::time_point last_cmd_vel_time_;
    std::chrono::steady_clock::time_point last_wheel_speed_time_;

    bool sendPacket(const std::vector<uint8_t> &buf);
    bool sendDebugCommand(const std::string &cmd, uint32_t timeout_ms, std::string *result = nullptr);
    bool debugResultHasError(const std::string &result) const;
    bool verifyPcControlModePersisted(const std::string &expected_mode, std::string *result = nullptr);
    uint32_t chassisModeToCurrentMode(uint32_t chassis_mode) const;
    void sendZeroCommands();
    uint32_t expectedChassisModeForPcMode(uint32_t pc_mode) const;
    uint32_t expectedDmModeForPcMode(uint32_t pc_mode) const;
    uint8_t expectedMotorCtrlModeForPcMode(uint32_t pc_mode) const;
    bool isModeReady(uint32_t required_mode);
    bool shouldAcceptSpeedSource(SpeedSource source, std::chrono::steady_clock::time_point now);
    uint32_t getReportedPcControlMode() const;
    void updateModeFromParamResult(const std::string &result);

    void velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);
    void wheelSpeedCallback(const tianbot_core::msg::DmWheelSpeedCommand::ConstSharedPtr &msg);
    void wheelMitCallback(const tianbot_core::msg::DmMitCommand::ConstSharedPtr &msg);
    void setControlModeCallback(
        const std::shared_ptr<tianbot_core::srv::SetControlMode::Request> req,
        std::shared_ptr<tianbot_core::srv::SetControlMode::Response> res);
};

#endif
