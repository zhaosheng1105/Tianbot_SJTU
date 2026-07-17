#include "dyn.h"

#include <algorithm>
#include <cctype>
#include <thread>

#include "protocol.h"

using namespace std::chrono_literals;

namespace
{
std::string trimWhitespace(const std::string &input)
{
    const auto begin = input.find_first_not_of(" \t\r\n:=\0");
    if (begin == std::string::npos)
    {
        return "";
    }
    const auto end = input.find_last_not_of(" \t\r\n");
    return input.substr(begin, end - begin + 1);
}

bool dynPcControlModeFromString(const std::string &mode, uint32_t &value)
{
    if (mode == "speed")
    {
        value = 0;
        return true;
    }
    if (mode == "mit")
    {
        value = 1;
        return true;
    }
    return false;
}

std::string dynPcControlModeToString(uint32_t mode)
{
    switch (mode)
    {
    case 0:
        return "speed";
    case 1:
        return "mit";
    default:
        return "unknown_" + std::to_string(mode);
    }
}

std::string dynDmControlModeToString(uint32_t mode)
{
    switch (mode)
    {
    case 1:
        return "mit";
    case 2:
        return "pos";
    case 3:
        return "speed";
    default:
        return "unknown_" + std::to_string(mode);
    }
}

std::string dynChassisModeToString(uint32_t mode)
{
    switch (mode)
    {
    case 0:
        return "pc_mit";
    case 1:
        return "rc_speed";
    case 2:
        return "rc_mit";
    case 3:
        return "pc_speed";
    default:
        return "unknown_" + std::to_string(mode);
    }
}

std::string extractPcControlModeValue(const std::string &result)
{
    const auto key = std::string("pc_control_mode");
    const auto start = result.find(key);
    if (start == std::string::npos)
    {
        return "";
    }

    const auto value_start = start + key.length();
    const auto value_end = result.find("\n", value_start);
    return trimWhitespace(result.substr(
        value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start));
}
}

TianbotDyn::TianbotDyn(const std::shared_ptr<rclcpp::Node> &node)
    : TianbotChasis(node),
      param_pc_control_mode_(MODE_UNKNOWN),
      has_param_pc_control_mode_(false),
      current_mode_(MODE_UNKNOWN),
      active_speed_source_(SPEED_SOURCE_NONE)
{
    motor_feedback_pub_ = node->create_publisher<tianbot_core::msg::DmMotorFeedbackArray>("motor_feedback", 1);
    motion_mode_status_pub_ = node->create_publisher<tianbot_core::msg::RoverMotionModeStatus>("motion_mode_status", 1);

    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&TianbotDyn::velocityCallback, this, std::placeholders::_1));
    wheel_speed_sub_ = node->create_subscription<tianbot_core::msg::DmWheelSpeedCommand>(
        "wheel_speed_cmd", 1, std::bind(&TianbotDyn::wheelSpeedCallback, this, std::placeholders::_1));
    wheel_mit_sub_ = node->create_subscription<tianbot_core::msg::DmMitCommand>(
        "wheel_mit_cmd", 1, std::bind(&TianbotDyn::wheelMitCallback, this, std::placeholders::_1));
    set_control_mode_srv_ = node->create_service<tianbot_core::srv::SetControlMode>(
        "set_control_mode", std::bind(&TianbotDyn::setControlModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    initDone_ = true;

    std::string result;
    if (sendDebugCommand("param get", 500, &result))
    {
        updateModeFromParamResult(result);
    }
}

bool TianbotDyn::sendPacket(const std::vector<uint8_t> &buf)
{
    if (comm_inf_->send(const_cast<uint8_t *>(&buf[0]), buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        RCLCPP_ERROR(this->node->get_logger(), "communication failed, reopen the device");
        heartbeat_timer_->cancel();
        communication_timer_->cancel();
        open();
        communication_timer_->reset();
        return false;
    }
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
    return true;
}

bool TianbotDyn::sendDebugCommand(const std::string &cmd, uint32_t timeout_ms, std::string *result)
{
    return sendDebugCommandAndWait(cmd, timeout_ms, result);
}

bool TianbotDyn::debugResultHasError(const std::string &result) const
{
    std::string normalized = result;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    return normalized.find("fail") != std::string::npos ||
           normalized.find("error") != std::string::npos ||
           normalized.find("invalid") != std::string::npos ||
           normalized.find("unknown cmd") != std::string::npos;
}

bool TianbotDyn::verifyPcControlModePersisted(const std::string &expected_mode, std::string *result)
{
    for (int attempt = 0; attempt < 5; ++attempt)
    {
        std::string current_result;
        if (!sendDebugCommand("param get", 1000, &current_result))
        {
            rclcpp::sleep_for(100ms);
            continue;
        }

        if (result)
        {
            *result = current_result;
        }
        updateModeFromParamResult(current_result);
        if (extractPcControlModeValue(current_result) == expected_mode)
        {
            return true;
        }
        rclcpp::sleep_for(100ms);
    }
    return false;
}

uint32_t TianbotDyn::chassisModeToCurrentMode(uint32_t chassis_mode) const
{
    switch (chassis_mode)
    {
    case 3:
        return PC_SPEED;
    case 0:
        return PC_MIT;
    default:
        return MODE_UNKNOWN;
    }
}

void TianbotDyn::sendZeroCommands()
{
    for (int i = 0; i < 3; ++i)
    {
        std::vector<uint8_t> buf;
        struct twist zero_twist = {};
        struct dm_speed_cmd zero_speed = {};
        struct dm_mit_cmd zero_mit = {};

        buildCmd(buf, PACK_TYPE_CMD_VEL, reinterpret_cast<uint8_t *>(&zero_twist), sizeof(zero_twist));
        sendPacket(buf);
        buf.clear();
        buildCmd(buf, PACK_TYPE_DM_SPEED_CMD, reinterpret_cast<uint8_t *>(&zero_speed), sizeof(zero_speed));
        sendPacket(buf);
        buf.clear();
        buildCmd(buf, PACK_TYPE_DM_MIT_CMD, reinterpret_cast<uint8_t *>(&zero_mit), sizeof(zero_mit));
        sendPacket(buf);
        rclcpp::sleep_for(20ms);
    }
}

uint32_t TianbotDyn::expectedChassisModeForPcMode(uint32_t pc_mode) const
{
    return pc_mode == PC_SPEED ? 3U : 0U;
}

uint32_t TianbotDyn::expectedDmModeForPcMode(uint32_t pc_mode) const
{
    return pc_mode == PC_SPEED ? 3U : 1U;
}

uint8_t TianbotDyn::expectedMotorCtrlModeForPcMode(uint32_t pc_mode) const
{
    return pc_mode == PC_SPEED ? 3U : 1U;
}

bool TianbotDyn::isModeReady(uint32_t required_mode)
{
    if (!(mode_cache_.has_status &&
          mode_cache_.ready &&
          mode_cache_.chassis_mode == expectedChassisModeForPcMode(required_mode) &&
          mode_cache_.dm_mode == expectedDmModeForPcMode(required_mode)))
    {
        return false;
    }

    const uint8_t expected_motor_mode = expectedMotorCtrlModeForPcMode(required_mode);
    for (int i = 0; i < 4; ++i)
    {
        if (mode_cache_.motor_ctrl_mode[i] != expected_motor_mode || mode_cache_.motor_state[i] != 1U)
        {
            return false;
        }
    }
    return true;
}

bool TianbotDyn::shouldAcceptSpeedSource(SpeedSource source, std::chrono::steady_clock::time_point now)
{
    const auto timeout = 200ms;
    const auto cmd_vel_valid = last_cmd_vel_time_.time_since_epoch().count() != 0 && (now - last_cmd_vel_time_) <= timeout;
    const auto wheel_valid = last_wheel_speed_time_.time_since_epoch().count() != 0 && (now - last_wheel_speed_time_) <= timeout;

    if (source == SPEED_SOURCE_CMD_VEL)
    {
        return !wheel_valid || !cmd_vel_valid || last_cmd_vel_time_ >= last_wheel_speed_time_;
    }
    if (source == SPEED_SOURCE_WHEEL)
    {
        return !cmd_vel_valid || !wheel_valid || last_wheel_speed_time_ >= last_cmd_vel_time_;
    }
    return false;
}

void TianbotDyn::updateModeFromParamResult(const std::string &result)
{
    const auto value = extractPcControlModeValue(result);
    if (value.empty())
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mode_mutex_);
    if (value == "mit")
    {
        param_pc_control_mode_ = PC_MIT;
    }
    else if (value == "speed")
    {
        param_pc_control_mode_ = PC_SPEED;
    }
    else
    {
        return;
    }
    has_param_pc_control_mode_ = true;
}

uint32_t TianbotDyn::getReportedPcControlMode() const
{
    if (mode_cache_.has_status)
    {
        return mode_cache_.pc_control_mode;
    }
    if (has_param_pc_control_mode_)
    {
        return param_pc_control_mode_;
    }
    return MODE_UNKNOWN;
}

void TianbotDyn::velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
{
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (!isModeReady(PC_SPEED))
        {
            RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 2000, "DYN ignores cmd_vel because current mode is not PC_SPEED");
            return;
        }
    }

    const auto now = std::chrono::steady_clock::now();
    last_cmd_vel_time_ = now;
    if (!shouldAcceptSpeedSource(SPEED_SOURCE_CMD_VEL, now))
    {
        return;
    }
    active_speed_source_ = SPEED_SOURCE_CMD_VEL;

    struct twist twist_cmd = {};
    std::vector<uint8_t> buf;

    twist_cmd.linear.x = msg->linear.x;
    twist_cmd.angular.z = msg->angular.z;
    buildCmd(buf, PACK_TYPE_CMD_VEL, reinterpret_cast<uint8_t *>(&twist_cmd), sizeof(twist_cmd));
    sendPacket(buf);
}

void TianbotDyn::wheelSpeedCallback(const tianbot_core::msg::DmWheelSpeedCommand::ConstSharedPtr &msg)
{
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (!isModeReady(PC_SPEED))
        {
            RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 2000, "DYN ignores wheel_speed_cmd because current mode is not PC_SPEED");
            return;
        }
    }

    const auto now = std::chrono::steady_clock::now();
    last_wheel_speed_time_ = now;
    if (!shouldAcceptSpeedSource(SPEED_SOURCE_WHEEL, now))
    {
        return;
    }
    active_speed_source_ = SPEED_SOURCE_WHEEL;

    struct dm_speed_cmd speed_cmd = {};
    std::vector<uint8_t> buf;
    for (int i = 0; i < 4; ++i)
    {
        speed_cmd.speed_rad_s[i] = msg->speed_rad_s[i];
    }
    buildCmd(buf, PACK_TYPE_DM_SPEED_CMD, reinterpret_cast<uint8_t *>(&speed_cmd), sizeof(speed_cmd));
    sendPacket(buf);
}

void TianbotDyn::wheelMitCallback(const tianbot_core::msg::DmMitCommand::ConstSharedPtr &msg)
{
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (!isModeReady(PC_MIT))
        {
            RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 2000, "DYN ignores wheel_mit_cmd because current mode is not PC_MIT");
            return;
        }
    }

    struct dm_mit_cmd mit_cmd = {};
    std::vector<uint8_t> buf;
    for (int i = 0; i < 4; ++i)
    {
        mit_cmd.p_des[i] = msg->p_des[i];
        mit_cmd.v_des[i] = msg->v_des[i];
        mit_cmd.kp[i] = msg->kp[i];
        mit_cmd.kd[i] = msg->kd[i];
        mit_cmd.t_ff[i] = msg->t_ff[i];
    }
    buildCmd(buf, PACK_TYPE_DM_MIT_CMD, reinterpret_cast<uint8_t *>(&mit_cmd), sizeof(mit_cmd));
    sendPacket(buf);
}

void TianbotDyn::setControlModeCallback(
    const std::shared_ptr<tianbot_core::srv::SetControlMode::Request> req,
    std::shared_ptr<tianbot_core::srv::SetControlMode::Response> res)
{
    const auto fill_response = [&](const MotionModeCache &mode_cache, uint32_t reported_pc_control_mode) {
        res->current_chassis_mode = dynChassisModeToString(mode_cache.chassis_mode);
        res->current_pc_control_mode = dynPcControlModeToString(reported_pc_control_mode);
        res->current_dm_mode = dynDmControlModeToString(mode_cache.dm_mode);
        res->ready = mode_cache.ready;
    };

    uint32_t requested_mode = MODE_UNKNOWN;
    if (!dynPcControlModeFromString(req->mode, requested_mode))
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        res->success = false;
        fill_response(mode_cache_, getReportedPcControlMode());
        res->message = "unsupported control mode, only 'speed' and 'mit' are allowed";
        return;
    }

    std::vector<uint8_t> buf;
    struct motion_mode motion_mode = {};

    motion_mode.mode = requested_mode;
    sendZeroCommands();
    buildCmd(buf, PACK_TYPE_SET_ROVER_MOTION_MODE, reinterpret_cast<uint8_t *>(&motion_mode), sizeof(motion_mode));
    if (!sendPacket(buf))
    {
        res->success = false;
        std::lock_guard<std::mutex> lock(mode_mutex_);
        fill_response(mode_cache_, getReportedPcControlMode());
        res->message = "failed to send mode switch packet";
        return;
    }

    {
        current_mode_ = MODE_SWITCHING;
        const auto deadline = std::chrono::steady_clock::now() + 10s;
        bool matched = false;

        while (std::chrono::steady_clock::now() < deadline)
        {
            {
                std::lock_guard<std::mutex> lock(mode_mutex_);
                if (isModeReady(requested_mode))
                {
                    matched = true;
                    current_mode_ = requested_mode;
                    fill_response(mode_cache_, getReportedPcControlMode());
                    break;
                }
            }
            rclcpp::sleep_for(20ms);
        }

        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (matched)
        {
            res->success = true;
            res->message = "mode switch confirmed by rover_motion_mode_status";
        }
        else
        {
            current_mode_ = mode_cache_.has_status ? chassisModeToCurrentMode(mode_cache_.chassis_mode) : MODE_UNKNOWN;
            res->success = false;
            res->message = "timeout waiting for rover_motion_mode_status";
            fill_response(mode_cache_, getReportedPcControlMode());
        }
    }

    if (!res->success || !req->save_to_flash)
    {
        return;
    }

    const std::string control_mode = requested_mode == PC_MIT ? "mit" : "speed";
    std::string result;
    if (!sendDebugCommand("param set pc_control_mode " + control_mode, 3000, &result))
    {
        res->success = false;
        res->message = "mode switched, but failed to set pc_control_mode";
        return;
    }
    if (debugResultHasError(result))
    {
        res->success = false;
        res->message = "mode switched, but device rejected pc_control_mode update";
        return;
    }
    if (!sendDebugCommand("param save", 3000, &result))
    {
        res->success = false;
        res->message = "mode switched, but failed to save pc_control_mode";
        return;
    }
    if (debugResultHasError(result))
    {
        res->success = false;
        res->message = "mode switched, but device rejected param save";
        return;
    }

    if (!verifyPcControlModePersisted(control_mode, &result))
    {
        res->success = false;
        res->message = "mode switched, but param get did not confirm saved pc_control_mode";
        return;
    }

    rclcpp::sleep_for(200ms);

    std::vector<uint8_t> reset_buf;
    std::string reset_cmd = "reset";
    buildCmd(
        reset_buf, PACK_TYPE_DEBUG, reinterpret_cast<uint8_t *>(reset_cmd.data()), reset_cmd.length());
    if (!sendPacket(reset_buf))
    {
        res->success = false;
        res->message = "mode switched and saved, but failed to request reset";
        return;
    }
    res->message = "mode switched, saved, verified by param get, and reset requested";
}

void TianbotDyn::onMotionModeStatus(const struct rover_motion_mode_status &status)
{
    std::lock_guard<std::mutex> lock(mode_mutex_);

    mode_cache_.chassis_mode = status.chassis_mode;
    mode_cache_.pc_control_mode = status.pc_control_mode;
    mode_cache_.dm_mode = status.dm_mode;
    mode_cache_.ready = status.ready != 0;
    mode_cache_.ctrl_source = status.ctrl_source;
    for (int i = 0; i < 4; ++i)
    {
        mode_cache_.motor_ctrl_mode[i] = status.motor_ctrl_mode[i];
        mode_cache_.motor_state[i] = status.motor_state[i];
    }
    mode_cache_.updated_at = std::chrono::steady_clock::now();
    mode_cache_.has_status = true;

    if (current_mode_ != MODE_SWITCHING)
    {
        current_mode_ = chassisModeToCurrentMode(status.chassis_mode);
    }
    mode_cv_.notify_all();
}
