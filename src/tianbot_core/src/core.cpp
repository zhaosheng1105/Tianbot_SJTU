#include "core.h"
#include "protocol.h"
#include <stdint.h>
#include <vector>

namespace
{
uint32_t debugTimeoutForCommand(const std::string &cmd)
{
    if (cmd == "reset")
    {
        return 0;
    }
    if (cmd == "param save" || cmd == "param reset")
    {
        return 2000;
    }
    if (cmd.find("set_") != cmd.npos)
    {
        return 3000;
    }
    return 200;
}
}

bool TianbotCore::sendDebugCommandAndWait(const std::string &cmd, uint32_t timeout_ms, std::string *result)
{
    std::lock_guard<std::mutex> debug_command_lock(debug_command_mutex_);

    {
        std::lock_guard<std::mutex> debug_result_lock(debug_result_mutex_);
        debugResultFlag_ = false;
        debugResultStr_.clear();
    }

    vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_DEBUG, reinterpret_cast<uint8_t *>(const_cast<char *>(cmd.c_str())), cmd.length());
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
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

    if (timeout_ms == 0)
    {
        if (result)
        {
            *result = cmd;
        }
        return true;
    }

    std::unique_lock<std::mutex> debug_result_lock(debug_result_mutex_);
    const bool got_result = debug_result_cv_.wait_for(
        debug_result_lock, std::chrono::milliseconds(timeout_ms), [this]() { return debugResultFlag_; });
    if (!got_result)
    {
        return false;
    }

    if (result)
    {
        *result = debugResultStr_;
    }
    return true;
}

void TianbotCore::dataProc(uint8_t *data, unsigned int data_len)
{
    static uint8_t state = 0;
    uint8_t *p = data;
    static vector<uint8_t> recv_msg;
    static uint32_t len;
    uint32_t j;

    while (data_len != 0)
    {
        switch (state)
        {
        case 0:
            if (*p == (PROTOCOL_HEAD & 0xFF))
            {
                recv_msg.clear();
                recv_msg.push_back(PROTOCOL_HEAD & 0xFF);
                state = 1;
            }
            p++;
            data_len--;
            break;

        case 1:
            if (*p == ((PROTOCOL_HEAD >> 8) & 0xFF))
            {
                recv_msg.push_back(((PROTOCOL_HEAD >> 8) & 0xFF));
                p++;
                data_len--;
                state = 2;
            }
            else
            {
                state = 0;
            }
            break;

        case 2: // len
            recv_msg.push_back(*p);
            len = *p;
            p++;
            data_len--;
            state = 3;
            break;

        case 3: // len
            recv_msg.push_back(*p);
            len += (*p) * 256;
            if (len > 1024 * 10)
            {
                state = 0;
                break;
            }
            p++;
            data_len--;
            state = 4;
            break;

        case 4: // pack_type
            recv_msg.push_back(*p);
            p++;
            data_len--;
            len--;
            state = 5;
            break;

        case 5: // pack_type
            recv_msg.push_back(*p);
            p++;
            data_len--;
            len--;
            state = 6;
            break;

        case 6: //
            if (len--)
            {
                recv_msg.push_back(*p);
                p++;
                data_len--;
            }
            else
            {
                state = 7;
            }
            break;

        case 7: {
            int i;
            uint8_t bcc = 0;
            recv_msg.push_back(*p);
            p++;
            data_len--;
            state = 0;

            for (i = 4; i < recv_msg.size(); i++)
            {
                bcc ^= recv_msg[i];
            }

            if (bcc == 0)
            {
                if (initDone_)
                {
                    tianbotDataProc(&recv_msg[0], recv_msg.size()); // process recv msg
                }
                communication_timer_->cancel();                    // restart timer for communication timeout
                communication_timer_->reset();
            }
            else
            {
                RCLCPP_INFO(this->node->get_logger(), "BCC error");
            }
            state = 0;
        }
        break;

        default:
            state = 0;
            break;
        }
    }
}

void TianbotCore::communicationErrorCallback()
{
    RCUTILS_LOG_ERROR_THROTTLE(RCUTILS_STEADY_TIME, 5, "Communication with base error");
}

void TianbotCore::heartCallback()
{
    std::vector<uint8_t> buf;
    uint16_t dummy = 0;

    buildCmd(buf, PACK_TYPE_HEART_BEAT, reinterpret_cast<uint8_t *>(&dummy), sizeof(dummy));
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
    heartbeat_timer_->reset();
}

void TianbotCore::debugCmdCallback(const std_msgs::msg::String::ConstSharedPtr &msg)
{
    vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_DEBUG, (uint8_t *)msg->data.c_str(), msg->data.length());
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

bool TianbotCore::debugCmdSrv(const std::shared_ptr<tianbot_core::srv::DebugCmd::Request> req, 
                              const std::shared_ptr<tianbot_core::srv::DebugCmd::Response> res)
{
    RCLCPP_INFO(this->node->get_logger(), "cmd: %s", req->cmd.c_str());
    std::string result;
    if (!sendDebugCommandAndWait(req->cmd, debugTimeoutForCommand(req->cmd), &result))
    {
        return false;
    }
    RCLCPP_INFO(this->node->get_logger(), "result: %s", result.c_str());
    res->result = result;
    return true;
}

void TianbotCore::checkDevType(void)
{
    string type_keyword_list[] = {"base_type: ", "end"};
    string type;
    string dev_param;
    string dev_type;
    string::size_type start;
    string::size_type end;

    uint32_t retry;

    for (retry = 0; retry < 5; retry++)
    {
        if (sendDebugCommandAndWait("param get", 300, &dev_param))
        {
            break;
        }
        else
        {
            RCLCPP_ERROR(this->node->get_logger(), "Get Device type failed, retry after 1s ...");
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }
    if (retry == 5)
    {
        RCLCPP_ERROR(this->node->get_logger(), "No valid device type found");
        return;
    }
    for (int i = 0; type_keyword_list[i] != "end"; i++)
    {
        start = dev_param.find(type_keyword_list[i]);
        if (start != dev_param.npos)
        {
            start += type_keyword_list[i].length();
            end = dev_param.find("\r\n", start);
            if (end == dev_param.npos)
            {
                end = dev_param.length();
            }
            dev_type = dev_param.substr(start, end - start);
            RCLCPP_INFO(this->node->get_logger(), "Get device type [%s]", dev_type.c_str());
            if (!this->node->has_parameter("type"))
            {
                this->node->declare_parameter("type", rclcpp::PARAMETER_STRING);
            }
            if (!this->node->get_parameter("type", type)) {
                type = DEFAULT_TYPE;
            }
            if (dev_type == "omni" || dev_type == "mecanum")
            {
                dev_type = "omni";
            }
            if (type == dev_type)
            {
                RCLCPP_INFO(this->node->get_logger(), "Device type match");
            }
            else
            {
                RCLCPP_ERROR(this->node->get_logger(), "Device type mismatch, set [%s] get [%s]", type.c_str(), dev_type.c_str());
            }
            return;
        }
    }
    RCLCPP_ERROR(this->node->get_logger(), "No valid device type found");
}

void TianbotCore::open(void)
{
    std::string param_serial_port;
    std::string client_ip;
    if (!this->node->get_parameter("serial_port", param_serial_port) && !this->node->get_parameter("client_ip", client_ip))
    {
        RCLCPP_FATAL(this->node->get_logger(), "Please specify the serial_port or client_ip");
        exit(-1);
    }
    else if (this->node->get_parameter("serial_port", param_serial_port))
    {
        comm_inf_ = new Serial();
        struct serial_cfg s_cfg;
        if (!this->node->has_parameter("serial_baudrate"))
        {
            this->node->declare_parameter<int>("serial_baudrate", DEFAULT_SERIAL_BAUDRATE);
        }
        this->node->get_parameter("serial_baudrate", s_cfg.rate);
        s_cfg.device = (char *)param_serial_port.c_str();
        s_cfg.databits = 8;
        s_cfg.flow_ctrl = 0;
        s_cfg.parity = 'N';
        s_cfg.stopbits = 1;
        RCLCPP_INFO(this->node->get_logger(), "Using %s for communication, baudrate: %d", param_serial_port.c_str(), s_cfg.rate);
        while (comm_inf_->open(&s_cfg, boost::bind(&TianbotCore::dataProc, this, _1, _2)) != true)
        {
            if (!rclcpp::ok())
                exit(0);
            RCUTILS_LOG_ERROR_THROTTLE(RCUTILS_STEADY_TIME, 5.0, "Device %s open failed", param_serial_port.c_str());
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_INFO(this->node->get_logger(), "Device %s open successfully", param_serial_port.c_str());
    }
    else if (this->node->get_parameter("client_ip", client_ip))
    {
        comm_inf_ = new Udp();
        struct udp_cfg u_cfg;
        this->node->declare_parameter<int>("client_port", DEFAULT_CLIENT_PORT);
        this->node->declare_parameter<int>("server_port", DEFAULT_SERVER_PORT);
        this->node->get_parameter("client_port", u_cfg.udp_send_port);
        this->node->get_parameter("server_port", u_cfg.udp_recv_port);
        u_cfg.client_addr = client_ip;
        while (comm_inf_->open(&u_cfg, boost::bind(&TianbotCore::dataProc, this, _1, _2)) != true)
        {
            RCUTILS_LOG_ERROR_THROTTLE(RCUTILS_STEADY_TIME, 5.0, "Lesten device %s:%d failed", client_ip.c_str(), u_cfg.udp_send_port);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_ERROR(this->node->get_logger(), "Listen device %s:%d, server port %d ", client_ip.c_str(), u_cfg.udp_send_port, u_cfg.udp_recv_port);
    }
}

TianbotCore::TianbotCore(const std::shared_ptr<rclcpp::Node> &nh)
    : node(nh), initDone_(false)
{
    open();
    debug_result_pub_ = node->create_publisher<std_msgs::msg::String>("debug_result", 1);
    debug_cmd_sub_ = node->create_subscription<std_msgs::msg::String>(
        "debug_cmd", 1, std::bind(&TianbotCore::debugCmdCallback, this, std::placeholders::_1));

    param_set_ = node->create_service<tianbot_core::srv::DebugCmd>(
        "debug_cmd_srv", std::bind(&TianbotCore::debugCmdSrv, this, std::placeholders::_1, std::placeholders::_2));

    timer_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    heartbeat_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&TianbotCore::heartCallback, this), timer_callback_group_);
    communication_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&TianbotCore::communicationErrorCallback, this), timer_callback_group_);
    heartbeat_timer_->cancel();
    communication_timer_->cancel();
    
    heartbeat_timer_->reset();
    communication_timer_->reset();
}
