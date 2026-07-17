#include "chassis.h"
#include "protocol.h"

namespace
{
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

std::string dynMotorStateToString(uint8_t state)
{
    switch (state)
    {
    case 0:
        return "offline";
    case 1:
        return "online";
    default:
        return "unknown_" + std::to_string(state);
    }
}

void fillCompactMotorFeedbackMsg(
    const dm_motor_feedback_array_compact &feedback,
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    tianbot_core::msg::DmMotorFeedbackArray &feedback_msg)
{
    feedback_msg.header.stamp = stamp;
    feedback_msg.header.frame_id = frame_id;
    feedback_msg.control_mode = dynDmControlModeToString(feedback.control_mode);
    for (int i = 0; i < 4; ++i)
    {
        tianbot_core::msg::DmMotorFeedback motor_msg;
        motor_msg.id = feedback.motors[i].id;
        motor_msg.state = dynMotorStateToString(feedback.motors[i].state);
        motor_msg.control_mode = dynDmControlModeToString(feedback.motors[i].control_mode);
        motor_msg.output_speed_rad_s = feedback.motors[i].output_speed_rad_s;
        motor_msg.output_torque_nm = feedback.motors[i].output_torque_nm;
        motor_msg.mos_temp_c = feedback.motors[i].mos_temp_c;
        motor_msg.coil_temp_c = feedback.motors[i].coil_temp_c;
        motor_msg.last_feedback_age_ms = feedback.motors[i].last_feedback_age_ms;
        feedback_msg.motors[i] = motor_msg;
    }
}

void fillCompactMotionModeStatusMsg(
    const rover_motion_mode_status_compact &status,
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    tianbot_core::msg::RoverMotionModeStatus &status_msg)
{
    status_msg.header.stamp = stamp;
    status_msg.header.frame_id = frame_id;
    status_msg.chassis_mode = dynChassisModeToString(status.chassis_mode);
    status_msg.dm_mode = dynDmControlModeToString(status.dm_mode);
    bool computed_ready = status.ready != 0;
    for (int i = 0; i < 4; ++i)
    {
        status_msg.motor_ctrl_mode[i] = dynDmControlModeToString(status.motor_ctrl_mode[i]);
        status_msg.motor_state[i] = dynMotorStateToString(status.motor_state[i]);
        if (status.motor_ctrl_mode[i] != status.dm_mode || status.motor_state[i] != 1U)
        {
            computed_ready = false;
        }
    }
    status_msg.ready = computed_ready;
}
}

void TianbotChasis::onMotorFeedback(const struct dm_motor_feedback_array &)
{
}

void TianbotChasis::onMotionModeStatus(const struct rover_motion_mode_status &)
{
}

void TianbotChasis::tianbotDataProc(unsigned char *buf, int)
{
    if (!publisher_init_done)
    {
        return;
    }
    struct protocol_pack *p = (struct protocol_pack *)buf;
    switch (p->pack_type)
    {
    case PACK_TYPE_ODOM_RESPONSE:
        if (sizeof(struct odom) == p->len - 2)
        {
            nav_msgs::msg::Odometry odom_msg;
            struct odom *pOdom = (struct odom *)(p->data);
            rclcpp::Time current_time = clock_->now();
            odom_msg.header.stamp = current_time;
            // odom_msg.header.frame_id = (nh_.getNamespace() + "/" + odom_frame_).erase(0,1);
            odom_msg.header.frame_id = odom_frame_;

            odom_msg.pose.pose.position.x = pOdom->pose.point.x;
            odom_msg.pose.pose.position.y = pOdom->pose.point.y;
            odom_msg.pose.pose.position.z = pOdom->pose.point.z;
            geometry_msgs::msg::Quaternion q = createQuaternionMsgFromYaw(pOdom->pose.yaw);          
            odom_msg.pose.pose.orientation = q;
            // set the velocity
            odom_msg.child_frame_id = base_frame_;
            odom_msg.twist.twist.linear.x = pOdom->twist.linear.x;
            odom_msg.twist.twist.linear.y = pOdom->twist.linear.y;
            odom_msg.twist.twist.linear.z = pOdom->twist.linear.z;
            odom_msg.twist.twist.angular.x = pOdom->twist.angular.x;
            odom_msg.twist.twist.angular.y = pOdom->twist.angular.y;
            odom_msg.twist.twist.angular.z = pOdom->twist.angular.z;
            // publish the message
            odom_pub_->publish(odom_msg);
            if (publish_tf_)
            {
                odom_tf_.header.stamp = current_time;
                odom_tf_.header.frame_id = odom_frame_;
                odom_tf_.child_frame_id = base_frame_;
                odom_tf_.transform.translation.x = pOdom->pose.point.x;
                odom_tf_.transform.translation.y = pOdom->pose.point.y;
                odom_tf_.transform.translation.z = pOdom->pose.point.z;

                odom_tf_.transform.rotation = odom_msg.pose.pose.orientation;
                tf_broadcaster_->sendTransform(odom_tf_);
            }
        }
        break;

    case PACK_TYPE_ODOM_V2_RESPONSE:
        if (sizeof(struct odom_v2) == p->len - 2)
        {
            nav_msgs::msg::Odometry odom_msg;
            struct odom_v2 *pOdom = (struct odom_v2 *)(p->data);
            rclcpp::Time current_time = clock_->now();
            odom_msg.header.stamp = current_time;
            // odom_msg.header.frame_id = (nh_.getNamespace() + "/" + odom_frame_).erase(0,1);
            odom_msg.header.frame_id = odom_frame_;

            odom_msg.pose.pose.position.x = pOdom->pose.point.x;
            odom_msg.pose.pose.position.y = pOdom->pose.point.y;
            odom_msg.pose.pose.position.z = pOdom->pose.point.z;
            //vector3 x->roll y->pitch z->yaw
            geometry_msgs::msg::Quaternion q = createQuaternionMsgFromRollPitchYaw(pOdom->pose.rpy.x, pOdom->pose.rpy.y, pOdom->pose.rpy.z);    
            odom_msg.pose.pose.orientation = q;
            // set the velocity
            odom_msg.child_frame_id = base_frame_;
            odom_msg.twist.twist.linear.x = pOdom->twist.linear.x;
            odom_msg.twist.twist.linear.y = pOdom->twist.linear.y;
            odom_msg.twist.twist.linear.z = pOdom->twist.linear.z;
            odom_msg.twist.twist.angular.x = pOdom->twist.angular.x;
            odom_msg.twist.twist.angular.y = pOdom->twist.angular.y;
            odom_msg.twist.twist.angular.z = pOdom->twist.angular.z;
            // publish the message
            odom_pub_->publish(odom_msg);
            if (publish_tf_)
            {
                odom_tf_.header.stamp = current_time;
                odom_tf_.header.frame_id = odom_frame_;
                odom_tf_.child_frame_id = base_frame_;
                odom_tf_.transform.translation.x = pOdom->pose.point.x;
                odom_tf_.transform.translation.y = pOdom->pose.point.y;
                odom_tf_.transform.translation.z = pOdom->pose.point.z;

                odom_tf_.transform.rotation = odom_msg.pose.pose.orientation;
                tf_broadcaster_->sendTransform(odom_tf_);
            }
        }
        break;

    case PACK_TYPE_UWB_RESPONSE:
        if (sizeof(struct uwb) == p->len - 2)
        {
            geometry_msgs::msg::Pose2D pose2d_msg;
            auto pUwb = reinterpret_cast<struct uwb *>(p->data);
            pose2d_msg.x = pUwb->x_m;
            pose2d_msg.y = pUwb->y_m;
            pose2d_msg.theta = pUwb->yaw;
            uwb_pub_->publish(pose2d_msg);
        }
        break;

    case PACK_TYPE_Voltage_RESPONSE:
        if (sizeof(struct voltage) == p->len - 2)
        {
            std_msgs::msg::Float32 battery_msg;
            auto voltage = reinterpret_cast<struct voltage *>(p->data);
            battery_msg.data = voltage->Battery_voltage;
            voltage_pub_->publish(battery_msg);
        }
        break;

    case PACK_TYPE_SIGNAL_STATUS:
        if (sizeof(struct signal_status) == p->len - 2)
        {
            std_msgs::msg::UInt8MultiArray signal_msg;
            auto signal = reinterpret_cast<struct signal_status *>(p->data);
            signal_msg.data.push_back(signal->red);
            signal_msg.data.push_back(signal->yellow);
            signal_msg.data.push_back(signal->green);
            signal_msg.data.push_back(signal->buzzer);
            if (stack_light_pub_)
            {
                stack_light_pub_->publish(signal_msg);
            }
        }
        break;

    case PACK_TYPE_ACTUATOR_STATUS:
        if (sizeof(struct actuator_status) == p->len - 2)
        {
            std_msgs::msg::UInt8 actuator_msg;
            auto actuator = reinterpret_cast<struct actuator_status *>(p->data);
            actuator_msg.data = actuator->state;
            if (lift_actuator_state_pub_)
            {
                lift_actuator_state_pub_->publish(actuator_msg);
            }
        }
        break;

    case PACK_TYPE_HAITAI_VELOCITY:
        if (sizeof(struct haitai_vel) == p->len - 2)
        {
            std_msgs::msg::Float32 haitai_vel_msg;
            auto haitai_vel = reinterpret_cast<struct haitai_vel *>(p->data);
            haitai_vel_msg.data = haitai_vel->velocity;
            if (spindle_vel_pub_)
            {
                spindle_vel_pub_->publish(haitai_vel_msg);
            }
        }
        break;

    case PACK_TYPE_EMM_V5_VELOCITY:
        if (sizeof(struct EmmV5_vel) == p->len - 2)
        {
            std_msgs::msg::Float32 emm_v5_msg;
            auto emm_v5 = reinterpret_cast<struct EmmV5_vel *>(p->data);
            emm_v5_msg.data = emm_v5->position;
            if (lift_actuator_position_pub_)
            {
                lift_actuator_position_pub_->publish(emm_v5_msg);
            }
        }
        break;

    case PACK_TYPE_DM_MOTOR_FEEDBACK:
        if (sizeof(struct dm_motor_feedback_array) == p->len - 2)
        {
            const auto feedback = reinterpret_cast<struct dm_motor_feedback_array *>(p->data);
            tianbot_core::msg::DmMotorFeedbackArray feedback_msg;

            feedback_msg.header.stamp = clock_->now();
            feedback_msg.header.frame_id = base_frame_;
            feedback_msg.control_mode = dynDmControlModeToString(feedback->control_mode);
            for (int i = 0; i < 4; ++i)
            {
                tianbot_core::msg::DmMotorFeedback motor_msg;
                motor_msg.id = feedback->motors[i].id;
                motor_msg.state = dynMotorStateToString(feedback->motors[i].state);
                motor_msg.control_mode = dynDmControlModeToString(feedback->motors[i].control_mode);
                motor_msg.output_speed_rad_s = feedback->motors[i].output_speed_rad_s;
                motor_msg.output_torque_nm = feedback->motors[i].output_torque_nm;
                motor_msg.mos_temp_c = feedback->motors[i].mos_temp_c;
                motor_msg.coil_temp_c = feedback->motors[i].coil_temp_c;
                motor_msg.last_feedback_age_ms = feedback->motors[i].last_feedback_age_ms;
                feedback_msg.motors[i] = motor_msg;
            }
            if (motor_feedback_pub_)
            {
                motor_feedback_pub_->publish(feedback_msg);
            }
            onMotorFeedback(*feedback);
        }
        else if (sizeof(struct dm_motor_feedback_array_compact) == p->len - 2)
        {
            const auto feedback = reinterpret_cast<struct dm_motor_feedback_array_compact *>(p->data);
            tianbot_core::msg::DmMotorFeedbackArray feedback_msg;
            dm_motor_feedback_array feedback_full = {};

            fillCompactMotorFeedbackMsg(*feedback, clock_->now(), base_frame_, feedback_msg);
            if (motor_feedback_pub_)
            {
                motor_feedback_pub_->publish(feedback_msg);
            }

            feedback_full.control_mode = feedback->control_mode;
            for (int i = 0; i < 4; ++i)
            {
                feedback_full.motors[i].id = feedback->motors[i].id;
                feedback_full.motors[i].state = feedback->motors[i].state;
                feedback_full.motors[i].control_mode = feedback->motors[i].control_mode;
                feedback_full.motors[i].output_speed_rad_s = feedback->motors[i].output_speed_rad_s;
                feedback_full.motors[i].output_torque_nm = feedback->motors[i].output_torque_nm;
                feedback_full.motors[i].mos_temp_c = feedback->motors[i].mos_temp_c;
                feedback_full.motors[i].coil_temp_c = feedback->motors[i].coil_temp_c;
                feedback_full.motors[i].last_feedback_age_ms = feedback->motors[i].last_feedback_age_ms;
            }
            onMotorFeedback(feedback_full);
        }
        else
        {
            RCLCPP_WARN(this->node->get_logger(),
                        "DYN dm motor feedback size mismatch, expect %zu or %zu got %u",
                        sizeof(struct dm_motor_feedback_array),
                        sizeof(struct dm_motor_feedback_array_compact), p->len - 2);
        }
        break;

    case PACK_TYPE_ROVER_MOTION_MODE_STATUS:
        if (sizeof(struct rover_motion_mode_status) == p->len - 2)
        {
            const auto status = reinterpret_cast<struct rover_motion_mode_status *>(p->data);
            tianbot_core::msg::RoverMotionModeStatus status_msg;
            bool computed_ready = status->ready != 0;

            status_msg.header.stamp = clock_->now();
            status_msg.header.frame_id = base_frame_;
            status_msg.chassis_mode = dynChassisModeToString(status->chassis_mode);
            status_msg.dm_mode = dynDmControlModeToString(status->dm_mode);
            for (int i = 0; i < 4; ++i)
            {
                status_msg.motor_ctrl_mode[i] = dynDmControlModeToString(status->motor_ctrl_mode[i]);
                status_msg.motor_state[i] = dynMotorStateToString(status->motor_state[i]);
                if (status->motor_ctrl_mode[i] != status->dm_mode || status->motor_state[i] != 1U)
                {
                    computed_ready = false;
                }
            }
            status_msg.ready = computed_ready;
            if (motion_mode_status_pub_)
            {
                motion_mode_status_pub_->publish(status_msg);
            }
            onMotionModeStatus(*status);
        }
        else if (sizeof(struct rover_motion_mode_status_compact) == p->len - 2)
        {
            const auto status = reinterpret_cast<struct rover_motion_mode_status_compact *>(p->data);
            tianbot_core::msg::RoverMotionModeStatus status_msg;
            rover_motion_mode_status status_full = {};

            fillCompactMotionModeStatusMsg(*status, clock_->now(), base_frame_, status_msg);
            if (motion_mode_status_pub_)
            {
                motion_mode_status_pub_->publish(status_msg);
            }

            status_full.chassis_mode = status->chassis_mode;
            status_full.dm_mode = status->dm_mode;
            status_full.ready = status_msg.ready ? 1U : 0U;
            for (int i = 0; i < 4; ++i)
            {
                status_full.motor_ctrl_mode[i] = status->motor_ctrl_mode[i];
                status_full.motor_state[i] = status->motor_state[i];
            }
            onMotionModeStatus(status_full);
        }
        else
        {
            RCLCPP_WARN(this->node->get_logger(),
                        "DYN motion mode status size mismatch, expect %zu or %zu got %u",
                        sizeof(struct rover_motion_mode_status),
                        sizeof(struct rover_motion_mode_status_compact), p->len - 2);
        }
        break;

    case PACK_TYPE_HEART_BEAT_RESPONSE:
        break;

    case PACK_TYPE_IMU_REPONSE:
        if (sizeof(struct imu_feedback) == p->len - 2)
        {
            sensor_msgs::msg::Imu imu_msg;
            auto pImu = reinterpret_cast<struct imu_feedback *>(p->data);

            imu_msg.header.stamp = clock_->now();
            // imu_msg.header.frame_id = (nh_.getNamespace() + "/" + imu_frame_).erase(0,1);
            imu_msg.header.frame_id = imu_frame_;
            imu_msg.orientation.x = pImu->quat.x;
            imu_msg.orientation.y = pImu->quat.y;
            imu_msg.orientation.z = pImu->quat.z;
            imu_msg.orientation.w = pImu->quat.w;
            imu_msg.angular_velocity.x = pImu->angular_vel.x;
            imu_msg.angular_velocity.y = pImu->angular_vel.y;
            imu_msg.angular_velocity.z = pImu->angular_vel.z;
            imu_msg.linear_acceleration.x = pImu->linear_acc.x;
            imu_msg.linear_acceleration.y = pImu->linear_acc.y;
            imu_msg.linear_acceleration.z = pImu->linear_acc.z;
            imu_pub_->publish(imu_msg);
        }
        break;

    case PACK_TYPE_DEBUG_RESPONSE: {
            std_msgs::msg::String debug_msg;
            p->data[p->len - 2] = '\0';
            debug_msg.data = (char *)(p->data);
            {
                std::lock_guard<std::mutex> debug_result_lock(debug_result_mutex_);
                debugResultStr_ = (char *)(p->data);
                debugResultFlag_ = true;
            }
            debug_result_cv_.notify_all();
            debug_result_pub_->publish(debug_msg);
        }
        break;

    default:
        break;
    }
}

TianbotChasis::TianbotChasis(const std::shared_ptr<rclcpp::Node> & node)
    : TianbotCore(node), publisher_init_done(false)
{
    if (!node->get_parameter("base_frame", base_frame_)) {
        base_frame_ = DEFAULT_BASE_FRAME;
    }
    if (!node->get_parameter("odom_frame", odom_frame_)) {
        odom_frame_ = DEFAULT_ODOM_FRAME;
    }
    if (!node->get_parameter("imu_frame", imu_frame_)) {
        imu_frame_ = DEFAULT_IMU_FRAME;
    }
    if (!node->get_parameter("publish_tf", publish_tf_)) {
        publish_tf_ = DEFAULT_PUBLISH_TF;
    }
 
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    uwb_pub_ = node->create_publisher<geometry_msgs::msg::Pose2D>("uwb", 1);
    voltage_pub_ = node->create_publisher<std_msgs::msg::Float32>("voltage", 1);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    publisher_init_done = true;

    odom_tf_.header.frame_id = odom_frame_;
    odom_tf_.child_frame_id = base_frame_;
}
