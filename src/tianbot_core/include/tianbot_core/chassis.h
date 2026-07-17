#ifndef __CHASIS_H__
#define __CHASIS_H__

#include <rclcpp/rclcpp.hpp>
#include "serial.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "tianbot_core/msg/dm_motor_feedback_array.hpp"
#include "tianbot_core/msg/rover_motion_mode_status.hpp"
#include "core.h"

#ifdef BUILD_BEFORE_HUMBLE
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
    #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#define DEFAULT_BASE_FRAME "base_link"
#define DEFAULT_ODOM_FRAME "odom"
#define DEFAULT_IMU_FRAME "imu_link"

#define DEFAULT_PUBLISH_TF true

using namespace std;

class TianbotChasis : public TianbotCore {
public:
    TianbotChasis(const std::shared_ptr<rclcpp::Node> & node);

protected:
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr stack_light_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr lift_actuator_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lift_actuator_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spindle_vel_pub_;
    rclcpp::Publisher<tianbot_core::msg::DmMotorFeedbackArray>::SharedPtr motor_feedback_pub_;
    rclcpp::Publisher<tianbot_core::msg::RoverMotionModeStatus>::SharedPtr motion_mode_status_pub_;
    virtual void onMotorFeedback(const struct dm_motor_feedback_array &feedback);
    virtual void onMotionModeStatus(const struct rover_motion_mode_status &status);

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr uwb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Clock::SharedPtr clock_;
    geometry_msgs::msg::TransformStamped odom_tf_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool publish_tf_;
    std::string base_frame_;
    std::string odom_frame_;
    std::string imu_frame_;
    bool publisher_init_done;
    virtual void tianbotDataProc(unsigned char *buf, int len);
};

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromRollPitchYaw(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

#endif
