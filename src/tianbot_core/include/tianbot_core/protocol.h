#ifndef _PROTOCOL_
#define _PROTOCOL_

#include "stdint.h"
#include <vector>

using namespace std;

#define PROTOCOL_HEAD 0xAA55

enum
{
    PACK_TYPE_HEART_BEAT = 0x0000,
    PACK_TYPE_CMD_VEL,
    PACK_TYPE_ACKMAN_VEL,
    PACK_TYPE_SIGNAL_CTRL,
    PACK_TYPE_ACTUATOR_CTRL,
    PACK_TYPE_HAITAI_CTRL,
    PACK_TYPE_LINE_OPTO_CTRL,
    PACK_TYPE_EMM_V5_CTRL,
    PACK_TYPE_SET_ROVER_MOTION_MODE,
    PACK_TYPE_DM_MIT_CMD,
    PACK_TYPE_DM_SPEED_CMD,
    PACK_TYPE_DEBUG = 0x4000,
    PACK_TYPE_ODOM_RESPONSE = 0x8000,
    PACK_TYPE_UWB_RESPONSE,
    PACK_TYPE_HEART_BEAT_RESPONSE,
    PACK_TYPE_IMU_REPONSE,
    PACK_TYPE_ODOM_V2_RESPONSE,
    PACK_TYPE_DM_MOTOR_FEEDBACK = 0x800A,
    PACK_TYPE_ROVER_MOTION_MODE_STATUS,
    PACK_TYPE_DEBUG_RESPONSE = 0xC000,
    PACK_TYPE_Voltage_RESPONSE,
    PACK_TYPE_SIGNAL_STATUS,
    PACK_TYPE_ACTUATOR_STATUS,
    PACK_TYPE_HAITAI_VELOCITY,
    PACK_TYPE_EMM_V5_VELOCITY,

};

struct vector3
{
    float x;
    float y;
    float z;
};

struct twist
{
    struct vector3 linear;
    struct vector3 angular;
};

struct quaternion
{
    float x;
    float y;
    float z;
    float w;
};

struct pose
{
    struct vector3 point;
    float yaw;
};

struct pose_v2
{
    struct vector3 point;
    struct vector3 rpy;
};

struct odom
{
    struct pose pose;
    struct twist twist;
};

struct odom_v2
{
    struct pose_v2 pose;
    struct twist twist;
};

struct uwb
{
    float x_m;
    float y_m;
    float yaw;
    // uint32_t sig_level;
};

struct imu_feedback
{
    struct quaternion quat;
    struct vector3 linear_acc;
    struct vector3 angular_vel;
};

struct ackermann_cmd
{
    float steering_angle;
    float speed;
};

struct voltage
{
    float Battery_voltage;
};

struct motion_mode
{
    uint32_t mode;
};

struct signal_status
{
    uint8_t red;
    uint8_t yellow;
    uint8_t green;
    uint8_t buzzer;
};

struct actuator_status
{
    uint8_t state;
};

struct line_opto_cmd
{
    uint32_t pulse_ms;
};

struct HaitaiCtrl_t
{
    float position;
    float velocity;
    float torque;
    float kp;
    float kd;
};

struct haitai_vel
{
    float velocity;
};

struct EmmV5Ctrl_t
{
    float distance_mm;
    uint16_t vel;
    uint8_t acc;
};

struct EmmV5_vel
{
    float position;
};

struct dm_speed_cmd
{
    float speed_rad_s[4];
};

struct dm_mit_cmd
{
    float p_des[4];
    float v_des[4];
    float kp[4];
    float kd[4];
    float t_ff[4];
};

struct dm_motor_feedback
{
    uint8_t id;
    uint8_t state;
    uint8_t control_mode;
    uint8_t reserved0;
    uint16_t raw_position;
    uint16_t raw_velocity;
    uint16_t raw_torque;
    uint16_t reserved1;
    float output_position_rad;
    float output_speed_rad_s;
    float output_torque_nm;
    float mos_temp_c;
    float coil_temp_c;
    float cmd_speed_rad_s;
    float cmd_tff_nm;
    float cmd_p_des_rad;
    float cmd_v_des_rad_s;
    float cmd_kp;
    float cmd_kd;
    uint32_t last_feedback_age_ms;
};

struct dm_motor_feedback_array
{
    uint32_t stamp_ms;
    uint8_t control_mode;
    uint8_t reserved[3];
    struct dm_motor_feedback motors[4];
};

struct __attribute__((packed)) dm_motor_feedback_compact
{
    uint8_t id;
    uint8_t state;
    uint8_t control_mode;
    float output_speed_rad_s;
    float output_torque_nm;
    float mos_temp_c;
    float coil_temp_c;
    uint32_t last_feedback_age_ms;
};

struct __attribute__((packed)) dm_motor_feedback_array_compact
{
    uint8_t control_mode;
    struct dm_motor_feedback_compact motors[4];
};

struct rover_motion_mode_status
{
    uint32_t stamp_ms;
    uint32_t chassis_mode;
    uint32_t pc_control_mode;
    uint32_t dm_mode;
    uint8_t ready;
    uint8_t ctrl_source;
    uint8_t reserved[2];
    uint8_t motor_ctrl_mode[4];
    uint8_t motor_state[4];
};

struct __attribute__((packed)) rover_motion_mode_status_compact
{
    uint32_t chassis_mode;
    uint32_t dm_mode;
    uint8_t ready;
    uint8_t motor_ctrl_mode[4];
    uint8_t motor_state[4];
};

struct protocol_pack
{
    uint16_t head;
    uint16_t len; // data len + 2 byte pack_type
    uint16_t pack_type;
    uint8_t data[]; // contain bcc byte
};

void buildCmd(vector<uint8_t> &buf, uint16_t cmd, uint8_t data[], uint8_t data_len);

#endif
