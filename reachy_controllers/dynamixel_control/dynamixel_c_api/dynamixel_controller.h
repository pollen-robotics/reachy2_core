#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

extern "C" {

int32_t dynamixel_joint_from_config(uint32_t *uid, const char *configfile);

int32_t dynamixel_joint_is_torque_on(uint32_t uid, bool *is_on);

int32_t dynamixel_joint_enable_torque(uint32_t uid, bool reset_target);

int32_t dynamixel_joint_disable_torque(uint32_t uid);

int32_t dynamixel_joint_get_current_position(uint32_t uid, double *position);

int32_t dynamixel_joint_set_torque(uint32_t uid, const bool *torque);

int32_t dynamixel_joint_set_target_position(uint32_t uid, const double *position);

int32_t dynamixel_joint_get_target_position(uint32_t uid, double *target);

int32_t dynamixel_joint_get_current_velocity(uint32_t uid, double *velocity);

int32_t dynamixel_joint_get_current_torque(uint32_t uid, double *torque);

int32_t dynamixel_joint_set_target_torque(uint32_t uid, const double *torque);

int32_t dynamixel_joint_get_target_torque(uint32_t uid, double *torque);

int32_t dynamixel_joint_get_control_mode(uint32_t uid, uint8_t *mode);

int32_t dynamixel_joint_set_control_mode(uint32_t uid, const uint8_t *mode);

int32_t dynamixel_joint_get_motors_temperature(uint32_t uid, double *temperature);

int32_t dynamixel_joint_get_raw_motors_torque_limit(uint32_t uid, double *limit);

int32_t dynamixel_joint_get_raw_motors_velocity_limit(uint32_t uid, double *limit);

int32_t dynamixel_joint_set_raw_motors_torque_limit(uint32_t uid, double *limit);

int32_t dynamixel_joint_set_raw_motors_velocity_limit(uint32_t uid, double *limit);

int32_t dynamixel_2joints_from_config(uint32_t *uid, const char *configfile);

int32_t dynamixel_2joints_is_torque_on(uint32_t uid, bool (*is_on)[2]);

int32_t dynamixel_2joints_enable_torque(uint32_t uid, bool reset_target);

int32_t dynamixel_2joints_disable_torque(uint32_t uid);

int32_t dynamixel_2joints_get_current_position(uint32_t uid, double (*position)[2]);

int32_t dynamixel_2joints_set_torque(uint32_t uid, const bool (*torque)[2]);

int32_t dynamixel_2joints_set_target_position(uint32_t uid, const double (*position)[2]);

int32_t dynamixel_2joints_get_target_position(uint32_t uid, double (*target)[2]);

int32_t dynamixel_2joints_get_current_velocity(uint32_t uid, double (*velocity)[2]);

int32_t dynamixel_2joints_get_current_torque(uint32_t uid, double (*torque)[2]);

int32_t dynamixel_2joints_set_target_torque(uint32_t uid, const double (*torque)[2]);

int32_t dynamixel_2joints_get_target_torque(uint32_t uid, double (*torque)[2]);

int32_t dynamixel_2joints_get_control_mode(uint32_t uid, uint8_t (*mode)[2]);

int32_t dynamixel_2joints_set_control_mode(uint32_t uid, const uint8_t (*mode)[2]);

int32_t dynamixel_2joints_get_motors_temperature(uint32_t uid, double (*temperature)[2]);

int32_t dynamixel_2joints_get_raw_motors_torque_limit(uint32_t uid, double (*limit)[2]);

int32_t dynamixel_2joints_get_raw_motors_velocity_limit(uint32_t uid, double (*limit)[2]);

int32_t dynamixel_2joints_set_raw_motors_torque_limit(uint32_t uid, double (*limit)[2]);

int32_t dynamixel_2joints_set_raw_motors_velocity_limit(uint32_t uid, double (*limit)[2]);

} // extern "C"
