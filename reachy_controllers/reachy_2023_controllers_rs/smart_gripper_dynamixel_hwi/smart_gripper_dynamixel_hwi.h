#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

extern "C" {

uint32_t smart_gripper_dynamixel_hwi_init(const char *serial_port, uint8_t id);

int32_t smart_gripper_dynamixel_hwi_get_position(uint32_t uid, double *position);

int32_t smart_gripper_dynamixel_hwi_get_goal_position(uint32_t uid, double *target_position);

int32_t smart_gripper_dynamixel_hwi_set_target_position(uint32_t uid, double target_position);

int32_t smart_gripper_dynamixel_hwi_is_torque_on(uint32_t uid, double *is_on);

int32_t smart_gripper_dynamixel_hwi_set_torque(uint32_t uid, double on);

} // extern "C"
