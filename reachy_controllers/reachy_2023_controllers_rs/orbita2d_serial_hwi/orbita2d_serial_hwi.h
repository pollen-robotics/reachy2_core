#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

extern "C" {

uint32_t orbita2d_serial_hwi_init(const char *serial_port_1,
                                  uint8_t id_1,
                                  const char *serial_port_2,
                                  uint8_t id_2,
                                  double *motors_offset,
                                  double *motors_ratio,
                                  double *orientation_limits);

int32_t orbita2d_serial_hwi_get_orientation(uint32_t uid, double *orientation);

int32_t orbita2d_serial_hwi_get_goal_orientation(uint32_t uid, double *target_orientation);

int32_t orbita2d_serial_hwi_set_target_orientation(uint32_t uid, double *target_orientation);

int32_t orbita2d_serial_hwi_is_torque_on(uint32_t uid, double *is_on);

int32_t orbita2d_serial_hwi_set_torque(uint32_t uid, double *on);

} // extern "C"
