// c-api
use dynamixel_controller::Dynamixel2Joints;
use dynamixel_controller::DynamixelJoint;

use crate::sync_map::SyncMap;
use env_logger;
// use log::debug;
use once_cell::sync::Lazy;
use std::{ffi::CStr, sync::Mutex};

fn print_error2(e: Box<dyn std::error::Error>) {
    log::debug!("[DYNAMIXEL_2JOINTS] Error: {:?}", e);
}

fn print_error(e: Box<dyn std::error::Error>) {
    log::debug!("[DYNAMIXEL_JOINT] Error: {:?}", e);
}
fn get_available_uid() -> u32 {
    let mut uid = UID.lock().unwrap();
    *uid += 1;
    *uid
}

pub fn convert<T, const N: usize>(arr: [Option<T>; N]) -> Option<[T; N]> {
    arr.try_map(|v| v)
}

static UID: Lazy<Mutex<u32>> = Lazy::new(|| Mutex::new(0));
static CONTROLLER2: Lazy<SyncMap<u32, Dynamixel2Joints>> = Lazy::new(SyncMap::new);
static CONTROLLER: Lazy<SyncMap<u32, DynamixelJoint>> = Lazy::new(SyncMap::new); // For the single joint case

// Single joint

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn dynamixel_joint_from_config(
    uid: &mut u32,
    configfile: *const libc::c_char,
) -> i32 {
    let configfile = unsafe { CStr::from_ptr(configfile) }.to_str().unwrap();

    let _ = env_logger::try_init();
    match DynamixelJoint::with_config_file(configfile) {
        Ok(controller) => {
            *uid = get_available_uid();
            CONTROLLER.insert(*uid, controller);
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_is_torque_on(uid: u32, is_on: &mut bool) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().is_torque_on() {
        Ok(torque) => {
            *is_on = torque;
            return 0;
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_enable_torque(uid: u32, reset_target: bool) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .enable_torque(reset_target)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_disable_torque(uid: u32) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().disable_torque() {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_get_current_position(uid: u32, position: &mut f64) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_current_orientation() {
        Ok(pos) => {
            *position = pos;
            return 0;
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_set_torque(uid: u32, torque: &bool) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_torque((*torque, true))
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_set_target_position(uid: u32, position: &f64) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_target_orientation(*position)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}
#[no_mangle]
pub extern "C" fn dynamixel_joint_get_target_position(uid: u32, target: &mut f64) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_target_orientation() {
        Ok(tgt) => {
            *target = tgt;
            return 0;
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_get_current_velocity(uid: u32, velocity: &mut f64) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_current_velocity() {
        Ok(vel) => {
            *velocity = vel;
            return 0;
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_get_current_torque(uid: u32, torque: &mut f64) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_current_torque() {
        Ok(t) => {
            *torque = t;
            return 0;
        }

        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_set_target_torque(uid: u32, torque: &f64) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().set_target_torque(*torque) {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_get_target_torque(uid: u32, torque: &mut f64) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_target_torque() {
        Ok(tq) => {
            *torque = tq;
            return 0;
        }

        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_get_control_mode(uid: u32, mode: &mut u8) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_control_mode() {
        Ok(m) => {
            *mode = m;
            return 0;
        }

        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_set_control_mode(uid: u32, mode: &u8) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().set_control_mode(*mode) {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_get_motors_temperature(uid: u32, temperature: &mut f64) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_motors_temperature() {
        Ok(temp) => {
            *temperature = temp;
            return 0;
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_get_raw_motors_torque_limit(uid: u32, limit: &mut f64) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_torque_limit()
    {
        Ok(tl) => {
            *limit = tl;
            return 0;
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_get_raw_motors_velocity_limit(uid: u32, limit: &mut f64) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_velocity_limit()
    {
        Ok(vl) => {
            *limit = vl;
            return 0;
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

//
#[no_mangle]
pub extern "C" fn dynamixel_joint_set_raw_motors_torque_limit(uid: u32, limit: &mut f64) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_torque_limit(*limit)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_joint_set_raw_motors_velocity_limit(uid: u32, limit: &mut f64) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_velocity_limit(*limit)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

// Two joints
#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn dynamixel_2joints_from_config(
    uid: &mut u32,
    configfile: *const libc::c_char,
) -> i32 {
    let configfile = unsafe { CStr::from_ptr(configfile) }.to_str().unwrap();

    let _ = env_logger::try_init();
    match Dynamixel2Joints::with_config_file(configfile) {
        Ok(controller) => {
            *uid = get_available_uid();
            CONTROLLER2.insert(*uid, controller);
            0
        }
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_is_torque_on(uid: u32, is_on: &mut [bool; 2]) -> i32 {
    match CONTROLLER2.get_mut(&uid).unwrap().is_torque_on() {
        Ok(torque) => match convert(torque) {
            Some(t) => {
                *is_on = t;
                return 0;
            }
            None => {
                return 1;
            }
        },
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_enable_torque(uid: u32, reset_target: bool) -> i32 {
    match CONTROLLER2
        .get_mut(&uid)
        .unwrap()
        .set_torque([(true, reset_target), (true, reset_target)])
    {
        Ok(_) => 0,
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_disable_torque(uid: u32) -> i32 {
    match CONTROLLER2
        .get_mut(&uid)
        .unwrap()
        .set_torque([(false, false), (false, false)])
    {
        Ok(_) => 0,
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_get_current_position(uid: u32, position: &mut [f64; 2]) -> i32 {
    match CONTROLLER2.get_mut(&uid).unwrap().get_current_position() {
        Ok(pos) => match convert(pos) {
            Some(p) => {
                *position = p;
                return 0;
            }
            None => {
                return 1;
            }
        },
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_set_torque(uid: u32, torque: &[bool; 2]) -> i32 {
    match CONTROLLER2
        .get_mut(&uid)
        .unwrap()
        .set_torque([(torque[0], true), (torque[1], true)])
    {
        Ok(_) => 0,
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_set_target_position(uid: u32, position: &[f64; 2]) -> i32 {
    match CONTROLLER2
        .get_mut(&uid)
        .unwrap()
        .set_target_position(*position)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error2(e);
            1
        }
    }
}
#[no_mangle]
pub extern "C" fn dynamixel_2joints_get_target_position(uid: u32, target: &mut [f64; 2]) -> i32 {
    match CONTROLLER2.get_mut(&uid).unwrap().get_target_position() {
        Ok(tgt) => match convert(tgt) {
            Some(t) => {
                *target = t;
                return 0;
            }
            None => {
                return 1;
            }
        },
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_get_current_velocity(uid: u32, velocity: &mut [f64; 2]) -> i32 {
    match CONTROLLER2.get_mut(&uid).unwrap().get_current_velocity() {
        Ok(vel) => match convert(vel) {
            Some(v) => {
                *velocity = v;
                return 0;
            }
            None => {
                return 1;
            }
        },
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_get_current_torque(uid: u32, torque: &mut [f64; 2]) -> i32 {
    match CONTROLLER2.get_mut(&uid).unwrap().get_current_torque() {
        Ok(t) => match convert(t) {
            Some(t) => {
                *torque = t;
                return 0;
            }
            None => {
                return 1;
            }
        },

        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_set_target_torque(uid: u32, torque: &[f64; 2]) -> i32 {
    match CONTROLLER2
        .get_mut(&uid)
        .unwrap()
        .set_target_torque(*torque)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_get_target_torque(uid: u32, torque: &mut [f64; 2]) -> i32 {
    match CONTROLLER2.get_mut(&uid).unwrap().get_target_torque() {
        Ok(tq) => match convert(tq) {
            Some(t) => {
                *torque = t;
                return 0;
            }
            None => {
                return 1;
            }
        },

        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_get_control_mode(uid: u32, mode: &mut [u8; 2]) -> i32 {
    match CONTROLLER2.get_mut(&uid).unwrap().get_control_mode() {
        Ok(m) => match convert(m) {
            Some(m) => {
                *mode = m;
                return 0;
            }
            None => {
                return 1;
            }
        },

        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_set_control_mode(uid: u32, mode: &[u8; 2]) -> i32 {
    match CONTROLLER2.get_mut(&uid).unwrap().set_control_mode(*mode) {
        Ok(_) => 0,
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_get_motors_temperature(
    uid: u32,
    temperature: &mut [f64; 2],
) -> i32 {
    match CONTROLLER2.get_mut(&uid).unwrap().get_motors_temperature() {
        Ok(temp) => match convert(temp) {
            Some(t) => {
                *temperature = t;
                return 0;
            }
            None => {
                return 1;
            }
        },
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_get_raw_motors_torque_limit(
    uid: u32,
    limit: &mut [f64; 2],
) -> i32 {
    match CONTROLLER2
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_torque_limit()
    {
        Ok(tl) => match convert(tl) {
            Some(t) => {
                *limit = t;
                return 0;
            }
            None => {
                return 1;
            }
        },
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_get_raw_motors_velocity_limit(
    uid: u32,
    limit: &mut [f64; 2],
) -> i32 {
    match CONTROLLER2
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_velocity_limit()
    {
        Ok(tl) => match convert(tl) {
            Some(t) => {
                *limit = t;
                return 0;
            }
            None => {
                return 1;
            }
        },
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

//
#[no_mangle]
pub extern "C" fn dynamixel_2joints_set_raw_motors_torque_limit(
    uid: u32,
    limit: &mut [f64; 2],
) -> i32 {
    match CONTROLLER2
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_torque_limit(*limit)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error2(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn dynamixel_2joints_set_raw_motors_velocity_limit(
    uid: u32,
    limit: &mut [f64; 2],
) -> i32 {
    match CONTROLLER2
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_velocity_limit(*limit)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error2(e);
            1
        }
    }
}
