use std::{ffi::CStr, sync::Mutex};

use gripper_dynamixel::GripperDynamixel;
use once_cell::sync::Lazy;
use sync_map::SyncMap;
pub mod gripper_dynamixel;
mod sync_map;

static UID: Lazy<Mutex<u32>> = Lazy::new(|| Mutex::new(0));
static CONTROLLER: Lazy<SyncMap<u32, GripperDynamixel>> = Lazy::new(SyncMap::new);

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn gripper_dynamixel_hwi_init(
    serial_port: *const libc::c_char,
    dxl_id: u8,
    uid: &mut u32,
) -> i32 {
    let serial_port = unsafe { CStr::from_ptr(serial_port) }.to_str().unwrap();

    match GripperDynamixel::new(serial_port, dxl_id) {
        Ok(gripper) => {
            *uid = get_available_uid();
            CONTROLLER.insert(*uid, gripper);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn gripper_dynamixel_hwi_get_position(uid: u32, position: &mut f64) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_current_position() {
        Ok(pos) => {
            *position = pos;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn gripper_dynamixel_hwi_get_goal_position(
    uid: u32,
    target_position: &mut f64,
) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_current_target_position()
    {
        Ok(pos) => {
            *target_position = pos;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn gripper_dynamixel_hwi_set_target_position(uid: u32, target_position: f64) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_target_position(target_position)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn gripper_dynamixel_hwi_is_torque_on(uid: u32, is_on: &mut f64) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().is_torque_on() {
        Ok(torque) => {
            *is_on = if torque { 1.0 } else { 0.0 };
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn gripper_dynamixel_hwi_set_torque(uid: u32, on: f64) -> i32 {
    let on = on != 0.0;
    match on {
        true => match CONTROLLER.get_mut(&uid).unwrap().enable_torque() {
            Ok(_) => 0,
            Err(_) => 1,
        },
        false => match CONTROLLER.get_mut(&uid).unwrap().disable_torque() {
            Ok(_) => 0,
            Err(_) => 1,
        },
    }
}

fn get_available_uid() -> u32 {
    let mut uid = UID.lock().unwrap();

    *uid += 1;
    *uid
}
