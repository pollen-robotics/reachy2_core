use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use smart_gripper_dynamixel::SmartGripperDynamixel;
mod smart_gripper_dynamixel;

#[macro_use]
extern crate lazy_static;

lazy_static! {
    static ref UID: Mutex<u32> = Mutex::new(0);
    static ref SMART_GRIPPER_DYNAMIXEL_CONTROLLER: Mutex<HashMap<u32, SmartGripperDynamixel>> =
        Mutex::new(HashMap::new());
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn smart_gripper_dynamixel_hwi_init(
    serial_port: *const libc::c_char,
    id: u8,
) -> u32 {
    let serial_port = unsafe { CStr::from_ptr(serial_port) }.to_str().unwrap();

    let c = SmartGripperDynamixel::new(serial_port, id).unwrap();

    let uid = get_available_uid();

    SMART_GRIPPER_DYNAMIXEL_CONTROLLER
        .lock()
        .unwrap()
        .insert(uid, c);

    uid
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn smart_gripper_dynamixel_hwi_get_position(uid: u32, position: &mut f64) -> i32 {
    match SMART_GRIPPER_DYNAMIXEL_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_current_position()
    {
        Ok(pos) => {
            *position = pos;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn smart_gripper_dynamixel_hwi_get_goal_position(
    uid: u32,
    target_position: &mut f64,
) -> i32 {
    match SMART_GRIPPER_DYNAMIXEL_CONTROLLER
        .lock()
        .unwrap()
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
pub extern "C" fn smart_gripper_dynamixel_hwi_set_target_position(
    uid: u32,
    target_position: f64,
) -> i32 {
    match SMART_GRIPPER_DYNAMIXEL_CONTROLLER
        .lock()
        .unwrap()
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
pub extern "C" fn smart_gripper_dynamixel_hwi_is_torque_on(uid: u32, is_on: &mut f64) -> i32 {
    match SMART_GRIPPER_DYNAMIXEL_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .is_torque_on()
    {
        Ok(torque) => {
            *is_on = if torque { 1.0 } else { 0.0 };
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn smart_gripper_dynamixel_hwi_set_torque(uid: u32, on: f64) -> i32 {
    let on = on != 0.0;
    match on {
        true => {
            match SMART_GRIPPER_DYNAMIXEL_CONTROLLER
                .lock()
                .unwrap()
                .get_mut(&uid)
                .unwrap()
                .enable_torque()
            {
                Ok(_) => 0,
                Err(_) => 1,
            }
        }
        false => {
            match SMART_GRIPPER_DYNAMIXEL_CONTROLLER
                .lock()
                .unwrap()
                .get_mut(&uid)
                .unwrap()
                .disable_torque()
            {
                Ok(_) => 0,
                Err(_) => 1,
            }
        }
    }
}

fn get_available_uid() -> u32 {
    let mut uid = UID.lock().unwrap();

    *uid += 1;
    *uid
}
