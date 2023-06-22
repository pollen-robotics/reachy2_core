use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use orbita2d_serial_controller::{Limit, Orbita2dController};

#[macro_use]
extern crate lazy_static;

lazy_static! {
    static ref UID: Mutex<u32> = Mutex::new(0);
    static ref ORBITA2D_SERIAL_CONTROLLER: Mutex<HashMap<u32, Orbita2dController>> =
        Mutex::new(HashMap::new());
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn orbita2d_serial_hwi_init(
    serial_port_1: *const libc::c_char,
    id_1: u8,
    serial_port_2: *const libc::c_char,
    id_2: u8,
    motors_offset: *mut f64,
    motors_ratio: *mut f64,
    orientation_limits: *mut f64,
) -> u32 {
    let serial_port_1 = unsafe { CStr::from_ptr(serial_port_1) }.to_str().unwrap();
    let serial_port_2 = unsafe { CStr::from_ptr(serial_port_2) }.to_str().unwrap();

    let motors_offset = unsafe { std::slice::from_raw_parts_mut(motors_offset, 2) };
    let motors_offset = (motors_offset[0] as f32, motors_offset[1] as f32);

    let motors_ratio = unsafe { std::slice::from_raw_parts_mut(motors_ratio, 2) };
    let motors_ratio = (motors_ratio[0] as f32, motors_ratio[1] as f32);

    let orientation_limits = unsafe { std::slice::from_raw_parts_mut(orientation_limits, 4) };
    let orientation_limits = Some((
        Limit {
            min: orientation_limits[0] as f32,
            max: orientation_limits[1] as f32,
        },
        Limit {
            min: orientation_limits[2] as f32,
            max: orientation_limits[3] as f32,
        },
    ));

    let c = Orbita2dController::new(
        (serial_port_1, serial_port_2),
        (id_1, id_2),
        motors_offset,
        motors_ratio,
        orientation_limits,
    )
    .unwrap();

    let uid = get_available_uid();

    ORBITA2D_SERIAL_CONTROLLER.lock().unwrap().insert(uid, c);

    uid
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn orbita2d_serial_hwi_get_orientation(uid: u32, orientation: *mut f64) -> i32 {
    let orientation = unsafe { std::slice::from_raw_parts_mut(orientation, 2) };

    match ORBITA2D_SERIAL_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_current_orientation()
    {
        Ok(o) => {
            orientation[0] = o.0 as f64;
            orientation[1] = o.1 as f64;

            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn orbita2d_serial_hwi_get_goal_orientation(
    uid: u32,
    target_orientation: *mut f64,
) -> i32 {
    let target_orientation = unsafe { std::slice::from_raw_parts_mut(target_orientation, 2) };

    match ORBITA2D_SERIAL_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_target_orientation()
    {
        Ok(o) => {
            target_orientation[0] = o.0 as f64;
            target_orientation[1] = o.1 as f64;

            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn orbita2d_serial_hwi_set_target_orientation(
    uid: u32,
    target_orientation: *mut f64,
) -> i32 {
    let target_orientation = unsafe { std::slice::from_raw_parts_mut(target_orientation, 2) };
    let target_orientation = (target_orientation[0] as f32, target_orientation[1] as f32);

    if ORBITA2D_SERIAL_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_target_orientation(target_orientation)
        .is_err()
    {
        return 1;
    }

    0
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn orbita2d_serial_hwi_is_torque_on(uid: u32, is_on: *mut f64) -> i32 {
    let is_on = unsafe { std::slice::from_raw_parts_mut(is_on, 2) };

    match ORBITA2D_SERIAL_CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .is_torque_on()
    {
        Ok(t) => {
            for i in 0..2 {
                is_on[i] = if t { 1.0 } else { 0.0 };
            }
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn orbita2d_serial_hwi_set_torque(uid: u32, on: *mut f64) -> i32 {
    let on = unsafe { std::slice::from_raw_parts_mut(on, 2) };

    // FIXME: This should not be done on each joint.
    let on = on[0] != 0.0;

    if on {
        match ORBITA2D_SERIAL_CONTROLLER
            .lock()
            .unwrap()
            .get_mut(&uid)
            .unwrap()
            .enable_torque()
        {
            Ok(_) => 0,
            Err(_) => 1,
        }
    } else {
        match ORBITA2D_SERIAL_CONTROLLER
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

fn get_available_uid() -> u32 {
    let mut uid = UID.lock().unwrap();

    *uid += 1;
    *uid
}
