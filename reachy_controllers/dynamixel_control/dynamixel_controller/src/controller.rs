use crate::DxlConfig;

use crate::DynamixelJoint;
use log::error;
use std::error::Error;

pub struct ForegroundDynamixelController {
    pub left: DynamixelJoint,
    pub right: DynamixelJoint,
}

impl ForegroundDynamixelController {
    pub fn with_config(
        left_config: DxlConfig,
        right_config: DxlConfig,
    ) -> Result<ForegroundDynamixelController, Box<dyn Error>> {
        let left = DynamixelJoint::with_config(left_config)?;
        let right = DynamixelJoint::with_config(right_config)?;

        Ok(ForegroundDynamixelController {
            left: left,
            right: right,
        })
    }

    pub fn is_torque_on(&mut self) -> Result<[Option<bool>; 2], Box<dyn Error>> {
        let left_torque = self.left.is_torque_on().map_or_else(|_e| None, |v| Some(v));

        let right_torque = self
            .right
            .is_torque_on()
            .map_or_else(|_e| None, |v| Some(v));

        Ok([left_torque, right_torque])
    }

    pub fn set_torque(&mut self, torques: [(bool, bool); 2]) -> Result<(), Box<dyn Error>> {
        let mut res;
        let (torque, reset_target) = torques[0];
        if torque {
            res = self.left.enable_torque(reset_target);
        } else {
            res = self.left.disable_torque();
        }
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_torque (left dxl) {:?}", res),
        }

        let (torque, reset_target) = torques[1];
        if torque {
            res = self.right.enable_torque(reset_target);
        } else {
            res = self.right.disable_torque();
        }

        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_torque(right dxl) {:?}", res),
        }

        Ok(())
    }

    pub fn get_current_position(&mut self) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        let left_pos = self.left.get_current_orientation();

        let left_pos = match left_pos {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_current_position (left dxl) {:?}", e);
                None
            }
        };

        let right_pos = self.right.get_current_orientation();

        let right_pos = match right_pos {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_current_position (right dxl) {:?}", e);
                None
            }
        };

        Ok([left_pos, right_pos])
    }
    pub fn get_target_position(&mut self) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        let left_pos = self.left.get_target_orientation();

        let left_pos = match left_pos {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_target_position (left dxl) {:?}", e);
                None
            }
        };
        let right_pos = self.right.get_target_orientation();

        let right_pos = match right_pos {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_target_position (right dxl) {:?}", e);
                None
            }
        };

        Ok([left_pos, right_pos])
    }

    pub fn set_target_position(&mut self, target: [f64; 2]) -> Result<(), Box<dyn Error>> {
        let res = self.left.set_target_orientation(target[0]);
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_target_position (left dxl) {:?}", res),
        }
        let res = self.right.set_target_orientation(target[1]);
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_target_position (right dxl) {:?}", res),
        }

        Ok(())
    }

    pub fn get_current_torque(&mut self) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        let left = self.left.get_current_torque();
        let left = match left {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_current_torque (left dxl) {:?}", e);
                None
            }
        };
        let right = self.right.get_current_torque();
        let right = match right {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_current_torque (right dxl) {:?}", e);
                None
            }
        };

        Ok([left, right])
    }

    pub fn set_target_torque(&mut self, target: [f64; 2]) -> Result<(), Box<dyn Error>> {
        let res = self.left.set_target_torque(target[0]);
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_target_torque (left dxl) {:?}", res),
        }
        let res = self.right.set_target_torque(target[1]);
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_target_torque (right dxl) {:?}", res),
        }

        Ok(())
    }

    pub fn get_target_torque(&mut self) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        let left = self.left.get_target_torque();

        let left = match left {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_target_torque (left dxl) {:?}", e);
                None
            }
        };

        let right = self.right.get_target_torque();

        let right = match right {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_target_torque (right dxl) {:?}", e);
                None
            }
        };

        Ok([left, right])
    }

    pub fn get_current_velocity(&mut self) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        let left = self.left.get_current_velocity();

        let left = match left {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_current_velocity (left dxl) {:?}", e);
                None
            }
        };

        let right = self.right.get_current_velocity();

        let right = match right {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_current_velocity (right dxl) {:?}", e);
                None
            }
        };

        Ok([left, right])
    }

    // pub fn set_target_velocity(&mut self, target: [f64; 8]) -> Result<(), Box<dyn Error>> {
    //     Ok(())
    // }

    pub fn set_control_mode(&mut self, mode: [u8; 2]) -> Result<(), Box<dyn Error>> {
        let res = self.left.set_control_mode(mode[0]);
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_control_mode (left dxl) {:?}", res),
        }
        let res = self.right.set_control_mode(mode[1]);
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_control_mode (right dxl) {:?}", res),
        }

        Ok(())
    }

    pub fn get_control_mode(&mut self) -> Result<[Option<u8>; 2], Box<dyn Error>> {
        let left = self.left.get_control_mode();

        let left = match left {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_control_mode (left dxl) {:?}", e);
                None
            }
        };
        let right = self.right.get_control_mode();

        let right = match right {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_control_mode (right dxl) {:?}", e);
                None
            }
        };

        Ok([left, right])
    }

    pub fn get_raw_motors_velocity_limit(
        &mut self,
    ) -> Result<[Option<f64>; 2], Box<dyn std::error::Error>> {
        let left_velocity = self.left.get_raw_motors_velocity_limit();

        let left_velocity = match left_velocity {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_raw_motors_velocity_limit (left dxl) {:?}", e);
                None
            }
        };
        let right_velocity = self.right.get_raw_motors_velocity_limit();

        let right_velocity = match right_velocity {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_raw_motors_velocity_limit (right dxl) {:?}", e);
                None
            }
        };

        Ok([left_velocity, right_velocity])
    }
    pub fn set_raw_motors_velocity_limit(
        &mut self,
        velocity_limit: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let res = self.left.set_raw_motors_velocity_limit(velocity_limit[0]);
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_raw_motors_velocity_limit (left dxl) {:?}", res),
        }
        let res = self.right.set_raw_motors_velocity_limit(velocity_limit[1]);
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_raw_motors_velocity_limit (right dxl) {:?}", res),
        }

        Ok(())
    }

    pub fn get_raw_motors_torque_limit(
        &mut self,
    ) -> Result<[Option<f64>; 2], Box<dyn std::error::Error>> {
        let left_torque = self.left.get_raw_motors_torque_limit();

        let left_torque = match left_torque {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_raw_motors_torque_limit (left dxl) {:?}", e);
                None
            }
        };
        let right_torque = self.right.get_raw_motors_torque_limit();

        let right_torque = match right_torque {
            Ok(v) => Some(v),
            Err(e) => {
                error!("Error: get_raw_motors_torque_limit (right dxl) {:?}", e);
                None
            }
        };

        Ok([left_torque, right_torque])
    }

    pub fn get_motors_temperature(
        &mut self,
    ) -> Result<[Option<f64>; 2], Box<dyn std::error::Error>> {
        let left_temp = self.left.get_motors_temperature();

        let left_temp = match left_temp {
            Ok(t) => Some(t),
            Err(e) => {
                error!("Error: get_motors_temperature (left dxl) {:?}", e);
                None
            }
        };
        let right_temp = self.right.get_motors_temperature();

        let right_temp = match right_temp {
            Ok(t) => Some(t),
            Err(e) => {
                error!("Error: get_motors_temperature (right dxl) {:?}", e);
                None
            }
        };

        Ok([left_temp, right_temp])
    }

    pub fn set_raw_motors_torque_limit(
        &mut self,
        torque_limit: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let res = self.left.set_raw_motors_torque_limit(torque_limit[0]);
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_raw_motors_torque_limit (left dxl) {:?}", res),
        }

        let res = self.right.set_raw_motors_torque_limit(torque_limit[1]);
        match res {
            Ok(_) => {}
            Err(res) => error!("Error: set_raw_motors_torque_limit (right dxl) {:?}", res),
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use log::info;
    use std::{thread, time::Duration};

    use super::*;
    //To be run sequentially: cargo test -- --test-threads=1

    fn init() {
        let _ = env_logger::builder().is_test(true).try_init();
    }

    #[test]
    fn single_orbita3d() {
        init();
        let controller1 = Orbita3dController::with_config("config/dxl_poulpe3d.yaml");
        match controller1 {
            Ok(mut controller1) => {
                info!("Controller1 created");
                thread::sleep(Duration::from_millis(1));
                let ret = controller1
                    .is_torque_on()
                    .expect("Failed to get torque status");
                thread::sleep(Duration::from_millis(1));
                info!("Torque status: {}", ret);
            }
            Err(e) => panic!("Controller1 creation failed: {}", e),
        }
    }

    #[test]
    fn two_orbita() {
        init();
        let controller1 = Orbita3dController::with_config("config/dxl_poulpe3d.yaml");
        match controller1 {
            Ok(mut controller1) => {
                info!("Controller1 created");
                thread::sleep(Duration::from_millis(1));
                let ret = controller1
                    .is_torque_on()
                    .expect("Failed to get torque status");
                thread::sleep(Duration::from_millis(1));
                info!("Torque status: {}", ret);
            }
            Err(e) => panic!("Controller1 creation failed: {}", e),
        }
        let controller2 = Orbita2dController::with_config("config/dxl_poulpe2d.yaml");
        match controller2 {
            Ok(mut controller2) => {
                info!("Controller2 created");
                thread::sleep(Duration::from_millis(1));
                let ret = controller2
                    .is_torque_on()
                    .expect("Failed to get torque status");
                thread::sleep(Duration::from_millis(1));
                info!("Torque status: {}", ret);
            }
            Err(e) => panic!("Controller2 creation failed: {}", e),
        }
    }
}
