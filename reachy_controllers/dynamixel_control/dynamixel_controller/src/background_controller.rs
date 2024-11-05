use std::{
    error::Error,
    sync::{Arc, RwLock},
    thread,
};

use log::debug;
use log::error;

use crate::controller::ForegroundDynamixelController;
use crate::DxlConfig;

pub struct BackgroundDynamixelController {
    last_read_torque: Arc<RwLock<[Option<bool>; 2]>>,
    last_write_torque: Arc<RwLock<Option<[(bool, bool); 2]>>>,

    last_read_current_position: Arc<RwLock<[Option<f64>; 2]>>,
    last_read_current_velocity: Arc<RwLock<[Option<f64>; 2]>>,
    last_read_current_torque: Arc<RwLock<[Option<f64>; 2]>>,

    last_read_target_position: Arc<RwLock<[Option<f64>; 2]>>,
    last_write_target_position: Arc<RwLock<Option<[f64; 2]>>>,

    last_read_target_torque: Arc<RwLock<[Option<f64>; 2]>>,
    last_write_target_torque: Arc<RwLock<Option<[f64; 2]>>>,

    last_read_control_mode: Arc<RwLock<[Option<u8>; 2]>>,
    last_write_control_mode: Arc<RwLock<Option<[u8; 2]>>>,

    last_read_raw_motors_velocity_limit: Arc<RwLock<[Option<f64>; 2]>>,
    last_write_raw_motors_velocity_limit: Arc<RwLock<Option<[f64; 2]>>>,

    last_read_raw_motors_torque_limit: Arc<RwLock<[Option<f64>; 2]>>,
    last_write_raw_motors_torque_limit: Arc<RwLock<Option<[f64; 2]>>>,

    last_read_motors_temperature: Arc<RwLock<[Option<f64>; 2]>>,
}

impl BackgroundDynamixelController {
    pub fn with_config(
        left_config: DxlConfig,
        right_config: DxlConfig,
    ) -> Result<BackgroundDynamixelController, Box<dyn Error>> {
        let mut inner = ForegroundDynamixelController::with_config(left_config, right_config)?;

        let last_read_torque = Arc::new(RwLock::new(inner.is_torque_on()?));
        let last_write_torque = Arc::new(RwLock::new(None));

        let last_read_control_mode = Arc::new(RwLock::new(inner.get_control_mode()?));
        let last_write_control_mode = Arc::new(RwLock::new(None));

        let last_read_current_position = Arc::new(RwLock::new(inner.get_current_position()?));
        let last_read_current_velocity = Arc::new(RwLock::new(inner.get_current_velocity()?));
        let last_read_current_torque = Arc::new(RwLock::new(inner.get_current_torque()?));

        let last_read_target_position = Arc::new(RwLock::new(inner.get_target_position()?));
        let last_write_target_position = Arc::new(RwLock::new(None));

        let last_read_target_torque = Arc::new(RwLock::new(inner.get_target_torque()?));
        let last_write_target_torque = Arc::new(RwLock::new(None));

        let last_read_raw_motors_velocity_limit =
            Arc::new(RwLock::new(inner.get_raw_motors_velocity_limit()?));
        let last_write_raw_motors_velocity_limit = Arc::new(RwLock::new(None));

        let last_read_raw_motors_torque_limit =
            Arc::new(RwLock::new(inner.get_raw_motors_torque_limit()?));
        let last_write_raw_motors_torque_limit = Arc::new(RwLock::new(None));

        let last_read_motors_temperature = Arc::new(RwLock::new(inner.get_motors_temperature()?));

        thread::spawn({
            let last_read_torque = last_read_torque.clone();
            let last_write_torque = last_write_torque.clone();

            let last_read_control_mode = last_read_control_mode.clone();
            let last_write_control_mode = last_write_control_mode.clone();

            let last_read_current_position = last_read_current_position.clone();
            let last_read_current_velocity = last_read_current_velocity.clone();
            let last_read_current_torque = last_read_current_torque.clone();

            let last_read_target_position = last_read_target_position.clone();
            let last_write_target_position = last_write_target_position.clone();

            let last_read_target_torque = last_read_target_torque.clone();
            let last_write_target_torque = last_write_target_torque.clone();

            let last_read_raw_motors_velocity_limit = last_read_raw_motors_velocity_limit.clone();
            let last_write_raw_motors_velocity_limit = last_write_raw_motors_velocity_limit.clone();
            let last_read_raw_motors_torque_limit = last_read_raw_motors_torque_limit.clone();
            let last_write_raw_motors_torque_limit = last_write_raw_motors_torque_limit.clone();

            let last_read_motors_temperature = last_read_motors_temperature.clone();
            let mut loop_counter: u32 = 0;
            move || loop {
                let tic = std::time::Instant::now();

                // Write and read torques ON/OFF
                let torque = { *last_write_torque.read().unwrap() };
                if let Some(t) = torque {
                    match inner.set_torque(t) {
                        Ok(_) => {}
                        Err(e) => {
                            error!("Error when writing torque: {}", e);
                        }
                    }
                }
                match inner.is_torque_on() {
                    Ok(t) => {
                        *last_read_torque.write().unwrap() = t;
                    }
                    Err(e) => {
                        error!("Error when reading torque: {}", e);
                    }
                }

                match inner.get_target_position() {
                    Ok(p) => {
                        *last_read_target_position.write().unwrap() = p;
                    }
                    Err(e) => {
                        error!("Error when reading target position: {}", e);
                    }
                }

                let target = { *last_write_target_position.read().unwrap() };

                if let Some(target) = target {
                    // let fb = inner.set_target_position_fb(target);
                    // if let Ok(fb) = fb {
                    //     *last_read_current_position.write().unwrap() = fb;
                    // } else {
                    //     //In case it completely fails (FIXME: disabled)
                    //     match inner.get_current_position() {
                    //         Ok(p) => {
                    //             *last_read_current_position.write().unwrap() = p;
                    //         }
                    //         Err(e) => {
                    //             error!("Error when reading current position: {}", e);
                    //         }
                    //     }
                    // }

                    let _ = inner.set_target_position(target);
                    match inner.get_current_position() {
                        Ok(p) => {
                            *last_read_current_position.write().unwrap() = p;
                        }
                        Err(e) => {
                            error!("Error when reading current position: {}", e);
                        }
                    }
                }

                /*
                       match inner.get_target_torque() {
                           Ok(p) => {
                               *last_read_target_torque.write().unwrap() = p;
                           }
                           Err(e) => {
                               error!("Error when reading target torque: {}", e);
                           }
                       }

                       let target_t = { *last_write_target_torque.read().unwrap() };

                       if let Some(target) = target_t {
                           match inner.set_target_torque(target) {
                               Ok(_) => {}
                               Err(e) => {
                                   error!("Error when writing target torque: {}", e);
                               }
                           }
                       }
                */
                match inner.get_current_velocity() {
                    Ok(v) => {
                        *last_read_current_velocity.write().unwrap() = v;
                    }
                    Err(e) => {
                        error!("Error when reading current velocity: {}", e);
                    }
                }

                match inner.get_current_torque() {
                    Ok(v) => {
                        *last_read_current_torque.write().unwrap() = v;
                    }
                    Err(e) => {
                        error!("Error when reading current torque: {}", e);
                    }
                }

                //Kind of slow register
                if loop_counter == 100 {
                    let velocity_limit = { *last_write_raw_motors_velocity_limit.read().unwrap() };
                    if let Some(velocity_limit) = velocity_limit {
                        match inner.set_raw_motors_velocity_limit(velocity_limit) {
                            Ok(_) => {}
                            Err(e) => {
                                error!("Error when writing velocity limit: {}", e);
                            }
                        }
                    }
                    match inner.get_raw_motors_velocity_limit() {
                        Ok(v) => {
                            *last_read_raw_motors_velocity_limit.write().unwrap() = v;
                        }
                        Err(e) => {
                            error!("Error when reading velocity limit: {}", e);
                        }
                    }

                    let torque_limit = { *last_write_raw_motors_torque_limit.read().unwrap() };
                    if let Some(torque_limit) = torque_limit {
                        match inner.set_raw_motors_torque_limit(torque_limit) {
                            Ok(_) => {}
                            Err(e) => {
                                error!("Error when writing torque limit: {}", e);
                            }
                        }
                    }
                    match inner.get_raw_motors_torque_limit() {
                        Ok(t) => {
                            *last_read_raw_motors_torque_limit.write().unwrap() = t;
                        }
                        Err(e) => {
                            error!("Error when reading torque limit: {}", e);
                        }
                    }

                    let mode = { *last_write_control_mode.read().unwrap() };
                    if let Some(m) = mode {
                        match inner.set_control_mode(m) {
                            Ok(_) => {}
                            Err(e) => {
                                error!("Error when writing control mode: {}", e);
                            }
                        }
                    }
                    match inner.get_control_mode() {
                        Ok(s) => {
                            *last_read_control_mode.write().unwrap() = s;
                        }
                        Err(e) => {
                            error!("Error when reading control mode: {}", e);
                        }
                    }

                    match inner.get_motors_temperature() {
                        Ok(t) => {
                            *last_read_motors_temperature.write().unwrap() = t;
                        }
                        Err(e) => {
                            error!("Error when reading motors temperature: {}", e);
                        }
                    }
                    loop_counter = 0;
                } else {
                    loop_counter += 1;
                }

                let toc = tic.elapsed();
                debug!("BackgroundDynamixelController loop duration: {:?}", toc);
            }
        });

        Ok(BackgroundDynamixelController {
            last_read_torque,
            last_write_torque,

            last_read_current_position,
            last_read_current_velocity,
            last_read_current_torque,

            last_read_target_position,
            last_write_target_position,

            last_read_target_torque,
            last_write_target_torque,

            last_read_control_mode,
            last_write_control_mode,

            last_read_raw_motors_velocity_limit,
            last_write_raw_motors_velocity_limit,
            last_read_raw_motors_torque_limit,
            last_write_raw_motors_torque_limit,
            last_read_motors_temperature,
        })
    }

    pub fn is_torque_on(&mut self) -> Result<[Option<bool>; 2], Box<dyn Error>> {
        Ok(*self.last_read_torque.read().unwrap())
    }
    pub fn set_torque(&mut self, torques: [(bool, bool); 2]) -> Result<(), Box<dyn Error>> {
        *self.last_write_torque.write().unwrap() = Some(torques);
        Ok(())
    }

    pub fn get_control_mode(&mut self) -> Result<[Option<u8>; 2], Box<dyn Error>> {
        Ok(*self.last_read_control_mode.read().unwrap())
    }
    pub fn set_control_mode(&mut self, modes: [u8; 2]) -> Result<(), Box<dyn Error>> {
        *self.last_write_control_mode.write().unwrap() = Some(modes);
        Ok(())
    }

    pub fn get_current_position(&mut self) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        Ok(*self.last_read_current_position.read().unwrap())
    }

    pub fn get_current_velocity(&mut self) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        Ok(*self.last_read_current_velocity.read().unwrap())
    }

    pub fn get_current_torque(&mut self) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        Ok(*self.last_read_current_torque.read().unwrap())
    }

    pub fn get_target_position(&mut self) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        Ok(*self.last_read_target_position.read().unwrap())
    }

    pub fn set_target_position(&mut self, target: [f64; 2]) -> Result<(), Box<dyn Error>> {
        *self.last_write_target_position.write().unwrap() = Some(target);
        Ok(())
    }

    pub fn get_target_torque(&mut self) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        Ok(*self.last_read_target_torque.read().unwrap())
    }

    pub fn set_target_torque(&mut self, target: [f64; 2]) -> Result<(), Box<dyn Error>> {
        *self.last_write_target_torque.write().unwrap() = Some(target);
        Ok(())
    }

    pub fn set_target_position_fb(
        &mut self,
        target: [f64; 2],
    ) -> Result<[Option<f64>; 2], Box<dyn Error>> {
        self.set_target_position(target)?;
        self.get_current_position()
    }

    pub fn get_raw_motors_velocity_limit(
        &mut self,
    ) -> Result<[Option<f64>; 2], Box<dyn std::error::Error>> {
        Ok(*self.last_read_raw_motors_velocity_limit.read().unwrap())
    }
    pub fn set_raw_motors_velocity_limit(
        &mut self,
        velocity_limit: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        *self.last_write_raw_motors_velocity_limit.write().unwrap() = Some(velocity_limit);
        Ok(())
    }

    pub fn get_raw_motors_torque_limit(
        &mut self,
    ) -> Result<[Option<f64>; 2], Box<dyn std::error::Error>> {
        Ok(*self.last_read_raw_motors_torque_limit.read().unwrap())
    }

    pub fn get_motors_temperature(
        &mut self,
    ) -> Result<[Option<f64>; 2], Box<dyn std::error::Error>> {
        Ok(*self.last_read_motors_temperature.read().unwrap())
    }

    pub fn set_raw_motors_torque_limit(
        &mut self,
        torque_limit: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        *self.last_write_raw_motors_torque_limit.write().unwrap() = Some(torque_limit);
        Ok(())
    }
}
